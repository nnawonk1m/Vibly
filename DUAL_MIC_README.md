# Two INMP441 microphones on the XIAO ESP32-S3 — same model, new front end

The model is **unchanged**. `audio_model.h` is byte-identical to the single-mic build (same 6 188 parameters, same 0.9204 test macro-F1). Only the audio front end changed: the I2S bus now carries two microphones instead of one, and there is a fusion stage between the microphones and the classifier.

## Files

| Path | What it is |
|---|---|
| `xiao_dual_inmp441_inference/` | Flash this. Contains the sketch **and** `audio_model.h` — open the folder in Arduino IDE. |
| `xiao_dual_inmp441_capture/` | Bring-up + data capture firmware. **Flash this one first.** |
| `capture_wav_dual.py` | Records both mics to WAV over USB. Writes mic A, mic B and the average as separate mono files. |
| `predict_wav.py`, `model_weights.npz` | Unchanged from before — score the recordings on your PC. |

---

## 1. Wiring — one bus, three GPIOs

The INMP441 places its data in the left or the right half of the I2S frame depending on its `L/R` pin, and tri-states its output during the other half. Two microphones therefore share a **single** data line. This is the intended stereo pairing from the datasheet, not a workaround.

```
                    XIAO ESP32-S3
                   +--------------+
                   |              |
      3V3 <--------| 3V3      D2  |--------> SCK  (BCLK, 1.024 MHz)
      GND <--------| GND      D3  |--------> WS   (LRCLK, 16 kHz)
                   |          D4  |<-------- SD   (data, both mics)
                   +--------------+

   MIC A  (left slot)                    MIC B  (right slot)
   +----------------+                    +----------------+
   | VDD  --- 3V3   |                    | VDD  --- 3V3   |
   | GND  --- GND   |                    | GND  --- GND   |
   | L/R  --- GND   |  <-- LOW           | L/R  --- 3V3   |  <-- HIGH
   | SCK  --- D2    |                    | SCK  --- D2    |
   | WS   --- D3    |                    | WS   --- D3    |
   | SD   ---+      |                    | SD   ---+      |
   +---------|------+                    +---------|------+
             |                                     |
             +------------------+------------------+
                                |
                               D4
```

| Signal | MIC A | MIC B | XIAO pin |
|---|---|---|---|
| VDD | VDD | VDD | 3V3 |
| GND | GND | GND | GND |
| **L/R** | **GND** | **3V3** | — |
| SCK (BCLK) | SCK | SCK | D2 / GPIO3 |
| WS (LRCLK) | WS | WS | D3 / GPIO4 |
| SD (DOUT) | SD | SD | D4 / GPIO5 — **both joined** |

**The only difference between the two modules is the `L/R` pin.** Tie both the same way and you get one silent channel plus two chips driving the same time slot — which is also the single most common failure on this build.

Practical notes:

- 100 nF + 10 µF from VDD to GND at **each** module, as close to the pins as you can manage. A MEMS mic on a noisy rail raises your noise floor, and the audit already showed how sensitive this model's quiet classes are.
- Keep SCK / WS / SD under ~15 cm and away from USB and power leads.
- Pin numbering is unchanged from your working single-mic sketch — D2/D3/D4 = GPIO3/4/5.

### Microphone spacing

| Fusion mode | Spacing constraint |
|---|---|
| `MODE_BOTH`, `MODE_BEST`, `MODE_LEFT/RIGHT` | **Any.** Opposite ends of an enclosure, different rooms, whatever suits the application. |
| `MODE_MIX` | **3–6 cm.** Wider spacing comb-filters off-axis sources above ~3 kHz, which produces spectra the model never saw in training. |

---

## 2. The five fusion modes

Set `#define FUSION` near the top of the inference sketch.

| Mode | What it does | Inferences per hop | When to use it |
|---|---|---|---|
| **`MODE_BOTH`** (default) | Classifies each microphone separately, then takes the per-class maximum and renormalises | **2** | Best default. Redundant, and matches training exactly. |
| `MODE_MIX` | Averages the two channels, one inference | 1 | Cheapest. ~3 dB better SNR against mic self-noise. Needs close spacing. |
| `MODE_BEST` | Picks whichever mic has the higher window RMS, one inference | 1 | Half the compute of `MODE_BOTH` with most of the coverage benefit. |
| `MODE_LEFT` / `MODE_RIGHT` | One microphone only | 1 | Debugging, or falling back when a mic dies. |

**Why `MODE_BOTH` is the default.** The model was trained on single-microphone audio. Each channel here *is* single-microphone audio, so each inference is exactly in-distribution — no averaging, no comb filtering, no change in noise statistics. And a microphone that is covered, facing the wrong way, or wind-loaded simply loses the vote instead of dragging the good one down. That is the actual reason to fit a second mic.

The serial output tells you which microphone produced each verdict:

```
[MFE  36 ms | NN  82 ms] A -47.2 dB  B -46.8 dB  src:B |  baby_crying 0.02  human_voice 0.93  noise_none 0.04  traffic 0.01
>>> EVENT: human_voice   (loudest on mic B)
```

**Be clear about what two mics buy you:** redundancy, wider spatial coverage, a rough left/right cue, and ~3 dB on self-noise in `MODE_MIX`. They do **not** make the classifier smarter. Every accuracy caveat in `MODE_README.md` still holds — most importantly, this model has still never heard a real baby or a real horn through your microphone.

---

## 3. Bring-up, in order

### Step 1 — flash the capture firmware and identify the mics

Open `xiao_dual_inmp441_capture/`, leave `LEVEL_METER 1`, upload, Serial Monitor at **115200**.

```
A: RMS  -63.1 dBFS peak   1.2% clip    0   |   B: RMS  -62.4 dBFS peak   1.4% clip    0
```

| Test | Expected |
|---|---|
| Silence | both channels −60 to −70 dBFS |
| **Tap mic A** | the **A** column jumps, B barely moves |
| **Tap mic B** | the **B** column jumps, A barely moves |
| Speak at 30 cm | both peak 20–60 % FS, `clip 0` |

- **Columns respond to the wrong mic** → set `SWAP_CHANNELS 1` in *both* sketches and re-flash. The interleave order of the legacy I2S driver is not worth arguing with; just measure it.
- **One column never moves, and the meter prints `<-- B looks dead / miswired`** → that mic's `L/R` pin is tied the same way as the other one, or its SD/VDD/GND is not connected.
- **Both columns move together for either tap** → the two `L/R` pins are both low or both high, so both mics are driving the same slot. Fix `L/R`.
- **Peaks at 2–3 %** → lower `SHIFT_BITS` to 13. **Clipping on normal speech** → raise it to 15.

**Write your final `SHIFT_BITS` down.** It must be identical in the capture sketch, the inference sketch, and every recording you ever make.

### Step 2 — record

Set `LEVEL_METER 0`, re-upload, **close the Serial Monitor**, then:

```
pip install pyserial numpy
python capture_wav_dual.py --list
python capture_wav_dual.py -p COM7 -l human_voice -d 20 -n 10
python capture_wav_dual.py -p COM7 -l baby_crying -d 20 -n 10
python capture_wav_dual.py -p COM7 -l traffic     -d 20 -n 10
python capture_wav_dual.py -p COM7 -l noise_none  -d 20 -n 10
```

Each take produces three mono WAVs — `..._A.wav`, `..._B.wav`, `..._mix.wav` — plus per-file peak / RMS / clipping, and a warning if the two mics disagree by more than 12 dB.

### Step 3 — measure before you flash the model

```
python predict_wav.py -w model_weights.npz xiao_recordings/ --score
```

Run it three ways and compare — this is how you choose your fusion mode with evidence rather than a guess:

```
python predict_wav.py -w model_weights.npz xiao_recordings/*_A.wav   --score
python predict_wav.py -w model_weights.npz xiao_recordings/*_B.wav   --score
python predict_wav.py -w model_weights.npz xiao_recordings/*_mix.wav --score
```

If `mix` beats both singles, use `MODE_MIX` and save half the CPU. If the two singles differ a lot, one mic is better placed — fix the placement, or use `MODE_BOTH`.

### Step 4 — flash the classifier

Open `xiao_dual_inmp441_inference/` (the folder, so `audio_model.h` comes along), set `SHIFT_BITS` and `SWAP_CHANNELS` to match, upload.

Arduino IDE: **XIAO_ESP32S3**, **PSRAM = OPI PSRAM**, **CPU = 240 MHz**, **USB CDC On Boot = Enabled**, **Partition Scheme = 8M with spiffs**.

---

## 4. Budget

| | single mic | two mics |
|---|---|---|
| GPIOs | 3 | **3** (unchanged — that is the point of sharing the bus) |
| BCLK | 1.024 MHz | 1.024 MHz (unchanged; the frame always had two slots) |
| Ring buffers | 32 KB | 64 KB |
| Work buffer + features + model scratch | ~78 KB | ~78 KB (reused between channels) |
| **Total RAM** | ~110 KB | **~142 KB** of 512 KB |
| Flash | +26 KB of model | same |
| Compute per 250 ms hop | 1 × (MFE + NN) | **2 ×** in `MODE_BOTH`, 1 × otherwise |

`MODE_BOTH` roughly doubles the per-hop work. The single-mic build was estimated at 30–90 ms against a 250 ms budget, so two inferences should still fit — but **read the printed `[MFE … | NN …]` numbers rather than trusting that estimate.** If they approach 250 ms, in order of preference: raise `HOP_SAMPLES` to `8000` (500 ms), or switch to `MODE_BEST`.

Watch for `[!] overruns=` in the output. Non-zero means audio is being dropped and every prediction after that point is suspect.

---

## 5. Two-microphone troubleshooting

| Symptom | Cause | Fix |
|---|---|---|
| One channel flat at ~−90 dBFS | `L/R` not connected, or SD/VDD open on that module | check `L/R` is hard-tied to GND *or* 3V3, never floating |
| Both channels carry the same audio | both `L/R` pins tied the same way | one to GND, one to 3V3 |
| Channels behave as if swapped | driver interleave order | `SWAP_CHANNELS 1` in both sketches |
| Both channels noisy/garbled | two chips contending for the slot, or SD wire too long | fix `L/R` first; then shorten wires, add decoupling |
| One channel much noisier | bad ground or long unshielded run on that module | shorten, add 100 nF + 10 µF at that mic |
| Everything worked with one mic, breaks with two | you left `channel_format` as `ONLY_LEFT`, or `DMA_FRAMES` sizing | the dual sketch uses `I2S_CHANNEL_FMT_RIGHT_LEFT` and reads `DMA_FRAMES × 2` int32 per buffer — don't mix the two sketches' I2S blocks |
| `MODE_MIX` scores worse than either single mic | comb filtering from too-wide spacing | bring the mics to 3–6 cm, or use `MODE_BOTH` |

---

## 6. What I verified

- Both sketches compile clean under `g++ -Wall` against ESP-IDF/Arduino API stubs, in **every** configuration: all five `FUSION` modes, `SWAP_CHANNELS` 0 and 1, and `LEVEL_METER` 0 and 1.
- `capture_wav_dual.py` round-trips synthetic interleaved stereo into A / B / mix mono WAVs that `predict_wav.py` reads and classifies correctly.
- `audio_model.h` is unchanged and was already checked against the Keras model on all 826 test windows: max probability difference 1.07 × 10⁻⁶, 100 % argmax agreement.

I could not test against real hardware — the I2S channel interleave order is the one thing that genuinely needs measuring on your bench, which is exactly what Step 1 is for.
