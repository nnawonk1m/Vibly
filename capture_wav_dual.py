#!/usr/bin/env python3
"""
capture_wav_dual.py - host companion to xiao_dual_inmp441_capture.ino
                      (LEVEL_METER 0)

Records from BOTH INMP441s at once and writes 16 kHz / mono / 16-bit WAVs in
the format Edge Impulse and predict_wav.py expect. By default it saves three
files per take: mic A, mic B, and the average of the two — so you can compare
them and decide which fusion mode to run.

Install:  pip install pyserial
Usage:
    python capture_wav_dual.py --list
    python capture_wav_dual.py -p COM7 -l human_voice -d 20 -n 10
    python capture_wav_dual.py -p COM7 -l noise_none  -d 3600 -n 1 --name soak
    python capture_wav_dual.py -p COM7 -l traffic -d 20 -n 5 --channel mix
    python capture_wav_dual.py -p COM7 -l noise_none -d 20 -n 5 --stereo

Score what you recorded straight away:
    python predict_wav.py -w model_weights.npz xiao_recordings/ --score
"""

import argparse, array, math, os, re, sys, time, wave

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    sys.exit("pyserial is required:  pip install pyserial")

SR = 16000
SAMPLE_WIDTH = 2
MAGIC = b"EIWAVD2\n"                 # note: differs from the single-mic magic
FRAME_BYTES = 2 * SAMPLE_WIDTH       # one frame = A + B
BYTES_PER_SEC = SR * FRAME_BYTES


def next_start_index(outdir, label, base):
    """Look at existing files for this label/name in outdir and return the
    index to continue from, so recordings keep counting up (001, 002, 003, ...)
    across separate runs of the script instead of restarting at 001 each time."""
    prefix = f"{label}.{base}"
    pattern = re.compile(rf"^{re.escape(prefix)}_(\d+)_")
    best = 0
    if os.path.isdir(outdir):
        for fname in os.listdir(outdir):
            m = pattern.match(fname)
            if m:
                best = max(best, int(m.group(1)))
    return best + 1


def list_serial_ports():
    ports = list(list_ports.comports())
    if not ports:
        print("No serial ports found.")
        return
    for p in ports:
        print(f"  {p.device:20s}  {p.description}")


def wait_for_magic(ser, timeout=10.0):
    deadline = time.time() + timeout
    window = b""
    while time.time() < deadline:
        c = ser.read(1)
        if not c:
            continue
        window = (window + c)[-len(MAGIC):]
        if window == MAGIC:
            return True
    return False


def stats(a):
    if not len(a):
        return 0.0, 0.0, 0.0
    peak = max(1, max(abs(int(x)) for x in a))
    rms = (sum(float(x) * float(x) for x in a) / len(a)) ** 0.5
    clipped = sum(1 for x in a if abs(int(x)) >= 32700)
    return (100.0 * peak / 32768.0,
            20 * math.log10(max(rms, 1e-9) / 32768.0),
            100.0 * clipped / len(a))


def write_wav(path, samples, nch=1):
    with wave.open(path, "wb") as w:
        w.setnchannels(nch)
        w.setsampwidth(SAMPLE_WIDTH)
        w.setframerate(SR)
        w.writeframes(samples.tobytes())


def record_one(ser, outdir, label, base, index, duration_s, channel, stereo):
    target = int(duration_s * BYTES_PER_SEC)
    target -= target % FRAME_BYTES

    ser.reset_input_buffer()
    ser.write(b"R")
    ser.flush()
    if not wait_for_magic(ser):
        raise RuntimeError(
            "no sync marker from the board.\n"
            "  - is xiao_dual_inmp441_capture.ino flashed with LEVEL_METER 0 ?\n"
            "  - is the Serial Monitor (or another program) holding the port ?"
        )

    buf = bytearray()
    t0 = time.time()
    last = 0.0
    while len(buf) < target:
        chunk = ser.read(min(8192, target - len(buf)))
        if not chunk:
            if time.time() - t0 > duration_s + 15:
                raise RuntimeError("timed out waiting for audio data")
            continue
        buf.extend(chunk)
        now = time.time()
        if now - last > 0.5:
            pct = 100.0 * len(buf) / target
            sys.stdout.write(f"\r    {pct:5.1f}%  ({len(buf)/BYTES_PER_SEC:6.1f} s)")
            sys.stdout.flush()
            last = now
    ser.write(b"S"); ser.flush()
    sys.stdout.write("\r" + " " * 44 + "\r")

    inter = array.array("h")
    inter.frombytes(bytes(buf[:target]))
    A = inter[0::2]
    B = inter[1::2]
    MIX = array.array("h", [(a + b) // 2 for a, b in zip(A, B)])

    written = []
    if stereo:
        p = os.path.join(outdir, f"{label}.{base}_{index:03d}_stereo.wav")
        write_wav(p, inter, nch=2); written.append(("A+B stereo", p, A))
    want = {"both": ["A", "B", "mix"], "a": ["A"], "b": ["B"], "mix": ["mix"]}[channel]
    for tag, data in (("A", A), ("B", B), ("mix", MIX)):
        if tag not in want:
            continue
        p = os.path.join(outdir, f"{label}.{base}_{index:03d}_{tag}.wav")
        write_wav(p, data)
        written.append((tag, p, data))

    for tag, p, data in written:
        pk, rms, cl = stats(data)
        print(f"    {tag:<10s} {os.path.basename(p):<44s} "
              f"peak {pk:5.1f}% FS  RMS {rms:6.1f} dBFS  clipped {cl:.3f}%")

    pkA, rmsA, _ = stats(A)
    pkB, rmsB, _ = stats(B)
    if abs(rmsA - rmsB) > 12:
        quiet = "B" if rmsA > rmsB else "A"
        print(f"      [!] mic {quiet} is {abs(rmsA-rmsB):.0f} dB quieter - "
              f"check its wiring / L-R pin / that it is not covered")
    if max(pkA, pkB) < 5:
        print("      [!] very quiet - consider DECREASING SHIFT_BITS by 1 (+6 dB)")
    if max(stats(A)[2], stats(B)[2]) > 0.1:
        print("      [!] clipping - consider INCREASING SHIFT_BITS by 1 (-6 dB)")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--list", action="store_true")
    ap.add_argument("-p", "--port", default="/dev/cu.usbmodem101")
    ap.add_argument("-b", "--baud", type=int, default=921600)
    ap.add_argument("-l", "--label", default="F_Ali")
    ap.add_argument("-d", "--duration", type=float, default=20.0)
    ap.add_argument("-n", "--count", type=int, default=1)
    ap.add_argument("-o", "--outdir", default="xiao_recordings")
    ap.add_argument("--name", default=None)
    ap.add_argument("--gap", type=float, default=2.0)
    ap.add_argument("--channel", choices=["both", "a", "b", "mix"], default="both",
                    help="which mono file(s) to write (default: all three)")
    ap.add_argument("--stereo", action="store_true",
                    help="also write a 2-channel WAV (not for Edge Impulse upload)")
    a = ap.parse_args()

    if a.list:
        list_serial_ports(); return
    if not a.port:
        print("--port is required (use --list)\n"); list_serial_ports(); sys.exit(1)

    os.makedirs(a.outdir, exist_ok=True)
    base = a.name or a.label
    start = next_start_index(a.outdir, a.label, base)
    print(f"port  : {a.port}\nlabel : {a.label}\nclips : {a.count} x {a.duration:.0f} s"
          f"\nfiles : {a.channel}{' + stereo' if a.stereo else ''}"
          f"\noutdir: {os.path.abspath(a.outdir)}"
          f"\nstart : {start:03d} (continuing from existing files)\n")

    with serial.Serial(a.port, a.baud, timeout=1.0) as ser:
        time.sleep(2.0)
        for n in range(a.count):
            index = start + n
            print(f"[{n+1}/{a.count}] recording {a.duration:.0f} s (#{index:03d}) ...")
            record_one(ser, a.outdir, a.label, base, index, a.duration, a.channel, a.stereo)
            if n + 1 < a.count and a.gap > 0:
                time.sleep(a.gap)

    print("\nDone. Upload the mono files in Edge Impulse Studio "
          "(Upload data -> Infer label from filename), or score them locally:")
    print(f"  python predict_wav.py -w model_weights.npz {a.outdir}/ --score")


if __name__ == "__main__":
    main()
