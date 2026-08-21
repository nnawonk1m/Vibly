#!/usr/bin/env python3
"""
predict_wav.py - run the trained 4-class audio model on WAV files, on your PC.

Pure numpy. No TensorFlow, no Edge Impulse. Uses exactly the same feature
extraction and forward pass as audio_model.h, so what you see here is what the
XIAO will produce.

    pip install numpy

    # one file
    python predict_wav.py -w model_weights.npz clip.wav

    # a whole folder, with per-window detail
    python predict_wav.py -w model_weights.npz recordings/ --verbose

    # score a labelled folder (filenames must start with "<label>.")
    python predict_wav.py -w model_weights.npz test_set/ --score

Input WAVs must be 16 kHz mono 16-bit PCM. Convert anything else with:
    ffmpeg -i in.ext -ac 1 -ar 16000 -sample_fmt s16 -c:a pcm_s16le out.wav
"""
import argparse, os, sys, wave
import numpy as np

# ------------------------------------------------------------------ features
SR, WIN, FRAME, HOP, NFFT, NMEL = 16000, 16000, 512, 256, 512, 32
NFRAMES = 1 + (WIN - FRAME) // HOP          # 61
F_LOW, F_HIGH = 100.0, 8000.0
DB_FLOOR, DB_RANGE, EPS = -80.0, 100.0, 1e-10
WIN_HOP = 8000                               # 0.5 s between windows

# filename-prefix aliases, so the original Edge Impulse export names also score
ALIAS = {"speech": "human_voice", "noise": "noise_none",
         "baby_cry": "baby_crying", "car_honk": "traffic"}
HANN = np.hanning(FRAME)


def _fb():
    h2m = lambda f: 2595.0 * np.log10(1.0 + f / 700.0)
    m2h = lambda m: 700.0 * (10.0 ** (m / 2595.0) - 1.0)
    pts = m2h(np.linspace(h2m(F_LOW), h2m(F_HIGH), NMEL + 2))
    b = np.clip(np.floor((NFFT + 1) * pts / SR).astype(int), 0, NFFT // 2)
    fb = np.zeros((NMEL, NFFT // 2 + 1))
    for m in range(NMEL):
        l, c, r = b[m], b[m + 1], b[m + 2]
        if c == l: c = l + 1
        if r == c: r = c + 1
        r = min(r, NFFT // 2)
        for k in range(l, c): fb[m, k] = (k - l) / float(c - l)
        for k in range(c, r): fb[m, k] = (r - k) / float(r - c)
    return fb


FB = _fb()


def mfe(w):
    fr = np.lib.stride_tricks.sliding_window_view(w, FRAME)[::HOP][:NFRAMES] * HANN
    sp = np.fft.rfft(fr, n=NFFT, axis=1)
    pw = (sp.real ** 2 + sp.imag ** 2) / float(NFFT)
    db = 10.0 * np.log10(pw @ FB.T + EPS)
    return np.clip((db - DB_FLOOR) / DB_RANGE, 0.0, 1.0)


# ------------------------------------------------------------------- network
def conv_relu_pool(x, W, b):
    """x (H,W,CI) -> conv3x3 'same' + relu + maxpool2 -> (H//2, W//2, CO)"""
    H, Wd, CI = x.shape
    p = np.zeros((H + 2, Wd + 2, CI)); p[1:-1, 1:-1] = x
    CO = W.shape[3]
    acc = np.zeros((H, Wd, CO))
    for ky in range(3):
        for kx in range(3):
            acc += p[ky:ky + H, kx:kx + Wd] @ W[ky, kx]
    acc = np.maximum(acc + b, 0.0)
    OH, OW = H // 2, Wd // 2
    a = acc[:OH * 2, :OW * 2].reshape(OH, 2, OW, 2, CO)
    return a.max(axis=(1, 3))


def predict(feat, m):
    x = feat[..., None]
    for i in range(3):
        x = conv_relu_pool(x, m[f"W{i}"], m[f"B{i}"])
    g = x.mean(axis=(0, 1))
    z = g @ m["Wd"] + m["bd"]
    z = np.exp(z - z.max())
    return z / z.sum()


# ---------------------------------------------------------------------- wav
def read_wav(path):
    w = wave.open(path, "rb")
    sr, ch, sw, n = w.getframerate(), w.getnchannels(), w.getsampwidth(), w.getnframes()
    d = np.frombuffer(w.readframes(n), dtype="<i2").astype(np.float64)
    w.close()
    if sw != 2:
        raise ValueError(f"{os.path.basename(path)}: need 16-bit PCM, got {sw*8}-bit")
    if ch != 1:
        d = d.reshape(-1, ch).mean(1)
    if sr != SR:
        raise ValueError(f"{os.path.basename(path)}: need {SR} Hz, got {sr} Hz")
    return d / 32768.0


def windows(a):
    if len(a) < WIN:
        w = np.zeros(WIN); w[:len(a)] = a; return [w]
    return [a[s:s + WIN] for s in range(0, len(a) - WIN + 1, WIN_HOP)]


# --------------------------------------------------------------------- main
def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("inputs", nargs="+", help="WAV files or folders")
    ap.add_argument("-w", "--weights", default="model_weights.npz")
    ap.add_argument("--verbose", action="store_true", help="print every window")
    ap.add_argument("--score", action="store_true",
                    help="treat the text before the first '.' as the true label "
                         "and print a confusion matrix")
    a = ap.parse_args()

    z = np.load(a.weights, allow_pickle=True)
    m = {k: z[k].astype(np.float64) for k in
         ("W0", "B0", "W1", "B1", "W2", "B2", "Wd", "bd")}
    labels = [str(s) for s in z["labels"]]

    files = []
    for p in a.inputs:
        if os.path.isdir(p):
            files += [os.path.join(p, f) for f in sorted(os.listdir(p))
                      if f.lower().endswith(".wav")]
        else:
            files.append(p)
    if not files:
        sys.exit("no .wav files found")

    cm = np.zeros((len(labels), len(labels)), int)
    n_scored = 0

    for path in files:
        try:
            audio = read_wav(path)
        except Exception as e:
            print(f"SKIP {e}"); continue

        probs = np.stack([predict(mfe(w), m) for w in windows(audio)])
        mean = probs.mean(0)
        top = int(mean.argmax())

        print(f"\n{os.path.basename(path)}   ({len(audio)/SR:.1f} s, "
              f"{len(probs)} windows)")
        order = np.argsort(-mean)
        print("  clip verdict: " + ", ".join(
            f"{labels[i]} {mean[i]:.3f}" for i in order))

        if a.verbose:
            for i, p in enumerate(probs):
                j = int(p.argmax())
                print(f"    t={i*WIN_HOP/SR:5.1f}s  {labels[j]:<12s} {p[j]:.3f}   "
                      + " ".join(f"{labels[k][:4]}:{p[k]:.2f}" for k in range(len(labels))))

        if a.score:
            true = ALIAS.get(os.path.basename(path).split(".")[0],
                             os.path.basename(path).split(".")[0])
            if true in labels:
                cm[labels.index(true), top] += 1
                n_scored += 1
            else:
                print(f"  (not scored: '{true}' is not one of {labels})")

    if a.score and n_scored:
        print("\nconfusion matrix (rows = true, cols = predicted), clip level")
        w = max(len(l) for l in labels)
        print(" " * (w + 2) + "  ".join(f"{l[:6]:>6s}" for l in labels))
        f1s = []
        for i, l in enumerate(labels):
            print(f"{l:>{w}s}  " + "  ".join(f"{x:6d}" for x in cm[i]))
        for i in range(len(labels)):
            tp = cm[i, i]
            pr = tp / max(cm[:, i].sum(), 1); rc = tp / max(cm[i].sum(), 1)
            f1s.append(0 if pr + rc == 0 else 2 * pr * rc / (pr + rc))
            print(f"  {labels[i]:<12s} precision {pr:.3f}  recall {rc:.3f}  F1 {f1s[-1]:.3f}")
        print(f"\naccuracy {np.trace(cm)/cm.sum():.4f}   macro-F1 {np.mean(f1s):.4f}"
              f"   ({n_scored} clips)")


if __name__ == "__main__":
    main()
