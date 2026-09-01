"""Convert the Sub-GHz IQ dataset .mat files (variable IQ_samples, 2.048 MS/s)
to .cf32 for s2g-rx. Usage: python convert_mat.py FILE.mat [FILE2.mat ...]"""
import sys
import numpy as np
import scipy.io


def convert(path):
    m = scipy.io.loadmat(path)
    keys = [k for k in m if not k.startswith("__")]
    key = "IQ_samples" if "IQ_samples" in m else keys[0]
    x = np.asarray(m[key]).squeeze()
    if np.iscomplexobj(x):
        iq = x.astype(np.complex64)
    elif x.ndim == 2 and x.shape[-1] == 2:
        iq = (x[:, 0] + 1j * x[:, 1]).astype(np.complex64)
    elif x.ndim == 2 and x.shape[0] == 2:
        iq = (x[0] + 1j * x[1]).astype(np.complex64)
    else:
        raise SystemExit(f"{path}: unexpected shape {x.shape} for {key}")
    out = path.rsplit(".", 1)[0] + ".cf32"
    iq.astype(np.complex64).tofile(out)
    print(f"{path}: {key} {x.shape} -> {out} ({len(iq)} samples, rms {np.sqrt(np.mean(np.abs(iq)**2)):.4f})")


for p in sys.argv[1:]:
    convert(p)
