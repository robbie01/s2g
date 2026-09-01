"""Download a public Mega.nz file link without the (broken on 3.11+) mega.py
package. Usage: python mega_get.py 'https://mega.nz/file/ID#KEY' OUT_DIR"""
import base64
import json
import os
import struct
import sys
import urllib.request

from Crypto.Cipher import AES
from Crypto.Util import Counter


def b64d(s):
    s = s.replace("-", "+").replace("_", "/")
    return base64.b64decode(s + "=" * (-len(s) % 4))


def api(req):
    r = urllib.request.Request(
        "https://g.api.mega.co.nz/cs", data=json.dumps([req]).encode(), headers={"Content-Type": "application/json"}
    )
    return json.loads(urllib.request.urlopen(r, timeout=60).read())[0]


url, out_dir = sys.argv[1], sys.argv[2]
fid, key = url.split("/file/")[1].split("#")
k = struct.unpack(">8I", b64d(key))
fk = struct.pack(">4I", k[0] ^ k[4], k[1] ^ k[5], k[2] ^ k[6], k[3] ^ k[7])
iv = (k[4] << 32) | k[5]
info = api({"a": "g", "g": 1, "p": fid})
if isinstance(info, int):
    raise SystemExit(f"mega API error {info}")
size, dl = info["s"], info["g"]
attrs = AES.new(fk, AES.MODE_CBC, b"\0" * 16).decrypt(b64d(info["at"]))
name = json.loads(attrs.decode("utf-8", "ignore").split("MEGA", 1)[1].rstrip("\0"))["n"]
name = os.path.basename(name)
print(f"{name}: {size / 1e6:.1f} MB")
os.makedirs(out_dir, exist_ok=True)
cipher = AES.new(fk, AES.MODE_CTR, counter=Counter.new(128, initial_value=iv << 64))
with urllib.request.urlopen(dl, timeout=120) as resp, open(os.path.join(out_dir, name), "wb") as f:
    done = 0
    while True:
        chunk = resp.read(1 << 20)
        if not chunk:
            break
        f.write(cipher.decrypt(chunk))
        done += len(chunk)
print(f"wrote {done} bytes to {os.path.join(out_dir, name)}")
