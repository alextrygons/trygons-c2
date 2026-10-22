"""Download the eWaSR obstacle-detection model into gateway/vision/models/ (not in git).

    python tools/get_vision_model.py

Model: ewasr_resnet18 (ResNet-18 backbone, no IMU input) - the lightest
published eWaSR variant - as the ONNX file from the official release 0.1.0
of https://github.com/tersekmatija/eWaSR. Code and weights: Apache License
2.0 (copied next to the model, as the licence asks); trained on MaSTr1325
(CC BY 4.0, Bovcon et al., IROS 2019).
"""
import hashlib
import os
import sys
import urllib.request

MODEL = "ewasr_resnet18"
URL = "https://github.com/tersekmatija/eWaSR/releases/download/0.1.0/%s.onnx" % MODEL
LICENSE_URL = "https://raw.githubusercontent.com/tersekmatija/eWaSR/main/LICENSE"
SIZE = 241076632                    # bytes of the published file (release 0.1.0)
HERE = os.path.dirname(os.path.abspath(__file__))
DEST = os.path.join(HERE, "..", "gateway", "vision", "models")


def main():
    os.makedirs(DEST, exist_ok=True)
    path = os.path.join(DEST, MODEL + ".onnx")
    if not (os.path.exists(path) and os.path.getsize(path) == SIZE):
        print("downloading %s (%.0f MB) ..." % (URL, SIZE / 1e6), flush=True)
        tmp = path + ".part"
        urllib.request.urlretrieve(URL, tmp)
        if os.path.getsize(tmp) != SIZE:
            print("unexpected size %d (expected %d) - download again" % (os.path.getsize(tmp), SIZE))
            return 1
        os.replace(tmp, path)
    with urllib.request.urlopen(LICENSE_URL, timeout=60) as r:
        with open(os.path.join(DEST, "LICENSE-eWaSR.txt"), "wb") as f:
            f.write(r.read())
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for b in iter(lambda: f.read(1 << 20), b""):
            h.update(b)
    print("model %s\nsha256 %s" % (os.path.abspath(path), h.hexdigest()))
    return 0


if __name__ == "__main__":
    sys.exit(main())
