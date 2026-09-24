import base64, json, sys, zlib

# A synthetic firmware container for the firmware pages' tests: the shape of an ArduPilot .apj
# (JSON around a base64 zlib image), with a made-up 3,000-byte image. Not a real firmware.
image = bytes((i * 7) % 251 for i in range(3000))
apj = {
    "board_id": 140,
    "magic": "APJFWv1",
    "description": "Synthetic test image, not a firmware",
    "image": base64.b64encode(zlib.compress(image, 9)).decode("ascii"),
    "summary": "CubeOrange",
    "version": "0.1",
    "image_size": len(image),
    "board_revision": 0,
    "git_identity": "0000000",
}
with open(sys.argv[1], "w") as f:
    json.dump(apj, f, indent=4)
