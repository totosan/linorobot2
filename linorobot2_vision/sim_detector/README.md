# Sim detector (local twin of Jetson `.71`)

CPU stand-in for the Jetson `.71` ZeroMQ detector, used so Gazebo tests do not
depend on the hardware being powered. It speaks the same wire protocol, so the
ROS adapter switches between both only by profile/endpoint.

| | Sim twin (this container) | Jetson `.71` |
|---|---|---|
| Weights | `yolo12n.pt`, PyTorch CPU | `yolo12n.engine`, TensorRT |
| Ports | PULL 5555 (image), REP 5556 (`detect`) | same |
| Reply | `{"detections":[{"box":[x1,y1,x2,y2],"label","confidence"}]}` | same |
| Inference failure | `{"error": ...}` | empty list (ambiguous) |
| Latency/accuracy | **not** equal to the Jetson | reference |

Detections are *similar*, not identical, to the engine. Do not treat sim results
as hardware accuracy or latency evidence.

## Run

```bash
docker build -t linorobot2-sim-detector:1 src/linorobot2/linorobot2_vision/sim_detector
docker run -d --name simdet -p 5555:5555 -p 5556:5556 \
  -v /path/to/yolo12n.pt:/models/yolo12n.pt:ro linorobot2-sim-detector:1
docker logs simdet   # prints weights sha256, "warm-up done; ready"
```

Use `weights-sha256:<printed hash>` as `zmq_model_version`. Stop with
`docker rm -f simdet`.

## Swap to the real Jetson (separate approval, Domain 44)

```bash
ROS_DOMAIN_ID=44 ros2 launch linorobot2_vision vision_overlay.launch.py backend:=zmq \
  zmq_profile:=hardware_71 zmq_image_endpoint:=tcp://192.168.0.71:5555 \
  zmq_reply_endpoint:=tcp://192.168.0.71:5556 zmq_model_version:=<engine sha256>
```

The first `.71` request loads TensorRT lazily (about 54 s observed); keep
`zmq_timeout_sec` at 60 or more. On `.71` an empty list can also mean an
internal inference error; it is not proof of a healthy service.

## Sim launch (Domain 42, from the Linux VM)

The VM reaches the Mac container via `host.docker.internal`. `pyzmq` must be
installed in the ROS environment.

```bash
ROS_DOMAIN_ID=42 ros2 launch linorobot2_vision vision_overlay.launch.py backend:=zmq \
  zmq_model_version:=weights-sha256:<hash>
```

## Positive person target in Gazebo

The geometric person proxy is not a reliable YOLO target. For a positive test,
`scripts/make_g6_photo_world.sh` generates a hash-named SDF in
`log/test_assets/` (a photo billboard 3 m ahead of the robot). It now requires
an explicit texture and provenance manifest, validates the CC0 1.0 license URL
and exact SHA-256 and pixel dimensions, preserves the source image aspect ratio,
and refuses to overwrite an existing world. It never pulls an image from the
running detector container. The unverified `zidane.jpg` and the previously
generated SDF are left untouched. A separate OrbStack-compatible SDF variant
was generated and XML-validated for the positive one-capture test below.

A candidate asset and manifest are available locally:
[Person in winter clothing in Quebec City](https://commons.wikimedia.org/wiki/File:Person_in_winter_clothing_in_Quebec_City.jpg),
by Wilfredor, CC0. Commons also marks personality-rights restrictions and
full-consent metadata; this candidate is for local fixture testing only, not
for redistribution. Its local positive detector result is documented below.
Generate a fresh world with:

```bash
scripts/make_g6_photo_world.sh \
  --texture log/test_assets/person_winter_quebec.jpg \
  --provenance log/test_assets/person_winter_quebec.provenance.json
```

The exact candidate has since produced one positive G6 result on 2026-10-13:
one simulated 640×480 capture yielded a `person` box at confidence 0.8685
through Contract v1. The recorded CPU/PyTorch model was `yolo12n` with weights
SHA-256 `419ff3dca37d69bacc93a50fa0c186a1c6f9fe62fae0f108b0872829689e9ca6`.
The JSON report is at
`log/person_runtime/g6_oneshot_20261013_1759/vision-probe.json`. This is local
SIL evidence only, not Jetson/TensorRT performance or hardware validation.
The old `zidane.jpg` fixture and previously generated SDF remain untouched.

For a controlled repeat, start Fortress with `world:=<that file>` on Domain 42,
run a dedicated detector on host ports 15555/15556, and send exactly one frame
through the isolated profile:

```bash
ROS_DOMAIN_ID=42 python3 scripts/probe_zmq_vision_service.py \
  --profile sim_local_isolated \
  --image-endpoint tcp://host.docker.internal:15555 \
  --reply-endpoint tcp://host.docker.internal:15556 \
  --model-version weights-sha256:<hash> --expected-label person \
  --test-scene fortress_g6_vision_photo --report log/person_runtime/zmq-sim-<date>.json
```
