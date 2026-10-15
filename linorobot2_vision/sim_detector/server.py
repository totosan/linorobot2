"""Local twin of the Jetson ``.71`` ZeroMQ detector for Gazebo simulation.

Same wire protocol as ``~/video_detection/app.py`` on the Jetson:
  * PULL ``tcp://*:5555``: one encoded image (PNG/JPEG/PPM) per request.
  * REP  ``tcp://*:5556``: request ``"detect"``, reply
    ``{"detections":[{"box":[x1,y1,x2,y2],"label":str,"confidence":float}]}``.
Differences (deliberate): inference failures are reported as ``{"error":...}``
instead of an empty list, and unpaired images are dropped after a timeout.
This is a simulation stand-in: CPU PyTorch weights, not the TensorRT engine.
"""

import argparse
import hashlib
import json
import logging
import signal
import threading

import cv2
import numpy as np
import zmq

LOG = logging.getLogger("sim_detector")


class YoloDetector:
    """Ultralytics YOLO on CPU, same predict() call shape as the Jetson service."""

    def __init__(self, weights, imgsz=640, device="cpu"):
        from ultralytics import YOLO

        with open(weights, "rb") as handle:
            self.weights_sha256 = hashlib.sha256(handle.read()).hexdigest()
        self.model = YOLO(weights)
        self.imgsz = imgsz
        self.device = device
        self.names = self.model.names

    def warm_up(self):
        self.detect(np.zeros((480, 640, 3), np.uint8))

    def detect(self, image_bgr):
        results = self.model.predict(
            source=image_bgr, verbose=False, imgsz=self.imgsz, device=self.device)
        detections = []
        if results and results[0].boxes is not None:
            boxes = results[0].boxes
            for xyxy, conf, cls in zip(boxes.xyxy.cpu().numpy(),
                                       boxes.conf.cpu().numpy(),
                                       boxes.cls.cpu().numpy()):
                detections.append({
                    "box": [float(v) for v in xyxy],
                    "label": str(self.names[int(cls)]),
                    "confidence": float(conf),
                })
        return detections


def handle_request(detector, image_bytes, signal_text):
    """Return the JSON-able reply for one image + signal pair."""
    if signal_text != "detect":
        return {"error": f"Unknown signal: {signal_text}"}
    image = cv2.imdecode(np.frombuffer(image_bytes, np.uint8), cv2.IMREAD_COLOR)
    if image is None:
        return {"error": "Failed to decode image"}
    try:
        return {"detections": detector.detect(image)}
    except Exception as exc:  # report, never mask as an empty result
        LOG.exception("inference failed")
        return {"error": f"inference failed: {type(exc).__name__}: {exc}"}


def serve(detector, stop_event, image_bind="tcp://*:5555",
          reply_bind="tcp://*:5556", pair_timeout_ms=5000, context=None):
    context = context or zmq.Context.instance()
    pull = context.socket(zmq.PULL)
    rep = context.socket(zmq.REP)
    pull.setsockopt(zmq.LINGER, 0)
    rep.setsockopt(zmq.LINGER, 0)
    rep.setsockopt(zmq.RCVTIMEO, pair_timeout_ms)
    pull.bind(image_bind)
    rep.bind(reply_bind)
    LOG.info("image PULL %s, reply REP %s", image_bind, reply_bind)
    poller = zmq.Poller()
    poller.register(pull, zmq.POLLIN)
    try:
        while not stop_event.is_set():
            if not dict(poller.poll(timeout=200)):
                continue
            image_bytes = pull.recv()
            try:
                signal_text = rep.recv_string()
            except zmq.error.Again:
                LOG.warning("dropped image: no 'detect' signal within %d ms",
                            pair_timeout_ms)
                continue
            reply = handle_request(detector, image_bytes, signal_text)
            LOG.info("request: %d bytes, %s", len(image_bytes),
                     "error" if "error" in reply
                     else f"{len(reply['detections'])} detections")
            rep.send_string(json.dumps(reply))
    finally:
        pull.close(linger=0)
        rep.close(linger=0)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--weights", default="/models/yolo12n.pt")
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--image-bind", default="tcp://*:5555")
    parser.add_argument("--reply-bind", default="tcp://*:5556")
    args = parser.parse_args()
    logging.basicConfig(level=logging.INFO,
                        format="%(asctime)s - %(name)s - %(levelname)s - %(message)s")
    detector = YoloDetector(args.weights, imgsz=args.imgsz)
    LOG.info("weights=%s sha256=%s labels=%d", args.weights,
             detector.weights_sha256, len(detector.names))
    detector.warm_up()
    LOG.info("warm-up done; ready")
    stop = threading.Event()
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stop.set())
    serve(detector, stop, args.image_bind, args.reply_bind)


if __name__ == "__main__":
    main()
