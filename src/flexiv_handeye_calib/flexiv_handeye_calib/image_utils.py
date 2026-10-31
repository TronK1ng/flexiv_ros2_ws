"""Minimal sensor_msgs/Image decoding without cv_bridge."""

from __future__ import annotations

import numpy as np


def to_bgr(message) -> np.ndarray | None:
    channels = {"rgb8": 3, "bgr8": 3, "mono8": 1, "8UC3": 3, "8UC1": 1}.get(message.encoding)
    if channels is None:
        return None
    data = np.frombuffer(message.data, dtype=np.uint8)
    row_bytes = message.width * channels
    if message.step == row_bytes:
        image = data.reshape(message.height, message.width, channels)
    else:
        image = data.reshape(message.height, message.step)[:, :row_bytes].reshape(
            message.height, message.width, channels
        )
    if channels == 1:
        return np.ascontiguousarray(np.repeat(image, 3, axis=2))
    if message.encoding == "rgb8":
        image = image[:, :, ::-1]
    return np.ascontiguousarray(image)
