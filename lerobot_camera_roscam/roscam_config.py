# Copyright 2024 The HuggingFace Inc. team. All rights reserved.
# Copyright 2026 University of Augsburg, Chair of Mechatronics by Noah Böckmann
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from dataclasses import dataclass
from pathlib import Path

from lerobot.cameras.configs import CameraConfig, ColorMode, Cv2Rotation

__all__ = ["ROS2CameraConfig", "ColorMode", "Cv2Rotation"]


@CameraConfig.register_subclass("ROS2Camera")
@dataclass
class ROS2CameraConfig(CameraConfig):
    """Configuration class for ros-based camera topics
    """

    node_name: str
    topic: str
    camera_type: str = "camera"
    depth_encoding: str = ""
    depth_topic: str = ""
    color_mode: ColorMode = ColorMode.RGB
    rotation: Cv2Rotation = Cv2Rotation.NO_ROTATION
    crop_box: tuple[int, int, int, int] | None = None
    warmup_s: int = 1

    def __post_init__(self) -> None:
        if self.color_mode not in (ColorMode.RGB, ColorMode.BGR):
            raise ValueError(
                f"`color_mode` is expected to be {ColorMode.RGB.value} or {ColorMode.BGR.value}, but {self.color_mode} is provided."
            )
        
        if self.camera_type not in ("camera", "compressed_camera", "dcamera"):
            raise ValueError(
                f"`Camera type` is expected to be camera, compressed_camera or dcamera, but {self.camera_type} is provided."
            )

        if self.rotation not in (
            Cv2Rotation.NO_ROTATION,
            Cv2Rotation.ROTATE_90,
            Cv2Rotation.ROTATE_180,
            Cv2Rotation.ROTATE_270,
        ):
            raise ValueError(
                f"`rotation` is expected to be in {(Cv2Rotation.NO_ROTATION, Cv2Rotation.ROTATE_90, Cv2Rotation.ROTATE_180, Cv2Rotation.ROTATE_270)}, but {self.rotation} is provided."
            )
