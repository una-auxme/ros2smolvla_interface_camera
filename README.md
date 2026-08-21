# ROS 2 Camera Integration for LeRobot

This module implements a ROS 2 camera wrapper for the [LeRobot](https://github.com/huggingface/lerobot)
library. It allows you to subscribe to ROS 2 image topics (raw or compressed) and use them transparently
as LeRobot `Camera` objects for data collection or inference.

## Features

* **Standard ROS 2 Integration:** Uses `rclpy` to subscribe to topics.
* **Multiple Image Transport:** Supports both `sensor_msgs/Image` (raw) and `sensor_msgs/CompressedImage`
    (compressed).
* **Depth Support (Experimental):** capable of synchronizing RGB and Depth topics using approximate
    time synchronization. Currently LeRobot does not support this natively, though depth could be interpreted
    as additional state.
* **LeRobot Compatible:** Inherits from `lerobot.cameras.Camera` and fully integrates with the LeRobot
    configuration system.
* **Image Processing:** Handles rotation (90°, 180°, 270°), cropping and color mode conversion (BGR -> RGB) on the
    fly.

## Prerequisites

Ensure you have a working ROS 2 installation (only Jazzy is tested). Install the ROS2 dependencies using
rosdep. When installing this package pip should take care of the rest.

* `rclpy`
* `cv_bridge`
* `numpy`
* `opencv-python`
* `lerobot`

## Configuration

The system uses `ROS2CameraConfig` to define connection parameters.

### Configuration Parameters

| Parameter | Type | Default | Description |
| :--- | :--- | :--- | :--- |
| `node_name` | `str` | **Required** | The name of the ROS 2 node created by the camera wrapper. |
| `topic` | `str` | **Required** | The ROS 2 topic name to subscribe to (e.g., `/camera/image_raw`). |
| `camera_type` | `str` | `"camera"` | The mode of operation. Options: `camera` (Raw), `compressed_camera`, or `dcamera` (RGB+Depth). |
| `color_mode` | `ColorMode`| `RGB` | The desired output color format (`RGB` or `BGR`). |
| `rotation` | `Cv2Rotation`| `NO_ROTATION`| Rotates the incoming frame. Options: `NO_ROTATION`, `ROTATE_90`, `ROTATE_180`, `ROTATE_270`. |
| `crop_box` | `tuple[int, int, int, int]`| `None`| Crops the image according to the given crop box. Options: `[ymin, ymax, xmin, xmax]`. |
| `depth_topic` | `str` | `""` | Required only if `camera_type` is `"dcamera"`. |
| `depth_encoding`| `str` | `""` | Encoding for depth conversion (used in synchronized mode). |
| `warmup_s` | `int` | `1` | Time to wait during initialization. |

## Usage

Below is an example of how to configure and initialize the camera in your Python script. The ROS2CameraConfig
can be passed as a LeRobot camera in a robot configuration. Check out 
[this configuration](https://github.com/una-auxme/lerobot-ros/blob/2ff1aa6e0eb8505220e4bd4c63c84ee171a0b5de/lerobot_robot_ros/lerobot_robot_ros/config.py#L136)
for an example.

```python
from lerobot.cameras.configs import ColorMode, Cv2Rotation
from lerobot_camera_roscam.roscam import ROS2Camera
from lerobot_camera_roscam.roscam_config import ROS2CameraConfig
import cv2

# 1. Create the configuration
config = ROS2CameraConfig(
    node_name="lerobot_listener_node",
    topic="/camera/image",
    camera_type="camera",              # Use "compressed_camera" for compressed topics
    color_mode=ColorMode.BGR,
    rotation=Cv2Rotation.NO_ROTATION
)

# 2. Initialize the camera
camera = ROS2Camera(config)

# 3. Connect (starts the ROS 2 spin thread)
camera.connect()

try:
    # 4. Read frames
    while True:
        # Returns a numpy array (H, W, C)
        frame = camera.read()
        
        # Display frame
        cv2.imshow("ROS 2 Camera Stream", frame)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
finally:
    # 5. Cleanup
    camera.disconnect()
    cv2.destroyAllWindows()
```

## Limitations & Known Issues

* **Depth Camera:** The `dcamera` type handles synchronized RGB and Depth, but the feature is currently incomplete.
* **Camera Discovery:** The `find_cameras()` method is not yet implemented.
* **Encoding:** The package **assumes BGR** color encoding for the input. Checking is not yet implemented.

## License
Apache 2.0

This project is based on the Hugging Face LeRobot structure. See the file headers for specific copyright information.