from lerobot.cameras.configs import ColorMode, Cv2Rotation
from lerobot_roscam.roscam import ROS2Camera
from lerobot_roscam.roscam_config import ROS2CameraConfig
import cv2

# 1. Create the configuration
config = ROS2CameraConfig(
    node_name="lerobot_listener_node",
    topic="/side_cam/image",
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