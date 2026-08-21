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

import rclpy
import threading
import numpy as np
from lerobot.cameras import Camera
from lerobot.cameras.configs import ColorMode, Cv2Rotation
from .roscam_config import ROS2CameraConfig

# --- ROS 2 Imports ---
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import CompressedImage as CompressedRosImage
from sensor_msgs.msg import Image as RawRosImage
from cv_bridge import CvBridge, CvBridgeError
from cv2 import cvtColor, rotate, COLOR_BGR2RGB, ROTATE_90_CLOCKWISE, ROTATE_90_COUNTERCLOCKWISE, ROTATE_180
import message_filters

# --- Typing Imports ---
from typing import Any, Optional, List, Dict
from numpy.typing import NDArray


class ROS2Camera(Camera):
    """
    A camera implementation that subscribes to a ROS 2 Image topic.

    This class initializes a ROS 2 node, subscribes to the specified
    image topic, and handles frame conversion from sensor_msgs/Image
    to numpy.ndarray using cv_bridge.
    """

    def __init__(self, config: ROS2CameraConfig):
        """
        Initialize the ROS 2 camera subscriber.

        Args:
            config: Configuration containing the ROS 2 topic and node name.
        """
        super().__init__(config)
        self.config = config

        self.node: Optional[Node] = None

        self.subscriber = None
        self.depth_subscriber = None
        self.spin_thread: Optional[threading.Thread] = None
        self.bridge = CvBridge()
        self.image_handler = None

        # --- Synchronization components ---
        self.rgb_sub: Optional[message_filters.Subscriber] = None
        self.depth_sub: Optional[message_filters.Subscriber] = None
        self.synchronizer: Optional[message_filters.ApproximateTimeSynchronizer] = None

        # Threading and frame management
        self._latest_frame: Optional[NDArray[Any]] = None
        self._latest_depth_frame: Optional[NDArray[Any]] = None
        self._frame_lock = threading.Lock()
        self._new_frame_event = threading.Event()
        self._is_connected_flag = False

    def connect(self, warmup: bool = True) -> None:
        """
        Initialize the ROS 2 node and subscriber.
        """
        if self.is_connected:
            print("Already connected.")
            return

        try:
            # Ensure rclpy is initialized (if not already)
            if not rclpy.ok():
                rclpy.init()

            self.node = rclpy.create_node(self.config.node_name)
            self.node.get_logger().info(f"Connecting to ROS 2 topic: {self.config.topic}")

            # Define a best effort QoS profile
            qos_profile = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST,
                depth=1
            )


            if self.config.camera_type == "dcamera":
                # 1. Create message_filters Subscribers
                self.rgb_sub = message_filters.Subscriber(
                    self.node,
                    RawRosImage,
                    self.config.topic,
                    qos_profile=qos_profile
                )
                self.depth_sub = message_filters.Subscriber(
                    self.node,
                    RawRosImage,
                    self.config.depth_topic,
                    qos_profile=qos_profile
                )
                # 2. Create the Synchronizer
                # ApproximateTimeSynchronizer is robust to small timestamp mismatches
                self.synchronizer = message_filters.ApproximateTimeSynchronizer(
                    [self.rgb_sub, self.depth_sub],
                    queue_size=10, # Number of message sets to buffer
                    slop=0.03         # Allow 30ms delta between message timestamps
                )
                self.synchronizer.registerCallback(self._synced_image_callback)

            elif self.config.camera_type == "compressed_camera":
                self.subscriber = self.node.create_subscription(
                    CompressedRosImage,
                    self.config.topic,
                    self._image_callback,
                    qos_profile
                )
                self.image_handler = self.bridge.compressed_imgmsg_to_cv2
                
            elif self.config.camera_type == "camera":
                self.subscriber = self.node.create_subscription(
                    RawRosImage,
                    self.config.topic,
                    self._image_callback,
                    qos_profile
                )
                self.image_handler = self.bridge.imgmsg_to_cv2

            # Start the spinning in a separate thread
            self.spin_thread = threading.Thread(target=self._ros_spin, daemon=True)
            self.spin_thread.start()

            self._is_connected_flag = True
            self.node.get_logger().info("Node created and spinning.")

            if warmup:
                self.node.get_logger().info("Warming up... waiting for first frame.")
                try:
                    # Use async_read's logic for waiting
                    self.async_read(timeout_ms=5000)  # 5-second warmup timeout
                    self.node.get_logger().info("Warmup complete. First frame received.")
                except TimeoutError:
                    self.node.get_logger().warn("Warmup timed out. No frame received.")
                    # Don't fail the connection, just warn.

        except Exception as e:
            print(f"Failed to connect to ROS 2: {e}")
            self._is_connected_flag = False
            if self.node:
                self.node.destroy_node()
                self.node = None

    @property
    def is_connected(self) -> bool:
        """Check if the ROS 2 node is alive and subscribing."""
        return self._is_connected_flag and self.node is not None and rclpy.ok()

    def find_cameras():
        print("Find Cameras is not implemented right now!")

    def read(self, color_mode: ColorMode | None = None) -> NDArray[Any]:
        """
        Get the latest frame received from the topic (non-blocking).

        This returns the most recent frame available without waiting.

        """
        if not self.is_connected:
            raise ConnectionError("Camera is not connected.")
        
        if color_mode is None:
            color_mode = self.config.color_mode

        with self._frame_lock:
            if self._latest_frame is None:
                raise RuntimeError("No frame has been received yet.")
            
            frame = self._latest_frame.copy()
            self._new_frame_event.clear()

        if color_mode == ColorMode.RGB:
            frame = cvtColor(frame, COLOR_BGR2RGB)
        elif color_mode == ColorMode.BGR:
            pass
        else:
            self.node.get_logger().warn("The selected color mode is not supported by lerobot-roscam! No conversion took place.")

        if self.config.rotation == Cv2Rotation.NO_ROTATION:
            pass
        elif self.config.rotation == Cv2Rotation.ROTATE_270:
            frame = rotate(frame, ROTATE_90_CLOCKWISE)
        elif self.config.rotation == Cv2Rotation.ROTATE_90:
            frame = rotate(frame, ROTATE_90_COUNTERCLOCKWISE)
        else:
            frame = rotate(frame, ROTATE_180)

        if self.config.crop_box is not None:
            ymin, ymax, xmin, xmax = self.config.crop_box
            frame = frame[ymin:ymax, xmin:xmax]

        return frame

    def async_read(self, timeout_ms: float = 1000.0) -> NDArray[Any]:
        """
        Waits for a previously unread frame to arrive or times out.

        Args:
            timeout_ms: Maximum time to wait for a frame in milliseconds.

        Returns:
            np.ndarray: The new frame.

        Raises:
            TimeoutError: If no new frame is received within the timeout.
            ConnectionError: If the camera is not connected.
        """
        if not self.is_connected:
            raise ConnectionError("Camera is not connected.")

        timeout_s = timeout_ms / 1000.0

        if not self._new_frame_event.wait(timeout=timeout_s):
            raise TimeoutError(f"No new frame received in {timeout_ms}ms.")

        return self.read()

    def disconnect(self) -> None:
        """Disconnect from the camera and release ROS 2 resources."""
        if not self._is_connected_flag or self.node is None:
            print("Not connected.")
            return

        self.node.get_logger().info("Disconnecting from ROS 2...")
        self._is_connected_flag = False

        # Destroy the subscriber and node
        if self.subscriber:
            self.node.destroy_subscription(self.subscriber)
            self.subscriber = None

        if self.node:
            self.node.destroy_node()
            self.node = None

        # The spin thread will exit automatically because rclpy.spin()
        # will raise ExternalShutdownException when the node is destroyed.
        if self.spin_thread:
            self.spin_thread.join(timeout=1.0)  # Wait briefly for it to exit

        print("ROS 2 node disconnected.")

    # TODO: The depth feature is incomplete, rotation etc. is not implemented
    def _synced_image_callback(self, rgb_msg: RawRosImage, depth_msg: RawRosImage):
        """
        Callback for the synchronized image subscribers.
        This is the "extra thread" (runs in the spin thread) that
        assembles the frames. This code is not up to date and needs more work!
        """
        try:
            # Convert both ROS Image messages to OpenCV images
            rgb_frame = self.bridge.imgmsg_to_cv2(
                rgb_msg, desired_encoding=self.config.rgb_encoding
            )
            depth_frame = self.bridge.imgmsg_to_cv2(
                depth_msg, desired_encoding=self.config.depth_encoding
            )

            rgb_frame = rgb_frame[..., :3].astype(np.float32) / 255.0 # Shape (width, height, 3)
            depth_frame = np.expand_dims(depth_frame, axis=-1)

            frame = np.concatenate((rgb_frame, depth_frame), axis=-1)

            with self._frame_lock:
                # Store the pair of frames
                self._latest_frame = frame

                # Update camera properties from the RGB frame
                if self.width is None or self.height is None:
                    self.height, self.width = rgb_frame.shape[:2]
                    # Note: FPS is not available from messages.
            
                # Signal any waiting async_read
                self._new_frame_event.set()

        except CvBridgeError as e:
            if self.node:
                self.node.get_logger().warn(f"CvBridgeError: {e}")
        except Exception as e:
            if self.node:
                self.node.get_logger().warn(f"Failed to process image messages: {e}")

    def _image_callback(self, msg):
        """Callback for the image subscriber."""
        try:
            # Convert ROS Image message to OpenCV image depending on camera stream type
            #TODO: The encoding differs for compressed and uncompressed images. There is no time to
            # fix this at the moment. Assuming the picture is BGR/JPEG BGR encoded for now.

            cv_frame = self.image_handler(
                msg, desired_encoding="passthrough" # Passthrough because of Numpy 1.X and 2.X incompatibility. This is handeled later by cv2.
            )
            
            with self._frame_lock:
                self._latest_frame = cv_frame

                # Update camera properties from the first message
                if self.width is None or self.height is None:
                    self.height, self.width = cv_frame.shape[:2]
            
                # Signal any waiting async_read
                self._new_frame_event.set()

        except CvBridgeError as e:
            if self.node:
                self.node.get_logger().warn(f"CvBridgeError: {e}")
        except Exception as e:
            if self.node:
                self.node.get_logger().warn(f"Failed to process image message: {e}")

    def _ros_spin(self):
        """Internal method to spin the ROS 2 node using a dedicated executor."""
        if self.node:
            # Create a single-threaded executor specifically for this node.
            # This isolates its execution from other nodes/threads.
            executor = rclpy.executors.SingleThreadedExecutor(context=self.node.context)
            executor.add_node(self.node)

            try:
                # Spin the executor. This blocks until the node is destroyed.
                executor.spin()
            except rclpy.executors.ExternalShutdownException:
                pass  # Expected when the node is destroyed/shutdown
            except Exception as e:
                # Log the error using the node's logger
                if rclpy.ok():
                    self.node.get_logger().error(f"Error in ROS spin thread: {e}")
            finally:
                # Clean up the executor and remove the node when finished
                executor.shutdown()
                executor.remove_node(self.node)