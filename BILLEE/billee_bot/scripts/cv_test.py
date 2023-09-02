import pyrealsense2 as rs
import numpy as np
import cv2

# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

# Start streaming
pipeline.start(config)

# Define the side portion parameters
side_width = 200  # Width of the side portion in pixels
side_height = 480  # Height of the side portion in pixels

# Capture video frames until interrupted
try:
    while True:
        # Wait for a coherent pair of frames: depth and color
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()

        # Convert the color frame to a numpy array
        color_image = np.asanyarray(color_frame.get_data())

        # Convert the depth frame to a numpy array
        depth_image = np.asanyarray(depth_frame.get_data())

        # Crop the side portion from the color and depth images
        side_color = color_image[:, :side_width, :]
        side_depth = depth_image[:, :side_width]

        # Apply depth visualization to the side depth image
        side_depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(side_depth, alpha=0.03), cv2.COLORMAP_JET)

        # Stack the side color and depth images horizontally
        side_combined = np.hstack((side_color, side_depth_colormap))

        # Resize the side_combined image to fit the window
        resized_side_combined = cv2.resize(side_combined, (640, side_height))

        # Display the resulting frame
        cv2.imshow('Live Video Stream', np.vstack((color_image, resized_side_combined)))

        # Exit the loop if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    # Stop streaming
    pipeline.stop()

# Close all OpenCV windows
cv2.destroyAllWindows()
