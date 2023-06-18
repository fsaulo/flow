#!/usr/bin/env python3

import cv2
import numpy as np
from pymavlink import mavutil

# Create a VideoWriter object to save the output video
fourcc = cv2.VideoWriter_fourcc(*"XVID")
# out = cv2.VideoWriter("trajectory.avi", fourcc, 20.0, (640, 480))

# Connect to the local Mavlink server
master = mavutil.mavlink_connection('udp:localhost:14445')

# Initialize variables for trajectory visualization
trajectory_color = (0, 0, 255)  # Red color
trajectory_thickness = 2

# Define the image dimensions
image_width = 640
image_height = 800

# Define the maximum number of points to be displayed on the trajectory
max_points = 1e6

# Initialize an empty list to store the trajectory points
trajectory_points = []

while True:
    # Wait for a message from the vehicle_local_position topic
    msg = master.recv_match(type='LOCAL_POSITION_NED', blocking=True)

    # Extract the x and y coordinates from the received message
    x = msg.x
    y = msg.y

    # Scale the coordinates to fit within the image dimensions
    scaled_x = int(x)*2 + image_width // 2
    scaled_y = int(y)*2 + image_height // 2


    # Append the scaled coordinates to the trajectory points list
    trajectory_points.append((scaled_x, scaled_y))

    # Trim the trajectory points list if it exceeds the maximum number of points
    if len(trajectory_points) > max_points:
        trajectory_points = trajectory_points[-max_points:]

    # Create a black image
    img = np.zeros((image_height, image_width, 3), dtype=np.uint8)

    # Draw the trajectory using scaled points
    for i in range(1, len(trajectory_points)):
        cv2.line(img, trajectory_points[i-1], trajectory_points[i], trajectory_color, trajectory_thickness)

    # Show the trajectory image
    cv2.imshow("Trajectory", img)
    # out.write(img)

    # Check if the 'q' key is pressed to exit the loop
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the video writer and close OpenCV windows
# out.release()
cv2.destroyAllWindows()
