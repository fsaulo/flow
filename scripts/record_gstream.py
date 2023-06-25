#!/usr/bin/env python3

import cv2

# GStreamer pipeline string
pipeline = "udpsrc port=5600 ! application/x-rtp, encoding-name=H264, payload=96 ! rtph264depay ! h264parse ! openh264dec ! videoconvert ! appsink"

# Create VideoCapture object
cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)

# Check if VideoCapture is successfully opened
if not cap.isOpened():
    print("Failed to open the video stream.")
    exit(1)

# Get video width, height, and FPS
frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
fps = cap.get(cv2.CAP_PROP_FPS)

# Define the codec for video output
fourcc = cv2.VideoWriter_fourcc(*"DIVX")
output_video = cv2.VideoWriter("output.mp4", fourcc, fps, (frame_height, frame_width))

while True:
    # Read frame from video stream
    ret, frame = cap.read()

    # Check if frame is successfully read
    if not ret:
        print("Failed to read frame from video stream.")
        break

    # Display the frame (optional)
    cv2.imshow("GStreamer Stream", frame)

    # Write the frame to the output video file
    output_video.write(frame)

    # Exit if 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release resources
cap.release()
output_video.release()
cv2.destroyAllWindows()
