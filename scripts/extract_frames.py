#!/usr/bin/env python3

import cv2
import os

def save_frames(video_path, output_folder):
    # Open the video file
    video = cv2.VideoCapture(video_path)

    # Check if the video file was successfully opened
    if not video.isOpened():
        print(f"Error opening video file: {video_path}")
        return

    # Create the output folder if it doesn't exist
    os.makedirs(output_folder, exist_ok=True)

    # Initialize variables
    frame_count = 0

    while True:
        # Read the next frame
        ret, frame = video.read()

        # Break the loop if no frame was retrieved
        if not ret:
            break

        # Generate the output image filename
        image_path = os.path.join(output_folder, f"frame_{frame_count:06d}.jpg")

        try:
            # Save the frame as an image
            cv2.imwrite(image_path, frame)

            # Increment the frame count
            frame_count += 1
        except Exception as e:
            print(f"Error saving frame {frame_count}:", str(e))

    # Release the video file
    video.release()

    print(f"Frames extracted: {frame_count}")

if __name__ == "__main__":
    video_path = ""
    output_folder = ""
    save_frames(video_path, output_folder)
