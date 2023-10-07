#!/usr/bin/env python3

import numpy as np
import cv2 as cv
import argparse
import sys


def computePartialDerivatives(patch : np.ndarray):
    kernel_x = np.array([
        [-1, 0, 1], 
        [-2, 0, 2], 
        [-1, 0, 1]
    ])
    kernel_y = np.array([
        [-1, -2, -1], 
        [0, 0, 0], 
        [1, 2, 1]
    ])

    size = patch.shape[:2]
    result = np.zeros(size)

def computeLKDenseFlow(prev_img, curr_img):
    pass

def main(args):
    cap = cv.VideoCapture(args.video)
    try:
        while(1):
            ret, frame = cap.read()
            
            if not ret:
                break

            frame_gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
            cv.imshow('frame', frame)
            k = cv.waitKey(30) & 0xff
            if k == 'q' or k == 27:
                break
    except KeyboardInterrupt:
        sys.exit()
    cv.destroyAllWindows()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='This sample demonstrates Lucas-Kanade Optical Flow.')
    parser.add_argument('video', type=str, help='path to video file')

    args = parser.parse_args()
    main(args)

