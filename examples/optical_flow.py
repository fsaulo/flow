#!/bin/python3

import numpy as np
import cv2 as cv
import argparse


def main():
    parser = argparse.ArgumentParser(description='This sample demonstrates Lucas-Kanade Optical Flow calculation.')
    parser.add_argument('image', type=str, help='path to image file')
    args = parser.parse_args()

    cap = cv.VideoCapture(args.image)

    # params for ShiTomasi corner detection
    feature_params = dict( maxCorners = 100,
                           qualityLevel = 0.3,
                           minDistance = 7,
                           blockSize = 7 )

    # Parameters for lucas kanade optical flow
    lk_params = dict( winSize  = (15, 15),
                      maxLevel = 2,
                      criteria = (cv.TERM_CRITERIA_EPS | cv.TERM_CRITERIA_COUNT, 10, 0.03))

    # Create some random colors
    color = np.random.randint(0, 255, (100, 3))

    # Take first frame and find corners in it
    ret, old_frame = cap.read()
    old_gray = cv.cvtColor(old_frame, cv.COLOR_BGR2GRAY)
    p0 = cv.goodFeaturesToTrack(old_gray, mask = None, **feature_params)

    # Create a mask image for drawing purposes
    mask = np.zeros_like(old_frame)

    while(1):
        ret, frame = cap.read()
        if not ret:
            print('No frames grabbed!')
            break

        frame_gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)

        # calculate optical flow
        p1, st, err = cv.calcOpticalFlowPyrLK(old_gray, frame_gray, p0, None, **lk_params)

        # Select good points
        if p1 is not None:
            good_new = p1[st==1]
            good_old = p0[st==1]

        # draw the tracks
        for i, (new, old) in enumerate(zip(good_new, good_old)):
            next_x, next_y = new.ravel()
            prev_x, prev_y = old.ravel()

            cv.arrowedLine(frame, (int(prev_x), int(prev_y)), (int(next_x), int(next_y)), (0, 255, 0), 5)
            # mask = cv.line(frame, (int(prev_x), int(prev_y)), (int(next_x), int(next_y)), (255,0,0), 2)
            frame = cv.circle(frame, (int(prev_x), int(prev_y)), 1, (0,0,0), 3)

        img = cv.add(frame, mask)
        cv.imshow('frame', img)
        k = cv.waitKey() & 0xff
        if k == 'q' or k == 113:
            break
        else:
            continue

        # Now update the previous frame and previous points
        old_gray = frame_gray.copy()
        p0 = good_new.reshape(-1, 1, 2)

    cv.destroyAllWindows()

if __name__ == '__main__':
    main()
