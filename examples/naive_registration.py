#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt
import cv2 as cv
import argparse
import sys

from naive_optical_flow import draw_flow

def computeImageRegistration(prev, curr):
    img_shape = prev.shape
    img_len = img_shape[0]
    square_size = 30
    search_size = 10
    step = 4
    result = np.zeros((*prev.shape,2))

    for j in range(0, img_len, step):
        for i in range(0, img_len, step):

            pad = search_size
            template = prev[j+pad:j+pad+square_size,i+pad:i+pad+square_size]
            patch = curr[j:j+pad*2+square_size,i:i+pad*2+square_size]

            w, h = template.shape[::-1]
            if w == 0 or h == 0: continue

            res = cv.matchTemplate(patch, template, cv.TM_CCOEFF_NORMED)
            min_val, max_val, min_loc, max_loc = cv.minMaxLoc(res)
            if (max_val < 0.98): continue

            flow = ((pad - max_loc[0]), (pad - max_loc[1]))
            result[i:i+step, j:j+step] = flow

    return result

def main(args):
    cap = cv.VideoCapture(args.video)
    _, prev = cap.read()
    im_size = 400
    prev = cv.resize(prev, (im_size, im_size))[:,:,0]
    try:
        while(1):
            ret, curr = cap.read()

            if not ret:
                break

            curr = cv.resize(curr, (im_size, im_size))[:,:, 0]
            flow = computeImageRegistration(prev, curr)
            vis = draw_flow(curr, flow, step=5)

            cv.imshow('frame', vis)
            k = cv.waitKey(30) & 0xff
            prev = curr
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
