#!/usr/bin/env python3

import matplotlib.pyplot as plt
import numpy as np
import cv2 as cv
import argparse
import sys

def draw_flow(img, flow, step=16):
    h, w = img.shape[:2]
    y, x = np.mgrid[step/2:h:step, step/2:w:step].reshape(2,-1).astype(int)
    fy, fx = flow[x,y].T
    lines = np.vstack([x, y, x+fx, y+fy]).T.reshape(-1, 2, 2)
    lines = np.int32(lines + 0.5)
    vis = cv.cvtColor(img, cv.COLOR_GRAY2BGR)
    cv.polylines(vis, lines, 0, (0, 255, 0))
    for (x1, y1), (_x2, _y2) in lines:
        cv.circle(vis, (x1, y1), 1, (0, 255, 0), -1)
    return vis

def convolve(image, kernel):
	(iH, iW) = image.shape[:2]
	(kH, kW) = kernel.shape[:2]
	pad = (kW - 1) // 2

	image = cv.copyMakeBorder(image, pad, pad, pad, pad, cv.BORDER_REPLICATE)
	output = np.zeros((iH, iW), dtype="float32")

	for y in np.arange(pad, iH + pad):
		for x in np.arange(pad, iW + pad):
			roi = image[y - pad:y + pad + 1, x - pad:x + pad + 1]
			k = (roi * kernel).sum()
			output[y - pad, x - pad] = k

	return output.flatten()

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
    kernel_t = np.ones((3, 3)) * (1/9)

    # gx = convolve(patch, kernel_x)
    # gy = convolve(patch, kernel_y)
    # gt = convolve(patch, kernel_t)

    gx = np.convolve(patch.flatten(), kernel_x.flatten())
    gy = np.convolve(patch.flatten(), kernel_y.flatten())
    gt = np.convolve(patch.flatten(), kernel_t.flatten())
    return gx, gy, gt

def computeLKDenseFlow(prev_img, curr_img):
    img_shape = prev_img.shape
    square_size = 16
    img_len = img_shape[0]
    flow_result = np.zeros((prev_img.shape[0], prev_img.shape[1], 2))

    for j in range(0, img_len, square_size):
        for i in range(0, img_len, square_size):
            flow = np.array([0, 0])

            prev_patch = prev_img[j:j+square_size,i:i+square_size]
            curr_patch = curr_img[j:j+square_size,i:i+square_size]

            g0_x, g0_y, g0_t = computePartialDerivatives(prev_patch)
            g1_x, g1_y, g1_t = computePartialDerivatives(curr_patch)

            grad_x = g1_x + g0_x
            grad_y = g1_y + g0_y
            grad_t = g1_t - g0_t

            A = np.array([grad_x, grad_y]).T
            B = np.array([-grad_t]).T.flatten()
            ATA = A.T @ A

            eps = 1e-12
            eig_x = np.linalg.norm(ATA[0])
            eig_y = np.linalg.norm(ATA[1])
            if eig_x < eps or eig_y < eps: continue

            if (np.linalg.det(ATA)) > eps:
                flow = np.linalg.inv(ATA) @ (A.T @ B)
            flow_result[i:i+square_size,j:j+square_size] = np.array([flow[0]*1e3, flow[1]*1e3])

    return np.array(flow_result)

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
            flow = computeLKDenseFlow(prev, curr)
            # flow = cv.calcOpticalFlowFarneback(prev, curr, None, 0.5, 2, 16, 3, 5, 1.2, 0)
            vis = draw_flow(curr, flow, step=10)
            # import pdb; pdb.set_trace()

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
