#!/usr/bin/env python3

from scipy.signal import convolve2d
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

	return output

def computePartialDerivatives(patch : np.ndarray):
    kernel_x = np.array([
        [0, 0, 0], 
        [-1, 0, 1], 
        [0, 0, 0]
    ])
    kernel_y = np.array([
        [0, -1, 0], 
        [0, 0, 0], 
        [0, 1, 0]
    ])

    # gx = convolve(patch, kernel_x).flatten()
    # gy = convolve(patch, kernel_y).flatten()
    gx = cv.Sobel(patch, cv.CV_64F, 1, 0, ksize=3).flatten()
    gy = cv.Sobel(patch, cv.CV_64F, 0, 1, ksize=3).flatten()
    return gx, gy

def computeLKDenseFlow(prev_img, curr_img):
    img_shape = prev_img.shape
    square_size = 5
    img_len = img_shape[0]
    flow_result = np.zeros((prev_img.shape[0], prev_img.shape[1], 2))

    for j in range(0, img_len, square_size):
        for i in range(0, img_len, square_size):
            flow = np.array([0, 0])

            prev_patch = prev_img[j:j+square_size,i:i+square_size]
            curr_patch = curr_img[j:j+square_size,i:i+square_size]

            # g0_x, g0_y = computePartialDerivatives(prev_patch)
            # g1_x, g1_y = computePartialDerivatives(curr_patch)

            # grad_x = g1_x - g0_x
            # grad_y = g1_y - g0_y

            kernelT = np.ones((2, 2)) * 0.25
            kernelX = np.array([[-1, 1], [-1, 1]]) * 0.25  # kernel for computing d/dx
            kernelY = np.array([[-1, -1], [1, 1]]) * 0.25  # kernel for computing d/dy

            grad_x = convolve2d(prev_patch, kernelX, "same") + convolve2d(curr_patch, kernelX, "same")
            grad_y = convolve2d(prev_patch, kernelY, "same") + convolve2d(curr_patch, kernelY, "same")
            grad_t = convolve2d(prev_patch, kernelT, "same") + convolve2d(curr_patch, kernelT, "same")

            grad_x = grad_x.flatten()
            grad_y = grad_y.flatten()

            # grad_t = curr_patch-prev_patch

            A = np.array([grad_x, grad_y]).T
            B = np.array([-grad_t]).T.flatten()
            ATA = A.T @ A

            # eps = 1e-9
            # threshold = 2
            # eig_x = np.linalg.norm(ATA[0])
            # eig_y = np.linalg.norm(ATA[1])
            # if eig_x > eig_y + threshold or eig_y > eig_x + threshold: continue
            # if eig_x < eps or eig_y < eps: continue

            if (np.linalg.det(ATA)) > 1e-9:
                flow = np.linalg.inv(ATA) @ (A.T @ B)
            flow_result[j:j+square_size,i:i+square_size] = np.array([flow[0]*23, flow[1]*22])

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
            # flow = cv.calcOpticalFlowFarneback(prev, curr, None, 0.5, 2, 3, 3, 5, 1.2, 0)
            vis = draw_flow(curr, flow, step=2)

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

