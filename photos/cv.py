#!/usr/bin/env python
import cv2
import sys
import numpy as np
import glob

#lower_color = np.array([75, 25, 50])
#high_color = np.array([100, 255, 255])

count = 0

img = None

kernel = np.ones((5, 5), np.uint8)

#images = [cv2.imread(file) for file in glob.glob('./bot_pics/*.png')]

cv2.namedWindow("original", flags = cv2.WINDOW_NORMAL)
cv2.namedWindow("processed", flags = cv2.WINDOW_NORMAL)


def update_img(x):
    # get current positions of four trackbars
    low_h = cv2.getTrackbarPos('Low H','original') * 5
    low_s = cv2.getTrackbarPos('Low S','original') * 5
    low_v = cv2.getTrackbarPos('Low V','original') * 5
    high_h = cv2.getTrackbarPos('High H','original') * 5
    high_s = cv2.getTrackbarPos('High S','original') * 5
    high_v = cv2.getTrackbarPos('High V','original') * 5
    kernel_size = cv2.getTrackbarPos('kernel_size','original')
    iterations = cv2.getTrackbarPos('iterations','original')
    blur = cv2.getTrackbarPos('blur','original')
    blur_x = cv2.getTrackbarPos('blur_x','original')

    kernel = np.ones((kernel_size * 2 + 1, kernel_size * 2 + 1), np.uint8)

    lower_color = np.array([low_h, low_s, low_v])
    high_color = np.array([high_h, high_s, high_v])

    cv2.imshow('processed', process_image(img, lower_color, high_color, kernel, iterations, blur, blur_x))
    cv2.resizeWindow('processed', 1600, 1200)
    cv2.moveWindow('processed', 2050, 500)


# create trackbars for color change
cv2.createTrackbar('Low H', 'original', 0, 51, update_img)
cv2.createTrackbar('Low S', 'original', 0, 51, update_img)
cv2.createTrackbar('Low V', 'original', 0, 51, update_img)
cv2.createTrackbar('High H', 'original', 0, 51, update_img)
cv2.createTrackbar('High S', 'original', 0, 51, update_img)
cv2.createTrackbar('High V', 'original', 0, 51, update_img)
cv2.createTrackbar('kernel_size', 'original', 0, 10, update_img)
cv2.createTrackbar('iterations', 'original', 0, 50, update_img)
cv2.createTrackbar('blur', 'original', 0, 50, update_img)
cv2.createTrackbar('blur_x', 'original', 0, 50, update_img)

print('80, 40, 120')
print('255, 255, 255')

print('0, 30, 110')
print('0, 0')

lower_color_magenta = np.array([0, 0, 0])
high_color_magenta = np.array([255, 255, 255])

lower_color_orange = np.array([0, 0, 0])
high_color_orange = np.array([255, 255, 255])

lower_color_yellow = np.array([0, 0, 0])
high_color_yellow = np.array([255, 255, 255])

lower_color_green = np.array([0, 0, 0])
high_color_green = np.array([255, 255, 255])

lower_color_blue = np.array([0, 0, 0])
high_color_blue = np.array([255, 255, 255])

lower_color_purple = np.array([0, 0, 0])
high_color_purple = np.array([255, 255, 255])

def process_image(input_image, lower_color, high_color, kernel, iterations, blur, blur_x):
    hsv = cv2.cvtColor(input_image, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower_color, high_color)
    proc = cv2.GaussianBlur(input_image, (blur * 2 + 1, blur * 2 + 1), blur_x)
    proc = cv2.bitwise_and(proc, proc, mask = mask)
    proc = cv2.erode(proc, kernel, iterations=iterations)
    proc = cv2.dilate(proc, kernel, iterations=iterations)

    return proc


while True:
    img = cv2.imread('./bot_pics/capture_{}.png'.format(count))

    cv2.imshow('original', img)
    cv2.resizeWindow('original', 1600, 1200)
    cv2.moveWindow('original', 250, 500)

    update_img(None)


    key = cv2.waitKey(0)
    if key == 32 or key == 106:
        count += 1
    elif key == 107:
        count -=1
    elif key == 27 or key == 113:
        break

cv2.destroyAllWindows()
