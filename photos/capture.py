#!/usr/bin/env python
import cv2
import numpy as np

cap = cv2.VideoCapture(1)
cv2.namedWindow("image", flags = cv2.WINDOW_NORMAL)

count = 0

while True:
    ret, img = cap.read()
    cv2.imshow('image', img)
    cv2.resizeWindow('image', 1600, 1200)
    cv2.moveWindow('image', 250, 500)

    key = cv2.waitKey(1)
    if key == 32:
        cv2.imwrite('capture_{}.png'.format(count), img)
        count += 1
    elif key == 27:
        break
cv2.destroyAllWindows()
