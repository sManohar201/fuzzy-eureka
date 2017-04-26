#!/usr/bin/env python
import cv2
import numpy as np

font = cv2.FONT_HERSHEY_SIMPLEX

cv2.namedWindow("ouput", flags = cv2.WINDOW_NORMAL)

img = np.zeros((512, 2400, 3), np.uint8)

def draw_egg(color, count, x, y):
    cv2.ellipse(img, (x, y), (140, 200), 0, 0, 360, color, -1)
    cv2.putText(img, count, (x - 80, y + 100), font, 8, (255, 255, 255), 12)

def show_output(eggs):
    draw_egg((255, 0, 255), str(eggs[0]), 200, 250)     # Magenta
    draw_egg((0, 165, 255), str(eggs[1]), 600, 250)     # Orange
    draw_egg((0, 215, 255), str(eggs[2]), 1000, 250)    # Yellow
    draw_egg((0, 255, 0), str(eggs[3]), 1400, 250)      # Green
    draw_egg((255, 80, 0), str(eggs[4]), 1800, 250)     # Blue
    draw_egg((220, 40, 140), str(eggs[5]), 2200, 250)   # Purple

cv2.imshow('ouput', img)
cv2.resizeWindow('ouput', 2400, 512)
cv2.moveWindow('ouput', 1125, 250)

key = cv2.waitKey(0)

cv2.destroyAllWindows()
