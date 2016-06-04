import cv2
import numpy as np
import sys

HUE_RED_LOW = 1
HUE_ORANGE = 40
HUE_YELLOW = 60
HUE_GREEN = 120
HUE_BLUE = 245
HUE_PINK = 300
HUE_RED_HIGH = 360

COLOUR = HUE_ORANGE

img = cv2.imread(sys.argv[1])                     # To read the image
img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

cv2.imshow('original', img)
# cv2.imshow('hsv', img_hsv)
cv2.waitKey(0)

# Arrays are values in order - BGR
# Blue
# lower = np.array([160, 10, 10], dtype='uint8')  # To set the lower boundary
# upper = np.array([255, 50, 50], dtype='uint8')  # To set the Upper boundary
# Red
# lower = np.array([10, 10, 160], dtype='uint8')  # To set the lower boundary
# upper = np.array([50, 50, 255], dtype='uint8')  # To set the Upper boundary
# Yellow
# lower = np.array([0, 180, 180], dtype='uint8')    # To set the lower boundary
# upper = np.array([150, 255, 255], dtype='uint8')  # To set the Upper boundary

# mask = cv2.inRange(img, lower, upper)         # search about the color in the range
# output = cv2.GaussianBlur(mask, (3, 3), 0)      # Using Gaussian Blur filter

lower_range = np.array([max(COLOUR - 15, 0), 175, 175], dtype=np.uint8)
upper_range = np.array([min(COLOUR + 15, 255), 255, 255], dtype=np.uint8)

# lower_range = np.array([0, 0, 205], dtype=np.uint8)
# upper_range = np.array([180, 50, 255], dtype=np.uint8)

mask = cv2.inRange(img_hsv, lower_range, upper_range)

output = cv2.bitwise_or(img_hsv, img_hsv, mask=mask)
output = cv2.cvtColor(output, cv2.COLOR_HSV2BGR)

cv2.imshow('output', output)                    # To show the image after detection
cv2.waitKey(0)

contours, h = cv2.findContours(mask, 1, 2)

for cnt in contours:
    approx = cv2.approxPolyDP(cnt, 0.01 * cv2.arcLength(cnt, True), True)
    print len(approx)

    if len(approx) == 3:
        print "triangle"
        cv2.drawContours(img, [cnt], 0, (0, 255, 0), -1)

# cv2.imwrite('ouput.jpg', output)                   # To save the image after detection
