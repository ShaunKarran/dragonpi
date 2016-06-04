import argparse
import cv2
import imutils
import numpy as np
import time
from itertools import combinations

from shapedetector import ShapeDetector


HUE_RED_LOW = 1
HUE_ORANGE = 40
HUE_YELLOW = 60
HUE_GREEN = 120
HUE_BLUE = 245
HUE_PINK = 300
HUE_RED_HIGH = 360

COLOUR = HUE_RED_HIGH

HSV_BLACK_LOWER = np.array((0, 0, 0), dtype=np.uint8)
HSV_BLACK_UPPER = np.array((180, 255, 90), dtype=np.uint8)

HSV_RED_LOWER = np.array((170, 200, 220), dtype=np.uint8)
HSV_RED_UPPER = np.array((180, 240, 250), dtype=np.uint8)


def is_close(a, b, rel_tol=1e-09, abs_tol=0.0):
    """
    Return True if the values a and b are close to each other and False otherwise.

    Implementation from https://docs.python.org/3/library/math.html#math.isclose
    """
    return abs(a - b) <= max(rel_tol * max(abs(a), abs(b)), abs_tol)


def contians_acid_sign(image):
    hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv_image, HSV_BLACK_LOWER, HSV_BLACK_UPPER)

    # cv2.imshow('Mask', mask)
    # cv2.waitKey(0)

    # Find contours in the thresholded image and initialize the shape detector.
    contours = cv2.findContours(mask.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    contours = contours[0] if imutils.is_cv2() else contours[1]

    # cv2.drawContours(image, contours, -1, (0, 255, 0), 2)

    sd = ShapeDetector()

    triangle_contours = []
    for contour in contours:
        # Detect the name of the shape using only the contour.
        shape = sd.detect(contour)

        if shape is 'triangle':
            triangle_contours.append(contour)

    # cv2.drawContours(image, triangle_contours, -1, (255, 0, 0), 2)

    sign_contours = []
    for contour1, contour2 in combinations(triangle_contours, 2):
        area1 = cv2.contourArea(contour1)
        area2 = cv2.contourArea(contour2)
        ratio1 = area1 / area2
        ratio2 = area2 / area1
        if (ratio1 > 1.1 and ratio1 < 1.8) or (ratio2 > 1.1 and ratio2 < 1.8):
            # Compute the center of the contour.
            M1 = cv2.moments(contour1)
            M2 = cv2.moments(contour2)
            c1X = int(M1["m10"] / M1["m00"])
            c1Y = int(M1["m01"] / M1["m00"])
            c2X = int(M2["m10"] / M2["m00"])
            c2Y = int(M2["m01"] / M2["m00"])
            if is_close(c1X, c2X, abs_tol=5) and is_close(c1Y, c2Y, abs_tol=5):
                sign_contours.extend([contour1, contour2])

    if len(sign_contours) == 2:
        cv2.drawContours(image, sign_contours, -1, (0, 0, 255), 2)
        return True

    return False


def contians_flammable_sign(image):
    hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv_image, HSV_RED_LOWER, HSV_RED_UPPER)

    # cv2.imshow('Mask', mask)
    # cv2.waitKey(0)

    # Find contours in the thresholded image and initialize the shape detector.
    contours = cv2.findContours(mask.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    contours = contours[0] if imutils.is_cv2() else contours[1]

    # cv2.drawContours(image, contours, -1, (0, 255, 0), 2)

    sd = ShapeDetector()

    square_contours = []
    for contour in contours:
        # Detect the name of the shape using only the contour.
        shape = sd.detect(contour)

        if shape in ['square', 'rectangle']:
            square_contours.append(contour)

    cv2.drawContours(image, square_contours, -1, (255, 0, 0), 2)

    sign_contours = []
    for contour1, contour2 in combinations(square_contours, 2):
        area1 = cv2.contourArea(contour1)
        area2 = cv2.contourArea(contour2)
        ratio1 = area1 / area2
        ratio2 = area2 / area1
        if (ratio1 > 1.1 and ratio1 < 1.8) or (ratio2 > 1.1 and ratio2 < 1.8):
            # Compute the center of the contour.
            M1 = cv2.moments(contour1)
            M2 = cv2.moments(contour2)
            c1X = int(M1["m10"] / M1["m00"])
            c1Y = int(M1["m01"] / M1["m00"])
            c2X = int(M2["m10"] / M2["m00"])
            c2Y = int(M2["m01"] / M2["m00"])
            if is_close(c1X, c2X, abs_tol=5) and is_close(c1Y, c2Y, abs_tol=5):
                sign_contours.extend([contour1, contour2])

    if len(sign_contours) == 2:
        return True

    return False


start_time = time.time()

args = argparse.ArgumentParser()
args.add_argument('-i', '--image', required=True, help='path to the input image')
args = vars(args.parse_args())

# Load the image and resize it to a smaller factor so that the shapes can be approximated better.
print '{:.0f}ms: Reading image...'.format((time.time() - start_time) * 1000)
image = cv2.imread(args['image'])
image = imutils.resize(image, width=500)

print '{:.0f}ms: Searching for acid sign...'.format((time.time() - start_time) * 1000)
acid_found = contians_acid_sign(image)
print '{:.0f}ms: Finshed searching.'.format((time.time() - start_time) * 1000)
print 'Acid sign found: {}'.format(acid_found)

# print '{:.0f}ms: Searching for flammable sign...'.format((time.time() - start_time) * 1000)
# flammable_found = contians_flammable_sign(image)
# print '{:.0f}ms: Finshed searching.'.format((time.time() - start_time) * 1000)
# print 'Flammable sign found: {}'.format(flammable_found)

cv2.imshow('Image', image)
cv2.waitKey(0)
# cv2.imwrite('found.jpg', image)
