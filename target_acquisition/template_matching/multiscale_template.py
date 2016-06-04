# Python libs
import argparse
import imutils
import ntpath
import sys
import time

from collections import namedtuple

# Raspberry Pi Camera
# from picamera import PiCamera
# from picamera.array import PiRGBArray

# OpenCV
import cv2

import numpy as np

# from matplotlib import pyplot as plt

MAX_VAL_THRESHOLD = 11000000


def main(args):
    """Main."""
    start_time = time.time()

    # Do this to initialise the camera.
    # camera = PiCamera()
    # rawCapture = PiRGBArray(camera)
    # time.sleep(0.1)

    # Define named tuple to store information relating to a given template.
    Template = namedtuple('Template', 'image width height')

    print '{:.0f}ms: Reading templates...'.format((time.time() - start_time) * 1000)
    templates = {}
    for template_path in args.templates:
        template = cv2.imread(template_path)  # Read image.
        template = imutils.resize(template, width=args.template_width)  # Scale to size.
        template = cv2.cvtColor(template, cv2.COLOR_BGR2GRAY)  # Convert to grayscale.
        template = cv2.Canny(template, 50, 200)  # Edge detection.
        (width, height) = template.shape[:2]  # Get width and height of template.

        templates[ntpath.basename(template_path)] = Template(image=template, width=width, height=height)

    print '{:.0f}ms: Reading image...'.format((time.time() - start_time) * 1000)
    if args.image:
        image = cv2.imread(args.image)
        image = imutils.resize(image, width=args.image_width)
    else:
        sys.exit()
        # camera.capture(rawCapture, format="bgr")
        # image = rawCapture.array

    # Convert it to grayscale, and initialize the bookkeeping variable to keep track of the matched region
    blur = cv2.GaussianBlur(image, (5, 5), 0)
    gray_image = cv2.cvtColor(blur, cv2.COLOR_BGR2GRAY)

    for template_name, template in templates.iteritems():
        result = None

        if args.visualise:
            cv2.imshow("Template", template.image)
            cv2.waitKey(0)

        # loop over the scales of the image
        print '{:.0f}ms: Running algorithm for template {}'.format((time.time() - start_time) * 1000, template_name)
        for scale in np.linspace(0.2, 1.0, 20)[::-1]:
            # resize the image according to the scale, and keep track of the ratio of the resizing
            resized = imutils.resize(gray_image, width=int(gray_image.shape[1] * scale))
            ratio = gray_image.shape[1] / float(resized.shape[1])

            # if the resized image is smaller than the template, then break from the loop
            if resized.shape[0] < template.width * 2 or resized.shape[1] < template.height * 2:
                break

            # detect edges in the resized, grayscale image and apply template matching to find the template in the image
            edged = cv2.Canny(resized, 50, 200)
            matched = cv2.matchTemplate(edged, template.image, cv2.TM_CCOEFF)
            (_, max_val, _, max_loc) = cv2.minMaxLoc(matched)

            # check to see if the iteration should be visualized
            if args.visualise:
                # draw a bounding box around the detected region
                clone = np.dstack([edged, edged, edged])
                cv2.rectangle(clone, (max_loc[0], max_loc[1]),
                              (max_loc[0] + template.height, max_loc[1] + template.width), (0, 0, 255), 2)
                cv2.imshow("Visualize", clone)
                cv2.waitKey(0)

            # if we have found a new maximum correlation value, then update the bookkeeping variable
            if (result is None) or (max_val > result[0]):
                result = (max_val, max_loc, ratio)

        # unpack the bookkeeping varaible and compute the (x, y) coordinates of the bounding box using the resized ratio
        (max_val, max_loc, ratio) = result
        # print max_val

        if max_val > MAX_VAL_THRESHOLD:
            top_left = (int(max_loc[0] * ratio), int(max_loc[1] * ratio))
            bottom_right = (int((max_loc[0] + height) * ratio), int((max_loc[1] + width) * ratio))

            # draw a bounding box around the detected result and display the image
            print '{:.0f}ms: Highlighting targets in image.'.format((time.time() - start_time) * 1000)
            cv2.rectangle(image, top_left, bottom_right, (0, 0, 255), 2)

            org = (top_left[0] - 20, bottom_right[1] + 20)
            cv2.putText(img=image,
                        text=template_name,
                        org=org,
                        fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                        fontScale=0.6,
                        color=(0, 0, 255),
                        thickness=2)
        else:
            print '{:.0f}ms: No targets found in image.'.format((time.time() - start_time) * 1000)

    # results = {}
    # for template_name, template in templates.iteritems():
    #     print '{:.0f}ms: Running algorithm for template {}'.format((time.time() - start_time) * 1000, template_name)
    #     results[template_name] = cv2.matchTemplate(image, template, cv2.TM_CCOEFF)
    #
    # print '{:.0f}ms: Highlighting targets in image.'.format((time.time() - start_time) * 1000)
    # for template_name, result in results.iteritems():
    #     _, _, _, top_left = cv2.minMaxLoc(result)
    #     bottom_right = (top_left[0] + args.template_width, top_left[1] + args.template_width)
    #
    #     image_highlighted = cv2.rectangle(image, top_left, bottom_right, (255, 0, 0), 2)
    #     plt.imshow(result, cmap='gray')
    #     plt.show()

    # print '{:.0f}ms: Writing highlighted image.'.format((time.time() - start_time) * 1000)
    # cv2.imwrite('image_highlighted.jpg', image_highlighted)

    # cv2.imshow('image_highlighted', image_highlighted)
    # cv2.waitKey(0)

    print '{:.0f}ms: Finished'.format((time.time() - start_time) * 1000)

    cv2.imshow("Image", image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()

    parser.add_argument('-i', '--image')
    parser.add_argument('-t', '--templates', nargs='*', required=True)
    parser.add_argument('-iw', '--image-width', type=int, default=500)
    parser.add_argument('-tw', '--template-width', type=int, default=500)
    parser.add_argument('-v', '--visualise', action='store_true', default=False)

    args = parser.parse_args(sys.argv[1:])
    main(args)
