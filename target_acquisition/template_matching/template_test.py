# Python libs
import argparse
import imutils
import sys
import time

# Raspberry Pi Camera
# from picamera import PiCamera
# from picamera.array import PiRGBArray

# OpenCV
import cv2

from matplotlib import pyplot as plt


def main(args):
    """Main."""
    start_time = time.time()

    # Do this to initialise the camera.
    # camera = PiCamera()
    # rawCapture = PiRGBArray(camera)
    # time.sleep(0.1)

    print '{:.0f}ms: Reading templates...'.format((time.time() - start_time) * 1000)
    templates = {}
    for template_name in args.templates:
        template = cv2.imread(template_name)
        template = imutils.resize(template, width=args.template_width)
        templates[template_name] = template

    print '{:.0f}ms: Reading image...'.format((time.time() - start_time) * 1000)
    if args.image:
        image = cv2.imread(args.image)
        image = imutils.resize(image, width=args.image_width)
    else:
        sys.exit()
        # camera.capture(rawCapture, format="bgr")
        # image = rawCapture.array

    # image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    results = {}
    for template_name, template in templates.iteritems():
        print '{:.0f}ms: Running algorithm for template {}'.format((time.time() - start_time) * 1000, template_name)
        results[template_name] = cv2.matchTemplate(image, template, cv2.TM_CCOEFF)

    print '{:.0f}ms: Highlighting targets in image.'.format((time.time() - start_time) * 1000)
    for template_name, result in results.iteritems():
        _, _, _, top_left = cv2.minMaxLoc(result)
        bottom_right = (top_left[0] + args.template_width, top_left[1] + args.template_width)

        image_highlighted = cv2.rectangle(image, top_left, bottom_right, (255, 0, 0), 2)
        plt.imshow(result, cmap='gray')
        plt.show()

    # print '{:.0f}ms: Writing highlighted image.'.format((time.time() - start_time) * 1000)
    # cv2.imwrite('image_highlighted.jpg', image_highlighted)

    cv2.imshow('image_highlighted', image_highlighted)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    print '{:.0f}ms: Finished'.format((time.time() - start_time) * 1000)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()

    parser.add_argument('-i', '--image')
    parser.add_argument('-iwidth', '--image-width', type=int, default=500)
    parser.add_argument('-t', '--templates', nargs='*', required=True)
    parser.add_argument('-twidth', '--template-width', type=int, default=500)

    args = parser.parse_args(sys.argv[1:])
    main(args)
