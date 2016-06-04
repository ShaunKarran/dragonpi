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


def main(args):
    """Main."""
    start_time = time.time()

    cascades = {}
    for target in args.targets:
        cascades[target] = cv2.CascadeClassifier(target)

    # Do this to initialise the camera.
    # camera = PiCamera()
    # rawCapture = PiRGBArray(camera)
    # time.sleep(0.1)

    print '{:.0f}ms: Reading image...'.format((time.time() - start_time) * 1000)
    if args.image:
        image = cv2.imread(args.image)
        image = imutils.resize(image, width=args.resize_width)
    else:
        sys.exit()
        # camera.capture(rawCapture, format="bgr")
        # image = rawCapture.array

    # image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    targets_detected = {}
    for target, cascade in cascades.iteritems():
        print '{:.0f}ms: Running algorithm for target {}'.format((time.time() - start_time) * 1000, target)
        targets_detected[target] = cascade.detectMultiScale(image)

    print '{:.0f}ms: Highlighting targets in image.'.format((time.time() - start_time) * 1000, target)
    for target, results in targets_detected.iteritems():
        for (x, y, w, h) in results:
            image_highlighted = cv2.rectangle(image, (x, y), (x + w, y + h), (255, 0, 0), 2)

    print '{:.0f}ms: Writing highlighted image.'.format((time.time() - start_time) * 1000)
    cv2.imwrite('image_highlighted.jpg', image_highlighted)

    cv2.imshow('image_highlighted', image_highlighted)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    print '{:.0f}ms: Finished'.format((time.time() - start_time) * 1000)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()

    parser.add_argument('-i', '--image')
    parser.add_argument('-t', '--targets', nargs='*', required=True)
    parser.add_argument('-r', '--resize-width', type=int, default=500)

    args = parser.parse_args(sys.argv[1:])
    main(args)
