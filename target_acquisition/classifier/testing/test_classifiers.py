# Python libs
import argparse
import os
import sys
import time

# Coloured print statements
from termcolor import colored

# OpenCV
import cv2
import imutils

expected_results = {
    "image_00.jpg": {"acid": 1, "misc": 0, "flammable": 0},
    "image_01.jpg": {"acid": 1, "misc": 0, "flammable": 0},
    "image_02.jpg": {"acid": 1, "misc": 0, "flammable": 0},
    "image_03.jpg": {"acid": 1, "misc": 0, "flammable": 0},
    "image_04.jpg": {"acid": 1, "misc": 0, "flammable": 0},
    "image_05.jpg": {"acid": 1, "misc": 0, "flammable": 0},
    "image_06.jpg": {"acid": 0, "misc": 0, "flammable": 0},
    "image_07.jpg": {"acid": 0, "misc": 0, "flammable": 0},
    "image_08.jpg": {"acid": 0, "misc": 0, "flammable": 1},
    "image_09.jpg": {"acid": 0, "misc": 0, "flammable": 1},
    "image_10.jpg": {"acid": 0, "misc": 0, "flammable": 1},
    "image_11.jpg": {"acid": 0, "misc": 0, "flammable": 1},
    "image_12.jpg": {"acid": 0, "misc": 0, "flammable": 0},
    "image_13.jpg": {"acid": 0, "misc": 1, "flammable": 0},
    "image_14.jpg": {"acid": 0, "misc": 1, "flammable": 0},
    "image_15.jpg": {"acid": 0, "misc": 1, "flammable": 0},
    "image_16.jpg": {"acid": 0, "misc": 1, "flammable": 1},
    "image_17.jpg": {"acid": 0, "misc": 0, "flammable": 0},
    "image_18.jpg": {"acid": 0, "misc": 0, "flammable": 0},
    "image_19.jpg": {"acid": 0, "misc": 0, "flammable": 0}
}


def main(args):
    """Main."""
    start_time = time.time()

    print 'Classifiers to be tested:'
    cascades = {}
    for file_name in os.listdir('./classifiers'):
        print '\t{}'.format(file_name)
        cascades[file_name[:-4]] = cv2.CascadeClassifier(os.path.join('./classifiers', file_name))
    print ""

    num_passed = 0
    for file_name in os.listdir('./images'):
        image = cv2.imread(os.path.join('./images', file_name))
        image = imutils.resize(image, width=args.resize_width)

        # Create a copy here so that the image exist and can be written even when no targets are found.
        image_highlighted = image.copy()

        false_positive = False
        false_negative = False
        print '{:.0f}ms: Testing {} -'.format((time.time() - start_time) * 1000, file_name),
        for cascade_name, cascade in cascades.iteritems():
            results = cascade.detectMultiScale(image)

            if len(results) > expected_results[file_name][cascade_name]:
                false_positive = True
            elif len(results) < expected_results[file_name][cascade_name]:
                false_negative = True

            for (x, y, w, h) in results:
                image_highlighted = cv2.rectangle(image_highlighted, (x, y), (x + w, y + h), (255, 0, 0), 2)
                cv2.putText(img=image_highlighted,
                            text=cascade_name,
                            org=(x, y),
                            fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                            fontScale=0.6,
                            color=(0, 0, 255),
                            thickness=2)

        if false_positive or false_negative:
            print colored('FAILED -', 'red'),
            if false_positive:
                print colored('False positive', 'red')
            if false_negative:
                print colored('False negative', 'yellow')
        else:
            print colored('PASSED', 'green')
            num_passed += 1

        cv2.imwrite(os.path.join('./output', file_name), image_highlighted)

    print '\nTesting complete. {} out of {} passed.'.format(num_passed, len(os.listdir('./images')))


if __name__ == '__main__':
    parser = argparse.ArgumentParser()

    parser.add_argument('-r', '--resize-width', type=int, default=500)

    args = parser.parse_args(sys.argv[1:])
    main(args)
