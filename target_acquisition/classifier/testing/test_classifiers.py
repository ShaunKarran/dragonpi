from __future__ import division

import argparse
import os
import sys
from collections import defaultdict

from termcolor import colored

import cv2
import imutils

expected_results = {
    "image_00.jpg": {"acid": 1, "misc": 0, "flammable": 0},
    "image_01.jpg": {"acid": 1, "misc": 0, "flammable": 0},
    "image_02.jpg": {"acid": 1, "misc": 0, "flammable": 0},
    "image_03.jpg": {"acid": 1, "misc": 0, "flammable": 0},
    "image_04.jpg": {"acid": 1, "misc": 0, "flammable": 0},
    "image_05.jpg": {"acid": 1, "misc": 0, "flammable": 0},
    "image_06.jpg": {"acid": 0, "misc": 0, "flammable": 1},
    "image_07.jpg": {"acid": 0, "misc": 0, "flammable": 1},
    "image_08.jpg": {"acid": 0, "misc": 0, "flammable": 1},
    "image_09.jpg": {"acid": 0, "misc": 0, "flammable": 1},
    "image_10.jpg": {"acid": 0, "misc": 1, "flammable": 0},
    "image_11.jpg": {"acid": 0, "misc": 1, "flammable": 0},
    "image_12.jpg": {"acid": 0, "misc": 1, "flammable": 0},
    "image_13.jpg": {"acid": 0, "misc": 1, "flammable": 1},
    "image_14.jpg": {"acid": 1, "misc": 1, "flammable": 0},
    "image_15.jpg": {"acid": 1, "misc": 1, "flammable": 1},
    "image_16.jpg": {"acid": 1, "misc": 0, "flammable": 1},
    "image_17.jpg": {"acid": 0, "misc": 0, "flammable": 1},
    "image_18.jpg": {"acid": 1, "misc": 0, "flammable": 0},
    "image_19.jpg": {"acid": 0, "misc": 1, "flammable": 0},
    "image_20.jpg": {"acid": 0, "misc": 1, "flammable": 1},
    "image_21.jpg": {"acid": 1, "misc": 0, "flammable": 0},
    "image_22.jpg": {"acid": 1, "misc": 0, "flammable": 0},
    "image_23.jpg": {"acid": 0, "misc": 0, "flammable": 0},
    "image_24.jpg": {"acid": 0, "misc": 0, "flammable": 0},
    "image_25.jpg": {"acid": 0, "misc": 0, "flammable": 0},
    "image_26.jpg": {"acid": 0, "misc": 0, "flammable": 0},
    "image_27.jpg": {"acid": 0, "misc": 0, "flammable": 0},
    "image_28.jpg": {"acid": 0, "misc": 0, "flammable": 0},
    "image_29.jpg": {"acid": 0, "misc": 0, "flammable": 0},
    "image_30.jpg": {"acid": 0, "misc": 0, "flammable": 0}
}


def main(args):
    """Main."""
    print 'Testing:\t',
    cascades = {}
    for file_name in os.listdir('./classifiers'):
        print '{}\t'.format(file_name),
        cascades[file_name[:-4]] = cv2.CascadeClassifier(os.path.join('./classifiers', file_name))
    print ""  # New line.

    # Store the total number of times each target appears in the set of test images.
    number_positives = defaultdict(int)
    for cascade_name, _ in cascades.iteritems():
        number_positives[cascade_name] += sum([results[cascade_name] for _, results in expected_results.iteritems()])

    number_detected_positives = defaultdict(int)  # Used to store a count of how many times each target is found.

    for file_name in os.listdir('./images'):
        image = cv2.imread(os.path.join('./images', file_name))
        image = imutils.resize(image, width=args.resize_width)

        # Create a copy here so that the image exist and can be written even when no targets are found.
        image_highlighted = image.copy()

        print '{}\t'.format(file_name),
        for cascade_name, cascade in cascades.iteritems():
            results = cascade.detectMultiScale(image)

            if len(results) > expected_results[file_name][cascade_name]:
                print colored('FALSE POSITIVE\t', 'red'),
            elif len(results) < expected_results[file_name][cascade_name]:
                print colored('FALSE NEGATIVE\t', 'yellow'),
            elif len(results) == 0:
                print colored('TRUE NEGATIVE\t', 'white'),
            else:
                print colored('TRUE POSITIVE\t', 'green'),
                number_detected_positives[cascade_name] += 1

            for (x, y, w, h) in results:
                image_highlighted = cv2.rectangle(image_highlighted, (x, y), (x + w, y + h), (255, 0, 0), 2)
                cv2.putText(img=image_highlighted,
                            text=cascade_name,
                            org=(x, y),
                            fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                            fontScale=0.6,
                            color=(0, 0, 255),
                            thickness=2)

        print ""  # New line.

        cv2.imwrite(os.path.join('./output', file_name), image_highlighted)

    print "\nAccuracy\t",
    for cascade_name, _ in cascades.iteritems():
        print "{:.1f}%\t\t".format(number_detected_positives[cascade_name] / number_positives[cascade_name] * 100),


if __name__ == '__main__':
    parser = argparse.ArgumentParser()

    parser.add_argument('-r', '--resize-width', type=int, default=500)

    args = parser.parse_args(sys.argv[1:])
    main(args)
