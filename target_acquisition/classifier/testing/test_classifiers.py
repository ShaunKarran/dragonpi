# Python libs
import argparse
import os
import sys
import time

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


class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'


def test_cascades_on_image(image, cascades, expected_results):
    """
    Apply the cascades to the image and compare the results to the expected results.

    image: An opencv image.
    cascades: A dictionary containing (cascade_name: cascade) pairs.
    expected_results: A dictionary containing (cascade_name: result) pairs where result is the number of results
        the cascade should return from detectMultiScale().

    return: A bool representing whether the test passed or failed to match the expected results.
    """
    test_result = True

    for cascade_name, cascade in cascades.iteritems():
        results = cascade.detectMultiScale(image)
        if len(results) != expected_results[cascade_name]:
            test_result = False

    return test_result


def main(args):
    """Main."""
    start_time = time.time()

    print 'Classifiers to be tested:'
    cascades = {}
    for file_name in os.listdir('./classifiers'):
        print '\t{}'.format(file_name)
        cascades[file_name[:-4]] = cv2.CascadeClassifier(os.path.join('./classifiers', file_name))
    print ""

    number_passed = 0
    for file_name in os.listdir('./images'):
        image = cv2.imread(os.path.join('./images', file_name))
        image = imutils.resize(image, width=args.resize_width)

        # Create a copy here so that the image exist and can be written even when no targets are found.
        image_highlighted = image.copy()

        test_result = test_cascades_on_image(image, cascades, expected_results[file_name])
        print '{:.0f}ms: Test for {} '.format((time.time() - start_time) * 1000, file_name),
        if test_result:
            print bcolors.OKGREEN + "PASSED" + bcolors.ENDC
            number_passed += 1
        else:
            print bcolors.FAIL + "FAILED" + bcolors.ENDC

        for cascade_name, cascade in cascades.iteritems():
            results = cascade.detectMultiScale(image)
            for (x, y, w, h) in results:
                image_highlighted = cv2.rectangle(image_highlighted, (x, y), (x + w, y + h), (255, 0, 0), 2)
                cv2.putText(img=image_highlighted,
                            text=cascade_name,
                            org=(x, y),
                            fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                            fontScale=0.6,
                            color=(0, 0, 255),
                            thickness=2)

        cv2.imwrite(os.path.join('./output', file_name), image_highlighted)

    print '\nTesting complete. {} out of {} passed.'.format(number_passed, len(os.listdir('./images')))


if __name__ == '__main__':
    parser = argparse.ArgumentParser()

    parser.add_argument('-r', '--resize-width', type=int, default=500)

    args = parser.parse_args(sys.argv[1:])
    main(args)
