# Python libs
import argparse
import os
import sys
import time

# OpenCV
import cv2
import imutils


def main(args):
    """Main."""
    start_time = time.time()

    print '{:.0f}ms: Using classifiers:'.format((time.time() - start_time) * 1000)
    cascades = {}
    for file_name in os.listdir('./classifiers'):
        print '\t{}'.format(file_name)
        cascades[file_name[:-4]] = cv2.CascadeClassifier(os.path.join('./classifiers', file_name))

    for file_name in os.listdir('./images'):
        print '{:.0f}ms: Finding targets in {}'.format((time.time() - start_time) * 1000, file_name)
        image = cv2.imread(os.path.join('./images', file_name))
        image = imutils.resize(image, width=args.resize_width)

        # Create a copy here so that the image exist and can be written even when no targets are found.
        image_highlighted = image.copy()

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

    print '{:.0f}ms: Finished'.format((time.time() - start_time) * 1000)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()

    parser.add_argument('-r', '--resize-width', type=int, default=500)

    args = parser.parse_args(sys.argv[1:])
    main(args)
