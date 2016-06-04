import cv2


class ShapeDetector:
    """Class to detect shapes from openCV contours."""

    def __init__(self):
        """Nothing to initialise."""
        pass

    def detect(self, contour):
        """
        Detect the shape from the contour (shape outline).

        contour: openCV contour
        returns: a string containing the name of the shape
        """
        # Initialize the shape name and get the shape perimeter.
        shape = "unidentified"
        perimeter = cv2.arcLength(contour, True)

        # Approximate the contour to 'smooth' the shape. Perimeter of appoximation can be up to 4% different.
        approx = cv2.approxPolyDP(contour, 0.03 * perimeter, True)

        # len() will give the number of vertices of the shape.
        if len(approx) == 3:
            shape = "triangle"

        # Check if the sides are all equal for special case of square.
        elif len(approx) == 4:
            # Compute the bounding box of the contour and use the bounding box to compute the aspect ratio.
            (x, y, w, h) = cv2.boundingRect(approx)
            aspect_ratio = w / float(h)
            # A square will have an aspect ratio that is close to 1, otherwise, the shape is a rectangle.
            shape = "square" if aspect_ratio >= 0.95 and aspect_ratio <= 1.05 else "rectangle"

        elif len(approx) == 5:
            shape = "pentagon"

        # Otherwise assume the shape is a circle.
        else:
            shape = "circle"

        return shape
