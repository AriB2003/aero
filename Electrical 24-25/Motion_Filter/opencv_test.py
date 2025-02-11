"""
Testing out how OpenCV works
"""

import cv2 as cv
import sys

print(cv.__version__)

img = cv.imread("testblurred.jpg")

if img is None:
    sys.exit("Could not read the image.")

# cv.rectangle(img, (384, 0), (510, 128), (0, 255, 0), 3)
img = cv.fastNlMeansDenoisingColored(img, None, 20, 20, 7, 15)
# roi: cv.rectangle = cv.rectangle(0, 0, img.cols & -2, img.rows & -2)

cv.imshow("Display window", img)
cv.waitKey(20000)
