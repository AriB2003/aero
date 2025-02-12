"""
Testing out how OpenCV works
"""

import cv2 as cv
import sys
import numpy as np

print(cv.__version__)

img = cv.imread("testblurred.jpg", cv.IMREAD_GRAYSCALE)
img = np.zeros((4000, 2666), dtype=np.single)

if img is None:
    sys.exit("Could not read the image.")

cv.rectangle(img, (384, 0), (510, 128), (0, 255, 0), 3)
center = (2000, 1333 // 2)
# center = (2000, 1333)
axes = (0, 780 // 2)
cv.ellipse(img, center, axes, 90 - 30, 0, 360, 255, -1)
# img = cv.fastNlMeansDenoisingColored(img, None, 20, 20, 7, 15)
# roi: cv.rectangle = cv.rectangle(0, 0, img.cols & -2, img.rows & -2)

cv.imshow("Display window", img)
cv.waitKey(20000)
