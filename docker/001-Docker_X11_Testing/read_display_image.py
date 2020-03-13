import numpy as np
import cv2

img = cv2.imread('test_image.jpg')

cv2.imshow('My Image', img)
cv2.waitKey(0)
cv2.destroyAllWindows()
