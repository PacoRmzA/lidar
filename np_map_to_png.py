import cv2
import numpy as np

arr = np.load('map.np.npy')
cv2.imwrite('map.png', arr)