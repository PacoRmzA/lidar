import cv2
import numpy as np
import glob, os

os.chdir("/home/thecubicjedi/lidar/map_captures")
for file in glob.glob("*.npy"):
    arr = np.load(file)
    cv2.imwrite(f'{file}.png', arr)

