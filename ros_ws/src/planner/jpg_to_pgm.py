import cv2
import netpbmfile
import numpy as np

file = "./maps/house.jpg"
out = file.split(".")[0] + ".pgm"

img = cv2.imread(file, cv2.IMREAD_GRAYSCALE)

mod = np.where(img < 205, 0, img)
mod = np.where(img > 205, 254, mod)

netpbmfile.imwrite(out, mod)
