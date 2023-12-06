import math
from random import randint

import cv2
import numpy as np

img = cv2.imread("./willowgarage.jpeg", cv2.IMREAD_GRAYSCALE)
# img = cv2.imread("./fp1.jpeg", cv2.IMREAD_GRAYSCALE)
# img = cv2.imread("./fp2.jpeg", cv2.IMREAD_GRAYSCALE)
# img = cv2.imread("./fp3.jpeg", cv2.IMREAD_GRAYSCALE)
# Resize image to 500xX keeping aspect ratio
img = cv2.resize(img, (500, int(500 * img.shape[0] / img.shape[1])))
# We want the background to be black, but the rooms to be white so
# let's invert it and mask out the background
# background = ~img

# Now we create a mask to isolate the background before we invert.
blurred = cv2.GaussianBlur(img, (11, 11), 0)
_, img_threshold = cv2.threshold(
    blurred, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU
)
cv2.imshow("Thresholded", img_threshold)
cv2.waitKey()

mask = np.zeros_like(img)
noise_removal_threshold = 25
contours, _ = cv2.findContours(
    img_threshold, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
)
# contours, _ = cv2.findContours(threshImg, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
for contour in contours:
    area = cv2.contourArea(contour)
    if area > noise_removal_threshold:
        cv2.fillPoly(mask, [contour], 255)

# cv2.imshow("mask", mask)
# cv2.waitKey()

# We now have a mask isolating the rooms, we want to mask the background
# so we want the opposite of the mask
# mask = ~mask
# We now want to take the initial image and set the background to black
# which is wherever the mask is
img[mask == 0] = 0

cv2.imshow("Rooms", img)
cv2.waitKey()

# Sharpen the image
kernel = np.array([[1, 1, 1], [1, -8, 1], [1, 1, 1]], dtype=np.float32)

laplacian = cv2.filter2D(img, cv2.CV_32F, kernel)
sharp = np.float32(img)
img = sharp - laplacian

# Convert back to 8bits gray scale
img = np.clip(img, 0, 255)
img = img.astype("uint8")
laplacian = np.clip(laplacian, 0, 255)
laplacian = np.uint8(laplacian)

# cv2.imshow("Sharpened", img)
# cv2.waitKey()

# Create a binary image from the source image
# binary = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
binary = img
_, binary_threshold = cv2.threshold(
    binary, 40, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU
)

cv2.imshow("Binary", binary_threshold)
cv2.waitKey()

# Perform the distance transform
distance = cv2.distanceTransform(binary_threshold, cv2.DIST_L2, 3)

# Normalize the distance and image for range = {0.0, 1.0}
cv2.normalize(distance, distance, 0, 1.0, cv2.NORM_MINMAX)

cv2.imshow("Distance", distance)
cv2.waitKey()

# Threshold the distance image for additional morpholgical operations
_, distance_threshold = cv2.threshold(distance, 0.2, 1.0, cv2.THRESH_BINARY)

# cv2.imshow("Distance Threshold", distance_threshold)
# cv2.waitKey()

# Dilate
kernel = np.ones((3, 3), dtype=np.uint8)
distance_dilated = cv2.dilate(distance_threshold, kernel)

# cv2.imshow("Dilated", distance_dilated)
# cv2.waitKey()

# We need to convert distance_dilated img to uint8 for future operations
distance_dilated = distance_dilated.astype("uint8")

# Find total markers via contours
contours, _ = cv2.findContours(
    distance_dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
)

# Marker image for watershed algorithm
markers = np.zeros_like(distance_dilated, dtype=np.int32)

# Draw foreground markers
# for i in range(len(contours)):
#     cv2.drawContours(markers, contours, i, (i + 1), -1)
# for contour in contours:
#     # print("contour", contour)
#     cv2.drawContours(markers, [contour], -1, color=255, thickness=-1)

for index, contour in enumerate(contours):
    cv2.drawContours(markers, contours, index, color=(index + 1), thickness=-1)

# cv2.imshow("Markers", markers.astype("uint8"))
# cv2.waitKey()

# Is this needed?
cv2.circle(markers, (5, 5), 3, (255, 255, 255), -1)

# cv2.imshow("markers post circle", markers.astype("uint8"))
# cv2.waitKey()


# result = img.astype("uint32")
# print out the img and markers type
print("img", img.dtype)
print("markers", markers.dtype)
# cv2.imshow("base image again", img)
# cv2.waitKey()

water = cv2.watershed(cv2.cvtColor(img, cv2.COLOR_GRAY2BGR), markers)

# cv2.imshow("markers post water", markers.astype("uint8"))
# cv2.waitKey()

print("contours - ", len(contours))
# Count how many unique values are in the np array markers
print("markers - ", np.unique(markers))


mark = markers.astype("uint8")
mark = cv2.bitwise_not(mark)
cv2.imshow("Markers_v2", mark)
cv2.waitKey()

cv2.imshow("Watershed", water.astype("uint8"))
cv2.waitKey()


final = np.zeros_like(img, dtype=np.uint8)
final = cv2.cvtColor(final, cv2.COLOR_GRAY2RGB)
colors = []
for _ in contours:
    colors.append((randint(0, 255), randint(0, 255), randint(0, 255)))

for y, row in enumerate(markers):
    for x, col in enumerate(row):
        index = markers[y, x]
        if index > 0 and index <= len(contours):
            final[y, x] = colors[index - 1]

cv2.imshow("Final", final)
cv2.waitKey()
