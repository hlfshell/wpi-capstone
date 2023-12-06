import math
from random import randint

import cv2
import numpy as np
from cv2 import (CHAIN_APPROX_NONE, CHAIN_APPROX_SIMPLE, Canny, GaussianBlur,
                 approxPolyDP, arcLength, contourArea, cvtColor, findContours,
                 imread)

# img = imread("./willowgarage.jpeg", cv2.IMREAD_GRAYSCALE)
img = imread("./willowgarage.jpeg", cv2.IMREAD_GRAYSCALE)

print(img.shape)

# cv2.imshow("Original", img)
# cv2.waitKey()

# Grayscale the img
# grayImg = cvtColor(img, cv2.COLOR_BGR2GRAY)
grayImg = img

cv2.imshow("Grayscale", grayImg)
cv2.waitKey()


edge_img = np.zeros_like(grayImg)
edge_img = cv2.cvtColor(edge_img, cv2.COLOR_GRAY2BGR)
edges = cv2.Canny(grayImg, 10, 250, apertureSize=3)

cv2.imshow("canny", edges)
cv2.waitKey()

lines = cv2.HoughLines(edges, 1, np.pi / 180, 200)
const = 1000
for line in lines:
    rho, theta = line[0]
    a = np.cos(theta)
    b = np.sin(theta)
    x0 = a * rho
    y0 = b * rho
    x1 = int(x0 + const * (-b))
    y1 = int(y0 + const * (a))
    x2 = int(x0 - const * (-b))
    y2 = int(y0 - const * (a))
    cv2.line(edge_img, (x1, y1), (x2, y2), (0, 0, 255), 2)

cv2.imshow("Edges", edge_img)
cv2.waitKey()


raise "stop"


# blurredImg = GaussianBlur(grayImg, (5, 5), 0)
# blurredImg = GaussianBlur(grayImg, (35, 35), 0)
# blurredImg = GaussianBlur(grayImg, (55, 55), 0)

# cv2.imshow("Blurred", blurredImg)
# cv2.waitKey()

# Contour detection
# canny_img = cv2.Canny(img, 95, 90)

# cv2.imshow("Canny", canny_img)
# cv2.waitKey()

img = np.zeros_like(grayImg)
img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
lines = cv2.HoughLines(grayImg, 1, np.pi / 180, 150, None, 0, 0)
print("LINES", len(lines), lines[0])

largest_line_threshold = 10
shortest_line_threshold = 0
filtered = 0


def polar_to_xy(line):
    rho = line[0][0]
    theta = line[0][1]
    a = math.cos(theta)
    b = math.sin(theta)
    x0 = a * rho
    y0 = b * rho
    pt1 = (int(x0 + 1000 * (-b)), int(y0 + 1000 * (a)))
    pt2 = (int(x0 - 1000 * (-b)), int(y0 - 1000 * (a)))

    return pt1, pt2


if lines is not None:
    for line in lines:
        # x1, y1, x2, y2 = line[0]
        pt1, pt2 = polar_to_xy(line)
        x1, y1 = pt1
        x2, y2 = pt2

        # Determine the length of the line and filter out
        # lines that are too small or too large
        length = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
        # if length > largest_line_threshold or length < shortest_line_threshold:
        if (
            0 < x1 < img.shape[1]
            and 0 < y1 < img.shape[0]
            and 0 < x2 < img.shape[1]
            and 0 < y2 < img.shape[0]
        ):
            filtered += 1
            continue

        cv2.line(img, (x1, y1), (x2, y2), color=(0, 0, 255), thickness=1)
        # rho = lines[i][0][0]
        # theta = lines[i][0][1]
        # a = math.cos(theta)
        # b = math.sin(theta)
        # x0 = a * rho
        # y0 = b * rho
        # pt1 = (int(x0 + 1000 * (-b)), int(y0 + 1000 * (a)))
        # pt2 = (int(x0 - 1000 * (-b)), int(y0 - 1000 * (a)))
        # color = (randint(0, 255), randint(0, 255), randint(0, 255))
        # cv2.line(img, pt1, pt2, color, 3, cv2.LINE_AA)

print(
    f"Filtered {filtered} out of {len(lines)} lines for {len(lines) - filtered} lines!"
)

cv2.imshow("Rooms", img)
cv2.waitKey()

raise "stop"


# Otsu thresholding
_, threshImg = cv2.threshold(
    blurredImg, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU
)

cv2.imshow("Thresholded", threshImg)
cv2.waitKey()

raise "stop"

# Now we create a mask to isolate the background before we invert.
mask = np.zeros_like(threshImg)
print("MASK", mask.shape)
noise_removal_threshold = 25
contours, _ = cv2.findContours(threshImg, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
# contours, _ = cv2.findContours(threshImg, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
for contour in contours:
    area = cv2.contourArea(contour)
    if area > noise_removal_threshold:
        cv2.fillPoly(mask, [contour], 255)

# cv2.imshow("mask", mask)
# cv2.waitKey()

# Now we invert the image so our rooms are values and our walls are negative space
# First we remove the mask from teh thresImg
isolated_rooms = cv2.bitwise_and(~threshImg, ~threshImg, mask=mask)
# cv2.imshow("Isolated", isolated_rooms)
# cv2.waitKey()

# Now we are going to erode the shapes

iterations = 5
eroded = isolated_rooms
# Perform an opening (vs closing)
kernel_sizes = [(3, 3), (5, 5), (7, 7), (9, 9), (11, 11)]
for kernel_size in kernel_sizes:
    # # Erode the image
    # eroded = cv2.erode(eroded, kernel_size, iterations=1)
    # # Dilate the image
    # eroded = cv2.dilate(eroded, kernel_size, iterations=1)
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, kernel_size)
    eroded = cv2.morphologyEx(eroded, cv2.MORPH_OPEN, kernel, iterations=2)

cv2.imshow("Dilated/Eroded", eroded)
cv2.waitKey()

# erosion_iterations = 3
# erosion_kernel = np.ones((3, 3), np.uint8)
# eroded = cv2.erode(isolated_rooms, erosion_kernel, iterations=erosion_iterations)
# # cv2.imshow("Eroded", eroded)
# # cv2.waitKey()

canny = Canny(eroded, 95, 90)

# cv2.imshow("Canny", canny)
# cv2.waitKey()

room_img = np.zeros_like(canny)
room_img = cv2.cvtColor(room_img, cv2.COLOR_GRAY2BGR)
contours, _ = cv2.findContours(canny, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
for contour in contours:
    # generate a random color for the room
    color = tuple(np.random.randint(0, 255, size=3, dtype=np.uint8))
    color = (randint(0, 255), randint(0, 255), randint(0, 255))
    cv2.fillPoly(room_img, [contour], color)

cv2.imshow("Rooms", room_img)
cv2.waitKey()

# raise "stop"

# Now we go through and remove canny "rooms" that are too small
img = np.zeros_like(room_img)
contours, _ = cv2.findContours(room_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
room_threshold = 5
for contour in contours:
    area = cv2.contourArea(contour)
    if area > room_threshold:
        cv2.fillPoly(img, [contour], 255)

cv2.imshow("Rooms", img)
cv2.waitKey()


raise "stop"
