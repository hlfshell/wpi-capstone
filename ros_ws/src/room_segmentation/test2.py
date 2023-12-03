import cv2
import numpy as np
from cv2 import (
    cvtColor,
    GaussianBlur,
    Canny,
    imread,
    findContours,
    CHAIN_APPROX_NONE,
    CHAIN_APPROX_SIMPLE,
    contourArea,
    arcLength,
    approxPolyDP,
)
from random import randint

img = imread("./willowgarage.jpeg", cv2.IMREAD_GRAYSCALE)
# img = imread("./fp1.jpeg", cv2.IMREAD_GRAYSCALE)

print(img.shape)

# cv2.imshow("Original", img)
# cv2.waitKey()

# Grayscale the img
# grayImg = cvtColor(img, cv2.COLOR_BGR2GRAY)
grayImg = img

cv2.imshow("Grayscale", grayImg)
cv2.waitKey()

# blurredImg = GaussianBlur(grayImg, (5, 5), 0)
blurredImg = GaussianBlur(grayImg, (35, 35), 0)
# blurredImg = GaussianBlur(grayImg, (55, 55), 0)

cv2.imshow("Blurred", blurredImg)
cv2.waitKey()

# Otsu thresholding
_, threshImg = cv2.threshold(
    blurredImg, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU
)

cv2.imshow("Thresholded", threshImg)
cv2.waitKey()


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

cv2.imshow("mask", mask)
cv2.waitKey()

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

raise "stop"

# Now we go through and remove canny "rooms" that are too small
img = np.zeros_like(canny)
contours, _ = cv2.findContours(canny, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
room_threshold = 5
for contour in contours:
    area = cv2.contourArea(contour)
    if area > room_threshold:
        cv2.fillPoly(img, [contour], 255)

cv2.imshow("Rooms", img)
cv2.waitKey()


raise "stop"
cannyImage = Canny(threshImg, 95, 90)

cv2.imshow("Canny", cannyImage)
cv2.waitKey()

raise "stop"
contours, hierarchy = findContours(
    image=cannyImage, method=CHAIN_APPROX_NONE, mode=cv2.RETR_EXTERNAL
)

# for contour in contours:
#     area = cv2.contourArea(contour)
#     # gives area of contour
#     perimeter = cv2.arcLength(contour, closed=True)
#     # gives perimeter of contour

#     borders = cv2.approxPolyDP(curve=contour, epsilon=0.05 * perimeter, closed=True)
#     # approximate boundaries for given polygon contour

# Create a new blank image the same size as our target image
drawn = np.zeros_like(grayImg)
# Set the background as white
drawn.fill(255)
drawn = cv2.cvtColor(drawn, cv2.COLOR_GRAY2BGR)
# From the given contour find hough lines
# lines = cv2.HoughLinesP(
#     image=cannyImage, rho=1, theta=np.pi / 180, threshold=100, minLineLength=0
# )

for contour in contours:
    print(contour)
    cv2.drawContours(drawn, contour, -1, color=(0, 0, 0), thickness=1)
    cv2.imshow("edges drawn", drawn)
    cv2.waitKey()

    # print(lines)
    # for line in lines:
    #     x1, y1, x2, y2 = line[0]
    #     cv2.line(drawn, (x1, y1), (x2, y2), color=(0, 0, 255), thickness=2)

    cv2.imshow("edges drawn", drawn)
    cv2.waitKey()

    # # Draw the lines on the image
    # for line in lines:
    #     x1, y1, x2, y2 = line[0]
    #     cv2.line(drawn, (x1, y1), (x2, y2), color=(0, 0, 255), thickness=2)

    # cv2.imshow("edges drawn", drawn)
    # cv2.waitKey()

    # area = cv2.contourArea(contour)
    # perimeter = cv2.arcLength(contour, closed=True)
    # borders = cv2.approxPolyDP(curve=contour, epsilon=0.05 * perimeter, closed=True)
    # cv2.drawContours(drawn, [borders], -1, color=(0, 0, 255), thickness=3)
    # cv2.imshow("edges drawn", drawn)
    # cv2.waitKey()

# for contour in contours:
#     cv2.drawContours(drawn, contour, -1, color=(0, 0, 0), thickness=3)

# cv2.imshow("edges drawn", drawn)
# cv2.waitKey()

# borders = []
# for contour in contours:
#     area = cv2.contourArea(contour)
#     perimeter = cv2.arcLength(contour, closed=True)

#     approx = cv2.approxPolyDP(curve=contour, epsilon=0.05 * perimeter, closed=True)
#     borders.append(approx)

# drawn = np.zeros_like(grayImg)
# drawn.fill(255)
# drawn = cv2.cvtColor(drawn, cv2.COLOR_GRAY2BGR)

# for approx in borders:
#     cv2.drawContours(drawn, [approx], -1, color=(0, 0, 255), thickness=3)

# cv2.imshow("approx drawn", drawn)
# cv2.waitKey()
