import cv2
import numpy as np
from random import randint
from typing import Optional, Tuple, List, Union
from skimage.transform import resize
import netpbmfile


def room_segmentation(
    input: cv2.typing.MatLike,
    blur: int = 7,
    noise_removal_threshold: int = 25,
    mask_background: bool = True,
    threshold_min: float = 0.3,
    threshold_max=1.0,
    working_size: int = 500,
    min_room_area: int = -1,
) -> np.ndarray:
    """
    room_segmentation uses watershed segmentation and distance
    transforms to try to identify what constitutes a "room". While
    not perfect, and needing some adjustment for images, it
    Accepts an image (and some configuration options) and ultimately

    """

    # Create a clone of inpu to work with without affecting the
    # original
    img = input.copy()

    # First we convert the image to grayscal if it is not already,
    # and then resize to WORKING_SIZE tall while keeping aspect
    # ratio
    if len(img.shape) == 3:
        img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Resize to a constant size to work with. Note we'll have to resize
    # back to the caller, especially since we expect pixels to represent
    # real world dimensions.
    height, width = img.shape
    img = cv2.resize(
        img, (working_size, int(working_size * img.shape[0] / img.shape[1]))
    )
    cv2.imshow("Resized", img)
    cv2.waitKey()

    # Once we have the resized image, we blur it
    blurred = cv2.GaussianBlur(img, (blur, blur), 0)

    _, img_threshold = cv2.threshold(
        blurred, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU
    )
    cv2.imshow("Thresholded", img_threshold)
    cv2.waitKey()

    # Take the contours of the image to find the overarching room
    # outline. We use the mask to isolate the room from surrounding area.
    if mask_background:
        mask = np.zeros_like(img)
        contours, _ = cv2.findContours(
            img_threshold, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > noise_removal_threshold:
                cv2.fillPoly(mask, [contour], 255)

        # Finally, mask out the background
        cv2.imshow("Mask", mask)
        cv2.waitKey()
        img[mask == 0] = 0

    # Blur again
    # img = cv2.GaussianBlur(img, (3, 3), 0)
    # cv2.imshow("Blurred", img)
    # cv2.waitKey()
    # # raise "stop"

    # Calculate the Laplacian and sharpen the image
    kernel = np.array([[1, 1, 1], [1, -8, 1], [1, 1, 1]], dtype=np.float32)
    laplacian = cv2.filter2D(img, cv2.CV_32F, kernel)
    # We need to convert the image to float32 for this step
    img = np.float32(img) - laplacian

    # Convert back to 8bits gray scale for the next steps
    img = np.clip(img, 0, 255)
    img = img.astype("uint8")

    # Create a binary threshold of the image
    _, binary_threshold = cv2.threshold(
        img, 40, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU
    )
    cv2.imshow("Binary Threshold", binary_threshold)
    cv2.waitKey()

    # Perform our distance transform
    distance = cv2.distanceTransform(binary_threshold, cv2.DIST_L2, 3)

    # Normalize our distances for the range {0.0, 1.0}
    cv2.normalize(distance, distance, 0, 1.0, cv2.NORM_MINMAX)

    # Threshold the distance image for additional morpholgical operations
    _, distance_thresholded = cv2.threshold(
        distance, threshold_min, threshold_max, cv2.THRESH_BINARY
    )
    cv2.imshow("Distance Thresholded", distance_thresholded)
    cv2.waitKey()

    # Dilate our image to increase the marker size
    # kernel = np.ones((3, 3), dtype=np.uint8)
    kernel = np.ones((3, 3), dtype=np.uint8)
    dilated = cv2.dilate(distance_thresholded, kernel)

    # Find markers via contours
    contours, _ = cv2.findContours(
        dilated.astype("uint8"), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )

    # Create our marker image for the watershed algorithm
    markers = np.zeros_like(dilated, dtype=np.int32)

    for index, contour in enumerate(contours):
        cv2.drawContours(markers, contours, index, color=(index + 1), thickness=-1)

    # Draw our background marker to separate background
    cv2.circle(markers, (5, 5), 3, (255, 255, 255), -1)

    # Finally, perfrom our watershed algorithm
    cv2.watershed(cv2.cvtColor(img, cv2.COLOR_GRAY2BGR), markers)

    # Now that we have our marker image, we need to resize back
    # to our original height and width with the sklearn resize
    # since we want to resize in a segmentation label-friendly
    # way.
    markers = resize(
        markers, (height, width), order=0, preserve_range=True, anti_aliasing=False
    )

    # If we have a minimum room area, we'll threshold the markers
    # to eliminate small rooms based on the area provided room
    # and set it to the most common surrounding room. If the option
    # is set to -1 then we skip this step.
    if min_room_area > 0:
        markers = threshold_zones(markers, min_area=min_room_area)

    return markers


def colorize_zones(
    markers: np.ndarray, colors: Optional[List[Tuple[int]]] = None, labels: bool = False
) -> cv2.typing.MatLike:
    """
    colorize_zones - Given a set of markers generated by
    room_segmentation, create an image of the zones colorized.
    Colors can be provided via colors, but are assumed to be
    of the same length as indexes in the markers array. If
    labels are set to True, then a small number representing
    the zone index will be placed in the center of the zone.
    """
    img = np.zeros_like(markers, dtype=np.uint8)
    img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)

    marker_indexes = np.unique(markers)

    # Eliminate -1 and 255 values
    marker_indexes = marker_indexes[marker_indexes > 0]
    marker_indexes = marker_indexes[marker_indexes < 255]

    # If no colors are provided, generate random colors for the marker
    # zones per our marker indexes
    if colors is None:
        # colors = []
        # for index in range(len(marker_indexes)):
        #     colors.append((randint(0, 255), randint(0, 255), randint(0, 255)))
        colors = {}
        for index in marker_indexes:
            colors[index] = (randint(0, 255), randint(0, 255), randint(0, 255))

    # for y, row in enumerate(markers):
    #     for x, _ in enumerate(row):
    #         index = markers[y, x]
    #         if index > 0 and index < 255:
    #             img[y, x] = colors[index]

    for index in marker_indexes:
        img[markers == index] = colors[index]

    if labels:
        # In the center of each marker polygon we will place a small
        # representing its index
        for index in marker_indexes:
            # Find the center of the polygon
            y, x = np.where(markers == index)
            y = int(np.mean(y))
            x = int(np.mean(x))
            cv2.putText(
                img,
                str(index + 1),
                (x, y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 0, 0),
                1,
                cv2.LINE_AA,
            )

    return img


def threshold_zones(markers: np.ndarray, min_area: float = 1500.0) -> np.ndarray:
    """
    threshold_zones accepts a 2d array of markers, which are segmented
    labeled zones, and a minimum threshold. Any marker whose area is
    below the specified marker will be set to the most common surrounding
    marker's value (ignoring 0, -1, and 255 as they have separate meanings
    in the segmentation code)
    """
    # Create a copy of the markers to work with
    markers = markers.copy()

    # Find the unique markers
    marker_indexes = np.unique(markers)

    # Eliminate -1 and 255 values
    marker_indexes = marker_indexes[marker_indexes > 0]
    marker_indexes = marker_indexes[marker_indexes < 255]

    # For each zone, we'll calculate the area and if it is less than
    # the minimum threshold, we'll set it to the surrounding zone's
    # value
    for index in marker_indexes:
        # First, determine the percentage of the marker index is of the
        # entire map
        truth_map = np.where(markers == index, 1, 0)

        # Find the upper left and bottom right of where the values
        # are 1
        y, x = np.where(truth_map == 1)
        y_min = np.min(y)
        y_max = np.max(y)
        x_min = np.min(x)
        x_max = np.max(x)

        # Calculate the area
        area = (y_max - y_min) * (x_max - x_min)
        if area < min_area:
            # Find within the bounding box the most used index that isn't
            # -1, 0, 255, or the matching index. But first, let's grow the
            # bounding box by a set percentage first to encapsulate a
            # better idea of what's around it.
            percentage = 0.1
            y_min = max(int(y_min - (y_max - y_min) * percentage), 0)
            y_max = min(int(y_max + (y_max - y_min) * percentage), markers.shape[0])
            x_min = max(int(x_min - (x_max - x_min) * percentage), 0)
            x_max = min(int(x_max + (x_max - x_min) * percentage), markers.shape[1])

            cutout = markers[y_min:y_max, x_min:x_max]
            cutout = np.where(cutout == index, 0, cutout)
            cutout = np.where(cutout <= 0, 0, cutout)
            cutout = np.where(cutout >= 255, 0, cutout)

            # Find the unique markers and their counts
            unique, counts = np.unique(cutout, return_counts=True)
            # Remove 0, since we set the most common index to that
            unique = unique[unique > 0]

            # If we are in a weird case where we don't see anything,
            # abort for this index
            if len(unique) <= 0:
                continue
            value = unique[np.argmax(counts)]

            # Set all values in the markers to the resulting value
            markers = np.where(markers == index, value, markers)

    return markers


def identify_unknown_and_known(
    img: np.ndarray,
    unknown: Union[int, Tuple[int, int, int]],
    empty: Union[int, Tuple[int, int, int]],
):
    """
    identify_unknown_and_known accepts an image and attempts to
    segment the image into known, unknown, and other (basically
    walls)
    """

    base = np.ones_like(img)

    base = np.where(img == unknown, -1, base)
    base = np.where(img == empty, 0, base)

    return base


# img = cv2.imread("./willowgarage.jpeg", cv2.IMREAD_GRAYSCALE)
# img = netpbmfile.imread("./house.pgm")
# img = netpbmfile.imread("./tb_new.pgm")
img = cv2.imread("./house.jpeg", cv2.IMREAD_GRAYSCALE)

# img = np.where(img >= 240, 255, img)
# img = np.where((img < 240) & (img > 0), 205, img)

# filter = identify_unknown_and_known(img, 255, 0)

# print(filter, np.unique(filter, return_counts=True), np.unique(img, return_counts=True))

markers = room_segmentation(img)
# final = colorize_zones(markers, labels=True)
thresholded_markers = threshold_zones(markers)
final = colorize_zones(markers, labels=True)
thresholded = colorize_zones(thresholded_markers, labels=True)

# cv2.imshow("Original", img)
cv2.imshow("Final", final)
# cv2.imshow("Markers", markers.astype("uint8"))
# cv2.imshow("Thresholded Markers", thresholded_markers.astype("uint8"))
cv2.imshow("Thresholded", thresholded)
cv2.waitKey()
print(np.unique(thresholded_markers, return_counts=True))

np.save("house.segmentation", thresholded_markers, allow_pickle=False)
