import cv2
import numpy as np
import time

# This image will be updated with the feed from the camera
image = cv2.imread('fullscreen.jpg')

images = ["centered.jpg", "offcenter.jpg", "fullscreen.jpg", "noisy.jpg"]

i = 0
while(True):
    # get frame from camera
    imagePath = images[i]
    image = cv2.imread(imagePath)

    # process image
    # This extracts the orange tape from the robot
    lower_bound = np.array([33, 47, 153])  # Lower bound for the orange (BGR)
    upper_bound = np.array([111, 153, 253]) # Upper bound for the orange (BGR)
    orange_mask = cv2.inRange(image, lower_bound, upper_bound)

    # This extracts the gray color of the robot
    lower_bound = np.array([31, 24, 21])
    upper_bound = np.array([95, 100, 109])
    gray_mask = cv2.inRange(image, lower_bound, upper_bound)

    # Combine the masks using bitwise OR
    combined_mask = cv2.bitwise_or(orange_mask, gray_mask)

    contours, _ = cv2.findContours(combined_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for contour in contours:
        x, y, w, h = cv2.boundingRect(contour)
        # Skip the tiny bounding boxes that are just noise
        if w < 25 or h < 25:
            continue
        cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)
        print(f"Bounding box coordinates: x={x}, y={y}, width={w}, height={h}")

    # publish bounding box of opponent
    smaller_image = cv2.resize(image, (1200, 800))
    cv2.imshow('Image with Bounding Boxes', smaller_image)
    
    # Press 0 to continue to the next frame
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    # This code is temporary. Its only until we get a live feed from the camera
    i += 1
    if (i == len(images)):
        i = 0

# result = cv2.bitwise_and(image, image, mask=combined_mask)
# # cv2.imshow('Original Image', image)
# # cv2.imshow('Mask', combined_mask)
# # cv2.imshow('Detected Color', result)

# cv2.rectangle(result, (100, 100), (200, 200), (255,255,255), 5)
# cv2.imshow("Drawing Shapes", result)

cv2.waitKey(0)
cv2.destroyAllWindows()