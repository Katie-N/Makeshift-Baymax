import cv2
import numpy as np

image = cv2.imread('fullscreen.jpg')

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

result = cv2.bitwise_and(image, image, mask=combined_mask)
# cv2.imshow('Original Image', image)
# cv2.imshow('Mask', combined_mask)
# cv2.imshow('Detected Color', result)

cv2.rectangle(result, (100, 100), (200, 200), (255,255,255), 5)
cv2.imshow("Drawing Shapes", result)

cv2.waitKey(0)
cv2.destroyAllWindows()