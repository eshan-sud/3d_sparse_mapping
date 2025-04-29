
# Performs Camera Calibration

import cv2
import numpy as np
import glob
import os

# Configuration
CHESSBOARD_SIZE = (10, 7)  # Chessboard inner corners
FRAME_CAPTURE_INTERVAL = 2  # Capture an image every 2 seconds
NUM_IMAGES = 20  # Number of images to collect for calibration
SAVE_DIR = "../calibration/calibration_images"
CALIBRATION_FILE = "../calibration/camera_calibration.npz"

# Prepare object points (0,0,0), (1,0,0), ..., (6,5,0)
objp = np.zeros((CHESSBOARD_SIZE[0] * CHESSBOARD_SIZE[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:CHESSBOARD_SIZE[0], 0:CHESSBOARD_SIZE[1]].T.reshape(-1, 2)

# Arrays to store object points and image points
objpoints = []
imgpoints = []

# Ensure the save directory exists
if not os.path.exists(SAVE_DIR):
    os.makedirs(SAVE_DIR)

# Initialize webcam
cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("Error: Could not open webcam")
    exit()

print("Starting camera calibration process...")

captured_images = 0
while captured_images < NUM_IMAGES:
    ret, frame = cap.read()
    if not ret:
        print("Failed to grab frame")
        continue

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Find the chessboard corners
    ret, corners = cv2.findChessboardCorners(gray, CHESSBOARD_SIZE, None)

    if ret:
        objpoints.append(objp)
        refined_corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1),
                                           (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))
        imgpoints.append(refined_corners)

        # Draw and display the corners
        cv2.drawChessboardCorners(frame, CHESSBOARD_SIZE, refined_corners, ret)
        img_name = os.path.join(SAVE_DIR, f"calib_{captured_images}.jpg")
        cv2.imwrite(img_name, frame)
        captured_images += 1
        print(f"Captured image {captured_images}/{NUM_IMAGES}")

    cv2.imshow("Calibration", frame)
    cv2.waitKey(FRAME_CAPTURE_INTERVAL * 1000)

cap.release()
cv2.destroyAllWindows()

# Camera Calibration
print("\nPerforming camera calibration...")
ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

# Compute reprojection error
mean_error = 0
for i in range(len(objpoints)):
    imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], camera_matrix, dist_coeffs)
    error = cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2) / len(imgpoints2)
    mean_error += error
total_error = mean_error / len(objpoints)

# Save calibration data
np.savez(CALIBRATION_FILE, camera_matrix=camera_matrix, dist_coeffs=dist_coeffs)
print(f"\nCalibration completed!")
print(f"Camera Matrix:\n{camera_matrix}")
print(f"Distortion Coefficients:\n{dist_coeffs}")
print(f"Reprojection Error: {total_error:.6f}")
print(f"\nCalibration data saved to {CALIBRATION_FILE}")
