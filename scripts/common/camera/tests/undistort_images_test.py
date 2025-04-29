import cv2
import numpy as np

# Load the saved calibration data
calib_data = np.load("camera_calibration.npz")
camera_matrix = calib_data["camera_matrix"]
dist_coeffs = calib_data["dist_coeffs"]

# Open webcam
cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("Error: Could not open webcam")
    exit()

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Undistort the frame
    undistorted_frame = cv2.undistort(frame, camera_matrix, dist_coeffs, None)

    # Show original and undistorted images side by side
    combined = np.hstack((frame, undistorted_frame))
    cv2.imshow("Original (Left) vs Undistorted (Right)", combined)

    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()
