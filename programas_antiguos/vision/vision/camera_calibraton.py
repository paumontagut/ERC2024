import cv2
import numpy as np

# Define the number of chessboard corners
num_corners_x = 8
num_corners_y = 6

# Create arrays to store object points and image points from all calibration images
obj_points = []  # 3D points in real world space
img_points = []  # 2D points in image plane

# Generate object points for the chessboard corners
objp = np.zeros((num_corners_x * num_corners_y, 3), np.float32)
objp[:, :2] = np.mgrid[0:num_corners_x, 0:num_corners_y].T.reshape(-1, 2)

cap = cv2.VideoCapture(0)
calibration_images = []


while len(calibration_images) < 10:
    ret, frame = cap.read()
    cv2.imshow("frame", frame)
    if cv2.waitKey(1):
        calibration_images.append(frame)
        print(f"Image {len(calibration_images)} saved")
    print(len(calibration_images))

cap.release()

for img in calibration_images:
    # Load the image
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Find chessboard corners
    ret, corners = cv2.findChessboardCorners(gray, (num_corners_x, num_corners_y), None)

    # If corners are found, add object points and image points
    if ret:
        obj_points.append(objp)
        img_points.append(corners)
    print(ret)

# Calibrate the camera
ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(obj_points, img_points, gray.shape[::-1], None, None)

# Print camera coefficients
print("Camera matrix:")
print(camera_matrix)
print("\nDistortion coefficients:")
print(dist_coeffs)