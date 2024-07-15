import numpy as np
import cv2
import glob
import yaml

def calibrate_camera():
    # Chessboard dimensions
    CHECKERBOARD = (6,9)  # Change this to match your chessboard
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    # Prepare object points
    objp = np.zeros((1, CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
    objp[0,:,:2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)

    # Arrays to store object points and image points
    objpoints = []  # 3d point in real world space
    imgpoints = []  # 2d points in image plane.

    # Get list of calibration images
    images = glob.glob('~/aruco_pose_prediction/aruco_pose_prediction/calibration_images/*.jpg')

    if not images:
        print("No calibration images found in 'calibration_images' directory. Please ensure images are present and in .jpg format.")
        return

    image_size = None

    for fname in images:
        img = cv2.imread(fname)
        if img is None:
            print(f"Failed to read image: {fname}")
            continue

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        if image_size is None:
            image_size = gray.shape[::-1]
        elif gray.shape[::-1] != image_size:
            print(f"Image size mismatch. Expected {image_size}, got {gray.shape[::-1]} for {fname}")
            continue

        # Find the chess board corners
        ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, None)

        # If found, add object points, image points
        if ret == True:
            objpoints.append(objp)
            corners2 = cv2.cornerSubPix(gray, corners, (11,11), (-1,-1), criteria)
            imgpoints.append(corners2)

            # Draw and display the corners
            cv2.drawChessboardCorners(img, CHECKERBOARD, corners2, ret)
            cv2.imshow('img', img)
            cv2.waitKey(500)

    cv2.destroyAllWindows()

    if not objpoints:
        print("No valid chessboard patterns found in the images. Please check your images and chessboard dimensions.")
        return

    print(f"Calibrating camera using {len(objpoints)} images...")

    # Calibration
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, image_size, None, None)

    if not ret:
        print("Camera calibration failed. Please try again with different images.")
        return

    # Save calibration results
    calibration_data = {
        'camera_matrix': mtx.tolist(),
        'dist_coeffs': dist.tolist()
    }

    with open('camera_calibration.yaml', 'w') as file:
        yaml.dump(calibration_data, file)

    print("Calibration complete. Results saved to camera_calibration.yaml")

    # Print calibration results
    print("\nCalibration Results:")
    print("Camera Matrix:")
    print(mtx)
    print("\nDistortion Coefficients:")
    print(dist)

if __name__ == "__main__":
    calibrate_camera()