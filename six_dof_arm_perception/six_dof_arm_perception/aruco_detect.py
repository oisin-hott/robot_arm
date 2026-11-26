import cv2
import numpy as np
from scipy.spatial.transform import Rotation

ARUCO_DICT = cv2.aruco.DICT_5X5_100
MARKER_LENGTH = 0.05        # marker side length in metres
AXIS_LENGTH = 0.03          # length of the drawn axes in metres
FX_PER_PX = 0.72            # Pixel 10a estimate: fx ≈ 0.72 × the frame's long side
DIST_COEFFS = np.zeros(5)

# Known marker centre positions in the world frame (metres)
ARUCO_WORLD_POSITIONS = {
    8:  [0.00, 0.00, 0.0],
    29: [-0.10, 0.00, 0.0],
    36: [0.00, 0.15, 0.0],
    24: [-0.10, 0.15, 0.0],
}


def make_camera_matrix(image):
    h, w = image.shape[:2]
    f = FX_PER_PX * max(w, h)
    return np.array([[f, 0.0, w / 2],
                     [0.0, f, h / 2],
                     [0.0, 0.0, 1.0]], dtype=np.float64)


def setup_detector():
    aruco_dict = cv2.aruco.Dictionary_get(ARUCO_DICT)
    aruco_params = cv2.aruco.DetectorParameters_create()
    return aruco_dict, aruco_params


def detect_markers(image, aruco_dict, aruco_params):
    corners, ids, _ = cv2.aruco.detectMarkers(image, aruco_dict, parameters=aruco_params)
    return corners, ids


def draw_marker(image, marker_corner, marker_id):
    pts = marker_corner.reshape((4, 2)).astype(int)
    top_left, top_right, bottom_right, bottom_left = [tuple(p) for p in pts]

    cv2.line(image, top_left, top_right, (0, 255, 0), 2)
    cv2.line(image, top_right, bottom_right, (0, 255, 0), 2)
    cv2.line(image, bottom_right, bottom_left, (0, 255, 0), 2)
    cv2.line(image, bottom_left, top_left, (0, 255, 0), 2)

    c_x = int((top_left[0] + bottom_right[0]) / 2.0)
    c_y = int((top_left[1] + bottom_right[1]) / 2.0)
    cv2.circle(image, (c_x, c_y), 4, (0, 0, 255), -1)

    cv2.putText(image, str(marker_id), (top_left[0], top_left[1] - 15),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)


def estimate_poses(corners, camera_matrix):
    rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
        corners, MARKER_LENGTH, camera_matrix, DIST_COEFFS)
    return rvecs, tvecs


def draw_axes(image, camera_matrix, rvec, tvec):
    cv2.drawFrameAxes(image, camera_matrix, DIST_COEFFS, rvec, tvec, AXIS_LENGTH)


def rvec_to_transformation_matrix(rvec, tvec):
    """Marker pose in the camera frame: T_CA."""
    R, _ = cv2.Rodrigues(rvec)
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = tvec.flatten()
    return T


def marker_world_transform(marker_id):
    """Known marker pose in the world frame: T_WA (assumes flat and aligned with the world)."""
    T = np.eye(4)
    T[:3, 3] = ARUCO_WORLD_POSITIONS[marker_id]
    return T


def invert_transform(T):
    R = T[:3, :3]
    t = T[:3, 3]
    T_inv = np.eye(4)
    T_inv[:3, :3] = R.T
    T_inv[:3, 3] = -R.T @ t
    return T_inv

def rotation_to_quaternion(R):
    """3×3 rotation matrix → [x, y, z, w]."""
    return Rotation.from_matrix(R).as_quat()


def camera_world_transform(rvec, tvec, marker_id):
    """Camera pose in the world frame: T_WC = T_WA · T_CA⁻¹."""
    T_CA = rvec_to_transformation_matrix(rvec, tvec)
    T_WA = marker_world_transform(marker_id)
    return T_WA @ invert_transform(T_CA)


def average_camera_pose(camera_estimates):
    """Average a list of T_WC estimates into one T_WC."""
    positions = np.array([T[:3, 3] for T in camera_estimates])
    rotations = Rotation.from_matrix([T[:3, :3] for T in camera_estimates])

    T_WC = np.eye(4)
    T_WC[:3, :3] = rotations.mean().as_matrix()
    T_WC[:3, 3] = positions.mean(axis=0)
    return T_WC


def detect_frame(frame, camera_matrix, aruco_dict, aruco_params):
    """Detects reference markers, draws them on the frame, and returns
    (T_WC, detected_ids). T_WC is the averaged camera pose, or None if no reference markers are visible."""
    camera_estimates = []
    detected_ids = []

    corners, ids = detect_markers(frame, aruco_dict, aruco_params)
    if ids is None:
        return None, detected_ids

    rvecs, tvecs = estimate_poses(corners, camera_matrix)
    for marker_corner, marker_id, rvec, tvec in zip(corners, ids.flatten(), rvecs, tvecs):
        draw_marker(frame, marker_corner, marker_id)
        draw_axes(frame, camera_matrix, rvec, tvec)

        if marker_id in ARUCO_WORLD_POSITIONS:
            detected_ids.append(int(marker_id))
            camera_estimates.append(camera_world_transform(rvec, tvec, marker_id))

    if not camera_estimates:
        return None, detected_ids
    return average_camera_pose(camera_estimates), detected_ids