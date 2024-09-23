import cv2
import numpy as np
import typing as T
from numpy.typing import NDArray, ArrayLike


class MarkerInfo(T.TypedDict):
    corners: np.ndarray
    tvec: np.ndarray
    rvec: np.ndarray
    num_id: int


def solve_marker_pnp(corners: NDArray, marker_size: int, mtx: NDArray, dist: NDArray):
    """
    This will estimate the rvec and tvec for each of the marker corners detected by:
       corners, ids, rejectedImgPoints = detector.detectMarkers(image)
    corners - is an array of detected corners for each detected marker in the image
    marker_size - is the size of the detected markers
    mtx - is the camera matrix
    distortion - is the camera distortion matrix
    RETURN list of rvecs, tvecs, and trash (so that it corresponds to the old estimatePoseSingleMarkers())
    """
    marker_points = np.array(
        [
            [-marker_size / 2, marker_size / 2, 0],
            [marker_size / 2, marker_size / 2, 0],
            [marker_size / 2, -marker_size / 2, 0],
            [-marker_size / 2, -marker_size / 2, 0],
        ],
        dtype=np.float32,
    )
    rvecs = []
    tvecs = []
    for corner in corners:
        retval, rvec, tvec = cv2.solvePnP(
            marker_points,
            corner,
            mtx,
            dist,
            flags=cv2.SOLVEPNP_IPPE_SQUARE,
        )
        if retval:
            rvecs.append(rvec)
            tvecs.append(tvec)

    rvecs = np.array(rvecs)  # type: ignore
    tvecs = np.array(tvecs)  # type: ignore
    (rvecs - tvecs).any()  # type: ignore
    return rvecs, tvecs


def draw_marker(frame: np.ndarray, corners, tvecs, rvecs, ids, mtx, dist) -> None:
    # cv2.aruco.drawDetectedMarkers(frame, corners, None, borderColor=(0, 255, 0))
    cv2.aruco.drawDetectedMarkers(frame, corners, ids, borderColor=(0, 200, 200))
    for i in range(len(ids)):
        corner, tvec, rvec, marker_id = corners[i], tvecs[i], rvecs[i], ids[i]
        cv2.drawFrameAxes(frame, mtx, dist, rvec, tvec, 60, 2)

