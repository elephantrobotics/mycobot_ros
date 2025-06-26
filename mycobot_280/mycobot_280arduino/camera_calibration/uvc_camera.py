import cv2
import numpy as np
import time
import typing


class UVCCamera:
    def __init__(
        self,
        cam_index=0,
        mtx=None,
        dist=None,
        capture_size: typing.Tuple[int, int] = (640, 480),
    ):
        super().__init__()
        self.cam_index = cam_index
        self.mtx = mtx
        self.dist = dist
        self.curr_color_frame: typing.Union[np.ndarray, None] = None
        self.capture_size = capture_size

    def capture(self):
        self.cap = cv2.VideoCapture(self.cam_index) #windows
        width, height = self.capture_size
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

    def update_frame(self) -> bool:
        ret, self.curr_color_frame = self.cap.read()
        return ret

    def color_frame(self) -> typing.Union[np.ndarray, None]:
        return self.curr_color_frame

    def release(self):
        self.cap.release()


if __name__ == "__main__":
    cam = UVCCamera(0)
    cam.capture()
    while True:
        if not cam.update_frame():
            continue

        frame = cam.color_frame()
        if frame is None:
            time.sleep(0.01)
            continue

        print(frame.shape)
        window_name = "preview"
        cv2.imshow(window_name, frame)
        if cv2.waitKey(1) == ord("q"):
            break
