import cv2 as cv

if __name__ == "__main__":
    cap_num = 0
    cap = cv.VideoCapture(cap_num)
    while cv.waitKey(1)<0:
        _, img = cap.read()
        cv.imshow("", img)
