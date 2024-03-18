import cv2


cap  = cv2.VideoCapture(0)
cap.set(3,640)
cap.set(4,480)

while cv2.waitKey(1)<0:
    ret, frame = cap.read()
    if not ret:
        break
    cv2.imshow('', frame)
    
