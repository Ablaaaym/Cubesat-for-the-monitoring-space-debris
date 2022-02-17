import cv2
import numpy as np

cap = cv2.VideoCapture(1)
cap0 = cv2.VideoCapture(0)
cv2.namedWindow("frame")
while True:
    ret, frame = cap.read()
    ret0, frame0 = cap0.read()


    cv2.imshow("frame", frame)
    cv2.imshow("frame0", frame0)
    k = cv2.waitKey(1) & 0xFF
    if k == 27:
        break

cap.release()
cv2.destroyAllWindows()