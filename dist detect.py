
import cv2
import numpy as np

def nothing(x): #пустая функция
    pass


def Distance_finder(Focal_Length, real_face_width, face_width_in_frame):
    distance = (real_face_width * Focal_Length) / face_width_in_frame
    # return the distance
    return distance

kernel =  np.ones((10,10), np.uint8)
# создаем ядро

cap = cv2.VideoCapture(0)
cv2.namedWindow("track", cv2.WINDOW_NORMAL)

cv2.createTrackbar('HL', "track", 0, 180, nothing)
cv2.createTrackbar('SL', "track", 0, 255, nothing)
cv2.createTrackbar('VL', "track", 0, 255, nothing)

cv2.createTrackbar('H', "track", 0, 180, nothing)
cv2.createTrackbar('S', "track", 0, 255, nothing)
cv2.createTrackbar('V', "track", 0, 255, nothing)

while True:


    ret, frame = cap.read()
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    # для конвертациипишем изначальное изображение и пишем как конвертировать надо

    hl = cv2.getTrackbarPos('HL', "track")
    # получаем значение с трекбара
    sl = cv2.getTrackbarPos('SL', "track")
    vl = cv2.getTrackbarPos('VL', "track")

    h = cv2.getTrackbarPos('H', "track")
    # получаем значение с трекбара
    s = cv2.getTrackbarPos('S', "track")
    v = cv2.getTrackbarPos('V', "track")

    lower = np.array([hl, sl, vl]) #нижний диапозон
    upper = np.array([h, s, v]) #верхний диапозон

    frame = cv2.bilateralFilter(frame, 9,75,75)
#для сглаживания используем билатеральный фильтер

    mask = cv2.inRange(hsv, lower, upper)
#позволяет цвет в диапозоне сделать белым а остальное черным
    res=cv2.bitwise_and(frame, frame, mask=mask)
    #позволяет белое сделать цветным обратно

    opening = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    closing = cv2.morphologyEx(opening, cv2.MORPH_CLOSE, kernel)
#используем морфологические операции для фильтрации изображения

    contours, h=cv2.findContours(closing, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    #находим контуры
    contours = sorted(contours, key=cv2.contourArea, reverse = True)
    #сортируем контуры указывая их, метод и реверс тру значит по убыванию

    for x in range(len(contours)):
        area = cv2.contourArea(contours[x])
        if area > 300:
            x, y, w, h = cv2.boundingRect(contours[x])
            frame = cv2.rectangle(frame, (x,y), (x+w, y+h), (0,255,0), 2)
            frame - cv2.rectangle(frame, (x,y), (x+60, y-25), (0,0,0), -1)
            #cv2.putText(frame, "o", (x,y), cv2.FONT_HERSHEY_SCRIPT_SIMPLEX, 1.0, (255,255,255), 2)
            if (w) != 0:
                distance = Distance_finder(140, 50, w)
                cv2.putText(frame, str(distance), (x + 10, y + 10), cv2.FONT_HERSHEY_SCRIPT_SIMPLEX, 1.0, (255, 255, 255), 1)

    cv2.imshow("cl", closing)
    cv2.imshow("frame", frame)
    cv2.imshow("hsv", hsv)
    cv2.imshow("mask", mask)
    cv2.imshow("res", res)
   # cv2.imshow("rotate", rotate)

    k = cv2.waitKey(1) & 0xFF
    if k == 27:  # если нажали ескейп вырубаем
        break
cap.release()
cv2.destroyAllWindows() # закрыть все
