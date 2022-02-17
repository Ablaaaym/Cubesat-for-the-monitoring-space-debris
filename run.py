from collections import deque
import numpy as np
import argparse
from imutils.video import VideoStream
import imutils
import cv2
#import RPi.GPIO as GPIO
import time
import socket
import pickle

# настройка сокета
soc = socket.socket()
host = socket.gethostname()
print(host)
port = 12345
soc.bind((host, port))
soc.listen(5)

# Инициализируем GPIO
motor = 21
#GPIO.setwarnings(False) #Отключение предупреждения
#GPIO.setmode(GPIO.BCM)
#GPIO.setup(motor, GPIO.OUT)
#GPIO.output(motor, GPIO.LOW)

braking_distances = 50 # Тормозной путь

# Определяем верхнею и нижнею границу цвета
colorLower = (24, 100, 100)
colorUpper = (44, 255, 255)
pts = deque(maxlen= 64)
h = 600 # Ширина кадра
otstup = 100 # Отступ от угла
y_cord = [100,600]
radius_obj = []

# инициализируем видеопоток
print("[INFO] waiting for camera ...")
#camera = VideoStream(1).start() # На Raspberry
camera = cv2.VideoCapture(2) # На PC
time.sleep(2.0)

#Функция опознования
def identification(y,radius):
    pass

    for i in range(len(y_cord)):
        if (y > (y_cord[i]-braking_distances)) and (y < (y_cord[i]+braking_distances)):
            for j in range(len(radius_obj)):
                if (radius > (radius_obj[j] - braking_distances)) and (radius < (radius_obj[j] + braking_distances)):
                    pass
        else:
            for j in range(len(radius_obj)):
                if (radius > (radius_obj[j] - braking_distances)) and (radius < (radius_obj[j] + braking_distances)):
                    pass
                else:
                    y_cord.append(y)
                    radius_obj.append(radius)
    print("[INFO] y_cord = ", y_cord)
    print("[INFO] radius_obj = ", radius_obj)
    senddata()


# Функция поворота
def runright():
    #GPIO.output(motor, GPIO.HIGH)
    time.sleep(1.0)
    print("[INFO] Run runright")
    #GPIO.output(motor, GPIO.LOW)

# Отправляем данные
def senddata():

    print("[INFO] отправка данных")
    data = pickle.dumps(y_cord)
    rasp, addr = soc.accept()
    print('adr：', addr)
    print(list(data))
    rasp.send(data)
    rasp.close()


while True:
    # Берем кадр с потока изменяем размер и преобразовываем в цветовое пространство HSV
    (grabbed, frame) = camera.read()
    frame = imutils.resize(frame, width = h)
    #frame = imutils.rotate(frame, angle = 180)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Создаем маску и удаляем мелкие детали
    mask = cv2.inRange(hsv, colorLower, colorUpper)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)

    # Находим контуры в маске
    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                            cv2.CHAIN_APPROX_SIMPLE)[-2]
    center = None

    # Если на кадре контуров больше 0
    if len(cnts) > 0:
        # Выбираем самый большой контур и вычисляем координаты и радиус
        c = max(cnts, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(c)
        M = cv2.moments(c)
        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

        # Если радиус больше ... то продолжаем
        if radius > 10:
            # Проверяем знакомый ли объект. Если нет то центруем на него камеру
            if x < ((float(h)/2) - braking_distances):
                runright()

            else:
                # Когда отцентровали камеру Рисуем круг и центр на кодре
                identification(y, radius)
                cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)
                cv2.circle(frame, center, 5, (0, 0, 255), -1)
                print("[INFO] to the center",int(x), int(y))

    # Выводим на экран
    cv2.imshow("Frame", frame)
    #senddata()

    # ESC - остановить выполнение
    key = cv2.waitKey(1) & 0xFF
    if key == 27:
        break

# Уборка
print("[INFO] Exiting Program")
#GPIO.cleanup()
cv2.destroyAllWindows()
camera.stop() # На Raspberry
camera.release() # На PC







