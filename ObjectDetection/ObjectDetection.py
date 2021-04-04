from collections import deque
import numpy as np
import argparse
from imutils.video import VideoStream
import imutils
import cv2
import RPi.GPIO as GPIO
import time
import socket
import pickle

# настройка сокета
soc = socket.socket()
host = '192.168.100.14'
print(host)
port = 12345
soc.bind((host, port))
soc.listen(5)

# Инициализируем GPIO
motorright = 21
motorleft = 20
motorstep = 16
on_off_motor = 12
GPIO.setwarnings(False) #Отключение предупреждения
GPIO.setmode(GPIO.BCM)
GPIO.setup(motorleft, GPIO.OUT)
GPIO.output(motorleft, 0)
GPIO.setup(motorright, GPIO.OUT)
GPIO.output(motorright, 0)
GPIO.setup(motorstep, GPIO.OUT)
GPIO.output(motorright, 0)
GPIO.setup(on_off_motor, GPIO.OUT)
GPIO.output(on_off_motor, 0)


braking_distances = 50 # Тормозной путь
number_obj = 1.000001
sss = 0

# Определяем верхнею и нижнею границу цвета
colorLower = (24, 100, 100)
colorUpper = (44, 255, 255)
pts = deque(maxlen= 64)
h = 1900 # Ширина кадра
otstup = 100 # Отступ от угла
y_cord = [540]
radius_obj = []
data_c = []

# инициализируем видеопоток
print("[INFO] waiting for camera ...")
#camera = PiCamera()
#camera.resolution = (1920, 1080)
#camera.framerate = 16
camera = VideoStream(0).start() # На Raspberry
#camera = cv2.VideoCapture(0) # На PC
camera.resolution = (1920, 1080)
time.sleep(2.0)


#Функция опознования
def identification(y,radius):
    pass
"""
    y = int(y)
    radius = int(radius)
    for i in y_cord:
        if (i > (y-braking_distances)) and (i < (y+braking_distances)):
            pass
        else:
            y_cord.append(y)
            print("COORDINATY ZAPISANY")
    #radius_obj.append(radius)
    #print("[INFO] y_cord = ", y_cord)
    #print("[INFO] radius_obj = ", radius_obj)
    
"""


# Функция поворота
def runtoobject(coob):
    if coob == 0:
        GPIO.output(motorright, 0)
        GPIO.output(motorleft, 0)
        GPIO.output(motorstep, 0)
    if coob > 1450:
        GPIO.output(motorright, 1)
        GPIO.output(motorleft, 1)
        GPIO.output(motorstep, 1)
    if coob > 1250 and coob <= 1450:
        GPIO.output(motorright, 1)
        GPIO.output(motorleft, 1)
        GPIO.output(motorstep, 0)
    if coob > 1050 and coob <= 1250:
        GPIO.output(motorright, 1)
        GPIO.output(motorleft, 0)
        GPIO.output(motorstep, 0)
    if coob > 850 and coob <= 1050:
        GPIO.output(motorright, 1)
        GPIO.output(motorleft, 0)
        GPIO.output(motorstep, 1)
    if coob > 650 and coob <= 850:
        GPIO.output(motorright, 0)
        GPIO.output(motorleft, 1)
        GPIO.output(motorstep, 1)
    if coob > 450 and coob <= 650:
        GPIO.output(motorright, 0)
        GPIO.output(motorleft, 0)
        GPIO.output(motorstep, 0)
    if coob > 0 and coob <= 450:
        GPIO.output(motorright, 0)
        GPIO.output(motorleft, 0)
        GPIO.output(motorstep, 0)


# Отправляем данные
def senddata(x,y,radius):
    try:
        data_c.clear()
        #print("[INFO] отправка данных")
        data_c.append(x)
        data_c.append(y)
        data_c.append(radius)
        send_c = pickle.dumps(data_c)
        rasp, addr = soc.accept()
        print('adr：', addr)
        print(list(send_c))
        rasp.send(send_c)
        rasp.close()
    except:
        print("[Error] senddata")
        rasp.close()


while True:
    # Берем кадр с потока изменяем размер и преобразовываем в цветовое пространство HSV
    frame = camera.read()
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
            counter = counter + 1
            GPIO.output(on_off_motor, 1)
            print('WREMYA = ', counter)
            cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)
            cv2.circle(frame, center, 5, (0, 0, 255), -1)
            print("[INFO] to the center",int(x), int(y))
            senddata(int(x), int(y), int(radius))
            runtoobject(int(x))

            if counter > 80:
                GPIO.output(on_off_motor, 0)
                time.sleep(10.0)
                sec = 0
            
            
            """
            # Проверяем знакомый ли объект. Если нет то центруем на него камеру
            if x < ((float(h)/2) - braking_distances):
                #runtoobject()
                pass
            else:
                # Когда отцентровали камеру Рисуем круг и центр на кодре
                #identification(y, radius)
                cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)
                cv2.circle(frame, center, 5, (0, 0, 255), -1)
                print("[INFO] to the center",int(x), int(y))
                senddata(int(x), int(y), int(radius))
            """
    else:
        senddata(0, 0, 0)
        runtoobject(0)

    # Выводим на экран
    cv2.imshow("Frame", frame)
    #senddata()

    # ESC - остановить выполнение
    key = cv2.waitKey(1) & 0xFF
    if key == 27:
        break

# Уборка
print("[INFO] Exiting Program")
GPIO.cleanup()
cv2.destroyAllWindows()
camera.stop() # На Raspberry
camera.release() # На PC







