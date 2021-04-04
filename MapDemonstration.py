import cv2
import numpy as np
import time
import socket
import pickle
i=0

if __name__ == '__main__':
    def nothing(*arg):
        pass

#cv2.namedWindow("img2")  # создаем главное окно


imgfile = cv2.imread('1.jpg') # создаем объект с картинки

#устанавливаем размер картинки и делаем окно изменяемым
screen_res = 1920, 1080
scale_width = screen_res[0] / imgfile.shape[1]
scale_height = screen_res[1] / imgfile.shape[0]
scale = min(scale_width, scale_height)
window_width = int(imgfile.shape[1] * scale)
window_height = int(imgfile.shape[0] * scale)

cv2.namedWindow('imgFile', cv2.WINDOW_NORMAL)
#cv2.resizeWindow('imgFile', window_width, window_height)

def socetclient():

    s_get = socket.socket()
    #host = '192.168.100.3'
    host = '192.168.100.14'
    port = 12345
    s_get.connect((host, port))
    data = s_get.recv(1024)
    data_variable = pickle.loads(data)
    print(data_variable)
    s_get.close()
    #time.sleep(3)
    return data_variable

while True:
    data = socetclient()

    #кординаты центра объекта

    cv2.imshow('imgFile', imgfile)  # отображаем кадр в окне с именем result

    #Обновляем картинку
    imgfile = cv2.imread('1.jpg')

    # выводим текст
    cv2.putText(imgfile, "System ON", (200, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 4)


    # Выводим кординаты объекта
    #for v in data:
    x = data[0]
    y = data[1]
    radius = data[2]

    if (x == 0) and (y == 0) and (radius == 0):
        print("ZERO")
        speed = 0

    else:
        speed = radius + 1147

        cv2.circle(imgfile, (x, y), 5, (0, 255, 0), 2)
        cv2.putText(imgfile, "coordinates: %d-%d , speed: %d " % (x, y, speed), (x + 10, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)



    key = cv2.waitKey(1) & 0xFF
    if key == 27:
        break

cap.release()
cv2.destroyAllWindows()
