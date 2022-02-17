import os, cv2, json
import numpy as np


def nothing(x): #пустая функция
    pass


def dict2vec(dict):
    x = dict['x']
    y = dict['y']
    z = dict['z']
    return np.array((x,y,z))

xPrev = 0
yPrev = 0
zPrev = 0
x=0
y=0
z=0
# ТУТ НАДО ИСПРАВИТЬ Х У W H В НА НАШИ ЗНАЧЕНИЯ
#def sattelite_telemetry(annotationPath, epoch):
def sattelite_telemetry(epoch, x, y, z, xPrev, yPrev, zPrev):
    # Read JSON file
    #with open(os.path.join(annotationPath, trueAnnot[0])) as json_file:
        #observations = json.load(json_file)
    # Read absolute bounding box boundaries
    #x = observations['bbox'][epoch]['Xcenter']
    #y = observations['bbox'][epoch]['Ycenter']
    #w = observations['bbox'][epoch]['width']
    #h = observations['bbox'][epoch]['height']
    # Read satellite-debris direction vector
    #sd = dict2vec(observations['telemetry'][epoch]['directionVector_actual'])
    # Read satellite position vector
    #es = dict2vec(observations['telemetry'][epoch]['satellitePosition'])
    # Read satellite speed
    #Sspeed = observations['telemetry'][epoch]['SatelliteSpeed']

    ssx = 100
    ssy = 100
    ssz = 100

    sd = np.array((x-xPrev, y-yPrev, z-zPrev))
    es = np.array((x, y, z))
    Sspeed = np.array((ssx, ssy, ssz))
    return (sd,es,Sspeed)


def process_readings(sd,sdPrev,es,esPrev,Nframes,focal_length,w,sensorSz,size):
    # Calculate satellite-debris range
    SD = np.linalg.norm(sd)
    SatSpeed = np.linalg.norm(Sspeed)
    # Calculate debris speed
    ed = np.add(es, sd) # debris position vector in ECI coordinate frame
    edPrev = np.add(esPrev, sdPrev)
    d = ed - edPrev # debris direction vector between current and previous states
    D = np.linalg.norm(d) # debris distance travelled between states (assuming straight line path)
    t = Nframes/fps
    DebSpeed = D/t
    # Calculate debris size
    #size_on_sensor_mm = sensorSz*(w*size[0])/size[0]
    size_on_sensor_mm = sensorSz * w
    DebSize = SD*size_on_sensor_mm/focal_length # [m]
    return (SD, DebSpeed, DebSize, ed, SatSpeed)


focal_length = 2680 # focal length [mm]
sensorSz = 26.624000549316406 # square sensor size [mm]
size = 60
fps = 30
OSF = 40 # specify OffSet Factor for a more visible bbox [in pixels]

kernel =  np.ones((10,10), np.uint8)
# создаем ядро

"""cap = cv2.VideoCapture(1)
Nframes = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))"""
videoFolder = 'B:\CubeSat'
cap = cv2.VideoCapture("test.avi")
Nframes = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))

cv2.namedWindow("track", cv2.WINDOW_NORMAL)

cv2.createTrackbar('HL', "track", 0, 180, nothing)
cv2.createTrackbar('SL', "track", 0, 255, nothing)
cv2.createTrackbar('VL', "track", 0, 255, nothing)

cv2.createTrackbar('H', "track", 0, 180, nothing)
cv2.createTrackbar('S', "track", 0, 255, nothing)
cv2.createTrackbar('V', "track", 0, 255, nothing)

epoch = 0

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
    if len(contours) > 0:
        # Выбираем самый большой контур и вычисляем координаты и радиус
        c = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(c)
        if area > 300:
            x, y, w, h = cv2.boundingRect(c)
            z=w
            frame = cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            frame - cv2.rectangle(frame, (x, y), (x + 60, y - 25), (0, 0, 0), -1)
            cv2.putText(frame, "object", (x, y), cv2.FONT_HERSHEY_SCRIPT_SIMPLEX, 1.0, (255, 255, 255), 2)

            sd, es, Sspeed = sattelite_telemetry(epoch, x, y, z, xPrev, yPrev, zPrev)

            if epoch>0:
                SD, DebSpeed, DebSize, ed, SatSpeed = process_readings(sd, sdPrev, es, esPrev, Nframes, focal_length, w, sensorSz, size)
                text1 = 'FPS: {}\nrange: {} km\ndetected size: {} m\ndetected speed: {} m/s\nsatellite speed: {} m/s'.format(
                    fps, round(SD / 1000, 2), round(DebSize, 1), round(DebSpeed), round(SatSpeed))
                text2 = 'detected position (ECI): {} m\n'.format(np.round(ed, 1))
                cv2.putText(frame, text1, (30, 30), cv2.FONT_HERSHEY_SCRIPT_SIMPLEX, 1.0, (255, 255, 255), 1)
                cv2.putText(frame, text2, (30, 60), cv2.FONT_HERSHEY_SCRIPT_SIMPLEX, 1.0, (255, 255, 255), 1)
                print(text1)
                print(text2)


            sdPrev = sd
            esPrev = es
            xPrev = x
            yPrev = y
            zPrev = z
            epoch = epoch+1



    cv2.imshow("cl", closing)
    cv2.imshow("frame", frame)
    #cv2.imshow("hsv", hsv)
    cv2.imshow("mask", mask)
    cv2.imshow("res", res)


    k = cv2.waitKey(1) & 0xFF
    if k == 27:  # если нажали ескейп вырубаем
        break
cap.release()
cv2.destroyAllWindows() # закрыть все
