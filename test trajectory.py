import os, cv2, json
import numpy as np


def nothing(x): #пустая функция
    pass


def dict2vec(dict):
    x = dict['x']
    y = dict['y']
    z = dict['z']
    return np.array((x,y,z))

# ТУТ НАДО ИСПРАВИТЬ Х У W H В НА НАШИ ЗНАЧЕНИЯ
#def sattelite_telemetry(annotationPath, epoch):
def sattelite_telemetry(epoch):
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
    sdx = 100
    sdy = 100
    sdz = 100

    esx = 100
    esy = 100
    esz = 100

    ssx = 100
    ssy = 100
    ssz = 100

    sd = np.array((sdx,sdy,sdz))
    es = np.array((esx,esy,esz))
    Sspeed = np.array((ssx, ssy, ssz))
    return (sd,es,Sspeed)

def Distance_finder(Focal_Length, real_face_width, face_width_in_frame):
    distance = (real_face_width * Focal_Length) / face_width_in_frame
    # return the distance
    return distance


def process_readings(sd,sdPrev,es,esPrev,Nframes,focal_length,w,sensorSz,size):
    # Calculate satellite-debris range
    SD = np.linalg.norm(sd)
    # Calculate debris speed
    ed = np.add(es, sd) # debris position vector in ECI coordinate frame
    edPrev = np.add(esPrev, sdPrev)
    d = ed - edPrev # debris direction vector between current and previous states
    D = np.linalg.norm(d) # debris distance travelled between states (assuming straight line path)
    t = Nframes/fps
    DebSpeed = D/t
    # Calculate debris size
    size_on_sensor_mm = sensorSz*(w*size[0])/size[0]
    DebSize = SD*size_on_sensor_mm/focal_length # [m]
    return (SD, DebSpeed, DebSize, ed)


focal_length = 2680 # focal length [mm]
sensorSz = 26.624000549316406 # square sensor size [mm]

OSF = 40 # specify OffSet Factor for a more visible bbox [in pixels]

kernel =  np.ones((10,10), np.uint8)
# создаем ядро


fps = 60
focal_length = 2680 # focal length [mm]
sensorSz = 26.624000549316406 # square sensor size [mm]

OSF = 40 # specify OffSet Factor for a more visible bbox [in pixels]
videoFolder = 'B:\CubeSat'
files = os.listdir(videoFolder)

# Separate true label files from animations (true labels = actual object location)
animations = []
trueAnnot = []
for file in files:
    if file.endswith('.avi'):
        animations.append(file)
    elif file.endswith('.txt'):
        trueAnnot.append(file)

# Extract frames from individual detection video and superimpose bounding box from annotation data
epoch = 2
frames = []
text1 = 'FPS: \nrange:  km\ndetected position (ECI): m\ndetected size:  m\ndetected speed:  m/s\nsatellite speed:  m/s'
text2 = 'detected position (ECI): m\n'
for vid in animations:
    vidcap = cv2.VideoCapture(os.path.join(videoFolder, vid))
    Nframes = int(vidcap.get(cv2.CAP_PROP_FRAME_COUNT))

    for n in range(Nframes):
        success, frame = vidcap.read()
        size = frame.shape[0:2]

        lower = np.array([0, 120, 0])  # нижний диапозон
        upper = np.array([255, 255, 255])  # верхний диапозон
        frame = cv2.bilateralFilter(frame, 9, 75, 75)
            # для сглаживания используем билатеральный фильтер

        mask = cv2.inRange(hsv, lower, upper)
            # позволяет цвет в диапозоне сделать белым а остальное черным
        res = cv2.bitwise_and(frame, frame, mask=mask)
            # позволяет белое сделать цветным обратно

        opening = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        closing = cv2.morphologyEx(opening, cv2.MORPH_CLOSE, kernel)
            # используем морфологические операции для фильтрации изображения

        contours, h = cv2.findContours(closing, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
            # находим контуры
        contours = sorted(contours, key=cv2.contourArea, reverse=True)
            # сортируем контуры указывая их, метод и реверс тру значит по убыванию

        if len(contours) > 0:
                # Выбираем самый большой контур и вычисляем координаты и радиус
           c = max(contours, key=cv2.contourArea)
           area = cv2.contourArea(c)
           if area > 300:
             x, y, w, h = cv2.boundingRect(c)
             frame = cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
             frame - cv2.rectangle(frame, (x, y), (x + 60, y - 25), (0, 0, 0), -1)
             cv2.putText(frame, "object", (x, y), cv2.FONT_HERSHEY_SCRIPT_SIMPLEX, 1.0, (255, 255, 255), 2)






            if epoch>0:
                # Process telemetry/detection data to output useful parameters
                SD, DebSpeed, DebSize, ed = process_readings(sd,sdPrev,es,esPrev,Nframes,focal_length,w,sensorSz,size)
                # Update parameters textbox
                # https://stackoverflow.com/questions/54607447/opencv-how-to-overlay-text-on-video
                text1 = 'FPS: {}\nrange: {} km\ndetected size: {} m\ndetected speed: {} m/s\nsatellite speed: {} m/s'.format(
                    fps, round(SD/1000,2), round(DebSize,1), round(DebSpeed), round(SatSpeed))
                text2 = 'detected position (ECI): {} m\n'.format(np.round(ed,1))
            # Assign current processed parameters as "previous" for next iteration
            sdPrev = np.array((1000, 1000, 1000))
            esPrev = np.array((1000, 1000, 1000))
        # Append textbox in frame
        draw_label(frame, text1, (size[0]-480, 50))
        draw_label(frame, text2, (round(size[0]/4), size[1]-30))
        frames.append(frame)
    epoch += 1

# Write final video combining all individual animations
video = cv2.VideoWriter(os.path.join(videoFolder, 'system_example.avi'),cv2.VideoWriter_fourcc(*'DIVX'), fps, size)
#video = cv2.VideoWriter(os.path.join(videoFolder, 'system_example.MP4'),cv2.VideoWriter_fourcc(*'DIVX'), fps, size)

for i in range(len(frames)):
    video.write(frames[i])
video.release()

