import socket
import pickle
import time


def socetclient():
    s_get = socket.socket()
    host = socket.gethostname()
    port = 12345
    s_get.connect((host, port))
    data = s_get.recv(1024)
    data_variable = pickle.loads(data)
    print(data_variable)
    s_get.close()


while True:
    socetclient()
    time.sleep(5)
