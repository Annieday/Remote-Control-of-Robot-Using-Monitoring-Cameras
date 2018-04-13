import socket, struct
import numpy as np
import cv2
import PIL
import _thread
import sys

host = "localhost"
# host = "192.168.0.17"
port = 60000
TYPE = 0
BUFFER_SIZE = 1024
QUALITY_LOW = 0
QUALITY_MEDIUM = 1
QUALITY_HIGH = 2

connection = {}


class SocketProducer:
    def __init__(self, host, port, quality):
        self.s = socket.socket()
        self.host = host
        self.port = port
        self.quality = quality
        self.status = False

        try:
            self.s.connect((host, port))
            print("Connected to server", host, "port", port)
            self.send(TYPE)
            self.send(quality)
            self.status = True

        except ConnectionRefusedError as e:
            print(str(e))
            self.status = False

    def send(self, data):
        # print(struct.pack("I", int(data)))
        self.s.send(struct.pack("I", int(data)))

    def receive_with_length(self, length):
        received_data = self.s.recv(length)
        return received_data

    def disconnect(self):
        self.s.close()
        self.status = False
        print("Disconnect from server", self.host, "port", self.port)

    def __del__(self):
        self.disconnect()


def compress_img(img, quality=QUALITY_HIGH):
    img = np.reshape(img, (1, -1))[0]
    return img


def send_img(conn, img, h, w):
    try:
        conn.send(h)
        conn.send(w)
        message = compress_img(img)
        conn.send(message.size)
        # conn.s.send(bytearray([self.L_value]))
        # send the image
        value = bytearray(message)
        print(h, w, message.size)
        conn.s.sendall(value)
    except Exception as e:
        print(str(e))
        conn.status = False
        conn.disconnect()
        return


if __name__ == "__main__":
    connection[QUALITY_HIGH] = SocketProducer(host, port, QUALITY_HIGH)
    connection[QUALITY_MEDIUM] = SocketProducer(host, port, QUALITY_MEDIUM)
    connection[QUALITY_LOW] = SocketProducer(host, port, QUALITY_LOW)

    # npy_depth = cv2.imread('Gray_Image.jpg', 0)
    cam = cv2.VideoCapture(0)
    # while connection[QUALITY_MEDIUM].status:
    while connection[QUALITY_LOW].status or connection[QUALITY_MEDIUM].status or connection[QUALITY_HIGH].status:
        ret, npy_depth = cam.read()
        npy_depth = cv2.cvtColor(npy_depth, cv2.COLOR_BGR2GRAY)
        h, w = npy_depth.shape
        # TODO compress the image, maybe use thread??
        for k, conn in connection.items():
            send_img(conn, npy_depth, h, w)
            # _thread.start_new_thread(send_img, (conn, npy_depth, h, w,))

    # _thread.exit()
    #     b_data += connection.receive_with_length(BUFFER_SIZE)
    # if self.size % BUFFER_SIZE != 0:
    #     b_data += connection.receive_with_length(self.size % BUFFER_SIZE)
# cv2.imshow('npy_depth', npy_depth)
# cv2.waitKey(0)

#     connection.send()
# connection.send("\nffdsafdsafds")
# connection.send(640)
