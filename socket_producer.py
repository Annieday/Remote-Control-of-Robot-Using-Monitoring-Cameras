import socket, struct
import numpy as np
import cv2
import PIL

host = "localhost"
# host = "192.168.0.17"
port = 60000
TYPE = 0
BUFFER_SIZE = 1024
connection = None


class SocketProducer:
    def __init__(self, host, port=60000):
        self.s = socket.socket()
        self.host = host
        self.port = port
        self.status = False

        try:
            self.s.connect((host, port))
            print("Connected to server", host, "port", port)
            self.send(TYPE)
            self.status = True

        except ConnectionRefusedError as e:
            print(str(e))
            self.status = False

    def send(self, data):
        print(struct.pack("I", int(data)))
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


def compress_img(image):
    image = np.reshape(image, (1, -1))[0]
    return image


if __name__ == "__main__":
    connection = SocketProducer(host, port)
    # npy_depth = cv2.imread('Gray_Image.jpg', 0)
    cam = cv2.VideoCapture(0)
    while connection.status:
        ret, npy_depth = cam.read()
        npy_depth = cv2.cvtColor(npy_depth, cv2.COLOR_BGR2GRAY)
        h, w = npy_depth.shape
        try:
            connection.send(h)
            connection.send(w)
            # TODO compress the image
            message = np.reshape(npy_depth, (1, -1))[0]
            connection.send(message.size)
            # connection.s.send(bytearray([self.L_value]))

            # send the image
            value = bytearray(message)
            print(h, w, message.size)
            connection.s.sendall(value)
        except Exception as e:
            print(str(e))
            connection.status = False
            connection.disconnect()
        #     b_data += connection.receive_with_length(BUFFER_SIZE)
        # if self.size % BUFFER_SIZE != 0:
        #     b_data += connection.receive_with_length(self.size % BUFFER_SIZE)
    # cv2.imshow('npy_depth', npy_depth)
    # cv2.waitKey(0)

    #     connection.send()
    # connection.send("\nffdsafdsafds")
    # connection.send(640)
