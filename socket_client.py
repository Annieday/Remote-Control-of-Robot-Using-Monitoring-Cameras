import socket, struct
import numpy as np
import cv2
import time
host = "localhost"
# host = "192.168.0.17"
port = 60000
TYPE = 1
BUFFER_SIZE = 1024


class SocketClient:
    def __init__(self, host, port=60000):
        self.s = socket.socket()
        self.host = host
        self.port = port

        try:
            self.s.connect((host, port))
            print("Connected to server", host, "port", port)
            self.send(TYPE)

        except ConnectionRefusedError as e:
            print(str(e))

    def send(self, data):
        print(struct.pack("I", int(data)))
        self.s.send(struct.pack("I", int(data)))

    def receive_with_length(self, length):
        data = self.s.recv(length)
        return data

    def disconnect(self):
        self.s.close()
        print("Disconnect from server", self.host, "port", self.port)

    def __del__(self):
        self.disconnect()


class SingleImage:
    def __init__(self):
        self.h = struct.unpack("I", connection.receive_with_length(4))[0]
        self.w = struct.unpack("I", connection.receive_with_length(4))[0]
        self.size = struct.unpack("I", connection.receive_with_length(4))[0]

        self.img = None

    def receive_img(self, connection):
        b_data = b''
        while len(b_data) < self.size:
            if len(b_data) + BUFFER_SIZE < self.size:
                b_data += connection.receive_with_length(BUFFER_SIZE)
            else:
                b_data += connection.receive_with_length(self.size-len(b_data))

        received_data = np.asarray(list(bytearray(b_data)))
        print(self.h,self.w, self.size, received_data.size)
        self.construct_img(received_data)

    def construct_img(self, received_data):
        # TODO This needs to be implement later, based on how we encode the image
        self.img = np.reshape(received_data, (self.h, self.w))
        self.img = self.img.astype(np.uint8)

    def show_img(self):
        cv2.imshow('img',self.img)
        cv2.waitKey(1000//24)

if __name__ == "__main__":
    connection = SocketClient(host, port)
    # TODO Create renderer, render window, etc for VTK
    while True:
        # try:
        single_image = SingleImage()
        single_image.receive_img(connection)
        print(single_image.img)
        single_image.show_img()
        # TODO 1.transform image to point cloud 2.vtk render
        #     #
        # except Exception as e:
        #     print(e)
        #     connection.disconnect()
