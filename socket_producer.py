import socket, struct
import numpy as np
import cv2
import PIL
host = "localhost"
# host = "192.168.0.17"
port = 60000
TYPE = 0
BUFFER_SIZE = 1024


class SocketProducer:
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
        received_data = self.s.recv(length)
        print(struct.unpack("I", received_data)[0])
        return received_data

    def disconnect(self):
        self.s.close()
        print("Disconnect from server", self.host, "port", self.port)

    def __del__(self):
        self.disconnect()

def compress_img(image):
    image = np.reshape(image, (1, -1))[0]
    return image


if __name__ == "__main__":
    connection = SocketProducer(host, port)
    while True:
        npy_depth = np.load('1520468813.npy')
        h, w = npy_depth.shape
        try:

            connection.s.send(struct.pack("I", int(h)))
            connection.s.send(struct.pack("I", int(w)))
            # TODO compress the image
            message = compress_img(npy_depth)
            connection.s.send(struct.pack("I", int(message.size)))
            # connection.s.send(bytearray([self.L_value]))

            # send the image
            value = bytearray(message)
            connection.s.sendall(value)
        except BrokenPipeError as e:
            connection.statusBar().showMessage(str(e))
            connection.s.close()


        #     b_data += connection.receive_with_length(BUFFER_SIZE)
        # if self.size % BUFFER_SIZE != 0:
        #     b_data += connection.receive_with_length(self.size % BUFFER_SIZE)
    # cv2.imshow('npy_depth', npy_depth)
    # cv2.waitKey(0)

    #     connection.send()
    # connection.send("\nffdsafdsafds")
    # connection.send(640)
