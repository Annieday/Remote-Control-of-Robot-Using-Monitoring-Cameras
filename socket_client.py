import socket, struct
import numpy as np
import cv2

host = "localhost"
# host = "192.168.0.17"
port = 60000
connection = None
TYPE = 1
BUFFER_SIZE = 1024


class SocketClient:
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


class SingleImage:
    def __init__(self):
        self.h = struct.unpack("I", connection.receive_with_length(4))[0]
        self.w = struct.unpack("I", connection.receive_with_length(4))[0]
        self.size = struct.unpack("I", connection.receive_with_length(4))[0]
        self.raw_img = None
        self.img = None

    # def receive_header(self):
    #     h = c
    #     w = connection.receive_with_length(4)
    #     size = connection.receive_with_length(4)
    #     if h and w and size:
    #         self.h =
    #         self.w = struct.unpack("I", w)[0]
    #         self.size = struct.unpack("I", size)[0]
    #     # else:
    #         # raise StreamingEnd("Header not received")

    def receive_img(self):
        b_data = b''
        try:
            while len(b_data) < self.size:
                print(len(b_data), self.size)
                if len(b_data) + BUFFER_SIZE < self.size:
                    b_data += connection.receive_with_length(BUFFER_SIZE)
                else:
                    b_data += connection.receive_with_length(self.size - len(b_data))

            self.raw_img = np.asarray(list(bytearray(b_data)))
            # print(self.h, self.w, self.size, self.raw_img.size)

        except ConnectionError as e:
            print(str(e))
            connection.status = False
            connection.disconnect()

    def construct_img(self):
        # TODO This needs to be implement later, based on how we encode/compress the image
        self.img = np.reshape(self.raw_img, (self.h, self.w))
        self.img = self.img.astype(np.uint8)

    def show_img(self):
        cv2.imshow('img', self.img)
        cv2.waitKey(30)


if __name__ == "__main__":

    connection = SocketClient(host, port)
    connection.s.settimeout(1)
    # TODO Add GUI for change view options (3d RGB) and quality
    # TODO Create renderer, render window, etc for VTK
    # TODO 1.transform image to point cloud 2.vtk render
    while connection.status:
        single_image = None
        try:
            single_image = SingleImage()
            single_image.receive_img()
            single_image.construct_img()
            single_image.show_img()
        except socket.timeout as e:
            print("Streaming not started")
            cv2.destroyAllWindows()
            del single_image
            continue
        except ValueError as e:
            print(str(e))
            cv2.destroyAllWindows()
            del single_image
            continue
        except ConnectionError as e:
            print(str(e))
            connection.status = False
            connection.disconnect()

    print('Client Disconnected')
