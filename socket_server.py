import socket, struct, time
import _thread
import numpy as np
import cv2
import logging
import asyncore

host = "localhost"
# host = "192.168.0.17"
port = 60000
TYPE_PRODUCER = 0
TYPE_CLIENT = 1


class SocketServer:
    def __init__(self, host, port=60000):
        self.s = socket.socket()  # Create a socket object
        self.host = host
        self.port = port
        self.conns = {}
        self.clients = {}
        self.producer = {}

        try:
            self.s.bind((host, port))  # Bind to the port
            print("Server established.")

        except Exception as e:
            print(str(e))

    def run_server(self):
        print("Server awaiting for client to connect ...")

        self.s.listen(5)  # Now wait for client connection.
        self.awaiting_connection()

    def awaiting_connection(self):
        while True:
            conn, addr = self.s.accept()  # Establish connection with client.
            print('Got connection from', addr)

            self.conns[addr] = conn

            _thread.start_new_thread(self.receive_header, (addr,))

            # self.receive_header(addr)
            print("Number of conns", len(self.conns))

            time.sleep(1)

    def broadcast(self, data):
        for k, v in self.conns:
            v.send(data)

    def subscribe(self, addr):
        self.clients[addr] = self.conns[addr]

    def unsubscribe(self, addr):
        del self.conns[addr]
        del self.producer[addr]

    def receive_header(self, addr):
        data = self.conns[addr].recv(4)
        if len(data) == 4:
            conn_type = struct.unpack("I", data)[0]
            print("receives from connection", addr, "with type:", conn_type)
            if conn_type == TYPE_PRODUCER:
                self.producer[addr] = self.conns[addr]
                self.producer_handler(addr)
                print("Number of clients conns", len(self.clients))
                print("Number of producer conns", len(self.producer))
            elif conn_type == TYPE_CLIENT:
                self.subscribe(addr)
                print("Number of clients conns", len(self.clients))
                print("Number of producer conns", len(self.producer))
                self.client_handler(addr)
                print("Number of clients conns", len(self.clients))
                print("Number of producer conns", len(self.producer))
            else:
                del self.conns[addr]
        else:
            del self.conns[addr]

        # self.conns[addr].send(data)
        # print(data)

    def client_handler(self, addr):
        print("this is a client handler:", addr)
        npy_depth = cv2.imread('Gray_Image.jpg', 0)
        h, w = npy_depth.shape
        while True:
            try:
                self.clients[addr].send(struct.pack("I", int(h)))
                self.clients[addr].send(struct.pack("I", int(w)))
                # TODO compress the image
                message = np.reshape(npy_depth, (1, -1))[0]
                self.clients[addr].send(struct.pack("I", int(message.size)))
                # connection.s.send(bytearray([self.L_value]))

                # send the image
                value = bytearray(message)
                print(npy_depth)
                self.clients[addr].sendall(value)
            except BrokenPipeError as e:
                print(e)
                del self.conns[addr]
                del self.clients[addr]
        # TODO if client is not reachable, remove it from the clients dict

    def producer_handler(self, addr):
        print("this is a producer handler:", addr)
        # TODO Use while loop to 1. receive image 2. send to client 3. machine learning 4. send instruction to robot
        # while True:


if __name__ == "__main__":
    server = SocketServer(host, port)
    server.run_server()
