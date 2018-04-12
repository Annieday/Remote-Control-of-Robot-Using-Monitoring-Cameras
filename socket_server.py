import socket, struct, time
import _thread
import numpy as np
import cv2
import logging

host = "localhost"
# host = "192.168.0.17"
port = 60000
TYPE_PRODUCER = 0
TYPE_CLIENT = 1
BUFFER_SIZE = 1024


class SocketServer:
    def __init__(self, host, port=60000):
        self.s = socket.socket()  # Create a socket object
        self.host = host
        self.port = port
        self.conns = {}
        self.clients = {}
        self.producer = {}
        self.connection_list_updated = False
        self.new_clients = {}
        self.rm_clients = {}
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

    def receive_with_length(self, length, addr):
        received_data = self.conns[addr].recv(length)
        return received_data

    def broadcast(self, data):
        # TODO need to handle unsubscribe
        self.update_subscribe_list()
        for k, v in self.clients.items():
            try:
                v.send(data)
            except ConnectionError as e:
                print(str(e))
                self.unsubscribe(k)
                continue

    def subscribe(self, addr):
            self.new_clients[addr] = self.conns[addr]
            self.connection_list_updated = True

    def unsubscribe(self, addr):
        self.rm_clients[addr] = self.conns[addr]
        del self.conns[addr]
        self.connection_list_updated = True

    def update_subscribe_list(self):
        if self.connection_list_updated:
            self.clients.update(self.new_clients)
            all(map(self.clients.pop, self.rm_clients))
            self.new_clients = {}
            self.rm_clients = {}
            self.connection_list_updated = False
            print("Number of clients conns", len(self.new_clients))
            print("Number of producer conns", len(self.producer))

    def producer_removed(self, addr):
        del self.conns[addr]
        del self.producer[addr]
        print("Number of clients conns", len(self.new_clients))
        print("Number of producer conns", len(self.producer))

    def receive_header(self, addr):
        data = self.conns[addr].recv(4)
        if len(data) == 4:
            conn_type = struct.unpack("I", data)[0]
            print("receives from connection", addr, "with type:", conn_type)
            if conn_type == TYPE_PRODUCER:
                self.producer[addr] = self.conns[addr]
                self.producer_handler(addr)
            elif conn_type == TYPE_CLIENT:
                self.subscribe(addr)
                self.client_handler(addr)
            else:
                del self.conns[addr]
        else:
            del self.conns[addr]

        # self.conns[addr].send(data)
        # print(data)

    def client_handler(self, addr):
        print("this is a client handler:", addr)

    def producer_handler(self, addr):
        print("this is a producer handler:", addr)

        while True:
            try:
                self.update_subscribe_list()

                h = self.receive_with_length(4, addr)
                self.broadcast(h)
                w = self.receive_with_length(4, addr)
                self.broadcast(w)
                size = self.receive_with_length(4, addr)
                self.broadcast(size)

                h = struct.unpack("I", h)[0]
                w = struct.unpack("I", w)[0]
                size = struct.unpack("I", size)[0]
                print(h, w)
                b_data = b''
                while len(b_data) < size:
                    if len(b_data) + BUFFER_SIZE < size:
                        temp = self.receive_with_length(BUFFER_SIZE, addr)
                        self.broadcast(temp)
                        b_data += temp
                    else:
                        temp = self.receive_with_length(size - len(b_data), addr)
                        self.broadcast(temp)
                        b_data += temp

            except ConnectionError as e:
                print(str(e))
                self.producer_removed(addr)
                self.update_subscribe_list()
                break

        # TODO Use while loop to 1. receive image 2. send to client 3. machine learning 4. send instruction to robot
        # while True:


if __name__ == "__main__":
    server = SocketServer(host, port)
    server.run_server()
