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
        data = self.conns[addr].recv(length)
        return data

    def broadcast(self, data):
        # TODO need to handle unsubscribe
        for k, v in self.clients.items():
            v.send(data)

    def subscribe(self, addr):
        self.new_clients = self.clients.copy()
        self.new_clients[addr] = self.conns[addr]
        self.connection_list_updated = True

    def unsubscribe(self, addr):
        del self.conns[addr]
        self.new_clients = self.clients.copy()
        del self.new_clients[addr]
        self.connection_list_updated = True

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
        # TODO if client is not reachable, remove it from the clients dict

    def producer_handler(self, addr):
        print("this is a producer handler:", addr)
        while True:
            h = self.receive_with_length(4, addr)
            self.broadcast(h)
            h = struct.unpack("I", h)[0]
            w = self.receive_with_length(4, addr)
            self.broadcast(w)
            w = struct.unpack("I", w)[0]
            print(h, w)
            size = self.receive_with_length(4, addr)
            self.broadcast(size)
            size = struct.unpack("I", size)[0]

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

            if self.connection_list_updated:
                self.clients = self.new_clients.copy()
                self.connection_list_updated = False

        # TODO Use while loop to 1. receive image 2. send to client 3. machine learning 4. send instruction to robot
        # while True:


if __name__ == "__main__":
    server = SocketServer(host, port)
    server.run_server()
