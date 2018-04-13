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

QUALITY_LOW = 0
QUALITY_MEDIUM = 1
QUALITY_HIGH = 2

CLIENT_REQUEST_QUALITY_CHANGE = 0
CLIENT_REQUEST_VTK = 1
CLIENT_REQUEST_RGB = 2
CLIENT_REQUEST_GREY = 3

conns = {}
producer = {}
subscriber = {}
subscriber[QUALITY_LOW] = {}
subscriber[QUALITY_MEDIUM] = {}
subscriber[QUALITY_HIGH] = {}


class Producer:
    def __init__(self, conn, addr, quality):
        self.conn = conn
        self.addr = addr
        self.quality = quality
        self.client_subscribed = False
        self.client_unsubscribed = False
        self.new_clients = {}
        self.rm_clients = {}

    def receive_with_length(self, length):
        received_data = self.conn.recv(length)
        return received_data

    def broadcast(self, data):
        if self.client_unsubscribed:
            self.update_subscribe_list_remove()

        for k, v in subscriber[self.quality].items():
            try:
                v.send(data)
            except ConnectionError as e:
                print(str(e))
                self.unsubscribe(k)
                del conns[k]
                continue

    def subscribe(self, addr):
        self.new_clients[addr] = conns[addr]
        self.client_subscribed = True

    def unsubscribe(self, addr):
        self.rm_clients[addr] = conns[addr]
        self.client_unsubscribed = True

    def update_subscribe_list_all(self):
        self.update_subscribe_list_add()
        self.update_subscribe_list_remove()

    def update_subscribe_list_add(self):
        if self.client_subscribed:
            subscriber[self.quality].update(self.new_clients)
            self.new_clients = {}
            self.client_subscribed = False
            print("Number of clients conns", len(subscriber[self.quality]))

    def update_subscribe_list_remove(self):
        if self.client_unsubscribed:
            all(map(subscriber[self.quality].pop, self.rm_clients))
            self.rm_clients = {}
            self.client_unsubscribed = False
            print("Number of clients conns", len(subscriber[self.quality]))

    def producer_removed(self):
        del conns[self.addr]
        del producer[self.quality]
        print("Number of clients conns", len(subscriber[self.quality]))

    def producer_handler(self):
        print("this is a producer handler:", self.addr)
        while True:
            try:
                b_data = b''
                h = self.receive_with_length(4)
                w = self.receive_with_length(4)
                size = self.receive_with_length(4)
                b_data += h
                b_data += w
                b_data += size
                self.broadcast(b_data)

                h = struct.unpack("I", h)[0]
                w = struct.unpack("I", w)[0]
                size = struct.unpack("I", size)[0]
                print(h, w, size, self.quality)
                b_data = b''
                while len(b_data) < size:
                    if len(b_data) + BUFFER_SIZE < size:
                        temp = self.receive_with_length(BUFFER_SIZE)
                        self.broadcast(temp)
                        b_data += temp
                    else:
                        temp = self.receive_with_length(size - len(b_data))
                        self.broadcast(temp)
                        b_data += temp

                # TODO 3. machine learning based on selected quality that's hard coded
                # TODO 4. send instruction to robot
                self.update_subscribe_list_all()
            except ConnectionError as e:
                print(str(e))
                self.update_subscribe_list_all()
                self.producer_removed()
                break


class SocketServer:
    def __init__(self, host, port=60000):
        self.s = socket.socket()  # Create a socket object
        self.host = host
        self.port = port
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

            conns[addr] = conn

            _thread.start_new_thread(self.receive_header, (addr,))

            # self.receive_header(addr)
            print("Number of conns", len(conns))

            time.sleep(1)

    def receive_with_length(self, length, addr):
        received_data = conns[addr].recv(length)
        return received_data

    def receive_header(self, addr):
        data = conns[addr].recv(4)
        if len(data) == 4:
            conn_type = struct.unpack("I", data)[0]
            print("receives from connection", addr, "with type:", conn_type)
            if conn_type == TYPE_PRODUCER:
                data = conns[addr].recv(4)
                if len(data) == 4:
                    quality = struct.unpack("I", data)[0]
                    producer[quality] = Producer(conns[addr], addr, quality)
                    producer[quality].producer_handler()
                else:
                    del conns[addr]
            elif conn_type == TYPE_CLIENT:
                if len(producer) == 0:
                    subscriber[QUALITY_HIGH][addr] = conns[addr]
                else:
                    producer[QUALITY_HIGH].subscribe(addr)
                self.client_handler(addr)
            else:
                del conns[addr]
        else:
            del conns[addr]

        # self.conns[addr].send(data)
        # print(data)

    def client_handler(self, addr):
        print("this is a client handler:", addr)
        # TODO use while loop to receive request and handle them
        # while True:


if __name__ == "__main__":
    server = SocketServer(host, port)
    server.run_server()

