import socket, struct, time
import _thread
import numpy as np
import cv2
import logging

from modules.color.BColor import BColor
from modules.server.Channel import Channel
from config.config import *

# host = "localhost"
# port = 60000

class SocketServer:
    def __init__(self, host='localhost', port=60000):
        self.s = socket.socket()  # Create a socket object
        self.host = host
        self.port = port
        self.conns = {}
        self.channels = {}
        self.running = False
        try:
            self.s.bind((host, port))  # Bind to the port
            logger.info(BColor.green("Server established. Host: " + host + "port: " + str(port)))
            self.running = True

        except Exception as e:
            logger.error("Server establish error. Host: " + host + "port: " + str(port) + " " + BColor.red(str(e)))

    def run_server(self):
        print("Server awaiting for client to connect ...")

        self.s.listen(5)  # Now wait for client connection.

        while True:
            conn, addr = self.s.accept()  # Establish connection with client.
            logger.info('Got connection from ' + str(addr))

            self.conns[addr] = conn

            _thread.start_new_thread(self.initialize_handshake, (conn, addr,))

            # print("Number of conns", len(conns))

            time.sleep(1)

    # def receive_with_length(self, length, addr):
    #     received_data = conns[addr].recv(length)
    #     return received_data

    def initialize_handshake(self, conn, addr):
        handshake_lenth = 8
        data = conn.recv(handshake_lenth)  # receive the conn_type
        if len(data) == handshake_lenth:
            conn_type = struct.unpack("I", data[:4])[0]  # extract the conn_type
            quality = struct.unpack("I", data[4:])[0]  # extract the quality
            print("receives from connection", addr, "with type:", conn_type, "quality:", quality)

            if conn_type == TYPE_PRODUCER or conn_type == TYPE_CLIENT:
                try:
                    channel = self.channels[quality]
                except KeyError as e:
                    # The channel is not exist, yet.
                    self.channels[quality] = Channel(quality, logger)  # create a channel

                if conn_type == TYPE_PRODUCER:
                    if self.channels[quality].is_producing: # there is a producer is producing
                        logger.warning(BColor.red(str(addr)+' try to publish to a producing channel. Dumped.'))
                        self.disconnect_from(conn, addr)
                    else:
                        self.channels[quality].setProducer(conn, addr)
                        self.channels[quality].producer_handler()

                elif conn_type == TYPE_CLIENT:
                    self.channels[quality].subscribe(conn, addr)
                    self.client_handler(addr)

            else:  # unknow conn_type shut it donw
                self.disconnect_from(conn, addr)
        else:
            # handshake error
            self.disconnect_from(conn, addr)

    def disconnect_from(self, conn, addr):
        conn.close()
        del conn
        del self.conns[addr]

    def client_handler(self, addr):
        print("this is a client handler:", addr)
        # TODO use while loop to receive request and handle them
        # while True:


if __name__ == "__main__":
    ### setup looger ###
    logger = logging.getLogger()
    logger.setLevel(logging.DEBUG)
    formatter = logging.Formatter(
        '%(asctime)s - %(levelname)s - %(message)s',
        datefmt='%Y-%m-%d %H:%M:%S')

    # get a file handler
    fh = logging.FileHandler('SocketServer.log')
    fh.setLevel(logging.DEBUG)
    fh.setFormatter(formatter)

    # get a stream handler
    ch = logging.StreamHandler()
    ch.setLevel(logging.DEBUG)
    ch.setFormatter(formatter)

    # add handlers to logger
    logger.addHandler(fh)
    logger.addHandler(ch)

    ### setup server and run ###
    server = SocketServer(host, port)
    if server.running:
        server.run_server()
