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


# https://stackoverflow.com/questions/287871/print-in-terminal-with-colors
class BColor:
    ## color codes
    CYAN = '\033[96m'
    PURPLE = '\033[95m'
    BLUE = '\033[94m'
    YELLOW = '\033[93m'
    GREEN = '\033[92m'
    RED = '\033[91m'
    DARK_CYAN = '\033[36m'
    DARK_PURPLE = '\033[35m'
    DARK_BLUE = '\033[34m'
    DARK_YELLOW = '\033[33m'
    DARK_GREEN = '\033[32m'
    DARK_RED = '\033[31m'

    ## sytle codes
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'

    def style(header, string):
        """ Adds a style header to string. """
        return header + string + BColor.ENDC

    def green(string):
        """ Turns a string green. """
        return BColor.style(BColor.GREEN, string)

    def red(string):
        """ Turns a string red. """
        return BColor.style(BColor.RED, string)

    def yellow(string):
        """ Turns a string blue. """
        return BColor.style(BColor.YELLOW, string)

    def cyan(string):
        """ Turns a string blue. """
        return BColor.style(BColor.CYAN, string)


class Channel:
    """ A channel within SocketServer """

    def __init__(self, quality):
        self.producer_conn = None
        self.producer_addr = None
        self.is_producing = False
        self.channel_id = quality
        self.subscribers = {}

    def setProducer(self, conn, addr):
        self.producer_conn = conn
        self.producer_addr = addr
        self.is_producing = True
        logger.info(BColor.green("Channel " + str(self.channel_id) + " got producer: " + str(addr)))

    def receive_with_length(self, length):
        received_data = self.producer_conn.recv(length)
        if len(received_data) == 0:
            self.is_producing = False
            raise ConnectionError("Loss producer " + str(self.producer_addr))
        return received_data

    def broadcast(self, clients, data):
        deleted_addrs = []
        for addr, conn in clients.items():
            try:
                conn.send(data)
            except ConnectionError as e:
                logger.info(BColor.yellow("Lose client ") + str(addr) + " " + BColor.red(str(e)))
                deleted_addrs.append(addr)
                self.unsubscribe(addr)

        # delete all the broken pipes
        for addr in deleted_addrs:
            del clients[addr]

    def subscribe(self, conn, addr):
        logger.info("Added subscriber from " +
                    BColor.green(str(addr)) +
                    " to channel " +
                    BColor.green(str(self.channel_id)))
        self.subscribers[addr] = conn
        # self.is_new_subscirbe = True

    def unsubscribe(self, addr):
        try:
            del self.subscribers[addr]
            logger.info("Deleted subscriber " +
                        BColor.yellow(str(addr)) +
                        " from channel " +
                        BColor.yellow(str(self.channel_id)))
        except KeyError:
            # already unsubscribed
            pass

    def producer_handler(self):
        print("this is a producer handler:", self.producer_addr)
        while True:
            clients = self.subscribers.copy()

            try:
                b_data = b''
                h = self.receive_with_length(4)
                w = self.receive_with_length(4)
                size = self.receive_with_length(4)
                b_data += h
                b_data += w
                b_data += size
                self.broadcast(clients, b_data)

                h = struct.unpack("I", h)[0]
                w = struct.unpack("I", w)[0]
                size = struct.unpack("I", size)[0]
                # print(h, w, size, self.quality)
                b_data = b''
                while len(b_data) < size:
                    if len(b_data) + BUFFER_SIZE < size:
                        temp = self.receive_with_length(BUFFER_SIZE)
                        self.broadcast(clients, temp)
                        b_data += temp
                    else:
                        temp = self.receive_with_length(size - len(b_data))
                        self.broadcast(clients, temp)
                        b_data += temp

                # TODO 3. machine learning based on selected quality that's hard coded
                # TODO 4. send instruction to robot
                # self.update_subscribe_list_all()
            except ConnectionError as e:
                logger.info(BColor.cyan("Channel " + str(self.channel_id) + " error: " + str(e)))
                break


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
                    self.channels[quality] = Channel(quality)  # create a channel

                if conn_type == TYPE_PRODUCER:
                    self.channels[quality].setProducer(conn, addr)
                    self.channels[quality].producer_handler()

                elif conn_type == TYPE_CLIENT:
                    self.channels[quality].subscribe(conn, addr)
                    self.client_handler(addr)

            else:  # unknow conn_type shut it donw
                conn.close()
                del conn
                del self.conns[addr]
        else:
            # handshake error
            conn.close()
            del conn

    def client_handler(self, addr):
        print("this is a client handler:", addr)
        # TODO use while loop to receive request and handle them
        # while True:


if __name__ == "__main__":
    # logging.basicConfig(#filename='SocketServer.log',
    #                     level=logging.DEBUG,
    #                     format='%(asctime)s %(message)s',
    #                     datefmt='%Y-%m-%d %H:%M:%S')

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

    server = SocketServer(host, port)
    if server.running:
        server.run_server()
