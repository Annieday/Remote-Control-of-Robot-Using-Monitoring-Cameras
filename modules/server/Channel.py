# import os, sys
# file_dir = os.path.dirname(os.path.abspath(__file__))
# root = file_dir# + '/..'
# sys.path.insert(0, root)
import struct
from modules.color.BColor import BColor
from config.config import *

class Channel:
    """ A channel within SocketServer """

    def __init__(self, quality, logger):
        self.producer_conn = None
        self.producer_addr = None
        self.is_producing = False
        self.channel_id = quality
        self.subscribers = {}
        self.logger = logger

    def setProducer(self, conn, addr):
        self.producer_conn = conn
        self.producer_addr = addr
        self.is_producing = True
        self.logger.info(BColor.green("Channel " + str(self.channel_id) + " got producer: " + str(addr)))

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
                self.logger.info(BColor.yellow("Lose client ") + str(addr) + " " + BColor.red(str(e)))
                deleted_addrs.append(addr)
                self.unsubscribe(addr)

        # delete all the broken pipes
        for addr in deleted_addrs:
            del clients[addr]

    def subscribe(self, conn, addr):
        self.logger.info("Added subscriber from " +
                    BColor.green(str(addr)) +
                    " to channel " +
                    BColor.green(str(self.channel_id)))
        self.subscribers[addr] = conn
        # self.is_new_subscirbe = True

    def unsubscribe(self, addr):
        try:
            del self.subscribers[addr]
            self.logger.info("Deleted subscriber " +
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
                self.logger.info(BColor.cyan("Channel " + str(self.channel_id) + " error: " + str(e)))
                self.is_producing = False
                break
