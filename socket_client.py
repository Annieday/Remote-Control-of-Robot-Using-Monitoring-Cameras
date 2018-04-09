import socket, struct

# host = "localhost"
host = "192.168.0.17"
port = 60000

class SocketClient:
    def __init__(self, host, port=60000):
        self.s = socket.socket()
        self.host = host
        self.port = port

        try:
            self.s.connect((host, port))
            print("Connected to server", host, "port", port)

        except ConnectionRefusedError as e:
            print(str(e))

    def send(self, data):
        print(struct.pack("I", int(data)))
        self.s.send(struct.pack("I", int(data)))

    def receive_with_length(self, length):
        gotdata = self.s.recv(length)
        print(struct.unpack("I", gotdata)[0])

    def disconnect(self):
        self.s.close()
        print("Disconnect from server", self.host, "port", self.port)

    def __del__(self):
        self.disconnect()


if __name__ == "__main__":
    connection = SocketClient(host, port)
    # connection.send("\nffdsafdsafds")
    # connection.send(640)

    input()
    connection2 = SocketClient(host, port)
    input()
    # connection3 = SocketClient(host, port)
    # input()
    # connection4 = SocketClient(host, port)
    # input()
    # connection5 = SocketClient(host, port)
    # input()
    # connection6 = SocketClient(host, port)
    # input()
