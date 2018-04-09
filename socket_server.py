import socket, struct, time
import _thread

# host = "localhost"
host = "192.168.0.17"
port = 60000

class SocketServer:
    def __init__(self, host, port=60000):
        self.s = socket.socket()        # Create a socket object
        self.host = host
        self.port = port
        self.conns = {}

        try:
            self.s.bind((host, port))   # Bind to the port
            print("Server established.")

        except Exception as e:
            print(str(e))

    def run_server(self):
        print("Server awaiting for client to connect ...")

        self.s.listen(5)                 # Now wait for client connection.
        self.awaiting_connection()


    def awaiting_connection(self):
        while True:
            conn, addr = self.s.accept()     # Establish connection with client.
            print('Got connection from', addr)

            self.conns[addr] = conn

            _thread.start_new_thread( self.receive_header, (addr, ) )

            # self.receive_header(addr)
            print(len(self.conns))

            time.sleep(1)

    def broadcast(self, data):
        for k, v in self.conns:
            v.send(data)

    def receive_header(self, addr):
        data = self.conns[addr].recv(4)
        if len(data) == 4:
            print("receives from client", addr, "with data:", struct.unpack("I", data)[0])
        else:
            del self.conns[addr]

        # self.conns[addr].send(data)
        # print(data)



if __name__ == "__main__":
    server = SocketServer(host, port)
    server.run_server()
