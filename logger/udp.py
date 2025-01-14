import socket
from time import sleep

class Client:
    """ Implements simple UDP client for data streaming. """
    def __init__(self, host:str, port:int):
        self.host = host
        self.port = port
        self.sock = None

    def connect(self):
        if self.sock != None:
            self.disconnect()
            sleep(1)

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.connect((self.host, self.port))
        self.sock.setblocking(False)

    def disconnect(self):
        if self.sock != None:
            self.sock.close()
            del self.sock
        self.sock = None

    def close(self):
        self.disconnect()

    def __del__(self):
        self.disconnect() 

    def isConnected(self) -> bool:
        """ returns true when connection is active """
        return (self.sock != None)
    
    def read(self) -> bytes:
        """ returns all available bytes received """
        try:  
            if self.sock is None:
                self.connect()

            return self.sock.recv(4096)
        except BlockingIOError:
            return bytes() # do not bark, just say there is nothing to read
        except ConnectionError:
            self.disconnect()
    
    def write(self, data:bytes):
        """ writes given data to the port """
        try:
            if self.sock is None:
                self.connect()

            self.sock.sendall(data)
            return
        except ConnectionError:
            self.disconnect()

class Server:
    """ Implements simple UDP server for data streaming. """
    def __init__(self, port:int):
        self.port = port
        self.sock = None
        self.clientIp = None

    def start(self):
        if self.isRunning():
            self.stop()
            sleep(1)

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setblocking(False)
        self.sock.bind(("", self.port))

    def stop(self):
        if self.sock != None:
            self.sock.close()
            del self.sock
        self.sock = None

    def __del__(self):
        self.stop() 

    def isRunning(self) -> bool:
        """ returns true when connection is active """
        return (self.sock != None)
    
    def read(self) -> bytes:
        """ returns all available bytes received """
        try: 
            if self.sock is None:
                self.start()

            data, ip = self.sock.recvfrom(4096)
            self.clientIp = ip
            return data
        except BlockingIOError:
            return bytes() # do not bark, just say there is nothing to read
        except ConnectionError:
            pass
    
    def write(self, data:bytes):
        """ writes given data to the port """
        try:
            if self.sock is None:
                self.start()

            if self.clientIp is None:
                return False

            self.sock.sendto(data, self.clientIp)
            return True
        except ConnectionError:
            self.clientIp = None   
