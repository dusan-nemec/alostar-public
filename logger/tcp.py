import socket
from time import sleep

class Client:
    """ Class for TCP/IP socket communication """
    
    def __init__(self, host:str, port:int):
        """ creates connection without opening it """
        self.sock = None
        self.host = host
        self.port = port
        
    def connect(self):
        """ connects to the TCP/IP socket """
        if self.isConnected():
            self.disconnect()
            sleep(1)  

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect((self.host, self.port))
        self.sock.setblocking(False)
        
    def disconnect(self):
        """ closes connection """
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
            return bytes()
    
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
    """ Implements simple TCP server for data streaming. """
    def __init__(self, port:int):
        self.port = port
        self.sock = None
        self.client = None
        self.clientIp = None

    def start(self):
        if self.isRunning():
            self.stop()
            sleep(1)

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.setblocking(False)
        self.sock.bind(("", self.port))
        self.sock.listen(1)

    def stop(self):
        if self.client != None:
            self.client.close()
            del self.client
        self.client = None
        self.clientIp = None

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
            if not self.isRunning():
                self.start()

            if self.client is None:
                self.client, self.clientIp = self.sock.accept()
                
            return self.client.recv(4096)

        except BlockingIOError:
            return bytes() # do not bark, just say there is nothing to read
        except ConnectionError:
            if self.client != None:
                self.client.close()
                self.client = None
            return bytes()
    
    def write(self, data:bytes):
        """ writes given data to the port """
        try:
            if not self.isRunning():
                self.start()

            if self.client is None:
                self.client, self.clientIp = self.sock.accept()
            
            self.client.send(data)
        except BlockingIOError:
            pass
        except ConnectionError:
            if self.client != None:
                self.client.close()
                self.client = None
