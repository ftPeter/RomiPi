#!/usr/bin/env python
#
# Peter F. Klemperer
#
# No-Frills Basic Broadcast Mechanism

from __future__ import print_function

import socket
import pickle

import threading


class BroadcastNode():
    def __init__(self):
        self.isActive = False
        self.name = None
        self.node_list = []

        self.port = 49152

        self.callback = (lambda self, m: print(m))
        return

    def send(self, address, message):
        """ send to just one node """
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.connect(address)
        try:
            sock.sendall(message)
        finally:
            sock.close()

    def broadcast(self, message):
        """ broadcast message to channel """
        for node in self.node_list:
            self.send(node, message)

    def start_server(self, name):
        """ start the receiving server with handle """
        self.server_address = (name, self.port)

        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.bind(self.server_address)
        self.server_socket.listen(5)
        self.server_socket.settimeout(0.1)

        # spawn thread for server
        self.server_thread = threading.Thread(target=self._server)
        self.server_thread.start()

        self.node_list.append(self.server_address)

        return

    def _server(self):
        self.isActive = True
        while self.isActive:
            try:
                conn, addr = self.server_socket.accept()
                mesg = conn.recv(1024)
                self._process_message(mesg)
            except socket.timeout:
                pass

    def stop_server(self):
        """ stop the receiver server """
        # TODO broadcast shutdown message
        # shutdown_message = "SHUTDOWN"
        # self.broadcast( shutdown_message )

        self.isActive = False
        self.server_thread.join()
        self.server_socket.close()

    def join(self, address):
        """ register node with the broadcast channel """
        message = ("JOIN", self.node_list)
        return

    def register_handler(self, handler=None):
        """ how to check if cb is valid type """
        if handler is not None and self.isActive:
            self.callback = handler

    def leave(self):
        """ unregister node with the broadcast channel """

    def _process_message(self, mesg):
        # unpickle the message
        msg_type, msg_data = pickle.loads(mesg)

        # cases to handle
        # join
        if msg_type == "JOIN":
            print("JOIN" + str(msg_data))
        # leave
        elif msg_type == "LEAVE":
            print("JOIN" + str(msg_data))
        # pass message to callback
        elif msg_type == "BROADCAST":
            print("JOIN" + str(msg_data))
            self.callback(msg_data)
        elif msg_type == "TEST":
            print("TEST " + str(msg_data))
        return

    def __str__(self):
        resp = "BroadcastNode: "
        if not self.isActive:
            resp += "inactive"
            return resp

        resp += "active, node_list: "
        resp += str(self.node_list)
        return resp

def test_client(ip, port, message):
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect((ip, port))
    try:
        sock.sendall(message)
        #response = sock.recv(1024)
        #print("Received: {}".format(response))
    finally:
        sock.close()

def test_node():
    node = BroadcastNode()
    try:
        node.start_server("localhost")

        test_client("localhost", 49152, pickle.dumps(("TEST","zero")))
        test_client("localhost", 49152, pickle.dumps(("TEST", "one")))
        test_client("localhost", 49152, pickle.dumps(("TEST", "two")))
        test_client("localhost", 49152, pickle.dumps(("TEST", "three")))

        # make sure server has time to process the clients.
        import time
        time.sleep(0.01)

    except:
        print("KeyboardInterrupt has been caught.")
        node.stop_server()

if __name__ == '__main__':
    test_node()
