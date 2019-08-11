#!/usr/bin/env python
#
# Peter F. Klemperer
#
# No-Frills Basic Broadcast Mechanism
#
# If re-writing this code, consider
# seperating the networking functions
# into a subclass apart from the
# broadcast specific functions
#
# TODO
# * Change the node_set into a Set()
#
#

from __future__ import print_function

import socket
import pickle

import threading


class BroadcastNode():
    def __init__(self):
        self.isActive = False
        self.node_set = set()

        self.port = 49152

        self.server_address = (socket.gethostname(), self.port)

        self.callback = self.test_callback
        return

    def test_callback(self, message):
        print("broadcast_test message: " + str(message))

    def set_callback(self, cb):
        self.callback = cb

    def send(self, address, pickled_message):
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.connect(address)
        try:
            sock.sendall(pickled_message)
        finally:
            sock.close()

    def send_with_return(self, address, pickled_message):
        """ send to just one node """
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(1.0)
        sock.connect((address,self.port))
        try:
            sock.sendall(pickled_message)
            ret_msg = sock.recv(4096)
            ret_msg = pickle.loads(ret_msg)
        finally:
            sock.close()

        return ret_msg

    def broadcast(self, message):
        broadcast_message = ("BROADCAST", message)
        msg = pickle.dumps(broadcast_message)
        """ broadcast message to channel """
        for node in self.node_set.copy():
            self.send(node, msg)

    def start_server(self):
        """ start the receiving server with handle """

        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.bind(("0.0.0.0", self.port))
        self.server_socket.listen(5)
        self.server_socket.settimeout(0.1)

        # spawn thread for server
        self.server_thread = threading.Thread(target=self._server)
        self.server_thread.start()

        # add server to list of nodes
        self.node_set.add(self.server_address)

        return

    def _server(self):
        self.isActive = True
        while self.isActive:
            try:
                conn, addr = self.server_socket.accept()
                mesg = conn.recv(4096)
                self._process_message(conn, addr, mesg)
                conn.close()
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
        """ register node with the broadcast channel
            * call any node in channel for their subscriber list
            * update my node_set
            * broadcast "JOIN"
        """
        # ASK FOR A NODE_SET
        viewers_mesg = ("SUBSCRIBERS","" )
        pickled_message = pickle.dumps(viewers_mesg)
        subscribers_reply = self.send_with_return(address, pickled_message)
        # UPDATE MY NODE_SET
        self.node_set |= subscribers_reply
        # BROADCAST JOIN
        broadcast_message = ("JOIN", self.node_set)
        broadcast_message_picked = pickle.dumps(broadcast_message)
        for node in self.node_set.copy():
            self.send(node, broadcast_message_picked)
        return

    def register_handler(self, handler=None):
        """ how to check if cb is valid type """
        if handler is not None and self.isActive:
            self.callback = handler

    def leave(self):
        """ unregister node with the broadcast channel """
        leave_message = ("LEAVE", self.server_address)
        leave_message_picked = pickle.dumps(leave_message)
        for node in self.node_set.copy():
            self.send(node, leave_message_picked)
        return

    def _process_message(self, conn, addr, mesg):
        # use conn and addr, but caller will close conn

        # unpickle the message
        msg_type, msg_data = pickle.loads(mesg)

        # cases to handle
        # join
        if msg_type == "TEST":
            print("TEST " + str(msg_data))
        elif msg_type == "SUBSCRIBERS":
            #print("SUBSCRIBERS" + str(msg_data))
            # send list back to caller
            conn.sendall(pickle.dumps(self.node_set))
        elif msg_type == "JOIN":
            peer_node_set = msg_data
            #print("JOIN" + str(peer_node_set))
            self.node_set |= peer_node_set
        # leave
        elif msg_type == "LEAVE":
            peer_name = msg_data
            #print("LEAVE" + str(peer_name))
            self.node_set.discard(peer_name)
        # pass message to callback
        elif msg_type == "BROADCAST":
            #print("BROADCAST" + str(msg_data))
            self.callback(msg_data)
        return

    def __str__(self):
        resp = "BroadcastNode: "
        if not self.isActive:
            resp += "inactive"
            return resp

        resp += "active, node_set: "
        resp += str(self.node_set)
        return resp

    def close(self):
        self.leave()
        self.stop_server()

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
        my_name = socket.gethostname()
        print("gethostname()" + " = " + str(my_name))

        node.start_server()

        node.join("jiffy.local")
        print(node.node_set)
        node.broadcast("hello from " + my_name)

        while True:
            pass
    except KeyboardInterrupt:
        print("KeyboardInterrupt has been caught.")

    node.leave()
    node.stop_server()

if __name__ == '__main__':
    test_node()
