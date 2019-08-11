#!/usr/bin/env python

import broadcast
import threading
import activity_manager
import time

class WaveNode():
    def __init__(self, name, assigned_wave, ):
        """ open the wave node """
        self.name = name

        self.assigned_wave = assigned_wave


        self.broadcast_node = broadcast.BroadcastNode()
        self.broadcast_node.set_callback(self.wave_callback)
        self.broadcast_node.start_server()

        self.wave_manager = activity_manager.WaveManager()

        self.start_wave_monitor_thread()

        return

    def start_wave_monitor_thread(self):
        self.running = True
        self.wave_monitor()

    def stop_wave_monitor_thread(self):
        self.running = False
        self.wave_monitor_thread.cancel()

    def wave_monitor(self):
        """ repeating wave_monitor_thread """
        print("wave_monitor_thread checking")


        if self.running:
            self.wave_monitor_thread = threading.Timer(1.0, self.wave_monitor)

    def leave(self):
        """ broadcast LEAVING message """
        self.broadcast_node.broadcast(("WAVE LEAVE", self.name))

    def wave_callback(self, message):
        """ pass to broadcast node to handle processing wave messages """
        print("wave node %s message: %s" % \
              (self.name, message))

        # unpickle the message
        msg_type, msg_data = message

        if msg_type == "WAVE TEST":
            print(msg_type, msg_data)
        elif msg_type == "CURRENT WAVE?":
            """ THIS IS CURRENTLY UNUSED """
            cw_mesg = ("STATUS", self.name, self.wave_manager.get_current_active_wave())
            self.broadcast_node.broadcast(cw_mesg)
        elif msg_type == "WAVE JOIN":
            new_name, assigned_wave = msg_data
            print("%s: %s -> assigned %d" % (msg_type, new_name, assigned_wave))
            self.wave_manager.join(new_name, assigned_wave)

            # JOINING ROBOT IS NEW, BROADCAST CURRENT WAVE MANAGER CONTENTS
            # TO SYNC UP NEW ROBOT
            update_msg = ("UPDATE WAVE", self.wave_manager.get_contents_list())
            self.broadcast_node.broadcast(update_msg)
        elif msg_type == "UPDATE WAVE":
            other_wave_list = msg_data
            print("%s: %s" % (msg_type, str(other_wave_list)))
            self.wave_manager.update(other_wave_list)
        elif msg_type == "WAVE LEAVE":
            leaving_name = msg_data
            print("%s: %s" % (msg_type, leaving_name))
            self.wave_manager.leave(leaving_name)

    """ 
    JOIN, ACTION_COMPLETE, LEAVE, UPDATE methods should not act directly
    except to BROADCAST an action request for wave_callback to handle 
    """

    def join(self, address):
        """ join the broadcast channel and then wave JOIN """
        self.broadcast_node.join(address)

        join_mesg = ("WAVE JOIN", (self.name, self.assigned_wave))
        self.broadcast_node.broadcast(join_mesg)

    def action_complete(self):
        print("WaveNode: action_complete")
        action_complete_msg = ("WAVE ACTION COMPLETE", self.name)
        self.broadcast_node.broadcast(action_complete_msg)
        return

    def is_active(self):
        print("WaveNode: is_active")
        return self.wave_manager.get_current_active_wave() == self.assigned_wave

    def close(self):
        """ shutdown the wave node """
        self.leave()
        time.sleep(1.0)
        self.broadcast_node.close()

    def __str__(self):
        ret = "WaveNode (%s) wave assigned (%d) current (%d), isActive (%s)\n" % (
                self.name, self.assigned_wave, self.wave_manager.get_current_active_wave(),
                str(self.is_active()))
        ret += "\t WaveManager   %s\n" % (str(self.wave_manager))
        ret += "\t BroadcastNode %s\n" % (str(self.broadcast_node))
        return ret

if __name__ == '__main__':
    import random
    try:
        import socket
        my_name = socket.gethostname()
        wave = WaveNode(name=my_name, assigned_wave=1)
        wave.join("jiffy.local")
        print(wave)

        while True:
            sleep_s = random.random()
            time.sleep(sleep_s)
            if wave.is_active():
                wave.action_complete()
            print(wave)
    except KeyboardInterrupt:
        print("KeyboardInterrupt has been caught.")

    wave.close()

