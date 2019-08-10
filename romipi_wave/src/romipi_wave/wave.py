#!/usr/bin/env python

import rospy

class WaveNode():
    def __init__(self, name, assigned_wave, node_list):
        """ open the wave node """
        self.name = name
        self.assigned_wave = assigned_wave
        self.node_list = node_list
        print(self)
        return

    def __str__(self):
        return "WaveNode (%s) assigned wave (%d) with node_list %s" % ( \
                self.name, self.assigned_wave, str(self.node_list))

    def test_node(self):
        return

if __name__ == '__main__':
    try:
        node_list = ['jiffy.local']
        wave = WaveNode("jiffy.local", 1, node_list)
            wave.test_node()
    except rospy.ROSInterruptException:
        pass

