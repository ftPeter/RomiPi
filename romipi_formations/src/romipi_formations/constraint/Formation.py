#!/usr/bin/python3
#
# Formation
#
# tool for storing a storing a complete
# ball-and-stick constrained formation
#
# see the FormationLibrary.py for usage examples
#
# Peter F. Klemperer
# July 29, 2018

class FormationNode:
    def __init__(self, name_str, wave_num):
        self.name_str = name_str
        self.wave_num = wave_num

    def __str__(self):
        return "{} (W{})".format(self.name_str, self.wave_num)


class Formation:
    def __init__(self):
        self.node_dict = dict()  # dict[name] = wave_num
        self.arc_dict = dict()  # dict[(name1, name2)] = distance_m
        self.pose_angle_dict = dict()  # dict[name_str] = angle_deg
        return

    def add_node(self, name, wave):
        """ add each Robot and it's wave to the formation """
        new_node = FormationNode(name, wave)
        self.node_dict[name] = new_node

    def add_arc_cm(self, source_name, target_name, dimension_cm, angle_deg=0):
        """ define the distance from source to target in centimeters """
        self.add_arc(source_name, target_name, dimension_cm/100.0, angle_deg)

    def add_arc(self, source_name, target_name, dimension_m, angle_deg=0):
        """ define the distance from source to target in meters """
        # add validation that:
        # * both are known nodes
        # * that the source is higher wave than target
        self.arc_dict[(source_name, target_name)] = dimension_m
        self.set_pose_angle(source_name, target_name, angle_deg)

    def get_constraint(self, source_name, target_name):
        """ get the distance from source to target in meters """
        if (source_name, target_name) in self.arc_dict.keys():
            return self.arc_dict[(source_name, target_name)]
        return None

    def get_formation_members(self):
        """ get the set of formation member names """
        return set(self.node_dict.keys())

    def get_wave(self, name_str):
        """ the wave number for the given robot """
        return self.node_dict[name_str].wave_num

    def set_pose_angle(self, from_name, to_name, angle_deg):
        """
        OPTIONAL: return the pose angle (degrees) for the formation
        to face the robot towards most likely visible targets
        the angle is defined relative to the bearing to the left target
        """
        self.pose_angle_dict[(from_name, to_name)] = float(angle_deg)

    def get_pose_angle(self, from_name, to_name):
        """
        OPTIONAL: return the pose angle (degrees) for the formation
        to face the robot towards most likely visible targets.
        the angle is defined relative to the bearing to the left target
        """
        if (from_name, to_name) in self.pose_angle_dict.keys():
            return self.pose_angle_dict[(from_name, to_name)]
        else:
            return 0.0

    def __str__(self):
        return "Nodes {}\nGraph {}".format(self.node_dict, self.arc_dict)
