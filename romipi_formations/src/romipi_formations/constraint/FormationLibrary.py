#!/usr/bin/python3
#
# FormationLibrary
#
# This file contains definitions
# for various ball-and-stick formations:
# * test_formation -
# * square_formation -
# * redundant_formation -
# * star_formation -
# * butterfly_formation -
# * wedge_formation - 10-robot triangle with leader at front narrow end
#
# Peter F. Klemperer
# July 29, 2018

from constraint.Formation import Formation

"""
TEST FORMATION
"""

test_formation = Formation()
test_formation.add_node("jiffy_board", 0)
test_formation.add_node("mars_board", 0)
test_formation.add_node("ivy_board", 1)

test_formation.add_arc_cm("ivy_board", "jiffy_board", 50.0, -15.0)
test_formation.add_arc_cm("ivy_board", "mars_board", 50.0, +15.0)

"""
SQUARE FORMATION (sometimes called "base")
"""

square_formation = Formation()
square_formation.add_node("port_board", 0)
square_formation.add_node("star_board", 0)
square_formation.add_node("eqypt_board", 1)
square_formation.add_node("calumet_board", 1)

square_formation.add_arc_cm("eqypt_board", "port_board", 50.0, -15.0)
square_formation.add_arc_cm("eqypt_board", "star_board", 58.0, +15.0)
square_formation.add_arc_cm("calumet_board", "port_board", 58.0, -15.0)
square_formation.add_arc_cm("calumet_board", "star_board", 50.0, +15.0)

"""
REDUNDANT FORMATION
"""

redundant_formation = Formation()
redundant_formation.add_node("port_board", 0)
redundant_formation.add_node("star_board", 0)
redundant_formation.add_node("eqypt_board", 1)
redundant_formation.add_node("calumet_board", 2)

redundant_formation.add_arc_cm("eqypt_board", "port_board", 0.5, -15.0)
redundant_formation.add_arc_cm("eqypt_board", "star_board", 0.58, +15.0)
redundant_formation.add_arc_cm("calumet_board", "egypt_board", 68.893, -45.0)
redundant_formation.add_arc_cm("calumet_board", "port_board", 97.4, -15.0)
redundant_formation.add_arc_cm("calumet_board", "star_board", 83.5, +15.0)

"""
STAR FORMATION
"""

star_formation = Formation()
star_formation.add_node("port_board", 0)
star_formation.add_node("star_board", 0)
star_formation.add_node("eqypt_board", 1)
star_formation.add_node("calumet_board", 1)
star_formation.add_node("brook_board", 2)

star_formation.add_arc_cm("calumet_board", "port_board", 40.00, -15.0)
star_formation.add_arc_cm("calumet_board", "star_board", 58.31, +35.0)

star_formation.add_arc_cm("eqypt_board", "port_board", 58.31, -15.0)
star_formation.add_arc_cm("eqypt_board", "star_board", 40.00, +35.0)

star_formation.add_arc_cm("brook_board", "port_board", 65.28, -10.0)
star_formation.add_arc_cm("brook_board", "star_board", 65.28, +10.0)
star_formation.add_arc_cm("brook_board", "eqypt_board", 40.00, +40.0)
star_formation.add_arc_cm("brook_board", "calumet_board", 40.00, -40.0)

"""
BUTTERFLY FORMATION
"""

butterfly_formation = Formation()
butterfly_formation.add_node("port_board", 0)
butterfly_formation.add_node("star_board", 0)
butterfly_formation.add_node("eqypt_board", 1)
butterfly_formation.add_node("calumet_board", 1)
butterfly_formation.add_node("brook_board", 2)

butterfly_formation.add_arc_cm("calumet_board", "port_board", 54.000, -15.0)
butterfly_formation.add_arc_cm("calumet_board", "star_board", 54.000, +15.0)

butterfly_formation.add_arc_cm("brook_board", "port_board", 121.145, -05.0)
butterfly_formation.add_arc_cm("brook_board", "star_board", 126.000, +05.0)
butterfly_formation.add_arc_cm("brook_board", "calumet_board", 72.000, +03.0)

butterfly_formation.add_arc_cm("eqypt_board", "port_board", 126.00, -05.0)
butterfly_formation.add_arc_cm("eqypt_board", "star_board", 121.145, +05.0)
butterfly_formation.add_arc_cm("eqypt_board", "calumet_board", 72.000, -03.0)

"""
10-Robot Formation: 30 cm Edges
https://docs.google.com/spreadsheets/d/1lRJDjkTz-1QdZPTXt7d_2LoRIjhbmHcVj5nP5GY-QKM/edit?usp=sharing
Also in Fusion 360
"""
wedge_formation = Formation()

wedge_formation.add_node("star_board", 0)
wedge_formation.add_node("port_board", 0)
wedge_formation.add_node("brook_board", 1)
wedge_formation.add_node("calumet_board", 1)
wedge_formation.add_node("danube_board", 2)
wedge_formation.add_node("egypt_board", 2)
wedge_formation.add_node("ivy_board", 2)
wedge_formation.add_node("jiffy_board", 3)
wedge_formation.add_node("kappa_board", 3)
wedge_formation.add_node("luna_board", 3)
wedge_formation.add_node("mars_board", 3)

wedge_formation.add_arc_cm("brook_board", "port_board", 33.193, -0.0)
wedge_formation.add_arc_cm("brook_board", "star_board", 50.48, -0.0)

wedge_formation.add_arc_cm("calumet_board", "port_board", 50.48, -0.0)
wedge_formation.add_arc_cm("calumet_board", "star_board", 33.193, -0.0)

wedge_formation.add_arc_cm("danube_board", "port_board", 79.962, -0.0)
wedge_formation.add_arc_cm("danube_board", "star_board", 89.841, -0.0)
wedge_formation.add_arc_cm("danube_board", "brook_board", 40.0, -0.0)
wedge_formation.add_arc_cm("danube_board", "calumet_board", 79.054, -0.0)

wedge_formation.add_arc_cm("egypt_board", "port_board", 65.576, -0.0)
wedge_formation.add_arc_cm("egypt_board", "star_board", 65.576, -0.0)
wedge_formation.add_arc_cm("egypt_board", "brook_board", 40.00, -0.0)
wedge_formation.add_arc_cm("egypt_board", "calumet_board", 40.00, -0.0)

wedge_formation.add_arc_cm("ivy_board", "port_board", 89.841, -0.0)
wedge_formation.add_arc_cm("ivy_board", "star_board", 71.962, -0.0)
wedge_formation.add_arc_cm("ivy_board", "brook_board", 79.054, -0.0)
wedge_formation.add_arc_cm("ivy_board", "calumet_board", 40.0, -0.0)

wedge_formation.add_arc_cm("jiffy_board", "port_board", 111.603, -0.0)
wedge_formation.add_arc_cm("jiffy_board", "star_board", 129.594, -0.0)
wedge_formation.add_arc_cm("jiffy_board", "brook_board", 80.00, -0.0)
wedge_formation.add_arc_cm("jiffy_board", "calumet_board", 115.647, -0.0)
wedge_formation.add_arc_cm("jiffy_board", "danube_board", 40.00, -0.0)
wedge_formation.add_arc_cm("jiffy_board", "egypt_board", 79.054, -0.0)
wedge_formation.add_arc_cm("jiffy_board", "ivy_board", 124.693, -0.0)

wedge_formation.add_arc_cm("kappa_board", "port_board", 98.724, -0.0)
wedge_formation.add_arc_cm("kappa_board", "star_board", 105.358, -0.0)
wedge_formation.add_arc_cm("kappa_board", "brook_board", 63.838, -0.0)
wedge_formation.add_arc_cm("kappa_board", "calumet_board", 81.256, -0.0)
wedge_formation.add_arc_cm("kappa_board", "danube_board", 40.00, -0.0)
wedge_formation.add_arc_cm("kappa_board", "egypt_board", 40.00, -0.0)
wedge_formation.add_arc_cm("kappa_board", "ivy_board", 78.782, -0.0)

wedge_formation.add_arc_cm("luna_board", "port_board", 105.358, -0.0)
wedge_formation.add_arc_cm("luna_board", "star_board", 98.724, -0.0)
wedge_formation.add_arc_cm("luna_board", "brook_board", 80.00, -0.0)
wedge_formation.add_arc_cm("luna_board", "calumet_board", 63.838, -0.0)
wedge_formation.add_arc_cm("luna_board", "danube_board", 79.054, -0.0)
wedge_formation.add_arc_cm("luna_board", "egypt_board", 40.00, -0.0)
wedge_formation.add_arc_cm("luna_board", "ivy_board", 40.00, -0.0)

wedge_formation.add_arc_cm("mars_board", "port_board", 129.594, -0.0)
wedge_formation.add_arc_cm("mars_board", "star_board", 111.603, -0.0)
wedge_formation.add_arc_cm("mars_board", "brook_board", 115.647, -0.0)
wedge_formation.add_arc_cm("mars_board", "calumet_board", 80.00, -0.0)
wedge_formation.add_arc_cm("mars_board", "danube_board", 124.693, -0.0)
wedge_formation.add_arc_cm("mars_board", "egypt_board", 79.054, -0.0)
wedge_formation.add_arc_cm("mars_board", "ivy_board", 40.00, -0.0)

"""
10-Robot Formation: 50 cm Edges
https://docs.google.com/spreadsheets/d/1lRJDjkTz-1QdZPTXt7d_2LoRIjhbmHcVj5nP5GY-QKM/edit?usp=sharing
Also in Fusion 360
"""

wedge_formation_50cm = Formation()

wedge_formation_50cm.add_node("star_board", 0)
wedge_formation_50cm.add_node("port_board", 0)
wedge_formation_50cm.add_node("brook_board", 1)
wedge_formation_50cm.add_node("calumet_board", 1)
wedge_formation_50cm.add_node("danube_board", 2)
wedge_formation_50cm.add_node("egypt_board", 2)
wedge_formation_50cm.add_node("ivy_board", 2)
wedge_formation_50cm.add_node("jiffy_board", 3)
wedge_formation_50cm.add_node("kappa_board", 3)
wedge_formation_50cm.add_node("luna_board", 3)
wedge_formation_50cm.add_node("mars_board", 3)

wedge_formation_50cm.add_arc_cm("brook_board", "port_board", 44.441, -30.0)
wedge_formation_50cm.add_arc_cm("brook_board", "star_board", 58.949, -30.0)

wedge_formation_50cm.add_arc_cm("calumet_board", "port_board", 58.949, 30.0)
wedge_formation_50cm.add_arc_cm("calumet_board", "star_board", 44.441, 30.0)

wedge_formation_50cm.add_arc_cm("danube_board", "port_board", 93.408, -30.0)
wedge_formation_50cm.add_arc_cm("danube_board", "star_board", 108.282, -30.0)
wedge_formation_50cm.add_arc_cm("danube_board", "brook_board", 50.000, -30.0)
wedge_formation_50cm.add_arc_cm("danube_board", "calumet_board", 86.603, -30.0)

wedge_formation_50cm.add_arc_cm("egypt_board", "port_board", 87.892, -0.0)
wedge_formation_50cm.add_arc_cm("egypt_board", "star_board", 87.892, -0.0)
wedge_formation_50cm.add_arc_cm("egypt_board", "brook_board", 50.000, -0.0)
wedge_formation_50cm.add_arc_cm("egypt_board", "calumet_board", 50.00, -0.0)

wedge_formation_50cm.add_arc_cm("ivy_board", "port_board", 108.282, 30.0)
wedge_formation_50cm.add_arc_cm("ivy_board", "star_board", 93.408, 30.0)
wedge_formation_50cm.add_arc_cm("ivy_board", "brook_board", 86.603, 30.0)
wedge_formation_50cm.add_arc_cm("ivy_board", "calumet_board", 50.0, 30.0)

wedge_formation_50cm.add_arc_cm("jiffy_board", "port_board", 143.091, -20.0)
wedge_formation_50cm.add_arc_cm("jiffy_board", "star_board", 158.035, -20.0)
wedge_formation_50cm.add_arc_cm("jiffy_board", "brook_board", 100.000, -20.0)
wedge_formation_50cm.add_arc_cm("jiffy_board", "calumet_board", 132.288, -20.0)
wedge_formation_50cm.add_arc_cm("jiffy_board", "danube_board", 50.000, -20.0)
wedge_formation_50cm.add_arc_cm("jiffy_board", "egypt_board", 86.603, -20.0)
wedge_formation_50cm.add_arc_cm("jiffy_board", "ivy_board", 132.288, -20.0)

wedge_formation_50cm.add_arc_cm("kappa_board", "port_board", 130.288, -0.0)
wedge_formation_50cm.add_arc_cm("kappa_board", "star_board", 135.923, -0.0)
wedge_formation_50cm.add_arc_cm("kappa_board", "brook_board", 86.603, -0.0)
wedge_formation_50cm.add_arc_cm("kappa_board", "calumet_board", 100.00, -0.0)
wedge_formation_50cm.add_arc_cm("kappa_board", "danube_board", 50.00, -0.0)
wedge_formation_50cm.add_arc_cm("kappa_board", "egypt_board", 50.00, -0.0)
wedge_formation_50cm.add_arc_cm("kappa_board", "ivy_board", 86.603, -0.0)

wedge_formation_50cm.add_arc_cm("luna_board", "port_board", 130.288, -0.0)
wedge_formation_50cm.add_arc_cm("luna_board", "star_board", 135.923, -0.0)
wedge_formation_50cm.add_arc_cm("luna_board", "brook_board", 100.00, -0.0)
wedge_formation_50cm.add_arc_cm("luna_board", "calumet_board", 86.603, -0.0)
wedge_formation_50cm.add_arc_cm("luna_board", "danube_board", 86.603, -0.0)
wedge_formation_50cm.add_arc_cm("luna_board", "egypt_board", 50.00, -0.0)
wedge_formation_50cm.add_arc_cm("luna_board", "ivy_board", 50.00, -0.0)

wedge_formation_50cm.add_arc_cm("mars_board", "port_board", 158.035, 20.0)
wedge_formation_50cm.add_arc_cm("mars_board", "star_board", 143.091, 20.0)
wedge_formation_50cm.add_arc_cm("mars_board", "brook_board", 132.288, 20.0)
wedge_formation_50cm.add_arc_cm("mars_board", "calumet_board", 100.00, 20.0)
wedge_formation_50cm.add_arc_cm("mars_board", "danube_board", 132.288, 20.0)
wedge_formation_50cm.add_arc_cm("mars_board", "egypt_board", 86.603, 20.0)
wedge_formation_50cm.add_arc_cm("mars_board", "ivy_board", 50.00, 20.0)

"""
10-Robot Formation: 60 cm Edges
https://docs.google.com/spreadsheets/d/1lRJDjkTz-1QdZPTXt7d_2LoRIjhbmHcVj5nP5GY-QKM/edit?usp=sharing
Also in Fusion 360
"""
wedge_formation_60cm = Formation()

wedge_formation_60cm.add_node("star_board", 0)
wedge_formation_60cm.add_node("port_board", 0)
wedge_formation_60cm.add_node("brook_board", 1)
wedge_formation_60cm.add_node("calumet_board", 1)
wedge_formation_60cm.add_node("danube_board", 2)
wedge_formation_60cm.add_node("egypt_board", 2)
wedge_formation_60cm.add_node("ivy_board", 2)
wedge_formation_60cm.add_node("jiffy_board", 3)
wedge_formation_60cm.add_node("kappa_board", 3)
wedge_formation_60cm.add_node("luna_board", 3)
wedge_formation_60cm.add_node("mars_board", 3)

wedge_formation_60cm.add_arc_cm("brook_board", "port_board", 54.083, -30.0)
wedge_formation_60cm.add_arc_cm("brook_board", "star_board", 68.739, -30.0)

wedge_formation_60cm.add_arc_cm("calumet_board", "port_board", 68.739, 30.0)
wedge_formation_60cm.add_arc_cm("calumet_board", "star_board", 54.083, 30.0)

wedge_formation_60cm.add_arc_cm("danube_board", "port_board", 54.083, -30.0)
wedge_formation_60cm.add_arc_cm("danube_board", "star_board", 128.16, -30.0)
wedge_formation_60cm.add_arc_cm("danube_board", "brook_board", 60.0, -30.0)
wedge_formation_60cm.add_arc_cm("danube_board", "calumet_board", 103.923, -30.0)

wedge_formation_60cm.add_arc_cm("egypt_board", "port_board", 105.000, -0.0)
wedge_formation_60cm.add_arc_cm("egypt_board", "star_board", 105.000, -0.0)
wedge_formation_60cm.add_arc_cm("egypt_board", "brook_board", 60.000, -0.0)
wedge_formation_60cm.add_arc_cm("egypt_board", "calumet_board", 60.00, -0.0)

wedge_formation_60cm.add_arc_cm("ivy_board", "port_board", 128.16, 30.0)
wedge_formation_60cm.add_arc_cm("ivy_board", "star_board", 54.083, 30.0)
wedge_formation_60cm.add_arc_cm("ivy_board", "brook_board", 103.923, 30.0)
wedge_formation_60cm.add_arc_cm("ivy_board", "calumet_board", 60.0, 30.0)

wedge_formation_60cm.add_arc_cm("jiffy_board", "port_board", 172.988, -20.0)
wedge_formation_60cm.add_arc_cm("jiffy_board", "star_board", 187.949, -20.0)
wedge_formation_60cm.add_arc_cm("jiffy_board", "brook_board", 120.00, -20.0)
wedge_formation_60cm.add_arc_cm("jiffy_board", "calumet_board", 158.745, -20.0)
wedge_formation_60cm.add_arc_cm("jiffy_board", "danube_board", 60.000, -20.0)
wedge_formation_60cm.add_arc_cm("jiffy_board", "egypt_board", 103.923, -20.0)
wedge_formation_60cm.add_arc_cm("jiffy_board", "ivy_board", 158.745, -20.0)

wedge_formation_60cm.add_arc_cm("kappa_board", "port_board", 156.605, -0.0)
wedge_formation_60cm.add_arc_cm("kappa_board", "star_board", 162.25, -0.0)
wedge_formation_60cm.add_arc_cm("kappa_board", "brook_board", 103.923, -0.0)
wedge_formation_60cm.add_arc_cm("kappa_board", "calumet_board", 120.00, -0.0)
wedge_formation_60cm.add_arc_cm("kappa_board", "danube_board", 60.00, -0.0)
wedge_formation_60cm.add_arc_cm("kappa_board", "egypt_board", 60.00, -0.0)
wedge_formation_60cm.add_arc_cm("kappa_board", "ivy_board", 103.923, -0.0)

wedge_formation_60cm.add_arc_cm("luna_board", "port_board", 162.25, -0.0)
wedge_formation_60cm.add_arc_cm("luna_board", "star_board", 156.605, -0.0)
wedge_formation_60cm.add_arc_cm("luna_board", "brook_board", 120.00, -0.0)
wedge_formation_60cm.add_arc_cm("luna_board", "calumet_board", 103.923, -0.0)
wedge_formation_60cm.add_arc_cm("luna_board", "danube_board", 103.923, -0.0)
wedge_formation_60cm.add_arc_cm("luna_board", "egypt_board", 60.00, -0.0)
wedge_formation_60cm.add_arc_cm("luna_board", "ivy_board", 60.00, -0.0)

wedge_formation_60cm.add_arc_cm("mars_board", "port_board", 187.949, 20.0)
wedge_formation_60cm.add_arc_cm("mars_board", "star_board", 172.988, 20.0)
wedge_formation_60cm.add_arc_cm("mars_board", "brook_board", 158.745, 20.0)
wedge_formation_60cm.add_arc_cm("mars_board", "calumet_board", 120.00, 20.0)
wedge_formation_60cm.add_arc_cm("mars_board", "danube_board", 158.745, 20.0)
wedge_formation_60cm.add_arc_cm("mars_board", "egypt_board", 103.923, 20.0)
wedge_formation_60cm.add_arc_cm("mars_board", "ivy_board", 60.00, 20.0)


def convert_formation_to_map(formation, robot_name):
    from constraint.ConstraintMapper import ConstraintMapper
    formation_map = ConstraintMapper()

    for arc in formation.arc_dict.keys():
        (source, target) = arc
        if source == robot_name:
            constraint = formation.get_constraint(source, target)
            offset_rad = formation.get_pose_angle(source, target)
            formation_map.add_constraint(target, constraint)
            formation_map.add_bearing_offset(target, offset_rad)
    return formation_map


def get_formation(formation_name):
    if formation_name == "test_formation" or formation_name == "test":
        return test_formation
    elif formation_name == "square_formation" or formation_name == "square":
        return square_formation
    elif formation_name == "wedge_50cm_formation" or \
            formation_name == "wedge_formation" or \
            formation_name == "wedge":
        return wedge_formation_50cm
    elif formation_name == "wedge_60cm_formation":
        return wedge_formation_60cm
    else:
        print("ERROR: FORMATION {:} not found.".format(formation_name))


def get_wave_number(formation_name, robot_name):
    formation = get_formation(formation_name=formation_name)
    return formation.get_wave(robot_name)


def get_formation_map(formation_name, robot_name):
    formation = get_formation(formation_name=formation_name)
    return convert_formation_to_map(formation, robot_name)


# Self Test
if __name__ == '__main__':
    print("SQUARE Formation")
    print(square_formation)
    print("REDUNDANT Formation")
    print(redundant_formation)
    print("STAR Formation")
    print(star_formation)
    print("BUTTERFLY Formation")
    print(butterfly_formation)
    print("10-ROBOT WEDGE Formation")
    print(wedge_formation)
