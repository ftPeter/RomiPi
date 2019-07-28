#!/usr/bin/python3
#
# ConstraintMapper
#
# tool for storing a retrieving
# navigation constraints
#
# Peter F. Klemperer
# July 27, 2018

class ConstraintMapper:
    def __init__(self):
        self.constraint_dict = dict()
        self.angle_dict = dict()
        self.boardname_priority = list()
        return

    def topTwoTargets(self, boardname_list):
        topTargets = []
        for boardname in self.boardname_priority:
            if boardname in boardname_list:
                topTargets.append(boardname)
        if len(topTargets) >= 2:
            return (topTargets[0], topTargets[1])
        return None

    def add_constraint(self, name, dimension):
        self.constraint_dict[name] = dimension
        self.boardname_priority.append(name)

    def add_bearing_offset(self, name, angle_rad):
        self.angle_dict[name] = angle_rad

    def get(self, boardname_list):
        targets = self.topTwoTargets(boardname_list)
        if targets is None:
            return None
        (b1, b2) = targets
        d1 = self.constraint_dict[b1]
        d2 = self.constraint_dict[b2]
        return (b1, d1, b2, d2)

    def get_bearing_offset(self, target):
        bearing_offset_in_rad = self.angle_dict[target]
        return bearing_offset_in_rad

    def __str__(self):
        return str(self.constraint_dict)


import unittest


class TestCM(unittest.TestCase):
    def test_cm_constructor(self):
        constraint_map = ConstraintMapper()
        constraint_map.add("star_board", 50)
        constraint_map.add("port_board", 60)
        constraint_map.add("egypt_board", 70)

        self.assertTrue(constraint_map.get(["calumet_board", "star_board", "port_board"]) ==
                        ('star_board', 50, 'port_board', 60))
        self.assertTrue(constraint_map.get(["egypt_board", "star_board", "calumet_board"]) ==
                        ('star_board', 50, 'egypt_board', 70))
        self.assertTrue(constraint_map.get(["egypt_board", "danube_board", "calumet_board"]) ==
                        None)
        return


# Self Test
if __name__ == '__main__':
    import unittest

    print("UNIT TESTING: CONSTRAINTMAPPER.PY BEGIN")
    unittest.main()
