from unittest import TestCase

from constraint.Formation import Formation


class TestFormation(TestCase):
    def setUp(self):
        self.line_formation = Formation()
        self.line_formation.add_node("star_board", 0)
        self.line_formation.add_node("port_board", 0)
        self.line_formation.add_node("egypt_board", 1)
        self.line_formation.add_node("calumet_board", 2)
        self.line_formation.add_node("brook_board", 3)

        self.line_formation.add_arc("star_board", "egypt_board", 0.1)
        self.line_formation.add_arc("star_board", "calumet_board", 0.2)
        self.line_formation.add_arc("star_board", "brook_board", 0.3)

        self.line_formation.add_arc("port_board", "egypt_board", 0.1)
        self.line_formation.add_arc("port_board", "calumet_board", 0.2)
        self.line_formation.add_arc("port_board", "brook_board", 0.3)

        self.line_formation.add_arc("egypt_board", "calumet_board", 0.1)
        self.line_formation.add_arc("egypt_board", "brook_board", 0.2)

        self.line_formation.add_arc("calumet_board", "brook_board", 0.1)

    def test_add_node(self):
        self.assertEquals(self.line_formation.node_dict['star_board'].wave_num, 0)
        self.assertEquals(self.line_formation.node_dict['star_board'].name_str, 'star_board')

        self.assertEquals(self.line_formation.node_dict['calumet_board'].wave_num, 2)
        self.assertEquals(self.line_formation.node_dict['calumet_board'].name_str, 'calumet_board')

    def test_add_arc(self):
        self.assertEquals(self.line_formation.arc_dict[('calumet_board', 'brook_board')], 0.1)
        self.assertEquals(self.line_formation.arc_dict[('port_board', 'brook_board')], 0.3)

    def test_get_constraint(self):
        self.assertIsNone(self.line_formation.get_constraint("port_board", "star_board"))
        self.assertIsNone(self.line_formation.get_constraint("calumet_board", "star_board"))
        self.assertAlmostEqual(self.line_formation.get_constraint("star_board", "calumet_board"), 0.2, 1)
        self.assertAlmostEqual(self.line_formation.get_constraint("star_board", "egypt_board"), 0.1, 1)

    def test_get_formation_members(self):
        self.assertSetEqual(self.line_formation.get_formation_members(),
                            {'port_board', 'star_board', 'calumet_board',
                             'brook_board', 'egypt_board'})
