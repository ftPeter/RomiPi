#!/usr/bin/python3
"""
board_dictionary.py

define a dictionary containing all the tag boards

Peter F. Klemperer
March 18, 2018
"""
import cv2.aruco as aruco
import numpy as np

def get_board_dictionary():
    # create empty dictionary
    board_dict = dict()

    # corner locations in meters
    # small 0.046 inch dual zumo hexagon
    hexagon_corners = [
        np.array([[-0.023, 0.048,0.044], [0.023, 0.048, 0.044],
                  [0.023,0.003,0.044], [-0.023, 0.003, 0.044]], dtype=np.float32),
        np.array([[0.027, 0.048,0.042], [0.050, 0.048, 0.002],
                  [0.050, 0.003,0.002], [0.027, 0.003, 0.042]], dtype=np.float32),
        np.array([[0.050, 0.048,-0.002], [0.027, 0.048,-0.042],
                  [0.028, 0.003,-0.042], [0.050, 0.003,-0.002]], dtype=np.float32),
        np.array([[0.023, 0.048, -0.044], [-0.023, 0.048, -0.044],
                  [-0.023,0.003, -0.044], [0.023, 0.003, -0.044]], dtype=np.float32),
        np.array([[-0.027, 0.048, -0.042], [-0.050, 0.048, -0.002],
                  [-0.050,0.003, -0.002], [-0.027, 0.003, -0.042]], dtype=np.float32),
        np.array([[-0.049, 0.048, 0.002], [-0.027, 0.048, 0.042],
                  [-0.028, 0.003, 0.042], [-0.049, 0.003, 0.002]], dtype=np.float32)]
    # 2 inch zumo octagon
    octagon_two_in_corners = [
        np.array([[-0.020,0.046,0.061], [0.020,0.046,0.061],  [0.020,0.005,0.061],  [-0.020,0.005,0.061] ],dtype=np.float32),
        np.array([[0.029,0.046,0.058],  [0.058,0.046,0.029],  [0.058,0.005,0.029],  [0.029,0.005,0.058]  ],dtype=np.float32),
        np.array([[0.061,0.046,0.020],  [0.061,0.046,-0.020], [0.061,0.005,-0.020], [0.061,0.005,0.020]  ],dtype=np.float32),
        np.array([[0.058,0.046,-0.029], [0.029,0.046,-0.058], [0.029,0.005,-0.058], [0.058,0.005,-0.029] ],dtype=np.float32),
        np.array([[0.020,0.046,-0.061], [-0.020,0.046,-0.061],[-0.020,0.005,-0.061],[0.020,0.005,-0.061] ],dtype=np.float32),
        np.array([[-0.029,0.046,-0.058],[-0.058,0.046,-0.029],[-0.058,0.005,-0.029],[-0.029,0.005,-0.058]],dtype=np.float32),
        np.array([[-0.061,0.046,-0.020],[-0.061,0.046,0.020], [-0.061,0.005,0.020], [-0.061,0.005,-0.020]],dtype=np.float32),
        np.array([[-0.058,0.046,0.029], [-0.029,0.046,0.058], [-0.029,0.005,0.058], [-0.058,0.005,0.029] ],dtype=np.float32)]
    # 2.735 in square face octagonal prism
    # 2.3 in corner tag centered on faces
    octagon_two_three_in_corners = [
        np.array([[-0.029,0.064,0.084], [0.030,0.064,0.084],  [0.030,0.006,0.084],  [-0.029,0.006,0.084] ],dtype=np.float32),
        np.array([[0.039,0.064,0.080],  [0.080,0.064,0.038],  [0.080,0.006,0.038],  [0.039,0.006,0.080]  ],dtype=np.float32),
        np.array([[0.084,0.064,0.029],  [0.084,0.064,-0.030], [0.084,0.006,-0.030], [0.084,0.006,0.029]  ],dtype=np.float32),
        np.array([[0.080,0.064,-0.039], [0.038,0.064,-0.080], [0.038,0.006,-0.080], [0.080,0.006,-0.039] ],dtype=np.float32),
        np.array([[0.029,0.064,-0.084], [-0.030,0.064,-0.084],[-0.030,0.006,-0.084],[0.029,0.006,-0.084] ],dtype=np.float32),
        np.array([[-0.039,0.064,-0.080],[-0.080,0.064,-0.038],[-0.080,0.006,-0.038],[-0.039,0.006,-0.080]],dtype=np.float32),
        np.array([[-0.084,0.064,-0.029],[-0.084,0.064,0.030], [-0.084,0.006,0.030], [-0.084,0.006,-0.029]],dtype=np.float32),
        np.array([[-0.080,0.064,0.039], [-0.038,0.064,0.080], [-0.038,0.006,0.080], [-0.080,0.006,0.039] ],dtype=np.float32)]

    def add_board(name, ids, board_description):
        board_ids = np.array(ids, dtype=np.int32)
        board_dict[name] = aruco.Board_create(board_description,
                                   aruco.getPredefinedDictionary(
                                   aruco.DICT_6X6_250),
                                   board_ids )

    add_board("calumet_board", [[66], [67], [68], [69], [70], [71], [72], [73]], octagon_two_three_in_corners)
    add_board("danube_board", [[74],[75],[76],[77],[78],[79],[80],[81]], octagon_two_three_in_corners)
    add_board("egypt_board", [[82],[83],[84],[85],[86],[87],[88],[89]], octagon_two_three_in_corners)
    add_board("french_board", [[90],[91],[92],[93],[94],[95],[96],[97]], octagon_two_three_in_corners)
    add_board("hoosic_board", [[98],[99],[100],[101],[102],[103],[104],[105]], octagon_two_three_in_corners)
    add_board("port_board", [[21], [22], [23], [24], [25], [26],[27],[28]], octagon_two_three_in_corners)
    add_board("star_board", [[29], [30], [31], [32],[33],[34],[35],[36]], octagon_two_three_in_corners)
    add_board("ivy_board", [[114], [115], [116], [117],[118],[119],[120],[121]], octagon_two_three_in_corners)
    add_board("jiffy_board", [[122], [123], [124], [125],[126],[127],[128],[129]], octagon_two_three_in_corners)
    add_board("kappa_board", [[130], [131], [132], [133],[134],[135],[136],[137]], octagon_two_three_in_corners)
    add_board("luna_board", [[138], [139], [140], [141],[142],[143],[144],[145]], octagon_two_three_in_corners)
    add_board("mars_board", [[146], [147], [148], [149],[150],[151],[152],[153]], octagon_two_three_in_corners)

    # ZUMO BRAVO TAG BOARD
    zumo_bravo_board_ids = np.array(
            [[58], [59], [60], [61], [62], [63], [64], [65]], dtype=np.int32)
    zumo_bravo_board = aruco.Board_create(octagon_two_in_corners,
                                               aruco.getPredefinedDictionary(aruco.DICT_6X6_250),
                                               zumo_bravo_board_ids)
    board_dict["bravo_board"] = zumo_bravo_board
    # ZUMO ALPHA TAG BOARD
    zumo_alpha_board_ids = np.array(
            [[50], [51], [52], [53], [54], [55], [56], [57]], dtype=np.int32)
    zumo_alpha_board = aruco.Board_create(octagon_two_in_corners,
                                               aruco.getPredefinedDictionary(aruco.DICT_6X6_250),
                                               zumo_alpha_board_ids)

    board_dict["alpha_board"] = zumo_alpha_board

    return board_dict

# Self Test
if __name__ == '__main__':
    print( get_board_dictionary() )
