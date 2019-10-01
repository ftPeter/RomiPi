#!/usr/bin/env python
#
# TagboardBoardPoser
#
# tool for identifying and calculating
# the distance to other robots using
# aruco tag boards
#
# Peter F. Klemperer
# February 10, 2018
#
# Potential next features:
# * thread image capture
# * demo threaded image display
# * complete board dictionary

import cv2
import numpy as np
import math
import cv2.aruco as aruco

from time import sleep
import threading

from romipi_fiducials.pose import Pose

VERBOSE = False

class BoardPoser:
    def __init__(self):
        # DEFINE TAG BOARDS
        from romipi_fiducials.board_dictionary import get_board_dictionary
        self.board_dict = get_board_dictionary()
        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
        
        # CAMERA SETUP
        # FIXME make this a proper ROS parameter
        # FIXME do I have to re-calibrate when using the raspberry pi node 
        #       instead of direct camera?
        camera_param_file = "/home/mhc/catkin_ws/src/RomiPi/romipi_fiducials/cameraParameters-PK-RasberryPi-Camera-8MP.xml"
        # configure cv2 aruco handling
        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
        camera_reader = cv2.FileStorage()
        camera_reader.open(camera_param_file, cv2.FileStorage_READ)
        # camera configurations
        self.camera_matrix = self.read_node_matrix(camera_reader, "cameraMatrix")
        self.dist_coeffs   = self.read_node_matrix(camera_reader, "dist_coeffs")
        width, height = self.read_resolution(camera_reader) # FIXME seems unused

        if self.camera_matrix is None or self.dist_coeffs is None:
            print("camera configurations not loaded.")

        if VERBOSE :
            print("camera matrix : %s" % self.camera_matrix)
            print("distance coeffs : %s" % self.dist_coeffs)
            print("width, height : %d, %d" % (width,height))

        return

    @staticmethod
    def read_resolution(reader):
        node = reader.getNode("cameraResolution")
        return int(node.at(0).real()), int(node.at(1).real())
 
    @staticmethod
    def read_node_matrix(reader, name):
        node = reader.getNode(name)
        return node.mat()

    def get_board_list(self):
        return self.get_board_dictionary().values()

    def process_frame(self, frame):
        """ read a new frame and process the frame. """
        self.unprocessed_frame = frame
        self.processed_frame = None
        self.is_processed()

    def is_processed(self):
        """
        process frame for markers. Generally not for users.

        This is written this way to allow users to always get the most updated results.
        FIXME may not be necessary to have this gated. putting this on the user is probably okay.
        """
        frame = self.unprocessed_frame
        if self.processed_frame is None:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)  #BGR2GRAY)
            self.visible_corners, self.visible_ids, rejectedImgPoints = aruco.detectMarkers(gray, self.aruco_dict)
            self.processed_frame = gray

    def get_frame(self):
        self.is_processed()
        return self.processed_frame

    def get_visible_ids(self):
        self.is_processed()
        return self.visible_ids

    def get_visible_corners(self):
        self.is_processed()
        return self.visible_corners

    def get_visible_board_set(self):
        self.is_processed()

        visible_boards_set = set()
        for board in self.get_board_list():
            for id in board.ids.tolist():
                if( self.is_id_visible( id ) ):
                    visible_boards_set.add( board )
        return visible_boards_set

    def get_visible_boardnames(self):
        self.is_processed()

        visible_board_list = list(self.get_visible_board_set())
        visible_boardnames_list = []
        for visible_board in visible_board_list:
            for name in self.get_board_dictionary().keys():
                if visible_board is self.get_board_dictionary()[name]:
                    visible_boardnames_list.append(name)
        return visible_boardnames_list

    def is_id_visible(self, id_num):
        self.is_processed()
        # ids is empty, then fail
        if(self.get_visible_ids() is None):
            return False
        # given id list, check for id_num in ids
        if(id_num in self.get_visible_ids()):
            return True
        # all other possibilities fail
        return False

    def get_board_dictionary(self):
        return self.board_dict

    def get_board(self, board_name_str):
        self.is_processed()

        dictionary = self.get_board_dictionary()
        return dictionary[board_name_str]

    def get_boardname_bearing_range(self, board_name_str):
        ''' user calls this with board name '''
        self.is_processed()

        board = self.get_board(board_name_str)
        if board:
            return self.get_board_bearing_range( board )
        return None,None

    def get_visible_ids_corners_lists(self, board):
        detected_board_ids     = list()
        detected_board_corners = list()

        if self.get_visible_ids() is None:
            return (None, None)

        # grab visible ids and corners for given board
        for vis_id, vis_corner in zip(self.get_visible_ids(), self.get_visible_corners()):
            if vis_id in board.ids.tolist():
                detected_board_ids.append(vis_id)
                detected_board_corners.append(vis_corner)
        return (detected_board_ids,detected_board_corners)

    def board_pose(self, board, corners, ids):
        retval, rvec, tvec = aruco.estimatePoseBoard(corners,
                                                     ids, board,
                                                     self.camera_matrix, self.dist_coeffs)
        # draw the boards origin
        # note: axis_length_inches is only related to drawing
        # the image of the axis, not any pose estimation
        draw_axis_length_m = 0.02
        self.processed_frame = aruco.drawAxis(
            self.processed_frame,
            self.camera_matrix,
            self.dist_coeffs,
            rvec,
            tvec,
            draw_axis_length_m)

        # translate TVEC from camera origin to robot origin
        CAMERA_OFFSET_M = [[0], [0.085], [0.084]]
        camera_tvec = np.asarray( CAMERA_OFFSET_M )
        corrected_tvec = np.add( tvec, camera_tvec )
        return retval, rvec, corrected_tvec

    def get_board_bearing_range(self, board):
        self.is_processed()

        (detected_board_ids,detected_board_corners) = self.get_visible_ids_corners_lists(board)

        # if board was visible, return bearing and range
        if(detected_board_ids):
            # convert ids and corners to numpy format arrays
            detected_board_ids = np.asarray(detected_board_ids)
            detected_board_corners = np.asarray(detected_board_corners)
            retval, rvec, tvec = self.board_pose(board, detected_board_corners, detected_board_ids)

            # calculate distance to the board
            range_to_board = np.linalg.norm(tvec)

            # calculate bearing to board in the XZ-plane
            # in degrees relative to Z (right pos, left neg)
            # this may not make sense, expect to change.
            x = tvec[0][0]
            z = tvec[2][0]
            bearing_to_board = np.degrees(np.arctan2(x, z))
            return (bearing_to_board, range_to_board)

        # board not visible, distance unknown
        return None, None

    def get_visible_board_poses(self):
        ''' return visible boards and poses '''
        self.is_processed()
        
        board_pose_dict = dict()

        for boardname in self.get_visible_boardnames():
            pose = self.get_boardname_pose(boardname)
            board_pose_dict[boardname] = pose

        return board_pose_dict

    # TODO test this with the robot
    # Return the pose in CM
    def get_boardname_pose(self, board_name_str):
        ''' get board poses one-at-a-time '''
        self.is_processed()

        board = self.get_board(board_name_str)
        if (board is not None):
            return self.get_board_pose(board)
        return None

    def get_board_pose(self, board):
        self.is_processed()

        (detected_board_ids, detected_board_corners) = self.get_visible_ids_corners_lists(board)

        # if board was visible, return bearing and range
        if (detected_board_ids):
            # convert ids and corners to numpy format arrays
            detected_board_ids = np.asarray(detected_board_ids)
            detected_board_corners = np.asarray(detected_board_corners)
            retval, rvec, tvec = self.board_pose(board, detected_board_corners, detected_board_ids)

            # calculate distance to the board
            board_rho = np.linalg.norm(tvec)

            # calculate bearing to board in the XZ-plane
            # in degrees relative to Z (right pos, left neg)
            # this may not make sense, expect to change.
            #
            # tvec and rvec are in the opencv coordinate
            # system where z is forward and x is left
            # and convert to cm for storage
            x = Pose.m_to_cm(tvec[0][0] * -1.0)
            z = Pose.m_to_cm(tvec[2][0])
            board_alpha_deg = -1 * np.degrees(np.arctan2(x, z))

            # calculate board pose
            rmat = cv2.Rodrigues(rvec)[0]
            (a, b, c) = BoardPoser.rotationMatrixToEulerAngles(rmat)
            # print("a ", a*180/3.1415, "b ", b*180/3.1415, "c ", c*180/3.1415)
            theta_deg = math.degrees(b) * 1.0

            # store pose in robot-coordinate system where x is forward
            # and y is left
            return Pose( z, x, theta_deg )

        # board not visible, distance unknown
        return None

    def get_boardname_pab(self, board_name_str):
        self.is_processed()

        board = self.get_board(board_name_str)
        if(board is not None):
            return self.get_board_pab( board )
        return None,None,None

    def get_board_pab(self, board):
        self.is_processed()

        (detected_board_ids,detected_board_corners) = self.get_visible_ids_corners_lists(board)

        # if board was visible, return bearing and range
        if(detected_board_ids):
            # convert ids and corners to numpy format arrays
            detected_board_ids     = np.asarray(detected_board_ids)
            detected_board_corners = np.asarray(detected_board_corners)
            retval, rvec, tvec = self.board_pose(board, detected_board_corners, detected_board_ids)

            # calculate distance to the board
            board_rho = np.linalg.norm(tvec)

            # calculate bearing to board in the XZ-plane
            # in degrees relative to Z (right pos, left neg)
            # this may not make sense, expect to change.
            x = tvec[0][0]
            z = tvec[2][0]
            board_alpha_deg = -1*np.degrees(np.arctan2(x, z))

            # calculate board pose
            rmat = cv2.Rodrigues(rvec)[0]
            (a,b,c) = BoardPoser.rotationMatrixToEulerAngles(rmat)
            #print("a ", a*180/3.1415, "b ", b*180/3.1415, "c ", c*180/3.1415)
            board_beta_deg = math.degrees(-1*b)

            return (board_rho, board_alpha_deg, board_beta_deg)

        # board not visible, distance unknown
        return None, None, None

    # Checks if a matrix is a valid rotation matrix.
    @staticmethod
    def isRotationMatrix(R) :
        Rt = np.transpose(R)
        shouldBeIdentity = np.dot(Rt, R)
        I = np.identity(3, dtype = R.dtype)
        n = np.linalg.norm(I - shouldBeIdentity)
        return n < 1e-6

    @staticmethod
    def rotationMatrixToEulerAngles(R) :
        """
        Calculates rotation matrix to euler angles
        The result is the same as MATLAB except the order
        of the euler angles ( x and z are swapped ).

        returns euler angles in radians
        """
        import math
        assert(BoardPoser.isRotationMatrix(R))

        sy = math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])

        singular = sy < 1e-6

        if  not singular :
            x = math.atan2(R[2,1] , R[2,2])
            y = math.atan2(-R[2,0], sy)
            z = math.atan2(R[1,0], R[0,0])
        else :
            x = math.atan2(-R[1,2], R[1,1])
            y = math.atan2(-R[2,0], sy)
            z = 0

        return np.array([x, y, z])

# TagboardBoardPoser Self Test
if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description='print 2-d pose of tagboards')
    parser.add_argument('--csv', dest='csv', action='store_true',
                        default=False, help="print results in csv format")
    parser.add_argument('--dual', dest='dual', action='store_true',
                        default=False, help="print dual tagboard target")
    parser.add_argument('--single', dest='single', action='store_true',
                        default=False, help="print single tagboard target")
    args = parser.parse_args()

    if args.csv:
        print("\"board name\",X cm,Y cm,Theta deg")

    # don't try to open windows
    # when working without GUI
    from os import getenv
    display = getenv("DISPLAY")
    if display:
        DISPLAY = True
    else:
        DISPLAY = False

    # open board_poser
    board_poser = BoardPoser()

    try:
        # FIXME re-write this to subscribe to raspberrypi cam and print out all visible board poses
        while(True):
            sleep(1.0/4.0)
            board_poser.process_frame()

            from boards.board_dictionary import get_board_dictionary
            danube = board_poser.get_boardname_pose("danube_board")
            hoosic = board_poser.get_boardname_pose("hoosic_board")
            port = board_poser.get_boardname_pose("port_board")
            star = board_poser.get_boardname_pose("star_board")
            from control.targeting import get_target_dual_leader_pose

            # reset the current pose
            if args.dual and danube and hoosic:
                print("dual leader target: ", get_target_dual_leader_pose(danube,hoosic))
            if args.dual and port and star:
                print("dual leader target: ", get_target_dual_leader_pose(port, 58.31, star, 50.00))

            for boardname in get_board_dictionary().keys():
                pose = board_poser.get_boardname_pose(boardname)
                if args.csv and pose is not None:
                    print("\"{}\",{:0.4f},{:0.4f},{:0.4f}".format(boardname,pose.getX(),
                                                                                 pose.getY(),pose.getTheta()))
                elif not args.csv:
                    if pose is not None:
                        print(boardname, board_poser.get_boardname_pose(boardname), "range = {:0.2f}".format(pose.getRange()))
                    else:
                        print(boardname, board_poser.get_boardname_pose(boardname))

            if DISPLAY:
                frame = board_poser.get_frame()
                cv2.imshow('frame', frame)
                if( cv2.waitKey(1) & 0xFF == ord('q') ):
                    break
    except(KeyboardInterrupt):
        print( "board_poser done now." )

    cv2.destroyAllWindows()

