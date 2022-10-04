#   Copyright (c) 2022 Jose Luis
#      <j.l.toledano.l@@gmail.com>
#
#   This file is part of QtPyVCP.
#
#   QtPyVCP is free software: you can redistribute it and/or modify
#   it under the terms of the GNU General Public License as published by
#   the Free Software Foundation, either version 2 of the License, or
#   (at your option) any later version.
#
#   QtPyVCP is distributed in the hope that it will be useful,
#   but WITHOUT ANY WARRANTY; without even the implied warranty of
#   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#   GNU General Public License for more details.
#
#   You should have received a copy of the GNU General Public License
#   along with QtPyVCP.  If not, see <http://www.gnu.org/licenses/>.

import cv2
import numpy as np
import yaml

from qtpy.QtCore import Qt
from qtpy.QtWidgets import QHBoxLayout, QVBoxLayout, QLabel, QPushButton, QFileDialog

from qtpyvcp.utilities import logger
from qtpyvcp.widgets.dialogs.base_dialog import BaseDialog

Log = logger.getLogger(__name__)


class CalibrateCameraDialog(BaseDialog):

    def __init__(self, parent=None):
        
        super(CalibrateCameraDialog, self).__init__(parent=parent, stay_on_top=True)
        
        self.setFixedSize(440, 320)
        self.log = Log

        # opencv stuff
        
        # Video device
        self.video_device = "/dev/video0"
        
        # Caputre
        self.capture = None
        
        # Define the dimensions of self.checkboard
        self.checkboard = (6, 9)
         
        # stop the iteration when specified
        # accuracy, epsilon, is reached or
        # specified number of iterations are completed.
        self.criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
         
        # Vector for 3D points
        self.threedpoints = []
         
        # Vector for 2D points
        self.twodpoints = []
         
        #  3D points real world coordinates
        self.objectp3d = np.zeros((1, self.checkboard[0] * self.checkboard[1],3), np.float32)
        self.objectp3d[0,:,:2] = np.mgrid[0:self.checkboard[0], 0:self.checkboard[1]].T.reshape(-1, 2)
        
        self.prev_img_shape = None

        self.matrix = None
        self.distortion = None
        self.r_vecs = None
        self.t_vecs = None


        self.run_label = QLabel("Calibrate Camera")
        self.save_label = QLabel("Save Results")
        
        self.run_button = QPushButton("Run")
        self.save_button = QPushButton("Save")

        main_layout = QVBoxLayout()

        run_layout = QHBoxLayout()

        run_layout.addWidget(self.run_label)
        run_layout.addWidget(self.run_button)
        
        save_layout = QHBoxLayout()

        save_layout.addWidget(self.save_label)
        save_layout.addWidget(self.save_button)

        main_layout.addLayout(run_layout)
        main_layout.addLayout(save_layout)
        
        self.setLayout(main_layout)
        self.setWindowTitle("Camera Calibration")

        self.save_button.setDisabled(True)

        self.run_button.clicked.connect(self.run_calibration)
        self.save_button.clicked.connect(self.save_results)

    def open_camera(self):
        
        self.log.debug("Open camera device")
        
        # aparently my webcam "showmewebcam on a pi" bugs on opencv 4.6
        self.capture = cv2.VideoCapture(self.video_device)

    def run_calibration(self):
        self.open_camera()
        
        self.log.info("Capturing 32 frames")
        self.log.info("large amount of frames can tale long ...")
        
        for i in range(32):
            if self.capture.isOpened():
                self.save_button.setEnabled(True)
                result, frame = self.capture.read()
        
                if result is True:
                    grayColor = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                 
                    # Find the chess board corners
                    # If desired number of corners are
                    # found in the image then ret = true
                    result, corners = cv2.findChessboardCorners(
                                    grayColor, self.checkboard,
                                    cv2.CALIB_CB_ADAPTIVE_THRESH
                                    +cv2.CALIB_CB_FAST_CHECK + 
                                    cv2.CALIB_CB_NORMALIZE_IMAGE)
                 
                    # If desired number of corners can be detected then,
                    # refine the pixel coordinates and display
                    # them on the images of checker board
                    if result is True:
                        threedpoints.append(objectp3d)
                 
                        # Refining pixel coordinates
                        # for given 2d points.
                        corners2 = cv2.cornerSubPix(
                            grayColor, corners, (11, 11), (-1, -1), criteria)
                 
                        twodpoints.append(corners2)
                 
                        # Draw and display the corners
                        frame = cv2.drawChessboardCorners(frame,
                                                          self.checkboard,
                                                          corners2, ret)
                 
        
                # h, w = frame.shape[:2]
                
                self.log.info("Runing calibration")
                
                # Perform camera calibration by
                # passing the value of above found out 3D points (threedpoints)
                # and its corresponding pixel coordinates of the
                # detected corners (twodpoints)
                
                result, matrix, distortion, r_vecs, t_vecs = cv2.calibrateCamera(self.threedpoints,
                                                                                 self.twodpoints,
                                                                                 grayColor.shape[::-1],
                                                                                 None,
                                                                                 None)
                
                if result is True:
                    
                    self.matrix = matrix
                    self.distortion = ditortion
                    self.r_vecs = r_vecs
                    self.t_vecs = tvecs
                    self.save_button.setEnabled(True)
            else:
                self.save_button.setDisabled(True)


    def save_results(self):
        
        self.log.info("Saving Results to yaml")
        
        options = QFileDialog.Options()
        options |= QFileDialog.DontUseNativeDialog
        
        file_name, _ = QFileDialog.getSaveFileName(self,"QFileDialog.getSaveFileName()","","All Files (*);;Text Files (*.txt)", options=options)
        if file_name:
            with open(file_name, "w") as yaml_file:
                if self.distortion is not None:
                    yaml_file.write(yaml.dump(self.distortion.tolist()))

    def close_method(self):
        
        self.hide()

