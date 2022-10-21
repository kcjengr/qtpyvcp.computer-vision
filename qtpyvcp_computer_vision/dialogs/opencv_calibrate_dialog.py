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
from qtpy.QtWidgets import QHBoxLayout, QVBoxLayout, QLabel, QPushButton, QFileDialog, QLineEdit

from qtpyvcp.utilities import logger
from qtpyvcp.widgets.dialogs.base_dialog import BaseDialog


LOG = logger.getLogger(__name__)


class CalibrateCameraDialog(BaseDialog):

    def __init__(self, parent=None):

        super(CalibrateCameraDialog, self).__init__(parent=parent, stay_on_top=True)

        self.setFixedSize(480, 340)
        self.log = LOG

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
        self.objectp3d = np.zeros((1, self.checkboard[0] * self.checkboard[1], 3), np.float32)
        self.objectp3d[0,:,:2] = np.mgrid[0:self.checkboard[0], 0:self.checkboard[1]].T.reshape(-1, 2)

        self.frame = None
        self.frame_bw = None

        self.matrix = None
        self.distortion = None
        self.r_vecs = None
        self.t_vecs = None

        self.video_label = QLabel("Device")
        self.video_input = QLineEdit()
        self.video_input.setText("/dev/video0")

        self.run_label = QLabel("Calibrate Camera")
        self.save_label = QLabel("Save Results")

        self.run_button = QPushButton("Run")
        self.save_button = QPushButton("Save")

        main_layout = QVBoxLayout()
        settings_layout = QHBoxLayout()
        run_layout = QHBoxLayout()
        save_layout = QHBoxLayout()

        settings_layout.addWidget(self.video_label)
        settings_layout.addWidget(self.video_input)

        run_layout.addWidget(self.run_label)
        run_layout.addWidget(self.run_button)

        save_layout.addWidget(self.save_label)
        save_layout.addWidget(self.save_button)

        main_layout.addLayout(settings_layout)
        main_layout.addLayout(run_layout)
        main_layout.addLayout(save_layout)

        self.setLayout(main_layout)
        self.setWindowTitle("Camera Calibration")

        # self.open_camera()

        self.save_button.setDisabled(True)

        self.run_button.clicked.connect(self.run_calibration)
        self.save_button.clicked.connect(self.save_results)

    def open_camera(self):

        self.log.debug("Opening camera device")

        # aparently my webcam "showmewebcam on a pi" bugs on opencv 4.6
        self.capture = cv2.VideoCapture(self.video_device)

    def run_calibration(self):
        self.log.info("Capturing 32 frames")
        self.log.info("large amount of frames can take long ...")

        self.open_camera()

        for i in range(32):
            self.log.info("Checking if device is open")
            if self.capture.isOpened():
                self.log.info(f"Capture frame {i}")
                result_1, frame = self.capture.read()

                if result_1 is True:
                    self.log.info("Got frame")
                    self.frame_bw = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

                    # Find the chess board corners
                    # If desired number of corners are
                    # found in the image then ret = true
                    result_2, corners = cv2.findChessboardCorners(
                                    self.frame_bw, self.checkboard,
                                    cv2.CALIB_CB_ADAPTIVE_THRESH
                                    +cv2.CALIB_CB_FAST_CHECK +
                                    cv2.CALIB_CB_NORMALIZE_IMAGE)

                    # If desired number of corners can be detected then,
                    # refine the pixel coordinates and display
                    # them on the images of checker board
                    if result_2 is True:
                        self.threedpoints.append(self.objectp3d)

                        # Refining pixel coordinates
                        corners2 = cv2.cornerSubPix(self.frame_bw, corners, (11, 11), (-1, -1), self.criteria)

                        self.twodpoints.append(corners2)

                        # Draw and display the corners
                        self.frame = cv2.drawChessboardCorners(self.frame, self.checkboard, corners2, result_2)
                else:
                    self.save_button.setDisabled(True)
            else:
                self.log.info("Error getting data")

        if self.frame is not None:
            h, w = self.frame.shape[:2]

        # Perform camera calibration by
        # passing the value of above found out 3D points (threedpoints)
        # and its corresponding pixel coordinates of the
        # detected corners (twodpoints)

        result_3, matrix, distortion, r_vecs, t_vecs = cv2.calibrateCamera(self.threedpoints, self.twodpoints, self.frame_bw.shape[::-1], None,  None)

        if result_3 is True:

            self.matrix = matrix
            self.distortion = ditortion
            self.r_vecs = r_vecs
            self.t_vecs = tvecs
            self.capture.release

            self.log.debug("Enable save, got results")
            self.save_button.setEnabled(True)
        else:
            self.log.debug("Disable save, no results")
            self.save_button.setDisabled(True)


    def save_results(self):

        self.log.info("Saving Results to yaml")

        options = QFileDialog.Options()
        options |= QFileDialog.DontUseNativeDialog

        file_name, _ = QFileDialog.getSaveFileName(self, "QFileDialog.getSaveFileName()", "", "All Files (*);;Text Files (*.txt)", options=options)
        if file_name:
            with open(file_name, "w") as yaml_file:
                if self.distortion is not None:
                    yaml_file.write(yaml.dump(self.distortion.tolist()))

    def close_method(self):

        self.hide()
