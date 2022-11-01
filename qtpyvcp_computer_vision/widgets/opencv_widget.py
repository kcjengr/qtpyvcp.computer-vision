import os
import sys
import cv2
import math
import numpy as np

from qtpy.QtGui import QImage, QPixmap
from qtpy.QtCore import Qt, QSize, QTimer, Slot
from qtpy.QtWidgets import QLabel


IN_DESIGNER = os.getenv('DESIGNER', False)

class OpenCVWidget(QLabel):

    def __init__(self, parent=None):
        super(OpenCVWidget, self).__init__(parent)

        self.video_size = QSize(320, 240)

        self.setAttribute(Qt.WA_OpaquePaintEvent, True)

        if not IN_DESIGNER:
            root_dir = os.path.dirname(os.path.abspath(__file__))
            logo_path = os.path.abspath(os.path.join(root_dir, os.pardir))

            self.no_video_image = f"{logo_path}/images/no_video.png"

            self._enable_camera = False
            self._enable_edge = False
            self._enable_crosshairs = False
            self._enable_hole_detect = False
            self._enable_slot_detect = False
            self._contour_min_frames = 5
            self._contour_frame_count = 0

            self._video_device = '/dev/video0'

            self._edge_min_threshold = 190
            self._edge_max_threshold = 200

            self._crosshairs_center = list()

            self._hole_dp = 1
            self._hole_min_dist = 20
            self._hole_param1 = 40
            self._hole_param2 = 50
            self._hole_min_radius = 1
            self._hole_max_radius = 20

            self._slot_number = 0
            self._slot_positions = list()
            self._slot_distance = list()
            self._slot_index = 0

            self._line_color = (255, 127, 0)  # R G B

            self._line_thickness = 1
            self._slot_line_color = (255, 0, 0)  # R G B
            self._slot_line_thickness = 3

            self._h_lines = 0
            self._v_lines = 0
            self._c_radius = 25

            self.setPixmap(QPixmap(self.no_video_image))

    # Video

    def setup_camera(self):
        """Initialize camera.
        """

        self.capture = cv2.VideoCapture(self._video_device)

        w = self.capture.get(cv2.CAP_PROP_FRAME_WIDTH)
        h = self.capture.get(cv2.CAP_PROP_FRAME_HEIGHT)

        self.video_size = QSize(int(w), int(h))

        self.setScaledContents(True)
        self.setMinimumSize(int(w), int(h))

        self.timer = QTimer()
        self.timer.timeout.connect(self.display_video_stream)
        self.timer.start(30)

    def minimumSizeHint(self):
        return self.video_size

    def timerEvent(self, QTimerEvent):
        pass

    def display_video_stream(self):
        """Read frame from camera and repaint QLabel widget.
        """
        if self._enable_camera is True:
            if self.capture.isOpened():

                result, frame = self.capture.read()

                if result is True:

                    if self._enable_edge is True:
                        frame = cv2.Canny(frame, self._edge_min_threshold, self._edge_max_threshold)

                        if self._enable_crosshairs is True:
                            self.draw_crosshairs(frame)

                        image = QImage(frame.data, self.video_size.width(), self.video_size.height(),
                                       QImage.Format_Indexed8)

                    else:
                        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                        frame = cv2.flip(frame, 1)

                        if self._enable_crosshairs is True:
                            self.draw_crosshairs(frame)

                        if self._enable_hole_detect is True:
                            self.hole_detect(frame)

                        if self._enable_slot_detect is True:
                            self.slot_detect(frame)

                        image = QImage(frame, frame.shape[1], frame.shape[0],
                                       frame.strides[0], QImage.Format_RGB888)

                    self.setPixmap(QPixmap.fromImage(image))
        else:
            self.capture.release()
            self.setPixmap(QPixmap(self.no_video_image))

    # Helpers

    def calculateDistance(self, x1,y1,x2,y2):
      dist = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
      return dist

    def line_intersection(self, line1, line2):

        xdiff = (line1[0][0] - line1[1][0], line2[0][0] - line2[1][0])
        ydiff = (line1[0][1] - line1[1][1], line2[0][1] - line2[1][1])

        def det(a, b):
            return a[0] * b[1] - a[1] * b[0]

        div = det(xdiff, ydiff)
        if div == 0:
           raise Exception('lines do not intersect')

        d = (det(*line1), det(*line2))
        x = int(det(d, xdiff) / div)
        y = int(det(d, ydiff) / div)

        return x, y

    def draw_crosshairs(self, frame):

        w = self.video_size.width()
        h = self.video_size.height()

        v_line_p1 = int(w / 2) + self._v_lines, 0
        v_line_p2 = int(w / 2) + self._v_lines, h

        h_line_p1 = 0, int(h / 2) - self._h_lines
        h_line_p2 = w, int(h / 2) - self._h_lines

        cv2.line(frame, v_line_p1, v_line_p2, self._line_color, self._line_thickness)
        cv2.line(frame, h_line_p1, h_line_p2, self._line_color, self._line_thickness)

        self._crosshairs_center = self.line_intersection((v_line_p1, v_line_p2), (h_line_p1, h_line_p2))

        if self._c_radius > 1:
            cv2.circle(frame, list(self._crosshairs_center), self._c_radius, self._line_color, self._line_thickness)

    def hole_detect(self, frame):

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        circles = cv2.HoughCircles(gray,
                                   cv2.HOUGH_GRADIENT,
                                   dp=self._hole_dp,
                                   minDist=self._hole_min_dist,
                                   param1=self._hole_param1,
                                   param2=self._hole_param2,
                                   minRadius=self._hole_min_radius,
                                   maxRadius=self._hole_max_radius)
        if circles is not None:
            circles = np.uint16(np.around(circles))
            for i in circles[0,:]:
                cv2.circle(frame, (i[0], i[1]), i[2], (246, 11, 11), 1)
                cv2.circle(frame, (i[0], i[1]), 2, (246, 11, 11), 1)


    def midpoint(self, ptA, ptB):

        return ((ptA[0] + ptB[0]) * 0.5, (ptA[1] + ptB[1]) * 0.5)


    def slot_detect(self, frame):

        self._slot_number = 0
        self._slot_positions.clear()

        img = cv2.pyrMeanShiftFiltering(frame, 21, 51)
        img = cv2.GaussianBlur(img, (5, 5), 0)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        (T, threshInv) = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY_INV | cv2.THRESH_OTSU)
        contours, hierarchy = cv2.findContours(threshInv, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

        for contour in contours:

            area = cv2.contourArea(contour)

            # print(area)

            M = cv2.moments(contour)
            x, y, w, h = cv2.boundingRect(contour)
            rect = cv2.minAreaRect(contour)
            box = cv2.boxPoints(rect)
            box = np.int0(box)

            if M["m00"] > 0:

                self._contour_frame_count += 1
                self._slot_number += 1

                cX = M["m10"] / M["m00"]
                cY = M["m01"] / M["m00"]

                self._slot_positions.append([int(cX), int(cY)])

                cv2.drawContours(frame, contour, -1, (0, 255, 0), 2)
                cv2.circle(frame, (np.int(cX), np.int(cY)), 7, (255, 255, 255), -1)

                # at this point we should have at least 5 frames if not take more
                if self._contour_frame_count < 5:
                    self._contour_frame_count = 0
                    continue

                _ , _ , angle = cv2.fitEllipse(contour)

                # print(cX , cY, angle)

                cv2.drawContours(frame, [box], 0, (0, 0, 255), 2)
                (tl, tr, br, bl) = box

                (tltrX, tltrY) = self.midpoint(tl, tr)
                (blbrX, blbrY) = self.midpoint(bl, br)
                (tlblX, tlblY) = self.midpoint(tl, bl)
                (trbrX, trbrY) = self.midpoint(tr, br)

                cv2.line(frame, (int(tltrX), int(tltrY)), (int(blbrX), int(blbrY)), (0, 255, 255), 1)
                cv2.line(frame, (int(tlblX), int(tlblY)), (int(trbrX), int(trbrY)), (0, 255, 255), 1)

        self._slot_distance.clear()

        for i, slot in enumerate(self._slot_positions):
            pos_x, pos_y = slot

            slot_distance =  self.calculateDistance(self._crosshairs_center[0], self._crosshairs_center[1], pos_x, pos_y)
            self._slot_distance.append(slot_distance)


        nearest_point = min(self._slot_distance)

        if nearest_point <= self._c_radius:
            self._slot_index = self._slot_distance.index(nearest_point)
            nearest_slot = self._slot_positions[self._slot_index]
            cv2.line(frame, self._crosshairs_center, nearest_slot, self._slot_line_color, self._slot_line_thickness)

    # Slots

    @Slot(bool)
    def enableCamera(self, enabled):
        self._enable_camera = enabled
        if enabled is True:
            self.setup_camera()

    @Slot(int)
    def setHorizontalLine(self, value):
        self._h_lines = value

    @Slot(int)
    def setVerticalLine(self, value):
        self._v_lines = value

    @Slot(int)
    def setCenterRadius(self, value):
        self._c_radius = value

    @Slot(bool)
    def enableCrosshairs(self, enabled):
        self._enable_crosshairs = enabled

    @Slot(bool)
    def enableEdge(self, enabled):
        self._enable_edge = enabled

    @Slot(bool)
    def enableHole(self, enabled):
        self._enable_hole_detect = enabled

    @Slot(bool)
    def enableSlot(self, enabled):
        self._enable_slot_detect = enabled

    @Slot(int)
    def setEdgeMinThreshold(self, value):
        self._edge_min_threshold = value

    @Slot(int)
    def setEdgeMaxThreshold(self, value):
        self._edge_max_threshold = value

    @Slot(float)
    def setHoleDp(self, value):
        self._hole_dp = value

    @Slot(float)
    def setHoleMinDist(self, value):
        self._hole_min_dist = value

    @Slot(float)
    def setHoleParam1(self, value):
        self._hole_param1 = value

    @Slot(float)
    def setHoleParam2(self, value):
        self._hole_param2 = value

    @Slot(int)
    def setHoleMinRadius(self, value):
        self._hole_min_radius = value

    @Slot(int)
    def setHoleMaxRadius(self, value):
        self._hole_max_radius = value

    @Slot(str)
    def setVideoDevice(self, path):
        self._video_device = path


if __name__ == "__main__":
    from qtpy.QtWidgets import QApplication

    app = QApplication(sys.argv)
    win = OpenCVWidget()
    win.show()
    sys.exit(app.exec_())

