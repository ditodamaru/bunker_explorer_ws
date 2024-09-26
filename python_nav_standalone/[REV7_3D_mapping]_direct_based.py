import sys
import os
import signal
import subprocess
from PyQt5 import QtWidgets, QtGui, QtCore

class TerminalLauncher(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()

        self.setupUi()

    def setupUi(self):
        self.setWindowTitle("3D Mapping GUI")

        layout = QtWidgets.QVBoxLayout(self)

        self.labels = [
            "Terminal 1 [Launch Camera]",
            "Terminal 2 [Launch IMU Camera Filter]",
            "Terminal 3 [Launch RTABMAP]",
            "Terminal 4 [Map Saver]",
            "Terminal 5 [Debugging]",
            "Terminal 6 [GPS Launch Script]",
            "Terminal 7 [Record data using ROS Bag]",
            "Terminal 8 [URDF Generator]"
        ]

        self.start_buttons = []

        for label in self.labels:
            layout.addWidget(QtWidgets.QLabel(label))
            start_button = QtWidgets.QPushButton("Start Terminal")
            self.start_buttons.append(start_button)
            start_button.clicked.connect(lambda _, idx=len(self.start_buttons)-1: self.startTerminal(idx))
            layout.addWidget(start_button)

        self.setFocusPolicy(QtCore.Qt.StrongFocus)
        self.keyPressEvent = self.customKeyPressEvent

    def customKeyPressEvent(self, event):
        if event.key() == QtCore.Qt.Key_Enter or event.key() == QtCore.Qt.Key_Return:
            focused_button = self.focusWidget()
            if focused_button in self.start_buttons:
                index = self.start_buttons.index(focused_button)
                self.startTerminal(index)

    def startTerminal(self, index):
        commands = [
            "roslaunch realsense2_camera rs_camera.launch align_depth:=true unite_imu_method:=linear_interpolation enable_gyro:=true enable_accel:=true",
            "rosrun imu_filter_madgwick imu_filter_node _use_mag:=false _publish_tf:=false _world_frame:=enu /imu/data_raw:=/camera/imu /imu/data:=/rtabmap/imu",
            "roslaunch rtabmap_ros rtabmap.launch clear_params:=false depth_topic:=/camera/aligned_depth_to_color/image_raw rgb_topic:=/camera/color/image_raw camera_info_topic:=/camera/color/camera_info approx_sync:=true wait_imu_to_init:=true imu_topic:=/rtabmap/imu", #database_path:=~/.ros/rtabmap2.db
            "rosrun map_server map_saver -f map:=/rtabmap/proj_map", #my_map_1
            "",
            "roslaunch ublox_gps", #ublox_zed-f9p0.launch #roslaunch ublox_gps ublox_device.launch",
            "rosbag record -a",
            "python3 /home/andi/catkin_devel_ws/src/bunker_explorer_bot/bunker_explorer_description/src/generate_urdf.py"
        ]

        if 0 <= index < len(commands):
            if index == 2:  # Terminal 3 (RTABMAP launch)
                custom_arguments, ok = QtWidgets.QInputDialog.getText(self, "Custom Arguments", "Enter custom arguments for Terminal 3:")
                if not ok:
                    return  # User canceled the input dialog, do not proceed with the process
                command = commands[index] + " " + custom_arguments
            elif index == 3:  # Terminal 4 (Map Saver)
                custom_arguments, ok = QtWidgets.QInputDialog.getText(self, "Custom Arguments", "Enter custom arguments for Terminal 4:")
                if not ok:
                    return  # User canceled the input dialog, do not proceed with the process
                command = commands[index] + " " + custom_arguments
            elif index == 4:  # Terminal 4 (Debugging)
                custom_arguments, ok = QtWidgets.QInputDialog.getText(self, "Debugging", "Enter custom arguments for Terminal 5:")
                if not ok:
                    return  # User canceled the input dialog, do not proceed with the process
                command = commands[index] + " " + custom_arguments            
            elif index == 5:  # Terminal 6 (GPS Launch)
                custom_arguments, ok = QtWidgets.QInputDialog.getText(self, "Choose Launch File", "example:ublox_zed-f9p")
                if not ok:
                    return  # User canceled the input dialog, do not proceed with the process
                command = commands[index] + " " + custom_arguments                
            else:
                command = commands[index]

            subprocess.Popen(["gnome-terminal", "--", "bash", "-c", command])

if __name__ == '__main__':
    app = QtWidgets.QApplication(sys.argv)
    launcher = TerminalLauncher()
    launcher.show()
    sys.exit(app.exec_())
