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
        self.setWindowTitle("AutoNav Visual SLAM")

        layout = QtWidgets.QVBoxLayout(self)

        self.labels = [
            "Terminal 1 [Source ROS Environment]",
            "Terminal 2 [Start Robot CAN Communication]",
            "Terminal 3 [Launch Camera]",
            "Terminal 4 [Bringup Bunker]",            
            "Terminal 6 [EKF Localization]",
            "Terminal 7 [Launch RTABMAP]",
            "Terminal 7 [Launch RTABMAP Localization]",
            "Terminal 8 [Launch Keyboard Control]",
            "Terminal 9 [Map Saver]",
            "Terminal 10 [Debugging]",
            "Terminal 11 [GPS Launch Script]",
            "Terminal 12 [Perform MoveBase-original]",
            "Terminal 12 [Perform MoveBase-husky]",
            "Terminal 13 [Launch Heading Calibration]",
            "Terminal 14 [Launch Collect GPS Waypoints]",
            "Terminal 15 [Launch Joystick Control]",
            "Terminal 16 [Perform Send Goals]",
            "Terminal 17 [Select Bunker URDF file to launch]",
            "Terminal 18 [Launch Plot GPS]",
            "Terminal 19 GPS Move Base"
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
            "source ~/.bashrc",
            "rosrun bunker_bringup bringup_can2usb.bash",
            "roslaunch bunker_explorer_perception start_rs_camera.launch",
            "roslaunch bunker_explorer_description robot_base_bringup.launch",
            #"roslaunch bunker_explorer_control robot_control_ekf.launch",
            #"roslaunch bunker_explorer_control localization_run.launch",
            "roslaunch bunker_explorer_waypoint localization_run.launch",
            "roslaunch bunker_explorer_perception rtab_mapping.launch", #database_path:=~/.ros/rtabmap2.db
            "roslaunch bunker_explorer_perception rtab_mapping.launch  localization:=true",
            "roslaunch bunker_explorer_description robot_base_teleopkey.launch",
            #"rosrun map_server map_saver -f map:=/rtabmap/proj_map", #my_map_1
            "roslaunch bunker_explorer_navigation map_saver.launch",
            "",
            #"roslaunch ublox_gps", #ublox_zed-f9p0.launch #roslaunch ublox_gps ublox_device.launch",
            #"roslaunch ntrip_ros ntrip_ros.launch",
            "roslaunch ~/catkin_redevel_ws/src/ntrip_ros/launch/ntrip_ros.launch",
            #"roslaunch bunker_explorer_navigation rev1_move_base.launch"
            "roslaunch bunker_explorer_navigation rev3_bunker_nav_amcl.launch",
            "roslaunch bunker_explorer_navigation rev1_move_base_gps_husky.launch",
            "roslaunch gps_waypoint_nav heading_calibration.launch",
            "roslaunch gps_waypoint_nav collect_goals.launch",
            "roslaunch gps_waypoint_nav joy_launch_control.launch",
            "roslaunch bunker_explorer_waypoint send_goals.launch",
            #"python3 ~/catkin_devel_ws/src/bunker_explorer_bot/bunker_explorer_description/src/rev1_launch_editor.py",
            "roslaunch gps_waypoint_nav plot_gps_data.launch", 
            "roslaunch bunker_explorer_navigation rev1_move_base_gps.launch"

        ]

        if 0 <= index < len(commands):
            if index == 5:  # Terminal 3 (RTABMAP launch)
                custom_arguments, ok = QtWidgets.QInputDialog.getText(self, "Custom Arguments", "Enter custom arguments for Terminal 3:")
                if not ok:
                    return  # User canceled the input dialog, do not proceed with the process
                command = commands[index] + " " + custom_arguments
            elif index == 6:  # Terminal 4 (Map Saver)
                custom_arguments, ok = QtWidgets.QInputDialog.getText(self, "Custom Arguments", "Enter custom arguments for Terminal 4:")
                if not ok:
                    return  # User canceled the input dialog, do not proceed with the process
                command = commands[index] + " " + custom_arguments
            elif index == 7:  # Terminal 4 (Map Saver)
                custom_arguments, ok = QtWidgets.QInputDialog.getText(self, "Custom Arguments", "Enter custom arguments for Terminal 4:")
                if not ok:
                    return  # User canceled the input dialog, do not proceed with the process
                command = commands[index] + " " + custom_arguments
            elif index == 8:  # Terminal 4 (Debugging)
                custom_arguments, ok = QtWidgets.QInputDialog.getText(self, "Debugging", "Enter custom arguments for Terminal 5:")
                if not ok:
                    return  # User canceled the input dialog, do not proceed with the process
                command = commands[index] + " " + custom_arguments            
            elif index == 9:  # Terminal 6 (GPS Launch)
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
