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
        self.setWindowTitle("AutoDriving Visual SLAM")

        layout = QtWidgets.QVBoxLayout(self)

        self.labels = [
            #"Terminal 1 [Source ROS Environment]",
            "Terminal 2 [Start Robot CAN Communication]",
            "Terminal 3 [Launc, GPS, IMU, Bunker Robot]",
            "Terminal 3a [Launch Camera]",            
            #"Terminal 4 [Robot Localization]",
            "Terminal 5 [Robot Localization and Navsat Transform]",
            "Terminal 5a [Robot Localization withour Navsat Transform]", 
            "Terminal 6 [Launch RTABMAP]",
            "Terminal 7 [RTABMAP Localization Mode]",
            "Terminal 8 [RTABMAP Mapping Mode]",
            "Terminal 8 [Launch Keyboard Control]",
            "Terminal 8a [Launch Move Base Default]",
            "Terminal 9 [Editor Map Saver launch file]",
            "Terminal 10 [Map Saver]",
            #"Terminal 11 [Debugging]",
            "Terminal 12 [Launch click and save 2D Nav Goal]",
            #"Terminal 13  [Launch Map chooser]",
            #"Terminal 13a [Launch Bunker3_navigation]",
            #"Terminal 14 [Launch Bunker3_navigation without Map]",
            #"Terminal 15 [Launch Goal File Chooser]",
            "Terminal 15a [Send Goals]",
            "Terminal 16a [COnverter Navgoal]",
            "Terminal 16b [Tracking_pid_Bunker]",
            "Terminal 17 [Publish Path]"
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
            #"source ~/.bashrc",
            "rosrun bunker_bringup bringup_can2usb.bash",
            "roslaunch bunker_explorer_description robot_base_bringup.launch",
            "roslaunch bunker_explorer_perception start_rs_camera.launch",
            #"roslaunch bunker_explorer_control robot_control_ekf.launch",
            #"roslaunch bunker_explorer_control localization_run_navsat.launch",
            "roslaunch bunker_explorer_waypoint localization_run.launch",
            "roslaunch bunker_explorer_control localization_run_no_navsat.launch",             
            "roslaunch bunker_explorer_perception rtab_mapping.launch", #database_path:=~/.ros/rtabmap2.db
            "rosservice call rtabmap/set_mode_localization", #rosservice call /rtabmap/reset_odom
            "rosservice call rtabmap/set_mode_mapping",       
            "roslaunch bunker_explorer_description robot_base_teleopkey.launch",
            "roslaunch bunker_explorer_navigation move_base_default.launch", 
            #"rosrun map_server map_saver -f map:=/rtabmap/proj_map", #my_map_1
            "roslaunch bunker_explorer_navigation editor_map_saver_launch.launch",
            "roslaunch bunker_explorer_navigation map_saver.launch",
            #"", #rosservice call /rtabmap/reset_odom
            "roslaunch bunker_explorer_waypoint collect_2dnav_goal.launch",
            #"roslaunch bunker_explorer_navigation bunker3_map_chooser.launch",
            #"roslaunch bunker_explorer_navigation bunker3_navigation.launch",
            #"roslaunch bunker_explorer_navigation bunker3_navigation_nomap.launch",
            "roslaunch bunker_explorer_waypoint send_goals.launch",
            "roslaunch bunker_explorer_waypoint converter_navgoal.launch",
            "roslaunch tracking_pid test_tracking_pid_bunker.test rviz:=false",
            "roslaunch tracking_pid test_publishing_path.test"
        ] 

        if 0 <= index < len(commands):
            if index == 5:  # Terminal 3 (RTABMAP launch)
                custom_arguments, ok = QtWidgets.QInputDialog.getText(self, "Custom Arguments", "Enter custom arguments :")
                if not ok:
                    return  # User canceled the input dialog, do not proceed with the process
                command = commands[index] + " " + custom_arguments
            elif index == 6:  # Terminal 4 (Map Saver)
                custom_arguments, ok = QtWidgets.QInputDialog.getText(self, "Custom Arguments", "Enter custom arguments :")
                if not ok:
                    return  # User canceled the input dialog, do not proceed with the process
                command = commands[index] + " " + custom_arguments
            elif index == 7:  # Terminal 4 (Map Saver)
                custom_arguments, ok = QtWidgets.QInputDialog.getText(self, "Custom Arguments", "Enter custom arguments :")
                if not ok:
                    return  # User canceled the input dialog, do not proceed with the process
                command = commands[index] + " " + custom_arguments
            elif index == 8:  # Terminal 4 (Debugging)
                custom_arguments, ok = QtWidgets.QInputDialog.getText(self, "Custom Arguments", "Enter custom arguments :")
                if not ok:
                    return  # User canceled the input dialog, do not proceed with the process
                command = commands[index] + " " + custom_arguments            
            elif index == 9:  # Terminal 6 (GPS Launch)
                custom_arguments, ok = QtWidgets.QInputDialog.getText(self, "Custom Arguments", "Enter custom arguments :")
                if not ok:
                    return  # User canceled the input dialog, do not proceed with the process
                command = commands[index] + " " + custom_arguments

            else:
                command = commands[index]

            #subprocess.Popen(["gnome-terminal", "--", "bash", "-c", command])

            try:
                #subprocess.Popen(["gnome-terminal", "--", "bash", "-c", command])
                print("Executing command:", command)
                subprocess.Popen(["gnome-terminal", "--", "bash", "-c", command])

            except Exception as e:
                print(f"Error: {e}")            

if __name__ == '__main__':
    app = QtWidgets.QApplication(sys.argv)
    launcher = TerminalLauncher()
    launcher.show()
    sys.exit(app.exec_())
