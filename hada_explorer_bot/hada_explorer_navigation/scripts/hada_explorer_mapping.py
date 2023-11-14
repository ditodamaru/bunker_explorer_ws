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
        self.setWindowTitle("HADA 3D Mapping")

        layout = QtWidgets.QVBoxLayout(self)

        self.labels = [
            #"Terminal 1 [Source ROS Environment]",
            "Terminal 1 [Start Robot CAN Communication]",
            "Terminal 2 [Launch HADA Robot]",
            "Terminal 3 [Launch Camera]",            
            #"Terminal 4 [Robot Localization]",
            #"Terminal 4 [Robot Localization and Navsat Transform]",
            #"Terminal 5a [Robot Localization withour Navsat Transform]", 
            "Terminal 4 [Launch RTABMAP]",
            "Terminal 5 [RTABMAP Localization Mode]",
            "Terminal 6 [RTABMAP Mapping Mode]",
            "Terminal 7 [Launch Keyboard Control]",
            "Terminal 8 [Map Saver]",

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
            "roslaunch hada_explorer_description hada_base_bringup.launch",
            "roslaunch hada_explorer_perception start_rs_camera.launch",          
            "roslaunch hada_explorer_perception rtab_mapping.launch", #database_path:=~/.ros/rtabmap2.db
            "rosservice call rtabmap/set_mode_localization", #rosservice call /rtabmap/reset_odom
            "rosservice call rtabmap/set_mode_mapping",       
            "roslaunch hada_explorer_description robot_base_teleopkey.launch",
            "roslaunch hada_explorer_navigation map_saver.launch",
        ] 

        if 0 <= index < len(commands):
            if index == 3:  # Terminal 3 (RTABMAP launch)
                custom_arguments, ok = QtWidgets.QInputDialog.getText(self, "Custom Arguments", "Enter custom arguments :")
                if not ok:
                    return  # User canceled the input dialog, do not proceed with the process
                command = commands[index] + " " + custom_arguments
            elif index == 4:  # Terminal 4 (Map Saver)
                custom_arguments, ok = QtWidgets.QInputDialog.getText(self, "Custom Arguments", "Enter custom arguments :")
                if not ok:
                    return  # User canceled the input dialog, do not proceed with the process
                command = commands[index] + " " + custom_arguments
            elif index == 5:  # Terminal 4 (Map Saver)
                custom_arguments, ok = QtWidgets.QInputDialog.getText(self, "Custom Arguments", "Enter custom arguments :")
                if not ok:
                    return  # User canceled the input dialog, do not proceed with the process
                command = commands[index] + " " + custom_arguments
            elif index == 6:  # Terminal 4 (Debugging)
                custom_arguments, ok = QtWidgets.QInputDialog.getText(self, "Custom Arguments", "Enter custom arguments :")
                if not ok:
                    return  # User canceled the input dialog, do not proceed with the process
                command = commands[index] + " " + custom_arguments            
            elif index == 7:  # Terminal 6 (GPS Launch)
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
