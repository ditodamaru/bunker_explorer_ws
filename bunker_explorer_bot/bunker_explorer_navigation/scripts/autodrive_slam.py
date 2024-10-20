import sys
import os
import signal
import subprocess
import time
from PyQt5 import QtWidgets, QtGui, QtCore
import functools

class TerminalLauncher(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()

        #Store sub process.Popen objects
        self.process_dict = {} 

        self.setupUi()

    def setupUi(self):
        self.setWindowTitle("AutoDriving Visual SLAM")

        layout = QtWidgets.QVBoxLayout(self)

        self.labels = [
            #"Terminal 1 [Source ROS Environment]", #"source ~/.bashrc",
            "Terminal [Start Robot CAN Communication]", #"rosrun bunker_bringup bringup_can2usb.bash",
            "Terminal [Launch GPS, IMU, Bunker Robot, Rviz]",
            "Terminal [Launch Depth Camera Node]",  #"roslaunch bunker_explorer_perception start_rs_camera.launch",          
            #"Terminal 4 [Robot Localization]", ##"roslaunch bunker_explorer_control robot_control_ekf.launch",
            #"Terminal 5 [Robot Localization and Navsat Transform]",   #"roslaunch bunker_explorer_control localization_run_navsat.launch",
            #"Terminal 5a [Robot Localization withour Navsat Transform]", #"roslaunch bunker_explorer_waypoint localization_run.launch",
            "Terminal [Launch RTABMAP]", #"roslaunch bunker_explorer_perception rtab_mapping.launch",
            "Terminal [RTABMAP Localization Mode]", #"rosservice call rtabmap/set_mode_localization",
            "Terminal [RTABMAP Mapping Mode]", #"rosservice call rtabmap/set_mode_mapping",
            "Terminal [Launch Keyboard Control]", #"roslaunch bunker_explorer_description robot_base_teleopkey.launch",
            #"Terminal 8a [Launch Outdoor Waypoint Nav]",#"Terminal 8a [Launch Move Base Default]", #"roslaunch bunker_explorer_navigation move_base_default.launch",
            "Terminal [Editor Map Saver launch file]",
            "Terminal [Launch Map Saver Node]",
            "Terminal [Launch Joystick Controller]",#"Terminal 11 [Debugging]", 
            "Terminal [Launch click and save 2D Nav Goal]",
            #"Terminal 13  [Launch Map chooser]",
            #"Terminal 13a [Launch Bunker3_navigation]",
            #"Terminal 14 [Launch Bunker3_navigation without Map]",
            #"Terminal 15 [Launch Goal File Chooser]",
            #"Terminal [Send Goals]",
            "Terminal [Launch Converter Navgoal Node]",
            "Terminal [Launch Look-ahead Tracking Node]",
            "Terminal [Launch Publish Path Node]",
            #"Terminal [Launch Outdoor Waypoint Localization]",
            #"Terminal [Launch Joystick Collect GPS]",
            #"Terminal [Launch Send Goals Node]",
            #"Terminal [Launch Plot GPS data Node]",
            #"Terminal [Launch Joystick Node]"
        ]

        self.start_buttons = []
        self.stop_buttons = []
        
        for index, label in enumerate(self.labels):
            # Create a horizontal layout for each terminal entry
            h_layout = QtWidgets.QHBoxLayout()

            label_widget = QtWidgets.QLabel(label)
            h_layout.addWidget(label_widget)

            start_button = QtWidgets.QPushButton("Start Terminal")
            h_layout.addWidget(start_button)

            stop_button = QtWidgets.QPushButton("Stop Terminal")
            stop_button.setEnabled(False) #Disable initially
            h_layout.addWidget(stop_button)

            #Connect signla for both start and stop buttons
            start_button.clicked.connect(lambda _, idx=index: self.startTerminal(idx, stop_button))

            #stop_button Method 1
            #stop_button.clicked.connect(lambda _, index=len(self.labels)-1, stop_button=stop_button: self.stopTerminal(index, stop_button))
            stop_button.clicked.connect(lambda _, idx=index, stop_button=stop_button:  self.stopTerminal(idx, stop_button))

            print(f"Stop button for index {index} connected.")

            # Add the horizontal layout to the main vertical layout
            layout.addLayout(h_layout)

            # Enable the stop button for the current index
            stop_button.setEnabled(True)            

        self.setFocusPolicy(QtCore.Qt.StrongFocus)
        self.keyPressEvent = self.customKeyPressEvent

    def customKeyPressEvent(self, event):
        if event.key() == QtCore.Qt.Key_Enter or event.key() == QtCore.Qt.Key_Return:
            focused_button = self.focusWidget()
            if focused_button in self.start_buttons:
                index = self.start_buttons.index(focused_button)
                self.startTerminal(index, self.stop_buttons[index])
                #print(f"Start button pressed on index:{index}")
            elif focused_button in self.stop_buttons:
                index = self.stop_buttons.index(focused_button)
                process = self.process(index)
                self.stopTerminal(process, self.stop_buttons[index])
                #print(f"Stop button pressed on index :{index}")

    def startTerminal(self, index, stop_button):
        print(f"Start button clicked for index: {index}")

        commands = [
            #"source ~/.bashrc",
            "rosrun bunker_bringup bringup_can2usb.bash",
            "roslaunch bunker_explorer_description robot_base_bringup.launch",
            "roslaunch bunker_explorer_perception start_rs_camera.launch",
            #"roslaunch bunker_explorer_control robot_control_ekf.launch",
            #"roslaunch bunker_explorer_control localization_run_navsat.launch",
            #"roslaunch bunker_explorer_waypoint localization_run.launch",
            #"roslaunch bunker_explorer_control localization_run_no_navsat.launch",             
            "roslaunch bunker_explorer_perception rtab_mapping.launch", #database_path:=~/.ros/rtabmap2.db
            "rosservice call rtabmap/set_mode_localization", #rosservice call /rtabmap/reset_odom
            "rosservice call rtabmap/set_mode_mapping",       
            "roslaunch bunker_explorer_description robot_base_teleopkey.launch",
            #"roslaunch outdoor_waypoint_nav outdoor_waypoint_nav_bunker.launch", #"roslaunch bunker_explorer_navigation move_base_default.launch", 
            #"rosrun map_server map_saver -f map:=/rtabmap/proj_map", #my_map_1
            "roslaunch bunker_explorer_navigation editor_map_saver_launch.launch",
            "roslaunch bunker_explorer_navigation map_saver.launch",
            "roslaunch joy2twist gamepad_controller.launch", #"", #rosservice call /rtabmap/reset_odom
            "roslaunch bunker_explorer_waypoint collect_2dnav_goal.launch",
            #"roslaunch bunker_explorer_navigation bunker3_map_chooser.launch",
            #"roslaunch bunker_explorer_navigation bunker3_navigation.launch",
            #"roslaunch bunker_explorer_navigation bunker3_navigation_nomap.launch",
            #"roslaunch outdoor_waypoint_nav send_goals.launch",
            "roslaunch bunker_explorer_waypoint converter_navgoal.launch",
            "roslaunch tracking_pid test_tracking_pid_bunker.test rviz:=false",
            "roslaunch tracking_pid test_publishing_path.test",
            #"roslaunch outdoor_waypoint_nav outdoor_waypoint_nav_bunker.launch", #"roslaunch bunker_explorer_navigation move_base_default.launch", 
            #"roslaunch outdoor_waypoint_nav joy_launch_control_ucom.launch",
            #"roslaunch outdoor_waypoint_nav send_goals.launch",
            #"roslaunch outdoor_waypoint_nav plot_gps_data_ucom.launch",
            #"roslaunch teleop_twist_joy teleop.launch"

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

            try:
                print("Executing command:", command)
                process = subprocess.Popen(["gnome-terminal", "--disable-factory", "--", "bash", "-c", f"{command}"])

                pid = process.pid

                print("PID number:", pid)

                #method2 based on pid process capturing
                self.process_dict[index] = pid
    
                print("index number:", {index})

                stop_button.setEnabled(True)  #Enable the stop button

            except Exception as e:
                print(f"Error: {e}")            

    #Method6
    def stopTerminal(self, index, stop_button):
        print(f"Stop button clicked for index: {index}")    
        pid = self.process_dict.get(index)

        if pid:
            try:
                os.killpg(os.getpgid(pid), signal.SIGINT)
                os.waitpid(pid, 0)
                print("PID number closed:", pid)
            except ProcessLookupError:
                print("Process not found.")
            except KeyboardInterrupt:
                print("Keyboard interupt received. Ignoring.")
            except Exception as e:
                print(f"Error stopping process: {e}")
        else:
            print("Process is not running")
        
        #Now close the associated gnome-terminal windows using pkill
        try:
            subprocess.run(["pkill", "-P", str(pid)])
            subprocess.run(["pkill", "-TERM", "-P", str(pid)])
            subprocess.run(["pkill", "-P", str(pid)])
            subprocess.run(["pkill", "-TERM", str(pid)])
            subprocess.run(["pkill", "-KILL", str(pid)])
            time.sleep(1)
            print("gnome-terminal window associated with PID", pid, "closed.")
        except Exception as e:
            print(f"Error closing terminal windows: {e}")
        
        #stop_button.setEnabled(False)  #Disable the stop button

if __name__ == '__main__':
    app = QtWidgets.QApplication(sys.argv)
    launcher = TerminalLauncher()
    launcher.show()
    sys.exit(app.exec_())
