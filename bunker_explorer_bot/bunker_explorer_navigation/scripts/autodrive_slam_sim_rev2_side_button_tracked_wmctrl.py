import sys
import os
import signal
import subprocess
import time
from PyQt5 import QtWidgets, QtGui, QtCore

class TerminalLauncher(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()

        #self.processes = [] # Store subprocess.Popen objects
        #change the method
        self.process_dict = {} #Store subprocess.Popen objects

        self.setupUi()


    def setupUi(self):
        self.setWindowTitle("AutoDriving Visual SLAM Sim")      

        layout = QtWidgets.QVBoxLayout(self)

        self.labels = [
            #"Terminal 1 [Source ROS Environment]", #"source ~/.bashrc",
            #"Terminal [Start Robot CAN Communication]", #"rosrun bunker_bringup bringup_can2usb.bash",
            "Terminal [Launch Bunker Gazebo Sim]",
            "Terminal [Launch GPS, Camera, IMU, Robot Base Sim, & Rviz]",
            #"Terminal [Launch Camera]",  #"roslaunch bunker_explorer_perception start_rs_camera.launch",          
            #"Terminal 4 [Robot Localization]", ##"roslaunch bunker_explorer_control robot_control_ekf.launch",
            #"Terminal 5 [Robot Localization and Navsat Transform]",   #"roslaunch bunker_explorer_control localization_run_navsat.launch",
            #"Terminal 5a [Robot Localization withour Navsat Transform]", #"roslaunch bunker_explorer_waypoint localization_run.launch",
            "Terminal [Launch RTABMAP]", #"roslaunch bunker_explorer_perception rtab_mapping.launch",
            "Terminal [RTABMAP Localization Mode]", #"rosservice call rtabmap/set_mode_localization",
            "Terminal [RTABMAP Mapping Mode]", #"rosservice call rtabmap/set_mode_mapping",
            "Terminal [Launch Keyboard Control]", #"roslaunch bunker_explorer_description robot_base_teleopkey.launch",
            #"Terminal 8a [Launch Outdoor Waypoint Nav]",#"Terminal 8a [Launch Move Base Default]", #"roslaunch bunker_explorer_navigation move_base_default.launch",
            "Terminal [Editor Map Saver launch file]", #"roslaunch bunker_explorer_navigation editor_map_saver_launch.launch",
            "Terminal [Launch Map Saver Node]",
            "Terminal [Launch Joystick Controller]",#"roslaunch joy2twist gamepad_controller.launch"
            "Terminal [Launch click and save 2D Nav Goal]", #roslaunch bunker_explorer_waypoint collect_2dnav_goal.launch"
            #"Terminal 13  [Launch Map chooser]", ##"roslaunch bunker_explorer_navigation bunker3_map_chooser.launch",
            #"Terminal 13a [Launch Bunker3_navigation]", ##"roslaunch bunker_explorer_navigation bunker3_navigation.launch",
            #"Terminal 14 [Launch Bunker3_navigation without Map]", ##"roslaunch bunker_explorer_navigation bunker3_navigation_nomap.launch",
            #"Terminal 15 [Launch Goal File Chooser]",
            #"Terminal [Send Goals]", #"roslaunch outdoor_waypoint_nav send_goals.launch"
            "Terminal [Launch Converter Navgoal Node]", #"roslaunch bunker_explorer_waypoint converter_navgoal.launch"
            "Terminal [Launch Look-ahead Tracking]", #"roslaunch tracking_pid test_tracking_pid_bunker.test rviz:=false"
            "Terminal [Launch Publish Path Node]", #"roslaunch tracking_pid test_publishing_path.test"
            "Terminal [Launch Outdoor Waypoint Localization]", #"roslaunch outdoor_waypoint_nav outdoor_waypoint_nav_bunker.launch"
            "Terminal [Launch Joystick Collect GPS]", #roslaunch outdoor_waypoint_nav joy_launch_control_ucom.launch"
            "Terminal [Launch Send Goals Node]", #roslaunch outdoor_waypoint_nav send_goals.launch"
            "Terminal [Launch Plot GPS data Node]", #"roslaunch outdoor_waypoint_nav plot_gps_data_ucom.launch"
            "Terminal [Launch Joystick Node]" #roslaunch teleop_twist_joy teleop.launch
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
            stop_button.setEnabled(False)  #Disable initially
            h_layout.addWidget(stop_button)

            #Connect signals for both start and stop buttons
            start_button.clicked.connect(lambda _, idx=index: self.startTerminal(idx, stop_button))
            #stop_button.clicked.connect(lambda _, processess=self.processes, stop_button=stop_button: self.stopTerminal(processess[-1] if processess else None, stop_button))
            stop_button.clicked.connect(lambda _, index=len(self.labels)-1, stop_button=stop_button: self.stopTerminal(index, stop_button))

            #self.start_buttons.append(start_button)
            #self.stop_buttons.append(stop_button)

            # Add the horizontal layout to the main vertical layout
            layout.addLayout(h_layout)
            

        self.setFocusPolicy(QtCore.Qt.StrongFocus)
        self.keyPressEvent = self.customKeyPressEvent

    def customKeyPressEvent(self, event):
        if event.key() == QtCore.Qt.Key_Enter or event.key() == QtCore.Qt.Key_Return:
            focused_button = self.focusWidget()
            if focused_button in self.start_buttons:
                index = self.start_buttons.index(focused_button)
                self.startTerminal(index, self.stop_buttons[index])
            elif focused_button in self.stop_buttons:
                index = self.stop_buttons.index(focused_button)
                process = self.processes(index)
                self.stopTerminal(process, self.stop_buttons[index])

    #def getCustomArguments(self):
    #    custom_arguments, ok = QtWidgets.QInputDialog.getText(self, "Custom arguments", "Enter custom arguments:")
    #    return custom_arguments if ok else None

    def startTerminal(self, index, stop_button):
        commands = [
            #"source ~/.bashrc",
            #"rosrun bunker_bringup bringup_can2usb.bash",
            "roslaunch bunker_explorer_description apple_tree_4times3random.launch", #"roslaunch bunker_explorer_description bunker_sim_camera_base-footprint.launch",
            "roslaunch bunker_explorer_description robot_base_bringup_sim.launch",
            #"roslaunch bunker_explorer_perception start_rs_camera.launch",
            #"roslaunch bunker_explorer_control robot_control_ekf.launch",
            #"roslaunch bunker_explorer_control localization_run_navsat.launch",
            #"roslaunch bunker_explorer_waypoint localization_run.launch",
            #"roslaunch bunker_explorer_control localization_run_no_navsat.launch",             
            "roslaunch bunker_explorer_perception rtab_mapping_sim.launch", #database_path:=~/.ros/rtabmap2.db
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
            "roslaunch bunker_explorer_waypoint converter_navgoal_sim.launch", 
            "roslaunch tracking_pid test_tracking_pid_bunker_sim.test rviz:=false",
            "roslaunch tracking_pid test_publishing_path_bunker_sim.test",
            "roslaunch outdoor_waypoint_nav outdoor_waypoint_nav_bunker.launch", #"roslaunch bunker_explorer_navigation move_base_default.launch", 
            "roslaunch outdoor_waypoint_nav joy_launch_control_ucom.launch",
            "roslaunch outdoor_waypoint_nav send_goals.launch",
            "roslaunch outdoor_waypoint_nav plot_gps_data_ucom.launch",
            "roslaunch teleop_twist_joy teleop.launch"

        ] 

        if 0 <= index < len(commands):
            if index == 2:  # Terminal 3 (RTABMAP launch)
                custom_arguments, ok = QtWidgets.QInputDialog.getText(self, "Custom Arguments", "Enter custom arguments :")
                if not ok:
                    return  # User canceled the input dialog, do not proceed with the process
                command = commands[index] + " " + custom_arguments
            elif index == 3:  # Terminal 4 (Map Saver)
                custom_arguments, ok = QtWidgets.QInputDialog.getText(self, "Custom Arguments", "Enter custom arguments :")
                if not ok:
                    return  # User canceled the input dialog, do not proceed with the process
                command = commands[index] + " " + custom_arguments
            elif index == 4:  # Terminal 4 (Map Saver)
                custom_arguments, ok = QtWidgets.QInputDialog.getText(self, "Custom Arguments", "Enter custom arguments :")
                if not ok:
                    return  # User canceled the input dialog, do not proceed with the process
                command = commands[index] + " " + custom_arguments
            elif index == 5:  # Terminal 4 (Debugging)
                custom_arguments, ok = QtWidgets.QInputDialog.getText(self, "Custom Arguments", "Enter custom arguments :")
                if not ok:
                    return  # User canceled the input dialog, do not proceed with the process
                command = commands[index] + " " + custom_arguments            
            elif index == 6:  # Terminal 6 (GPS Launch)
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
                #open terminal using gnome
                #pid = os.system(f"gnome-terminal -- bash -c '{command}'; echo $!")
                
                #process = subprocess.Popen(["gnome-terminal", "--", "bash", "-c", f"{command} && echo $!"]) # capturing using echo
                #process = subprocess.Popen(["gnome-terminal", "--", "bash", "-c", f"{command}"], stdout=subprocess.PIPE) # capturing using stdout
                process = subprocess.Popen(["gnome-terminal", "--disable-factory", "--", "bash", "-c", f"{command}"], stdout=subprocess.PIPE) #closed when gives stop signal
                #process = subprocess.Popen(["gnome-terminal", "--disable-factory", "--", "bash", "-c", f"{command}"], stdout=subprocess.PIPE) #closed when gives stop signal
                #process = subprocess.Popen(["gnome-terminal", "--disable-factory", "--", "bash", "-c", f"{command}"])
 
                #process.wait(timeout=1) #Wait for the process to finish to capturing the PID
                pid = process.pid

                print("PID number:", pid)

                #method 2 based on pid process capturing
                self.process_dict[index] = pid

                stop_button.setEnabled(True)  #Enable the stop button

            except Exception as e:
                print(f"Error: {e}")

                
    # # Method 4
    # def stopTerminal(self, index, stop_button):
    #     #process = self.process_dict.get(index)
    #     pid = self.process_dict.get(index)

    #     if pid:
    #         try:
    #             os.kill(pid, signal.SIGINT)
    #             os.waitpid(pid, 0)
    #             print("PID number closed:", pid)
    #             stdout, stderr = (pid, communicate())
    #         except ProcessLookupError:
    #             print("Proces not found.")
    #         except Exception as e:
    #             print(f"Errror stopping process: {e}")
    #     else:
    #         print("Process is not running")

    #     # Now close the associated gnome-terminal window
    #     # try:
    #     #     windows_title = f"Terminal [index={index}]"
    #     #     subprocess.run(["wmctrl", "-c", windows_title])
    #     #     time.sleep(1) # Add delay to ensure the terminal is closed before further actions
    #     # except Exception as e:
    #     #     print(f"Error closing terminal window: {e}") 

    #     stop_button.setEnabled(False)  # Disable the stop button
                
    # Method 6
    def stopTerminal(self, index, stop_button):
        pid = self.process_dict.get(index)

        if pid:
            try:
                # Send SIGINT to the entire process group
                os.killpg(os.getpgid(pid), signal.SIGINT)
                os.waitpid(pid, 0)
                print("Process with PID", pid, "terminated successfully.")

            except ProcessLookupError:
                print("Process not found.")
            except Exception as e:
                print(f"Error stopping process: {e}")
        else:
            print("Process is not running")

        # Now close the associated gnome-terminal window using pkill
        try:
            subprocess.run(["pkill", "-P", str(pid)])
            subprocess.run(["pkill", "-TERM", "-P", str(pid)])
            subprocess.run(["pkill", "-P", str(pid)])
            subprocess.run(["pkill", "-TERM", str(pid)])
            subprocess.run(["pkill", "-KILL", str(pid)])
            time.sleep(1)
            print("Gnome-terminal window associated with PID", pid, "closed.")
        except Exception as e:
            print(f"Error closing terminal window: {e}")

        stop_button.setEnabled(False)  # Disable the stop button



if __name__ == '__main__':
    app = QtWidgets.QApplication(sys.argv)
    launcher = TerminalLauncher()
    launcher.show()
    sys.exit(app.exec_())
