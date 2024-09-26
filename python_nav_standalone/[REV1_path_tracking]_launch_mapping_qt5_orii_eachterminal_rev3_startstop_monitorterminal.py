import sys
import os
import signal
import subprocess
import psutil
from PyQt5 import QtCore, QtGui, QtWidgets

class TerminalLauncher(QtWidgets.QWidget):
    def __init__(self):
        QtWidgets.QWidget.__init__(self)

        self.processes = []
        self.setupUi()

    def setupUi(self):
        self.setWindowTitle("Path Tracking Test GUI")

        layout = QtWidgets.QVBoxLayout(self)

        self.text_edit_widgets = []
        self.labels = [
            "Terminal 1 [Convert Robot Pose to yaml FILE]",
            "Terminal 2 [Launch Path Tracking Test]",
            "Terminal 3 [Launch Bunker ROS Node]",
            "Terminal 4 [Launch Record Database]",
            "Terminal 5 [Launch ROSBag play to move the Robot]"
            ]

        # for i in range(5):
        #     label = QtWidgets.QLabel("Terminal {}: ".format(i+1))
        #     layout.addWidget(label)
        #     self.labels.append(label)

        #     text_edit = QtWidgets.QTextEdit()
        #     layout.addWidget(text_edit)
        #     self.text_edit_widgets.append(text_edit)


        for i in range(5):
            label = QtWidgets.QLabel(self.labels[i])
            layout.addWidget(label)

            text_edit = QtWidgets.QTextEdit()
            layout.addWidget(text_edit)
            self.text_edit_widgets.append(text_edit)

            start_button_layout = QtWidgets.QHBoxLayout()
    

        for i in range(5):
            start_button = QtWidgets.QPushButton("Start Terminal {}".format(i+1))
            start_button.clicked.connect(lambda _, idx=i: self.startTerminal(idx))
            start_button_layout.addWidget(start_button)

        layout.addLayout(start_button_layout)

        stop_button_layout = QtWidgets.QHBoxLayout()

        for i in range(5):
            stop_button = QtWidgets.QPushButton("Stop Terminal {}".format(i+1))
            stop_button.clicked.connect(lambda _, idx=i: self.stopTerminal(idx))
            stop_button_layout.addWidget(stop_button)

        layout.addLayout(stop_button_layout)

    def startTerminal(self, index):
        commands = [
            "roslaunch realsense2_camera rs_camera.launch     align_depth:=true     unite_imu_method:=linear_interpolation     enable_gyro:=true      enable_accel:=true",
            "rosrun imu_filter_madgwick imu_filter_node     _use_mag:=false   _publish_tf:=false     _world_frame:=enu    /imu/data_raw:=/camera/imu     /imu/data:=/rtabmap/imu ",
            "roslaunch rtabmap_ros rtabmap.launch   clear_params:=false     depth_topic:=/camera/aligned_depth_to_color/image_raw     rgb_topic:=/camera/color/image_raw     camera_info_topic:=/camera/color/camera_info     approx_sync:=true     wait_imu_to_init:=true     imu_topic:=/rtabmap/imu  database_path:=~/.ros/rtabmap2.db",
            "rosrun map_server map_saver -f map:=/rtabmap/proj_map my_map_1",
            "roscore"
        ]

        if 0 <= index < len(commands):
            command = commands[index]
            process = QtCore.QProcess(self)
            process.readyReadStandardOutput.connect(lambda: self.updateOutput(process, index))
            process.readyReadStandardError.connect(lambda: self.updateOutput(process, index))
            process.finished.connect(lambda _, exit_code, index=index: self.processFinished(exit_code, index))
            process.start(command)
            self.processes.append(process)
            self.text_edit_widgets[index].append("Terminal {} started.\n".format(index+1))

    def updateOutput(self, process, index):
        output = process.readAllStandardOutput().data().decode()
        error_output = process.readAllStandardError().data().decode()
        if output:
            self.text_edit_widgets[index].append(output)
        if error_output:
            self.text_edit_widgets[index].append(error_output)

    # def stopTerminal(self, index):
    #     if 0 <= index < len(self.processes):
    #         process = self.processes[index]
    #         process.terminate()

    # def stopTerminal(self, index):
    #     if 0 <= index < len(self.processes):
    #         process = self.processes[index]
    #         # Send SIGINT signal to the process to stop it as if Ctrl+C is pressed
    #         process.terminate()
    #         process.waitForFinished()

    def stopTerminal(self, index):
        if 0 <= index < len(self.processes):
            process = self.processes[index]
            process.terminate()
            process.waitForFinished()

            # Terminate the running roscore process if Terminal 5 is stopped
            if index == 4:
                try:
                    # Find the running roscore process by name
                    for proc in psutil.process_iter(['pid', 'name']):
                        if 'roscore' in proc.info['name']:
                            os.kill(proc.info['pid'], signal.SIGINT)
                    self.text_edit_widgets[index].append("Terminal 5 stopped (roscore terminated).\n")
                except Exception as e:
                    self.text_edit_widgets[index].append("Failed to terminate roscore process: {}\n".format(e))
            else:
                self.text_edit_widgets[index].append("Terminal {} stopped.\n".format(index+1))




    def processFinished(self, exit_code, index):
        self.text_edit_widgets[index].append("Terminal {} stopped with exit code {}.\n".format(index+1, exit_code))

if __name__ == '__main__':
    app = QtWidgets.QApplication(sys.argv)
    launcher = TerminalLauncher()
    launcher.show()
    sys.exit(app.exec_())
