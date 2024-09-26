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
        self.setWindowTitle("Terminal Launcher")

        layout = QtWidgets.QVBoxLayout(self)

        self.text_edit_widgets = []
        self.labels = []

        for i in range(5):
            label = QtWidgets.QLabel("Terminal {}: ".format(i+1))
            layout.addWidget(label)
            self.labels.append(label)

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
            "roslaunch realsense_explorer_perception start_rs_camera.launch filters:=pointcloud",
            "roslaunch realsense_explorer_perception rtab_mapping.launch",
            "roslaunch realsense_explorer_perception rtab_mapping.launch localization:=true",
            "roslaunch realsense_explorer_navigation move_base.launch",
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
