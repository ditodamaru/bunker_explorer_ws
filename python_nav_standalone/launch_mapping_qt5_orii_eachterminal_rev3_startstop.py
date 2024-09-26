import sys
import os
import signal
import subprocess
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
            "roslaunch realsense_explorer_navigation move_base.launch"
            "roscore"
        ]

        if 0 <= index < len(commands):
            command = commands[index]
            process = subprocess.Popen(command.split(), stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                       universal_newlines=True)
            self.processes.append(process)
            self.text_edit_widgets[index].append("Terminal {} started.\n".format(index+1))

    def stopTerminal(self, index):
        if 0 <= index < len(self.processes):
            process = self.processes[index]
            process.send_signal(signal.SIGINT)
            self.text_edit_widgets[index].append("Terminal {} stopped.\n".format(index+1))

if __name__ == '__main__':
    app = QtWidgets.QApplication(sys.argv)
    launcher = TerminalLauncher()
    signal.signal(signal.SIGINT, signal.SIG_DFL)  # Reset signal handling to default
    launcher.show()
    sys.exit(app.exec_())
