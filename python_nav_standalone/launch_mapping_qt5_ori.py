import sys
import os
import subprocess
import signal
from PyQt5 import QtWidgets

class TerminalLauncher(QtWidgets.QWidget):
    def __init__(self):
        super(TerminalLauncher, self).__init__()
        self.initUI()

    def initUI(self):
        # Create buttons
        self.btnStart = QtWidgets.QPushButton('Start', self)
        self.btnStop = QtWidgets.QPushButton('Stop', self)

        # Create text edit widgets
        self.txtTerminal1Output = QtWidgets.QTextEdit(self)
        self.txtTerminal2Output = QtWidgets.QTextEdit(self)
        self.txtTerminal3Output = QtWidgets.QTextEdit(self)
        self.txtTerminal4Output = QtWidgets.QTextEdit(self)

        # Arrange widgets using layouts
        layout = QtWidgets.QVBoxLayout()
        layout.addWidget(self.btnStart)
        layout.addWidget(self.btnStop)
        layout.addWidget(self.txtTerminal1Output)
        layout.addWidget(self.txtTerminal2Output)
        layout.addWidget(self.txtTerminal3Output)
        layout.addWidget(self.txtTerminal4Output)

        self.setLayout(layout)

        # Connect button signals to functions
        self.btnStart.clicked.connect(self.startTerminals)
        self.btnStop.clicked.connect(self.stopTerminals)

        self.setWindowTitle('Terminal Launcher')
        self.show()

    def startTerminals(self):
        # Start each terminal command using subprocess
        terminal_commands = [
            "roslaunch realsense_explorer_perception start_rs_camera.launch filters:=pointcloud",
            "roslaunch realsense_explorer_perception rtab_mapping.launch",
            "roslaunch realsense_explorer_perception rtab_mapping.launch localization:=true",
            "roslaunch realsense_explorer_navigation move_base.launch"
        ]

        self.processes = []
        for command in terminal_commands:
            process = subprocess.Popen(command, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            self.processes.append(process)

        self.btnStart.setEnabled(False)
        self.btnStop.setEnabled(True)

    def stopTerminals(self):
        # Terminate all running terminal processes
        for process in self.processes:
            process.terminate()

        self.btnStart.setEnabled(True)
        self.btnStop.setEnabled(False)

def signal_handler(sig, frame):
    # Signal handler function to handle Ctrl+C
    global launcher
    launcher.stopTerminals()
    sys.exit(0)

if __name__ == '__main__':
    app = QtWidgets.QApplication(sys.argv)
    launcher = TerminalLauncher()
    signal.signal(signal.SIGINT, signal_handler)  # Register Ctrl+C signal handler
    sys.exit(app.exec_())
