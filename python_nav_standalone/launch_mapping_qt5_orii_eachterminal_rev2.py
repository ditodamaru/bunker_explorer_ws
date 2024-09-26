import sys
import os
import signal
import subprocess
from PyQt5 import QtWidgets

class TerminalLauncher(QtWidgets.QWidget):
    def __init__(self):
        super(TerminalLauncher, self).__init__()
        self.processes = []
        self.text_edit_widgets = []  # Store the text edit widgets for each terminal
        self.initUI()

    def initUI(self):
        self.setWindowTitle("Terminal Launcher")
        self.setGeometry(100, 100, 600, 400)

        layout = QtWidgets.QVBoxLayout()

        # Terminal 1
        label_terminal1 = QtWidgets.QLabel("Terminal 1 - Start RGB-D Stream from Realsense D435i:")
        self.txtTerminal1Output = QtWidgets.QTextEdit()
        self.text_edit_widgets.append(self.txtTerminal1Output)  # Add to the list
        layout.addWidget(label_terminal1)
        layout.addWidget(self.txtTerminal1Output)

        # Terminal 2
        label_terminal2 = QtWidgets.QLabel("Terminal 2 - Start RTAB Map for Mapping:")
        self.txtTerminal2Output = QtWidgets.QTextEdit()
        self.text_edit_widgets.append(self.txtTerminal2Output)  # Add to the list
        layout.addWidget(label_terminal2)
        layout.addWidget(self.txtTerminal2Output)

        # Terminal 3
        label_terminal3 = QtWidgets.QLabel("Terminal 3 - Start RTAB Map for Localization:")
        self.txtTerminal3Output = QtWidgets.QTextEdit()
        self.text_edit_widgets.append(self.txtTerminal3Output)  # Add to the list
        layout.addWidget(label_terminal3)
        layout.addWidget(self.txtTerminal3Output)

        # Terminal 4
        label_terminal4 = QtWidgets.QLabel("Terminal 4 - Robot Navigation:")
        self.txtTerminal4Output = QtWidgets.QTextEdit()
        self.text_edit_widgets.append(self.txtTerminal4Output)  # Add to the list
        layout.addWidget(label_terminal4)
        layout.addWidget(self.txtTerminal4Output)

        # Buttons
        self.btnStart = QtWidgets.QPushButton("Start Terminals")
        self.btnStart.clicked.connect(self.startTerminals)
        layout.addWidget(self.btnStart)

        self.btnStop = QtWidgets.QPushButton("Stop Terminals")
        self.btnStop.clicked.connect(self.stopTerminals)
        layout.addWidget(self.btnStop)

        self.setLayout(layout)

    def startTerminals(self):
        commands = [
            "roslaunch realsense_explorer_perception start_rs_camera.launch filters:=pointcloud",
            "roslaunch realsense_explorer_perception rtab_mapping.launch",
            "roslaunch realsense_explorer_perception rtab_mapping.launch localization:=true",
            "roslaunch realsense_explorer_navigation move_base.launch"
        ]

        for i, command in enumerate(commands):
            process = subprocess.Popen(command.split(), stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                       universal_newlines=True)
            self.processes.append(process)
            output_widget = self.text_edit_widgets[i]  # Use the list to access the correct text edit widget
            output_widget.append("Terminal {} started.\n".format(i+1))

    def stopTerminals(self):
        for i, process in enumerate(self.processes):
            process.send_signal(signal.SIGINT)
            output_widget = self.text_edit_widgets[i]  # Use the list to access the correct text edit widget
            output_widget.append("Terminal {} stopped.\n".format(i+1))

    def closeEvent(self, event):
        self.stopTerminals()
        event.accept()

if __name__ == '__main__':
    app = QtWidgets.QApplication(sys.argv)
    launcher = TerminalLauncher()
    signal.signal(signal.SIGINT, signal.SIG_DFL)  # Reset signal handling to default
    launcher.show()
    sys.exit(app.exec_())
