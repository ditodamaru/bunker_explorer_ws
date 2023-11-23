import sys
import subprocess
from PyQt5 import QtWidgets, QtGui, QtCore

class TerminalLauncher(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()

        self.labels = ["Terminal 1", "Terminal 2", "Terminal 3"]
        self.commands = ["rosrun package_name node1", "roslaunch package_name launch_file", "rosrun package_name node2"]

        self.setupUi()

    def setupUi(self):
        self.setWindowTitle("Custom Terminal Launcher")

        layout = QtWidgets.QVBoxLayout(self)

        self.name_inputs = []
        self.command_inputs = []
        self.start_buttons = []

        for i, label in enumerate(self.labels):
            name_label = QtWidgets.QLabel(label)
            name_input = QtWidgets.QLineEdit(self)
            name_input.setText(label)
            self.name_inputs.append(name_input)

            command_label = QtWidgets.QLabel("Command:")
            command_input = QtWidgets.QTextEdit(self)
            command_input.setPlainText(self.commands[i])
            self.command_inputs.append(command_input)

            layout.addWidget(name_label)
            layout.addWidget(name_input)
            layout.addWidget(command_label)
            layout.addWidget(command_input)

            start_button = QtWidgets.QPushButton("Start Terminal")
            self.start_buttons.append(start_button)
            start_button.clicked.connect(lambda _, idx=i: self.startTerminal(idx))
            layout.addWidget(start_button)
            layout.addWidget(QtWidgets.QLabel(""))  # Add a spacer

        self.setFocusPolicy(QtCore.Qt.StrongFocus)
        self.keyPressEvent = self.customKeyPressEvent

    def customKeyPressEvent(self, event):
        if event.key() == QtCore.Qt.Key_Enter or event.key() == QtCore.Qt.Key_Return:
            focused_button = self.focusWidget()
            if focused_button in self.start_buttons:
                index = self.start_buttons.index(focused_button)
                self.startTerminal(index)

    def startTerminal(self, index):
        if 0 <= index < len(self.commands):
            custom_arguments, ok = QtWidgets.QInputDialog.getText(self, "Custom Arguments", "Enter custom arguments:")
            if not ok:
                return  # User canceled the input dialog, do not proceed with the process

            name = self.name_inputs[index].text()
            command = self.command_inputs[index].toPlainText() + " " + custom_arguments

            try:
                print(f"Executing command for {name}:", command)
                subprocess.Popen(["gnome-terminal", "--", "bash", "-c", command])
            except Exception as e:
                print(f"Error: {e}")

if __name__ == '__main__':
    app = QtWidgets.QApplication(sys.argv)
    launcher = TerminalLauncher()
    launcher.show()
    sys.exit(app.exec_())
