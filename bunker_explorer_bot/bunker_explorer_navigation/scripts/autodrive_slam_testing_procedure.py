import sys
import subprocess
import os
from PyQt5 import QtWidgets, QtGui, QtCore

class TerminalLauncher(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()

        self.command_data = []

        self.setupUi()

    def setupUi(self):
        self.setWindowTitle("Custom Terminal Launcher")

        layout = QtWidgets.QVBoxLayout(self)

        self.name_inputs = []
        self.command_inputs = []
        self.start_buttons = []

        for i in range(3):  # Change the number of terminals as needed
            name_label = QtWidgets.QLabel(f"Terminal {i + 1}")
            name_input = QtWidgets.QLineEdit(self)
            self.name_inputs.append(name_input)

            command_label = QtWidgets.QLabel("Command:")
            command_input = QtWidgets.QTextEdit(self)
            self.command_inputs.append(command_input)

            layout.addWidget(name_label)
            layout.addWidget(name_input)
            layout.addWidget(command_label)
            layout.addWidget(command_input)

            edit_button = QtWidgets.QPushButton("Edit")
            edit_button.clicked.connect(lambda _, idx=i: self.editCommand(idx))
            layout.addWidget(edit_button)

            start_button = QtWidgets.QPushButton("Start Terminal")
            self.start_buttons.append(start_button)
            start_button.clicked.connect(lambda _, idx=i: self.startTerminal(idx))
            layout.addWidget(start_button)
            layout.addWidget(QtWidgets.QLabel(""))  # Add a spacer

        save_button = QtWidgets.QPushButton("Save Commands")
        save_button.clicked.connect(self.saveCommands)
        layout.addWidget(save_button)

        load_label = QtWidgets.QLabel("Load Commands:")
        self.load_dropdown = QtWidgets.QComboBox(self)
        self.load_dropdown.addItem("Select Command Set")
        self.load_dropdown.currentIndexChanged.connect(self.loadSelectedCommands)
        layout.addWidget(load_label)
        layout.addWidget(self.load_dropdown)

        self.loadCommands()  # Load commands on startup

        self.setFocusPolicy(QtCore.Qt.StrongFocus)
        self.keyPressEvent = self.customKeyPressEvent

    def customKeyPressEvent(self, event):
        if event.key() == QtCore.Qt.Key_Enter or event.key() == QtCore.Qt.Key_Return:
            focused_button = self.focusWidget()
            if focused_button in self.start_buttons:
                index = self.start_buttons.index(focused_button)
                self.startTerminal(index)

    def editCommand(self, index):
        # Edit the command for the selected terminal
        command, ok = QtWidgets.QInputDialog.getText(self, f"Edit Command for Terminal {index + 1}", "Enter command:")
        if ok:
            self.command_inputs[index].setPlainText(command)

    def startTerminal(self, index):
        # Start the terminal with the specified command
        if 0 <= index < len(self.command_inputs):
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

    def saveCommands(self):
        # Save the current commands to a text file
        file_path, _ = QtWidgets.QFileDialog.getSaveFileName(self, "Save Commands", "", "Text Files (*.txt)")
        if file_path:
            with open(file_path, 'w') as file:
                for i in range(len(self.command_inputs)):
                    name = self.name_inputs[i].text()
                    command = self.command_inputs[i].toPlainText()
                    file.write(f"{name}\t{command}\n")
            print("Commands saved to:", file_path)

    def loadCommands(self):
        # Load commands from a text file and populate the drop-down menu
        file_path, _ = QtWidgets.QFileDialog.getOpenFileName(self, "Load Commands", "", "Text Files (*.txt)")
        if file_path:
            with open(file_path, 'r') as file:
                lines = file.readlines()
                self.command_data = [line.strip().split('\t') for line in lines]
                self.load_dropdown.clear()
                self.load_dropdown.addItem("Select Command Set")
                for command_set in self.command_data:
                    self.load_dropdown.addItem(command_set[0])

    def loadSelectedCommands(self, index):
        # Load the selected commands from the drop-down menu
        if 0 < index < len(self.command_data) + 1:
            command_set = self.command_data[index - 1]
            self.name_inputs[0].setText(command_set[0])  # Set terminal name
            self.command_inputs[0].setPlainText(command_set[1])  # Set command

if __name__ == '__main__':
    app = QtWidgets.QApplication(sys.argv)
    launcher = TerminalLauncher()
    launcher.show()
    sys.exit(app.exec_())
