import sys
import subprocess
from PyQt5 import QtCore, QtGui, QtWidgets, uic

class TerminalLauncherApp(QtWidgets.QMainWindow):
    def __init__(self):
        super(TerminalLauncherApp, self).__init__()
        uic.loadUi('terminal_launcher.ui', self)

        self.start_button.clicked.connect(self.launch_terminal_scripts)
        self.stop_button.clicked.connect(self.stop_terminal_scripts)

        self.terminal_processes = {}
        self.terminal_outputs = {
            1: self.terminal_output_1,
            2: self.terminal_output_2,
            3: self.terminal_output_3,
            4: self.terminal_output_4
        }

    def run_terminal_command(self, terminal, command):
        process = subprocess.Popen(['gnome-terminal', '--', 'bash', '-c', command])
        self.terminal_processes[terminal] = process

    def launch_terminal_scripts(self):
        self.run_terminal_command(1, "roslaunch realsense_explorer_perception start_rs_camera.launch filters:=pointcloud")
        self.run_terminal_command(2, "roslaunch realsense_explorer_perception rtab_mapping.launch")
        self.run_terminal_command(3, "roslaunch realsense_explorer_perception rtab_mapping.launch localization:=true")
        self.run_terminal_command(4, "roslaunch realsense_explorer_navigation move_base.launch")

    def stop_terminal_scripts(self):
        for process in self.terminal_processes.values():
            process.terminate()

if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    window = TerminalLauncherApp()
    window.show()
    sys.exit(app.exec_())           
