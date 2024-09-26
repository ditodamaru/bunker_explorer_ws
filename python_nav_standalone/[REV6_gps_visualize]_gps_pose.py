import sys
import math
import matplotlib.pyplot as plt
from PyQt5.QtWidgets import QApplication, QFileDialog, QMainWindow, QPushButton, QMessageBox

class RobotPositionVisualizationApp(QMainWindow):
    def __init__(self):
        super().__init__()

        self.initUI()
        self.data = []
        self.x0 = 0.0  # Initial robot x position
        self.y0 = 0.0  # Initial robot y position
        self.yaw0 = 0.0  # Initial robot orientation

    def initUI(self):
        self.setGeometry(100, 100, 400, 250)
        self.setWindowTitle('Robot Position Visualization')

        self.select_file_button = QPushButton('Select File', self)
        self.select_file_button.setGeometry(150, 50, 100, 40)
        self.select_file_button.clicked.connect(self.select_file)

    def select_file(self):
        options = QFileDialog.Options()
        options |= QFileDialog.DontUseNativeDialog
        selected_file, _ = QFileDialog.getOpenFileName(self, "Open Data File", "", "Text Files (*.txt)", options=options)
        if selected_file:
            self.load_data(selected_file)

    def load_data(self, selected_file):
        try:
            with open(selected_file, 'r') as data_file:
                self.data = data_file.readlines()

            # Extract the initial robot position and orientation from the first entry
            initial_entry = self.data[1].split()
            self.x0 = float(initial_entry[6])
            self.y0 = float(initial_entry[7])
            self.yaw0 = math.atan2(float(initial_entry[11]), float(initial_entry[10]))

            self.visualize_robot_positions()

        except Exception as e:
            QMessageBox.critical(self, "Error", f"An error occurred: {str(e)}")

    def visualize_robot_positions(self):
        x_gps_column_index = 1  # Replace with the actual index of the longitude column in your data
        y_gps_column_index = 2  # Replace with the actual index of the latitude column in your data
        x_robot_column_index = 6  # Replace with the actual index of the x column in your data
        y_robot_column_index = 7  # Replace with the actual index of the y column in your data

        gps_data = self.data[1:]
        robot_data = self.data[1:]

        # Transform GPS data from global to local coordinates
        x_local_gps_values = []
        y_local_gps_values = []
        for entry in gps_data:
            values = entry.split()
            x_global_gps = float(values[1])
            y_global_gps = float(values[2])
            x_local_gps = (x_global_gps - self.x0) * math.cos(-self.yaw0) - (y_global_gps - self.y0) * math.sin(-self.yaw0)
            y_local_gps = (x_global_gps - self.x0) * math.sin(-self.yaw0) + (y_global_gps - self.y0) * math.cos(-self.yaw0)
            x_local_gps_values.append(x_local_gps)
            y_local_gps_values.append(y_local_gps)

        # Get robot position data
        x_robot_values = [float(entry.split()[x_robot_column_index]) for entry in robot_data]
        y_robot_values = [float(entry.split()[y_robot_column_index]) for entry in robot_data]

        # Plot transformed GPS data and robot position data
        plt.figure(figsize=(10, 8))
        plt.plot(x_local_gps_values, y_local_gps_values, marker='o', linestyle='-', color='g', label='Local GPS Position')
        plt.plot(x_robot_values, y_robot_values, marker='x', linestyle='-', color='r', label='Robot Position')
        plt.title('Local GPS and Robot Position Visualization')
        plt.xlabel('X (m)')
        plt.ylabel('Y (m)')
        plt.legend()
        plt.grid(True)
        plt.show()

if __name__ == '__main__':
    app = QApplication(sys.argv)
    mainWindow = RobotPositionVisualizationApp()
    mainWindow.show()
    sys.exit(app.exec_())
