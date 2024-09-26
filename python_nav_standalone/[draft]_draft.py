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
        # ... (same as before) ...

    def select_file(self):
        # ... (same as before) ...

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
        # ... (same as before) ...

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

        # Plot transformed GPS data and robot position data
        plt.figure(figsize=(10, 8))
        plt.plot(x_local_gps_values, y_local_gps_values, marker='o', linestyle='-', color='g', label='Local GPS Position')
        plt.plot(x_robot_values, y_robot_values, marker='x', linestyle='-', color='r', label='Robot Position')
        plt.title('Local GPS and Robot Position Visualization')
        plt.xlabel('Local X (m)')
        plt.ylabel('Local Y (m)')
        plt.legend()
        plt.grid(True)
        plt.show()

if __name__ == '__main__':
    app = QApplication(sys.argv)
    mainWindow = RobotPositionVisualizationApp()
    mainWindow.show()
    sys.exit(app.exec_())
