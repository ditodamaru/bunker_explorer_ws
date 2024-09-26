import sys
import math
import matplotlib.pyplot as plt
from PyQt5.QtWidgets import QApplication, QFileDialog, QMainWindow, QPushButton, QMessageBox

class RobotPositionVisualizationApp(QMainWindow):
    def __init__(self):
        super().__init__()

        self.initUI()
        self.data = []

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

            self.visualize_robot_positions()

        except Exception as e:
            QMessageBox.critical(self, "Error", f"An error occurred: {str(e)}")

    def visualize_robot_positions(self):
        x_column_index = 6  # Replace with the actual index of the x column in your data
        y_column_index = 7  # Replace with the actual index of the y column in your data

        x_values = [float(entry.split()[x_column_index]) for entry in self.data[1:]]
        y_values = [float(entry.split()[y_column_index]) for entry in self.data[1:]]

        x_values_gps = [float(entry.split()[1]) for entry in self.data[1:]]
        y_values_gps = [float(entry.split()[2]) for entry in self.data[1:]]

        plt.figure(figsize=(10, 8))
        plt.plot(x_values, y_values, marker='o', linestyle='-', color='b', label='Robot Position')
        plt.plot(x_values_gps, y_values_gps, marker='o', linestyle='-', color='r', label='GPS position')
        plt.title('Robot Position Visualization')
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
