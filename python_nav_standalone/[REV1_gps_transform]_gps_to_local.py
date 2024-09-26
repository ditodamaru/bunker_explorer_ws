import sys
import math
import matplotlib.pyplot as plt
from PyQt5.QtWidgets import QApplication, QFileDialog, QMainWindow, QPushButton, QMessageBox

class DataPreprocessingApp(QMainWindow):
    def __init__(self):
        super().__init__()

        self.initUI()
        self.robot_pose_x = 0.0  # Example robot pose, provide actual values
        self.robot_pose_y = 0.0
        self.robot_pose_theta = 0.0

    def initUI(self):
        self.setGeometry(100, 100, 400, 250)
        self.setWindowTitle('Data Preprocessing')

        self.select_files_button = QPushButton('Select Files', self)
        self.select_files_button.setGeometry(150, 50, 100, 40)
        self.select_files_button.clicked.connect(self.select_save_location)

    def select_save_location(self):
        options = QFileDialog.Options()
        options |= QFileDialog.DontUseNativeDialog
        self.gps_file, _ = QFileDialog.getOpenFileName(self, "Open GPS Data File", "", "Text Files (*.txt)", options=options)
        if self.gps_file:
            self.preprocess_data()

    def preprocess_data(self):
        try:
            with open(self.gps_file, 'r') as gps_file:
                gps_data = gps_file.readlines()

            transformed_data = []
            for line in gps_data[1:]:
                values = line.strip().split()
                timestamp, longitude, latitude = float(values[0]), float(values[1]), float(values[2])
                local_x, local_y = self.transform_global_to_local(latitude, longitude)
                values[1] = str(local_x)
                values[2] = str(local_y)
                transformed_data.append(' '.join(values))

            # Write the transformed data to a new file
            save_location, _ = QFileDialog.getSaveFileName(self, "Save Transformed Data As", "", "Text Files (*.txt)")
            if save_location:
                with open(save_location, 'w') as output_file:
                    header = "#timestamp x y altitude error bearing x y z qx qy qz qw\n"
                    output_file.write(header)
                    output_file.write('\n'.join(transformed_data))

                QMessageBox.information(self, "Success", "Transformation successful and data saved.")
                self.visualize_data(transformed_data)

        except Exception as e:
            QMessageBox.critical(self, "Error", f"An error occurred: {str(e)}")

    def transform_global_to_local(self, global_latitude, global_longitude):
        # Transformation logic, replace with your actual logic
        delta_latitude = global_latitude - self.robot_pose_x
        delta_longitude = global_longitude - self.robot_pose_y
        # Replace the following with your actual transformation equations
        local_x = delta_longitude
        local_y = delta_latitude
        return local_x, local_y

    def visualize_data(self, transformed_data):
        # Extract x and y values from transformed data
        x_values = [float(entry.split()[1]) for entry in transformed_data]
        y_values = [float(entry.split()[2]) for entry in transformed_data]

        plt.figure(figsize=(8, 6))
        plt.plot(x_values, y_values, marker='o', linestyle='-', color='b')
        plt.title('Transformed Data Visualization')
        plt.xlabel('Local X')
        plt.ylabel('Local Y')
        plt.grid(True)
        plt.show()

if __name__ == '__main__':
    app = QApplication(sys.argv)
    mainWindow = DataPreprocessingApp()
    mainWindow.show()
    sys.exit(app.exec_())
