import sys
from PyQt5.QtWidgets import QApplication, QFileDialog, QMessageBox, QMainWindow, QPushButton

class DataPreprocessingApp(QMainWindow):
    def __init__(self):
        super().__init__()

        self.init_ui()

    def init_ui(self):
        self.setGeometry(100, 100, 300, 200)
        self.setWindowTitle("Data Preprocessing")

        self.gps_button = QPushButton("Select GPS Data File", self)
        self.gps_button.setGeometry(50, 50, 200, 30)
        self.gps_button.clicked.connect(self.select_gps_file)

        self.robot_button = QPushButton("Select Robot Position File", self)
        self.robot_button.setGeometry(50, 100, 200, 30)
        self.robot_button.clicked.connect(self.select_robot_file)

        self.save_button = QPushButton("Select Save Location", self)
        self.save_button.setGeometry(50, 150, 200, 30)
        self.save_button.clicked.connect(self.select_save_location)

    def select_gps_file(self):
        options = QFileDialog.Options()
        gps_file, _ = QFileDialog.getOpenFileName(self, "Select GPS Data File", "", "Text Files (*.txt);;All Files (*)", options=options)
        self.gps_file = gps_file

    def select_robot_file(self):
        options = QFileDialog.Options()
        robot_file, _ = QFileDialog.getOpenFileName(self, "Select Robot Position Data File", "", "Text Files (*.txt);;All Files (*)", options=options)
        self.robot_file = robot_file

    def select_save_location(self):
        options = QFileDialog.Options()
        save_location, _ = QFileDialog.getSaveFileName(self, "Select Save Location", "", "Text Files (*.txt);;All Files (*)", options=options)
        self.save_location = save_location

        self.preprocess_data()

    #def preprocess_data(self):
        # Implement your data preprocessing logic here using self.gps_file, self.robot_file, and self.save_location
        # For example:
        # gps_data = read_data_file(self.gps_file)
        # robot_data = read_data_file(self.robot_file)
        # aligned_data = align_data(gps_data, robot_data)
        # write_aligned_data_to_file(aligned_data, self.save_location)
    #    pass

    def preprocess_data(self):
        # Load GPS data and robot position data
        with open(self.gps_file, 'r') as gps_file:
            gps_data = gps_file.readlines()

        with open(self.robot_file, 'r') as robot_file:
            robot_data = robot_file.readlines()

        # Perform data alignment or processing (you need to implement this)
        aligned_data = self.align_data(gps_data, robot_data)  # Use 'self.align_data'

        # Convert aligned data to strings
        aligned_data_strings = [' '.join(entry) + '\n' for entry in aligned_data]

        # Add header to the aligned data
        header = "#timestamp longitude latitude altitude error bearing x y z qx qy qz qw\n"
        aligned_data_strings.insert(0, header)

        # Save the aligned data to the specified location
        with open(self.save_location, 'w') as save_file:
            save_file.writelines(aligned_data_strings)  # Write the strings

        # Show a success message
        QMessageBox.information(self, "Success", "Alignment process was successful and data has been saved.")


    def align_data(self, gps_data, robot_data):
        aligned_data = []

        # Skip the header lines in GPS data and robot data
        gps_data = gps_data[1:]
        robot_data = robot_data[1:]

        for gps_entry in gps_data:
            gps_values = gps_entry.strip().split()
            gps_timestamp = float(gps_values[0])

            closest_robot_entry = min(robot_data, key=lambda x: abs(float(x.strip().split()[0]) - gps_timestamp))
            robot_values = closest_robot_entry.strip().split()[1:]

            aligned_data.append(gps_values + robot_values)
        return aligned_data


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = DataPreprocessingApp()
    window.show()
    sys.exit(app.exec_())
