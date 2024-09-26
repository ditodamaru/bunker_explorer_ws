import sys
import matplotlib.pyplot as plt
import numpy as np
from PyQt5.QtWidgets import QApplication, QMainWindow, QPushButton, QVBoxLayout, QWidget, QFileDialog
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas

class DataVisualizer(QMainWindow):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("GPS and Robot Pose Data Visualizer")
        self.setGeometry(100, 100, 800, 600)

        self.figure, self.axes = plt.subplots(4, 1, figsize=(8, 8))
        self.canvas = FigureCanvas(self.figure)
        self.canvas.setSizePolicy(self.canvas.sizePolicy().Expanding, self.canvas.sizePolicy().Expanding)

        self.select_file_button = QPushButton("Select Data File")
        self.select_file_button.clicked.connect(self.open_file_dialog)

        layout = QVBoxLayout()
        layout.addWidget(self.canvas)
        layout.addWidget(self.select_file_button)

        central_widget = QWidget(self)
        central_widget.setLayout(layout)
        self.setCentralWidget(central_widget)

    def open_file_dialog(self):
        options = QFileDialog.Options()
        options |= QFileDialog.ReadOnly
        data_file, _ = QFileDialog.getOpenFileName(self, "Select Data File", "", "Text files (*.txt);;All Files (*)", options=options)

        if data_file:
            self.visualize_data(data_file)

    def visualize_data(self, data_file):
        self.figure.clf()
        timestamps, longitudes, latitudes, altitudes, poses = [], [], [], [], []

        with open(data_file, 'r') as file:
            lines = file.readlines()
            for line in lines[1:]:  # Skip the header line
                parts = line.split()
                timestamps.append(float(parts[0]))
                longitudes.append(float(parts[1]))
                latitudes.append(float(parts[2]))
                altitudes.append(float(parts[3]))
                poses.append([float(p) for p in parts[4:]])

        timestamps = np.array(timestamps)
        longitudes = np.array(longitudes)
        latitudes = np.array(latitudes)
        altitudes = np.array(altitudes)
        poses = np.array(poses)

        self.axes[0].plot(timestamps, longitudes, label='Longitude')
        self.axes[0].plot(timestamps, latitudes, label='Latitude')
        self.axes[0].set_ylabel('Longitude / Latitude')
        self.axes[0].legend()

        self.axes[1].plot(timestamps, altitudes, label='Altitude')
        self.axes[1].set_ylabel('Altitude')
        self.axes[1].legend()

        for i in range(4):
            self.axes[2].plot(timestamps, poses[:, i + 4], label=f'Pose {i+1}')

        self.axes[2].set_xlabel('Timestamp')
        self.axes[2].set_ylabel('Position')
        self.axes[2].legend()

        for i in range(4, 8):
            self.axes[3].plot(timestamps, poses[:, i], label=f'Pose {i+1}')

        self.axes[3].set_xlabel('Timestamp')
        self.axes[3].set_ylabel('Quaternion Components')
        self.axes[3].legend()

        self.canvas.draw()

if __name__ == '__main__':
    app = QApplication(sys.argv)
    app.setStyle('Fusion')  # Use the Fusion style
    window = DataVisualizer()
    window.show()
    sys.exit(app.exec_())
