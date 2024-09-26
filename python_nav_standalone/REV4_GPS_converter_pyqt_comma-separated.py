import csv
import yaml
import sys
from PyQt5.QtWidgets import QApplication, QMainWindow, QLabel, QLineEdit, QPushButton, QVBoxLayout, QWidget, QFileDialog, QMessageBox

def convert_csv_to_yaml(csv_file, save_path):
    gps_data = []
    with open(csv_file, 'r') as csvfile:
        reader = csv.reader(csvfile)
        header = next(reader)  # Skip the header row
        for row in reader:
            x, y, _ = map(float, row)  # Assuming X and Y are the first two columns
            gps_data.append((x, y))

    yaml_data = {
        'header': {
            'seq': 1,
            'stamp': {
                'secs': 0,
                'nsecs': 0
            },
            'frame_id': 'map'
        },
        'poses': []
    }

    for idx, (x, y) in enumerate(gps_data):
        pose = {
            'header': {
                'seq': idx,
                'stamp': {
                    'secs': 0,
                    'nsecs': 0
                },
                'frame_id': 'map'
            },
            'pose': {
                'position': {
                    'x': x,
                    'y': y,
                    'z': 0.0
                },
                'orientation': {
                    'x': 0.0,
                    'y': 0.0,
                    'z': 0.0,
                    'w': 1.0
                }
            }
        }
        yaml_data['poses'].append(pose)

    yaml_output = yaml.dump(yaml_data)

    with open(save_path, 'w') as f:
        f.write(yaml_output)

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("GPS CSV to YAML Converter")
        self.setGeometry(100, 100, 400, 200)

        self.init_ui()

    def init_ui(self):
        self.csv_label = QLabel("CSV File:", self)
        self.csv_entry = QLineEdit(self)
        self.csv_browse_button = QPushButton("Browse", self)
        self.csv_browse_button.clicked.connect(self.browse_csv_file)

        self.save_label = QLabel("Save As:", self)
        self.save_entry = QLineEdit(self)
        self.save_browse_button = QPushButton("Browse", self)
        self.save_browse_button.clicked.connect(self.browse_save_location)

        self.convert_button = QPushButton("Convert", self)
        self.convert_button.clicked.connect(self.convert)

        layout = QVBoxLayout()
        layout.addWidget(self.csv_label)
        layout.addWidget(self.csv_entry)
        layout.addWidget(self.csv_browse_button)
        layout.addWidget(self.save_label)
        layout.addWidget(self.save_entry)
        layout.addWidget(self.save_browse_button)
        layout.addWidget(self.convert_button)

        central_widget = QWidget(self)
        central_widget.setLayout(layout)
        self.setCentralWidget(central_widget)

    def browse_csv_file(self):
        file_path, _ = QFileDialog.getOpenFileName(self, "Select CSV File", "", "CSV Files (*.csv)")
        self.csv_entry.setText(file_path)

    def browse_save_location(self):
        save_path, _ = QFileDialog.getSaveFileName(self, "Save YAML File", "", "YAML Files (*.yaml)")
        self.save_entry.setText(save_path)

    def convert(self):
        csv_file = self.csv_entry.text()
        save_path = self.save_entry.text()
        convert_csv_to_yaml(csv_file, save_path)
        QMessageBox.information(self, "Conversion Complete", "CSV to YAML conversion is complete!")

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())
