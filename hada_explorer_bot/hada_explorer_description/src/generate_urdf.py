import sys
from PyQt5.QtWidgets import QApplication, QMainWindow, QLabel, QLineEdit, QPushButton, QFileDialog
import subprocess  # Import the subprocess module


class URDFGeneratorApp(QMainWindow):
    def __init__(self):
        super().__init__()

        self.initUI()

    def initUI(self):
        self.setGeometry(100, 100, 400, 200)
        self.setWindowTitle('URDF Generator')

        self.xacro_label = QLabel('Xacro File:', self)
        self.xacro_label.move(20, 20)

        self.xacro_entry = QLineEdit(self)
        self.xacro_entry.move(120, 20)

        self.browse_button = QPushButton('Browse', self)
        self.browse_button.move(300, 20)
        self.browse_button.clicked.connect(self.browse_xacro_file)

        self.urdf_output_label = QLabel('URDF Output File:', self)
        self.urdf_output_label.move(20, 60)

        self.urdf_output_entry = QLineEdit(self)
        self.urdf_output_entry.move(120, 60)

        self.save_button = QPushButton('Save As', self)
        self.save_button.move(300, 60)
        self.save_button.clicked.connect(self.choose_urdf_output)

        self.generate_button = QPushButton('Generate URDF', self)
        self.generate_button.move(150, 120)
        self.generate_button.clicked.connect(self.generate_urdf)

    def browse_xacro_file(self):
        file_dialog = QFileDialog()
        xacro_file, _ = file_dialog.getOpenFileName(self, 'Select Xacro File', '', 'Xacro Files (*.xacro)')
        self.xacro_entry.setText(xacro_file)

    def choose_urdf_output(self):
        file_dialog = QFileDialog()
        urdf_output, _ = file_dialog.getSaveFileName(self, 'Save URDF File As', '', 'URDF Files (*.urdf)')
        self.urdf_output_entry.setText(urdf_output)

    def generate_urdf(self):
        xacro_file = self.xacro_entry.text()
        urdf_output = self.urdf_output_entry.text()
        # Generate URDF using Xacro
        try:
            xacro_command = f'xacro -o {urdf_output} {xacro_file}'
            subprocess.run(xacro_command, shell=True, check=True)
            print(f'Successfully generated URDF and saved to {urdf_output}')
        except subprocess.CalledProcessError:
            print('Error: Failed to generate URDF')

if __name__ == '__main__':
    app = QApplication(sys.argv)
    urdf_app = URDFGeneratorApp()
    urdf_app.show()
    sys.exit(app.exec_())
