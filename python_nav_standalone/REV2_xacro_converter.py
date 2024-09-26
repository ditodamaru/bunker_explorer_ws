import sys
import os
import subprocess
from PyQt5.QtWidgets import QApplication, QMainWindow, QPushButton, QLabel, QFileDialog, QLineEdit, QMessageBox

class XacroGeneratorApp(QMainWindow):
    def __init__(self):
        super().__init__()

        self.initUI()

    def initUI(self):
        self.setGeometry(100, 100, 400, 250)  # Increased height for the new button
        self.setWindowTitle('Xacro Generator')

        self.file_path_label = QLabel('No xacro file selected', self)
        self.file_path_label.setGeometry(20, 20, 360, 30)

        self.browse_button = QPushButton('Browse Xacro File', self)
        self.browse_button.setGeometry(20, 70, 150, 30)
        self.browse_button.clicked.connect(self.browse_xacro_file)

        self.select_directory_button = QPushButton('Select Save Directory', self)
        self.select_directory_button.setGeometry(20, 120, 150, 30)
        self.select_directory_button.clicked.connect(self.select_save_directory)

        self.generate_button = QPushButton('Generate URDF', self)
        self.generate_button.setGeometry(20, 170, 150, 30)
        self.generate_button.clicked.connect(self.generate_urdf)

        self.save_as_label = QLabel('Save URDF as:', self)
        self.save_as_label.setGeometry(200, 70, 100, 30)

        self.save_as_lineedit = QLineEdit(self)
        self.save_as_lineedit.setGeometry(200, 100, 150, 30)


    def browse_xacro_file(self):
        options = QFileDialog.Options()
        xacro_file, _ = QFileDialog.getOpenFileName(self, 'Select Xacro File', '', 'Xacro Files (*.xacro)', options=options)
        
        if xacro_file:
            self.xacro_file = xacro_file
            self.file_path_label.setText(f'Selected Xacro File: {os.path.basename(self.xacro_file)}')

    def select_save_directory(self):
        options = QFileDialog.Options()
        save_directory = QFileDialog.getExistingDirectory(self, 'Select Save Directory', '', options=options)
        
        if save_directory:
            self.save_directory = save_directory

    def generate_urdf(self):
        if not hasattr(self, 'xacro_file'):
            QMessageBox.warning(self, 'Error', 'No xacro file selected.')
            return
        
        if not hasattr(self, 'save_directory'):
            QMessageBox.warning(self, 'Error', 'No save directory selected.')
            return
        
        output_name = self.save_as_lineedit.text()
        if not output_name:
            QMessageBox.warning(self, 'Error', 'Please provide a name for the URDF file.')
            return

        urdf_file = os.path.join(self.save_directory, f'{output_name}.urdf')
        cmd = f"rosrun xacro xacro {self.xacro_file} --inorder > {urdf_file}"
        try:
            subprocess.run(cmd, shell=True, check=True)
            QMessageBox.information(self, 'Success', f'URDF file generated as: {urdf_file}\nFinished.')
        except subprocess.CalledProcessError:
            QMessageBox.critical(self, 'Error', 'An error occurred while generating the URDF file.')

if __name__ == '__main__':
    app = QApplication(sys.argv)
    xacro_app = XacroGeneratorApp()
    xacro_app.show()
    sys.exit(app.exec_())