import sys
from PyQt5.QtWidgets import QApplication, QMainWindow, QLabel, QTextEdit, QPushButton, QFileDialog

class URDFGeneratorApp(QMainWindow):
    def __init__(self):
        super().__init__()

        self.initUI()

    def initUI(self):
        self.setGeometry(100, 100, 600, 400)
        self.setWindowTitle('Launch Editor')

        self.launch_script_label = QLabel('Launch Script:', self)
        self.launch_script_label.move(20, 10)

        self.launch_script_text = QTextEdit(self)
        self.launch_script_text.setGeometry(20, 40, 560, 180)

        self.browse_button = QPushButton('Browse and Open Launch Script', self)
        self.browse_button.move(150, 230)
        self.browse_button.clicked.connect(self.browse_and_open_launch_file)

        self.save_button = QPushButton('Save and Update Launch Script', self)
        self.save_button.move(150, 280)
        self.save_button.clicked.connect(self.save_and_update_launch_file)

    def browse_and_open_launch_file(self):
        file_dialog = QFileDialog()
        launch_file_path, _ = file_dialog.getOpenFileName(self, 'Open Launch Script', '', 'Launch Files (*.launch)')

        if launch_file_path:
            with open(launch_file_path, 'r') as f:
                launch_content = f.read()
                self.launch_script_text.setPlainText(launch_content)

    def save_and_update_launch_file(self):
        launch_content = self.launch_script_text.toPlainText()

        try:
            launch_file_path, _ = QFileDialog.getSaveFileName(self, 'Save Launch Script As', '', 'Launch Files (*.launch)')

            if launch_file_path:
                with open(launch_file_path, 'w') as f:
                    f.write(launch_content)

                print('Launch script saved and updated successfully.')
        except Exception as e:
            print(f'Error saving and updating launch script: {e}')

if __name__ == '__main__':
    app = QApplication(sys.argv)
    urdf_app = URDFGeneratorApp()
    urdf_app.show()
    sys.exit(app.exec_())
