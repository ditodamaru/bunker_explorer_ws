import os
import sys
from PyQt5.QtWidgets import QApplication, QMainWindow, QLabel, QTextEdit, QPushButton, QFileDialog, QVBoxLayout, QWidget, QScrollArea

class URDFGeneratorApp(QMainWindow):
    def __init__(self):
        super().__init__()

        self.initUI()

    def initUI(self):
        self.setGeometry(100, 100, 600, 400)
        self.setWindowTitle('Launch Editor')

        self.scroll_area = QScrollArea(self)
        self.scroll_area.setGeometry(20, 40, 560, 180)
        self.scroll_area.setWidgetResizable(True)

        self.scroll_widget = QWidget()
        self.scroll_area.setWidget(self.scroll_widget)

        self.layout = QVBoxLayout(self.scroll_widget)

        self.launch_script_label = QLabel('Launch Script:', self)
        self.layout.addWidget(self.launch_script_label)

        self.launch_script_text = QTextEdit(self)
        self.layout.addWidget(self.launch_script_text)

        self.browse_button = QPushButton('Open Launch Script', self)
        self.browse_button.clicked.connect(self.open_launch_file)
        self.layout.addWidget(self.browse_button)

        self.save_button = QPushButton('Save and Update Launch Script', self)
        self.save_button.clicked.connect(self.save_and_update_launch_file)
        self.layout.addWidget(self.save_button)

        # Set the correct absolute path to the launch file
        self.launch_file_path = os.path.expanduser('~/catkin_devel_ws/src/bunker_explorer_bot/bunker_explorer_description/launch/robot_base_bringup.launch')
        self.open_launch_file()

    def open_launch_file(self):
        if self.launch_file_path:
            try:
                with open(self.launch_file_path, 'r') as f:
                    launch_content = f.read()
                    self.launch_script_text.setPlainText(launch_content)
            except Exception as e:
                print(f'Error opening launch script: {e}')
        else:
            print('Launch file path is not set.')

    def save_and_update_launch_file(self):
        if self.launch_file_path:
            launch_content = self.launch_script_text.toPlainText()
            urdf_output, _ = QFileDialog.getOpenFileName(self, 'Select URDF File', '', 'URDF Files (*.urdf)')

            try:
                updated_content = launch_content.replace(
                    '<param name="robot_description" textfile="$(find bunker_explorer_description)/urdf/REV3_bunker_realsense.urdf" />',
                    f'<param name="robot_description" textfile="{urdf_output}" />'
                )

                with open(self.launch_file_path, 'w') as f:
                    f.write(updated_content)

                print('Launch script saved and updated successfully.')
            except Exception as e:
                print(f'Error saving and updating launch script: {e}')
        else:
            print('Please set the launch file path.')

if __name__ == '__main__':
    app = QApplication(sys.argv)
    urdf_app = URDFGeneratorApp()
    urdf_app.show()
    sys.exit(app.exec_())
