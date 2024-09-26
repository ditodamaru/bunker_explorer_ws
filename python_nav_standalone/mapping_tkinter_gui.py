import subprocess
import tkinter as tk
from tkinter import scrolledtext
import threading

# Define the terminal commands to be executed
commands = [
    "roslaunch realsense_explorer_perception start_rs_camera.launch filters:=pointcloud",
    "roslaunch realsense_explorer_perception rtab_mapping.launch",
    "roslaunch realsense_explorer_perception rtab_mapping.launch localization:=true",
    "roslaunch realsense_explorer_navigation move_base.launch",
]

# Function to execute a terminal command and capture the output
def execute_command(command, output_widget):
    process = subprocess.Popen(
        command,
        shell=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        bufsize=1,
    )
    for line in process.stdout:
        output_widget.insert(tk.END, line)
        output_widget.see(tk.END)
    process.stdout.close()
    process.stderr.close()

# Function to launch terminal commands in separate threads
def start_threads():
    for i, command in enumerate(commands):
        thread = threading.Thread(target=execute_command, args=(command, outputs[i]))
        thread.start()

# Create the GUI
root = tk.Tk()
root.title("Terminal Launcher")

# Create text widgets to display output
outputs = []
for _ in commands:
    output_widget = scrolledtext.ScrolledText(root, wrap=tk.WORD, height=10, width=50)
    output_widget.pack(padx=10, pady=5)
    outputs.append(output_widget)

# Create a button to start the terminal commands
start_button = tk.Button(root, text="Start", command=start_threads)
start_button.pack(pady=10)

root.mainloop()
