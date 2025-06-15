import yaml
import numpy as np
from PyQt6.QtWidgets import (
    QDialog, QVBoxLayout, QLabel, QSlider, QComboBox, QFileDialog,
    QPushButton, QHBoxLayout, QLineEdit, QMessageBox, QGroupBox
)
from PyQt6.QtCore import pyqtSignal
from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtGui import QImage, QPixmap


class CVCalibrationDialog(QDialog):
    start_calibration_signal = pyqtSignal()
    def __init__(self, yaml_path):
        super().__init__()
        self.setWindowTitle("CV Calibration Settings")
        self.setFixedSize(480, 600)
        self.yaml_path = yaml_path

        self.defaults = {
            "exposure": 10000,
            "size": "HD",
            "vehicle_model_path": "/home/ubuntu/Desktop/Model/Heavy_duty.pt",
            "vehicle_class_path": "/home/ubuntu/Desktop/Model/Heavy_duty.txt",
            "person_model_path": "/home/ubuntu/Desktop/Model/Person.pt",
            "person_class_path": "/home/ubuntu/Desktop/Model/Person.txt",
            "sign_model_path": "/home/ubuntu/Desktop/Model/Sign.pt",
            "sign_class_path": "/home/ubuntu/Desktop/Model/Sign.txt"
        }

        self.settings = self.defaults.copy()
        self.load_yaml()

        layout = QVBoxLayout()
        layout.setContentsMargins(12, 12, 12, 12)
        layout.setSpacing(10)

        # Exposure slider
        exposure_layout = QHBoxLayout()
        exposure_label = QLabel("Exposure:")
        self.exposure_slider = QSlider(Qt.Orientation.Horizontal)
        self.exposure_slider.setMinimum(0)
        self.exposure_slider.setMaximum(30000)
        self.exposure_slider.setValue(self.settings.get("exposure", 10000))  # fallback if key missing
        self.exposure_value_label = QLabel(str(self.exposure_slider.value()))

        # Update label when slider value changes
        self.exposure_slider.valueChanged.connect(
            lambda value: self.exposure_value_label.setText(str(value))
        )

        exposure_layout.addWidget(exposure_label)
        exposure_layout.addWidget(self.exposure_slider)
        exposure_layout.addWidget(self.exposure_value_label)
        layout.addLayout(exposure_layout)


        # Size dropdown
        layout.addWidget(QLabel("Image Size"))
        self.size_dropdown = QComboBox()
        self.size_dropdown.addItems(["HD", "Full HD", "4MP"])
        self.size_dropdown.setCurrentText(self.settings["size"])
        self.size_dropdown.setStyleSheet("""
            QComboBox {
                background-color: #f8f8f2;
                color: #272822;
                padding: 5px;
                border-radius: 4px;
            }
            QComboBox QAbstractItemView {
                background-color: #ffffff;
                color: #000000;
                selection-background-color: #cceeff;
            }
        """)

        layout.addWidget(self.size_dropdown)

        
        # Model groups
        vehicle_group, self.vehicle_model_path_line, self.vehicle_class_path_line = self.create_model_section(
            "Vehicle Model", self.settings["vehicle_model_path"], self.settings["vehicle_class_path"]
        )
        layout.addWidget(vehicle_group)

        person_group, self.person_model_path_line, self.person_class_path_line = self.create_model_section(
            "Person Model", self.settings["person_model_path"], self.settings["person_class_path"]
        )
        layout.addWidget(person_group)

        sign_group, self.sign_model_path_line, self.sign_class_path_line = self.create_model_section(
            "Sign Model", self.settings["sign_model_path"], self.settings["sign_class_path"]
        )
        layout.addWidget(sign_group)

        # Start Calibration Button
        self.calibration_button = QPushButton("Start Calibration")
        self.calibration_button.clicked.connect(self.start_calibration)
        layout.addWidget(self.calibration_button)

        # Save and Reset buttons
        action_layout = QHBoxLayout()
        save_button = QPushButton("Save to YAML")
        save_button.clicked.connect(self.save_yaml)
        reset_button = QPushButton("Reset to Default")
        reset_button.clicked.connect(self.reset_defaults)
        action_layout.addWidget(save_button)
        action_layout.addWidget(reset_button)
        layout.addLayout(action_layout)

        self.setLayout(layout)

        # Styling
        self.setStyleSheet("""
            QLabel {
                font-weight: bold;
                color: #777;
            }

            QLabel#exposureLabel,
            QLabel#sizeLabel {
                color: black;
            }

            QGroupBox#vehicleGroup::title,
            QGroupBox#personGroup::title,
            QGroupBox#signGroup::title {
                color: black;
                font-weight: bold;
            }

            QGroupBox {
                border: 1px solid gray;
                border-radius: 6px;
                margin-top: 10px;
            }

            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 3px;
            }

            QLineEdit, QComboBox, QSlider {
                background-color: white;
                padding: 5px;
                border: 1px solid #ccc;
                border-radius: 4px;
            }

            QPushButton {
                background-color: #0066cc;
                color: white;
                padding: 6px 12px;
                border-radius: 6px;
            }

            QPushButton:hover {
                background-color: #004c99;
            }
        """)

        self.timer = QTimer()
        self.timer.timeout.connect(self.capture_image)
        self.capture_count = 0

    def create_model_section(self, title, model_path, class_path):
        group = QGroupBox(title)
        vbox = QVBoxLayout()

        # Model path
        model_layout = QHBoxLayout()
        model_line = QLineEdit(model_path)
        model_line.setReadOnly(True)
        model_line.setStyleSheet("""
            QLineEdit {
                background-color: #f8f8f2;
                color: #272822;
                padding: 5px;
                border-radius: 4px;
            }
        """)
        model_button = QPushButton("Browse Model")
        model_button.clicked.connect(lambda: self.browse_model_file(model_line))
        model_layout.addWidget(model_line)
        model_layout.addWidget(model_button)
        vbox.addLayout(model_layout)

        # Class path
        class_layout = QHBoxLayout()
        class_line = QLineEdit(class_path)
        class_line.setReadOnly(True)
        class_line.setStyleSheet("""
            QLineEdit {
                background-color: #f8f8f2;
                color: #272822;
                padding: 5px;
                border-radius: 4px;
            }
        """)
        class_button = QPushButton("Browse Class")
        class_button.clicked.connect(lambda: self.browse_class_file(class_line))
        class_layout.addWidget(class_line)
        class_layout.addWidget(class_button)
        vbox.addLayout(class_layout)

        group.setLayout(vbox)
        return group, model_line, class_line


    def load_yaml(self):
        try:
            with open(self.yaml_path, 'r') as f:
                loaded = yaml.safe_load(f)
                if loaded:
                    self.settings.update(loaded)
        except Exception as e:
            print("Could not load YAML file:", e)

    def save_yaml(self):
        self.settings["exposure"] = self.exposure_slider.value()
        self.settings["size"] = self.size_dropdown.currentText()
        self.settings["vehicle_model_path"] = self.vehicle_model_path_line.text()
        self.settings["vehicle_class_path"] = self.vehicle_class_path_line.text()
        self.settings["person_model_path"] = self.person_model_path_line.text()
        self.settings["person_class_path"] = self.person_class_path_line.text()
        self.settings["sign_model_path"] = self.sign_model_path_line.text()
        self.settings["sign_class_path"] = self.sign_class_path_line.text()

        try:
            with open(self.yaml_path, 'w') as f:
                yaml.dump(self.settings, f)
            QMessageBox.information(self, "Saved", "Settings saved successfully.")
            self.accept()
        except Exception as e:
            QMessageBox.warning(self, "Error", f"Could not save YAML file:\n{e}")

    def browse_model_file(self, line_edit):
        file_path, _ = QFileDialog.getOpenFileName(self, "Select Model File", "", "Model Files (*.tflite *.onnx *.pb);;All Files (*)")
        if file_path:
            line_edit.setText(file_path)
            line_edit.setToolTip(file_path)  # Show full path on hover

    def browse_class_file(self, line_edit):
        file_path, _ = QFileDialog.getOpenFileName(self, "Select Class File", "", "Text Files (*.txt);;All Files (*)")
        if file_path:
            line_edit.setText(file_path)
            line_edit.setToolTip(file_path)  # Show full path on hover

    def reset_defaults(self):
        self.exposure_slider.setValue(self.defaults["exposure"])
        self.size_dropdown.setCurrentText(self.defaults["size"])
        self.vehicle_model_path_line.setText(self.defaults["vehicle_model_path"])
        self.vehicle_class_path_line.setText(self.defaults["vehicle_class_path"])
        self.person_model_path_line.setText(self.defaults["person_model_path"])
        self.person_class_path_line.setText(self.defaults["person_class_path"])
        self.sign_model_path_line.setText(self.defaults["sign_model_path"])
        self.sign_class_path_line.setText(self.defaults["sign_class_path"])

    def start_calibration(self):
        self.capture_count = 0
        self.timer.start(5000)
        self.start_calibration_signal.emit()

    def capture_image(self):
        self.capture_count += 1
        dummy_img = (np.random.rand(480, 640) * 255).astype(np.uint8)
        height, width = dummy_img.shape
        q_img = QImage(dummy_img.data, width, height, width, QImage.Format.Format_Grayscale8)
        pixmap = QPixmap.fromImage(q_img)

        img_label = QLabel()
        img_label.setPixmap(pixmap)
        img_label.setWindowTitle(f"Capture {self.capture_count}")
        img_label.resize(pixmap.width(), pixmap.height())
        img_label.show()

        if self.capture_count >= 3:
            self.timer.stop()
