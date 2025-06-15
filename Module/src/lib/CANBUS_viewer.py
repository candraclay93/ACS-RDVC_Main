import sys
import time
import numpy as np
from queue import Empty

from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QLabel, QPushButton,
    QVBoxLayout, QHBoxLayout, QTableWidget, QTableWidgetItem, QHeaderView, QGridLayout
)
from PyQt6.QtCore import QTimer, Qt
from PyQt6.QtGui import QPixmap 
import pyqtgraph as pg
from lib.CAM_reader import CameraFeed

class CANPlotter(pg.PlotWidget):
    def __init__(self, data_queue):
        super().__init__(background='k')

        self.setLabel('bottom', 'Camera Coordinate (X, meters)')
        self.setLabel('left', 'Depth from Camera (Z, meters)')
        self.setRange(xRange=(-3.0, 3.0), yRange=(0, 5.0))
        self.showGrid(x=True, y=True, alpha=0.3)
        self.setAspectLocked(False)

        self.scatter = pg.ScatterPlotItem(size=10, brush=pg.mkBrush(249, 38, 114, 200))
        self.addItem(self.scatter)

        self.data_queue = data_queue
        self.objects = {}
        self.arrows = []
        self.labels = []
        
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_plot)
        self.timer.start(100)
    
    def update_plot(self):
        current_time = time.time()
        new_data = []

        for arrow in self.arrows:
            self.removeItem(arrow)
        self.arrows.clear()

        for label in self.labels:
            self.removeItem(label)
        self.labels.clear()

        while not self.data_queue.empty():
            try:
                parsed = self.data_queue.get_nowait()
                obj_id = parsed["id"]
                x = parsed["dist_lat"]
                z = parsed["dist_long"]
                vx = parsed["vel_rel_lat"]
                vz = parsed["vel_rel_long"]

                new_data.append(parsed)

                if obj_id not in self.objects:
                    self.objects[obj_id] = []
                self.objects[obj_id].append((current_time, x, z, vx, vz))
            except Empty:
                break

        expired_ids = []
        for obj_id, entries in self.objects.items():
            self.objects[obj_id] = [
                (ts, x, z, vx, vz) for ts, x, z, vx, vz in entries if current_time - ts <= 5
            ]
            if not self.objects[obj_id]:
                expired_ids.append(obj_id)

        for obj_id in expired_ids:
            del self.objects[obj_id]

        x_vals = []
        z_vals = []

        for obj_id, points in self.objects.items():
            if points:
                _, x, z, vx, vz = points[-1]
                x_vals.append(x)
                z_vals.append(z)

                arrow = pg.ArrowItem(pos=(x, z), angle=np.degrees(np.arctan2(vz, vx)),
                                    headLen=10, tipAngle=30, baseAngle=20,
                                    brush=pg.mkBrush(166, 226, 46), pen=pg.mkPen(166, 226, 46))
                self.addItem(arrow)
                self.arrows.append(arrow)

                label = pg.TextItem(text=str(obj_id), color=(248, 248, 242), anchor=(0.5, -0.5))
                label.setPos(x, z)
                self.addItem(label)
                self.labels.append(label)

        self.scatter.setData(x=x_vals, y=z_vals)
        return new_data
    
class DashboardWindow(QMainWindow):
    def __init__(self, data_queue, record_queue):
        super().__init__()
        self.setWindowTitle("Radar Dashboard")
        self.setGeometry(100, 100, 1200, 700)

        self.setStyleSheet("""
            QWidget {
                background-color: #272822;
                color: #f8f8f2;
            }
            QPushButton {
                background-color: #66d9ef;
                color: #272822;
                font-weight: bold;
                border: none;
                border-radius: 5px;
            }
            QPushButton:hover {
                background-color: #a1efe4;
            }
            QLabel {
                color: #f8f8f2;
            }
            QTableWidget {
                background-color: #3e3d32;
                color: #f8f8f2;
                gridline-color: #75715e;
                font-size: 12px;
            }
            QHeaderView::section {
                background-color: #66d9ef;
                color: #272822;
                font-weight: bold;
                padding: 4px;
                border: 1px solid #75715e;
            }
        """)

        central_widget = QWidget()
        self.setCentralWidget(central_widget)

        layout = QHBoxLayout(central_widget)

        # Left Panel: Buttons and Table
        left_panel = QVBoxLayout()
        layout.addLayout(left_panel, 3)

        # Buttons
        button_layout = QVBoxLayout()
        button_layout.setSpacing(8)
        button_layout.setContentsMargins(5, 5, 5, 5)
        for text in ["CV Calibration", "Radar Test", "Test CAN", "Network Setup", "ACS Run"]:
            btn = QPushButton(text)
            btn.setFixedHeight(40)
            button_layout.addWidget(btn)
        button_layout.addStretch()
        left_panel.addLayout(button_layout)

        # Video and Table section
        from CAM_reader import CameraFeed  # Make sure camera_feed.py is in same directory
        self.video_label = QLabel()
        self.video_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.video_label.setFixedSize(400, 300)  # Match your scaling
        self.video_label.setStyleSheet("background-color: #49483e; color: #f8f8f2;")
        left_panel.addWidget(self.video_label)

        # Initialize and start the camera
        self.camera_thread = CameraFeed(record_queue = record_queue)
        self.camera_thread.init_camera()
        self.camera_thread.frame_ready.connect(self.update_camera_frame)
        self.camera_thread.start_capture()
        
        """
        video_label = QLabel("Video stream from OpenCV")
        video_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        video_label.setFixedHeight(400)
        video_label.setStyleSheet("background-color: #49483e; color: #f8f8f2;")
        left_panel.addWidget(video_label)
        """

        self.table_widget = QTableWidget(0, 6)
        self.table_widget.setHorizontalHeaderLabels(["ID", "Lat dist", "Long dist", "vel Lat", "vel Long", "RCS"])
        self.table_widget.horizontalHeader().setStretchLastSection(True)
        self.table_widget.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Stretch)
        left_panel.addWidget(self.table_widget)

        # Right Panel: Radar Grid
        right_panel = QVBoxLayout()
        layout.addLayout(right_panel, 5)

        radar_label = QLabel("RADAR GRID")
        radar_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        radar_label.setStyleSheet("background-color: #66d9ef; color: #272822; font-weight: bold;")
        radar_label.setFixedHeight(30)
        right_panel.addWidget(radar_label)

        self.plot_widget = CANPlotter(data_queue)
        self.timer = QTimer()
        self.timer.timeout.connect(self.refresh_dashboard)
        self.timer.start(10)

        right_panel.addWidget(self.plot_widget)
    
    def refresh_dashboard(self):
        new_objects = self.plot_widget.update_plot()

        for parsed in new_objects:
            obj_id = parsed["id"]
            lat = f"{parsed['dist_lat']:.2f}"
            long = f"{parsed['dist_long']:.2f}"
            vlat = f"{parsed['vel_rel_lat']:.2f}"
            vlong = f"{parsed['vel_rel_long']:.2f}"
            raw = f"{parsed['rcs']:.2f}"

            found = False
            for row in range(self.table_widget.rowCount()):
                if self.table_widget.item(row, 0).text() == str(obj_id):
                    self.table_widget.setItem(row, 1, QTableWidgetItem(lat))
                    self.table_widget.setItem(row, 2, QTableWidgetItem(long))
                    self.table_widget.setItem(row, 3, QTableWidgetItem(vlat))
                    self.table_widget.setItem(row, 4, QTableWidgetItem(vlong))
                    self.table_widget.setItem(row, 5, QTableWidgetItem(raw))
                    found = True
                    break

            if not found:
                row_pos = self.table_widget.rowCount()
                self.table_widget.insertRow(row_pos)
                self.table_widget.setItem(row_pos, 0, QTableWidgetItem(str(obj_id)))
                self.table_widget.setItem(row_pos, 1, QTableWidgetItem(lat))
                self.table_widget.setItem(row_pos, 2, QTableWidgetItem(long))
                self.table_widget.setItem(row_pos, 3, QTableWidgetItem(vlat))
                self.table_widget.setItem(row_pos, 4, QTableWidgetItem(vlong))
                self.table_widget.setItem(row_pos, 5, QTableWidgetItem(raw))
    
    def update_camera_frame(self, image):
        self.video_label.setPixmap(QPixmap.fromImage(image))
    
    def closeEvent(self, event):
        if hasattr(self, 'camera_thread'):
            self.camera_thread.stop()
        event.accept()    

    class CameraViewer(QWidget):
        def __init__(self):
            super().__init__()
            self.setWindowTitle("Hikrobot Camera Viewer")
            self.setGeometry(100, 100, 1280, 720)
            
            # Setup UI
            self.video_label = QLabel()
            self.video_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
            layout = QVBoxLayout()
            layout.addWidget(self.video_label)
            self.setLayout(layout)
            
            # Setup camera
            self.camera = CameraFeed
            try:
                self.camera.init_camera()
                self.camera.frame_ready.connect(self.update_frame)
                self.camera.start_capture()
            except Exception as e:
                print(f"Camera initialization failed: {e}")
                self.close()

def run_gui(data_queue, record_queue):
    app = QApplication(sys.argv)
    win = DashboardWindow(data_queue, record_queue)
    win.show()
    sys.exit(app.exec())
