import sys
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import numpy as np
from PyQt6.QtWidgets import (
    QApplication,
    QWidget,
    QVBoxLayout,
    QPushButton,
    QHBoxLayout,
    QLabel
)
import pyqtgraph.opengl as gl
from PyQt6.QtCore import QThread, pyqtSignal, QObject


class MapSubscriber(Node, QObject):
    map_received = pyqtSignal(np.ndarray, int, int)

    def __init__(self):
        Node.__init__(self, 'map_subscriber')
        QObject.__init__(self)

        self.subscription = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )

    def map_callback(self, msg):
        try:
            width = msg.info.width
            height = msg.info.height
            data = np.array(msg.data, dtype=np.int8).reshape((height, width))

            # Convert occupancy data to grayscale
            grayscale = np.where(data == -1, 128, np.where(data == 0, 255, 0)).astype(np.uint8)
            image_data = np.stack((grayscale, grayscale, grayscale, np.full_like(grayscale, 255)), axis=-1)

            self.map_received.emit(image_data, width, height)
        except Exception as e:
            print(f"Hata alındı: {e}")


class Ros2Thread(QThread):
    def __init__(self, node):
        super().__init__()
        self.node = node

    def run(self):
        rclpy.spin(self.node)  # Spin the ROS2 event loop


class ThreeDWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("ROS2 Map 3D View")
        self.setGeometry(100, 100, 1280, 720)

        # Store map dimensions
        self.map_width = 0
        self.map_height = 0

        # === Style the entire window in a dark theme ===
        self.setStyleSheet("""
            QWidget {
                background-color: #121212; 
                color: white; 
            }
        """)

        # Calculate bottom bar height
        self.bar_height = self.height() // 12

        # Calculate area widths
        self.left_area_width = self.width() // 3
        self.right_area_width = self.width() - self.left_area_width

        # === Create bottom area ===
        self.bottom_area = QWidget(self)
        self.bottom_area.setGeometry(0, self.height() - self.bar_height,
                                     self.width(), self.bar_height)
        self.bottom_area.setStyleSheet("""
            background-color: #181818;
            border-top: 1px solid #333;
        """)
        self.bottom_layout = QHBoxLayout(self.bottom_area)
        self.bottom_layout.setContentsMargins(10, 5, 10, 5)
        self.bottom_layout.setSpacing(20)

        # === Create left area for buttons/labels ===
        self.left_area = QWidget(self)
        self.left_area.setGeometry(0, 0, self.left_area_width, self.height() - self.bar_height)
        self.left_area.setStyleSheet("""
            background-color: #1f1f1f; 
            border-right: 1px solid #333;
        """)
        self.left_layout = QVBoxLayout(self.left_area)
        self.left_layout.setContentsMargins(20, 20, 20, 20)
        self.left_layout.setSpacing(20)

        # == Buttons: Start, Pause, Stop ==
        self.start_button = QPushButton("Start", self.left_area)
        self.start_button.setStyleSheet("""
            QPushButton {
                background-color: #2b2b2b;
                border: 1px solid #444;
                border-radius: 8px;
                font-size: 16px;
                padding: 10px;
            }
            QPushButton:hover {
                background-color: #3c3c3c;
            }
        """)
        self.left_layout.addWidget(self.start_button)

        self.pause_button = QPushButton("Pause", self.left_area)
        self.pause_button.setStyleSheet("""
            QPushButton {
                background-color: #2b2b2b;
                border: 1px solid #444;
                border-radius: 8px;
                font-size: 16px;
                padding: 10px;
            }
            QPushButton:hover {
                background-color: #3c3c3c;
            }
        """)
        self.left_layout.addWidget(self.pause_button)

        self.stop_button = QPushButton("Stop", self.left_area)
        self.stop_button.setStyleSheet("""
            QPushButton {
                background-color: #2b2b2b;
                border: 1px solid #444;
                border-radius: 8px;
                font-size: 16px;
                padding: 10px;
            }
            QPushButton:hover {
                background-color: #3c3c3c;
            }
        """)
        self.left_layout.addWidget(self.stop_button)

        # == Battery Label (smaller) ==
        self.battery_label = QLabel("🔋 Battery: 80%", self.left_area)
        self.battery_label.setStyleSheet("""
            QLabel {
                font-size: 14px;
                color: #FFFFFF;
                background-color: #2b2b2b;
                border: 1px solid #4CAF50;
                border-radius: 6px;
                padding: 4px 8px;
            }
        """)
        self.left_layout.addWidget(self.battery_label)

        # == Temperature Label ==
        self.temperature_label = QLabel("🌡 Temperature: 25°C", self.left_area)
        self.temperature_label.setStyleSheet("""
            QLabel {
                font-size: 14px;
                color: #FFFFFF;
                background-color: #2b2b2b;
                border: 1px solid #4CAF50;
                border-radius: 6px;
                padding: 4px 8px;
            }
        """)
        self.left_layout.addWidget(self.temperature_label)

        # == Voltage + Ampere side by side ==
        voltage_current_layout = QHBoxLayout()
        self.voltage_label = QLabel("⚡ Voltage: 12.6V", self.left_area)
        self.voltage_label.setStyleSheet("""
            QLabel {
                font-size: 14px;
                color: #FFFFFF;
                background-color: #2b2b2b;
                border: 1px solid #4CAF50;
                border-radius: 6px;
                padding: 4px 8px;
            }
        """)
        voltage_current_layout.addWidget(self.voltage_label)

        self.current_label = QLabel("⚡ Current: 2.5A", self.left_area)
        self.current_label.setStyleSheet("""
            QLabel {
                font-size: 14px;
                color: #FFFFFF;
                background-color: #2b2b2b;
                border: 1px solid #4CAF50;
                border-radius: 6px;
                padding: 4px 8px;
            }
        """)
        voltage_current_layout.addWidget(self.current_label)

        self.left_layout.addLayout(voltage_current_layout)

        # === Create right area (3D view) ===
        self.right_area = QWidget(self)
        self.right_area.setGeometry(self.left_area_width, 0,
                                    self.right_area_width,
                                    self.height() - self.bar_height)
        self.right_area.setStyleSheet("background-color: black;")
        self.right_layout = QVBoxLayout(self.right_area)
        self.right_layout.setContentsMargins(0, 0, 0, 0)
        self.right_layout.setSpacing(0)

        # == 3D ViewWidget ==
        self.view_widget = gl.GLViewWidget()
        self.view_widget.setBackgroundColor((50, 50, 50))
        self.view_widget.setCameraPosition(distance=200)
        self.right_layout.addWidget(self.view_widget)

        self.image_item = None

        # == "Top-Down View" button (bottom bar) ==
        self.topdown_button = QPushButton("Top-Down View", self.bottom_area)
        self.topdown_button.setStyleSheet("""
            QPushButton {
                background-color: #2b2b2b;
                border: 1px solid #444;
                border-radius: 8px;
                font-size: 16px;
                padding: 10px;
            }
            QPushButton:hover {
                background-color: #3c3c3c;
            }
        """)
        self.topdown_button.clicked.connect(self.set_top_down_view)
        self.bottom_layout.addWidget(self.topdown_button)

        # === Initialize ROS subscription ===
        self.map_subscriber = MapSubscriber()
        self.map_subscriber.map_received.connect(self.add_map_plane)

        # === Start ROS thread ===
        self.ros_thread = Ros2Thread(self.map_subscriber)
        self.ros_thread.start()

    def add_map_plane(self, image_data, width, height):
        """Create or update the GLImageItem with new map data."""
        if self.image_item:
            self.view_widget.removeItem(self.image_item)

        self.map_width = width
        self.map_height = height

        # Flip the image vertically for correct orientation in OpenGL
        image_data = np.flipud(image_data)
        image_data = np.ascontiguousarray(image_data, dtype=np.uint8)

        self.image_item = gl.GLImageItem(image_data)

        # Translate & scale to fit in view
        self.image_item.translate(-width / 8, -height / 4, 0)
        self.image_item.scale(200 / width, 200 / height, 1)

        self.view_widget.addItem(self.image_item)

    def set_top_down_view(self):
        """Set the camera to a top-down view over the map."""
        if self.map_width == 0 or self.map_height == 0:
            print("Harita henüz yüklenmedi.")
            return

        distance = max(self.map_width, self.map_height) / 2
        # Elevation = 89 for near top-down, azimuth = 0 facing 'north'
        self.view_widget.setCameraPosition(distance=distance, elevation=89, azimuth=0)


if __name__ == "__main__":
    app = QApplication(sys.argv)
    rclpy.init()

    window = ThreeDWindow()
    window.show()

    try:
        sys.exit(app.exec())
    finally:
        rclpy.shutdown()
