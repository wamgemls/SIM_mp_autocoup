import sys
import random
from PyQt5.QtGui import QIcon
from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget, QVBoxLayout, QPushButton,QSizePolicy
from PyQt5.QtCore import Qt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        # Set the window title and icon
        self.setWindowTitle("PyQt5 Matplotlib Example")
        self.setWindowIcon(QIcon("icon.png"))

        # Set the stylesheet for the main window
        self.setStyleSheet("""
            QMainWindow {
                border: 2px solid #007bff;
                border-image: url();
            }
            QWidget {
                background-color: #2d2d2d;
                color: #f0f0f0;
            }
            QPushButton {
                background-color: #007bff;
                color: #f0f0f0;
                border-radius: 4px;
                padding: 8px 16px;
                font-size: 16px;
                font-weight: bold;
                margin-top: 16px;
            }
            QPushButton:hover {
                background-color: #0069d9;
            }
            """)

        # Create the main widget and layout
        self.main_widget = QWidget()
        self.layout = QVBoxLayout(self.main_widget)

        # Create the figure
        self.figure = Figure(figsize=(5, 5), dpi=100, facecolor='#2d2d2d')

        # Create the canvas
        self.canvas = FigureCanvas(self.figure)

        # Add the canvas to the layout
        self.layout.addWidget(self.canvas)

        # Create a button to update the plot data
        self.button = QPushButton('Update Plot')
        self.button.setSizePolicy(QSizePolicy.Minimum, QSizePolicy.Fixed)
        self.button.clicked.connect(self.update_plot)
        self.layout.addWidget(self.button, alignment=Qt.AlignRight)

        # Set the main widget as the central widget
        self.setCentralWidget(self.main_widget)

    def update_plot(self):
        # Clear the figure
        self.figure.clear()

        # Generate random data
        x = [random.random() for i in range(10)]
        y = [random.random() for i in range(10)]

        # Add the data to the plot
        ax = self.figure.add_subplot(111)
        ax.scatter(x, y)

        # Draw the plot
        self.canvas.draw()


if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())