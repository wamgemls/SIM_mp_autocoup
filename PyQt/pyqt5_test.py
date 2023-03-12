import sys
from PyQt5.QtWidgets import QApplication, QMainWindow, QTabWidget, QWidget, QVBoxLayout, QPushButton
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure

class MyWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        # create the main widget and layout
        self.main_widget = QWidget()
        self.layout = QVBoxLayout(self.main_widget)

        # create the tab widget
        self.tab_widget = QTabWidget()

        # create the first tab
        self.tab1 = QWidget()
        self.layout1 = QVBoxLayout(self.tab1)
        self.figure1 = Figure(figsize=(5, 5), dpi=100)
        self.canvas1 = FigureCanvas(self.figure1)
        self.layout1.addWidget(self.canvas1)

        # create the second tab
        self.tab2 = QWidget()
        self.layout2 = QVBoxLayout(self.tab2)
        self.figure2 = Figure(figsize=(5, 5), dpi=100)
        self.canvas2 = FigureCanvas(self.figure2)
        self.layout2.addWidget(self.canvas2)

        # add the tabs to the tab widget
        self.tab_widget.addTab(self.tab1, "Figure 1")
        self.tab_widget.addTab(self.tab2, "Figure 2")

        # add the tab widget to the main layout
        self.layout.addWidget(self.tab_widget)

        # create a button to update the plot data on the first figure
        self.button = QPushButton('Update Plot')
        self.layout.addWidget(self.button)
        self.button.clicked.connect(self.update_plot)

        # set the main widget as the central widget
        self.setCentralWidget(self.main_widget)

        # plot the initial data on the first figure
        self.plot_data()

    def plot_data(self):
        # plot the initial data on the first figure
        ax1 = self.figure1.add_subplot(111)
        ax1.plot([1, 2, 3, 4], [1, 4, 2, 3])

        # plot the initial data on the second figure
        ax2 = self.figure2.add_subplot(111)
        ax2.plot([1, 2, 3, 4], [3, 2, 4, 1])

    def update_plot(self):
        # update the plot data on the first figure
        ax1 = self.figure1.add_subplot(111)
        ax1.clear()
        ax1.plot([1, 2, 3, 4], [3, 1, 4, 2])

        # redraw the figure canvas to show the updated plot
        self.canvas1.draw()

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = MyWindow()
    window.show()
    sys.exit(app.exec_())