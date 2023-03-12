import sys
import random
from PyQt5.QtWidgets import QApplication, QWidget, QPushButton, QLabel

class Example(QWidget):

    def __init__(self):
        super().__init__()

        self.initUI()

    def initUI(self):

        self.setGeometry(300, 300, 250, 150)
        self.setWindowTitle('Random Number Example')

        self.label = QLabel('Press the button to toggle a random number', self)
        self.label.move(50, 50)

        self.btn = QPushButton('Toggle Number', self)
        self.btn.clicked.connect(self.toggleNumber)
        self.btn.resize(self.btn.sizeHint())
        self.btn.move(50, 100)

        self.show()

    def toggleNumber(self):
        number = random.randint(1, 100)
        self.label.setText(f'The random number is: {number}')

if __name__ == '__main__':

    app = QApplication(sys.argv)
    ex = Example()
    sys.exit(app.exec_())