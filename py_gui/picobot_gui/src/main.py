import sys
from PyQt5.QtWidgets import QApplication
from gui import PicobotGui

if __name__ == '__main__':
    app = QApplication(sys.argv)
    gui = PicobotGui()
    gui.show()
    sys.exit(app.exec_())