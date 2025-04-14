from PyQt5.QtGui import QPalette, QColor

def set_checkbox_style(checkbox, color):
    palette = checkbox.palette()
    palette.setColor(QPalette.WindowText, QColor(color))
    checkbox.setPalette(palette)
    checkbox.setStyleSheet("QCheckBox { font-size: 25px; } QCheckBox::indicator { width: 40px; height: 40px; }")

def set_label_color(label, color):
        label.setStyleSheet(f"color: {color}; font-size: 30px;")

def set_button_color(self, button, color):
        button.setStyleSheet("background-color: {}".format(color))