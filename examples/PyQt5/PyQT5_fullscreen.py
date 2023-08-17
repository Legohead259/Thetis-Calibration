# From https://stackoverflow.com/questions/50664527/pyqt5-show-fullscreen-dialog

import sys
from PyQt5.QtWidgets import QApplication, QDialog
from PyQt5.QtCore import Qt


app = QApplication(sys.argv)
dialog = QDialog()

dialog.setWindowFlags(Qt.WindowCloseButtonHint | Qt.WindowType_Mask)
dialog.showFullScreen()
app.exec()