# -*- coding: utf-8 -*-

################################################################################
## Form generated from reading UI file 'mainUI.ui'
##
## Created by: Qt User Interface Compiler version 6.9.0
##
## WARNING! All changes made in this file will be lost when recompiling UI file!
################################################################################

from PySide6.QtCore import (QCoreApplication, QDate, QDateTime, QLocale,
    QMetaObject, QObject, QPoint, QRect,
    QSize, QTime, QUrl, Qt)
from PySide6.QtGui import (QBrush, QColor, QConicalGradient, QCursor,
    QFont, QFontDatabase, QGradient, QIcon,
    QImage, QKeySequence, QLinearGradient, QPainter,
    QPalette, QPixmap, QRadialGradient, QTransform)
from PySide6.QtWidgets import (QApplication, QLabel, QMainWindow, QSizePolicy,
    QWidget)

class Ui_guiDlg(object):
    def setupUi(self, guiDlg):
        if not guiDlg.objectName():
            guiDlg.setObjectName(u"guiDlg")
        guiDlg.resize(702, 877)
        self.color = QLabel(guiDlg)
        self.color.setObjectName(u"color")
        self.color.setGeometry(QRect(5, 5, 640, 480))
        guiDlg.setCentralWidget(self.color)
        self.depth = QLabel(guiDlg)
        self.depth.setObjectName(u"depth")
        self.depth.setGeometry(QRect(80, 550, 480, 270))
        guiDlg.setCentralWidget(self.depth)

        self.retranslateUi(guiDlg)

        QMetaObject.connectSlotsByName(guiDlg)
    # setupUi

    def retranslateUi(self, guiDlg):
        guiDlg.setWindowTitle(QCoreApplication.translate("guiDlg", u"camera_kinova", None))
        self.color.setText(QCoreApplication.translate("guiDlg", u"Color space", None))
        self.depth.setText(QCoreApplication.translate("guiDlg", u"Depth space", None))
    # retranslateUi

