# -*- coding: utf-8 -*-

################################################################################
## Form generated from reading UI file 'mainUI.ui'
##
## Created by: Qt User Interface Compiler version 6.11.0
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
from PySide6.QtWidgets import (QApplication, QHBoxLayout, QLabel, QMainWindow,
    QSizePolicy, QWidget)

class Ui_guiDlg(object):
    def setupUi(self, guiDlg):
        if not guiDlg.objectName():
            guiDlg.setObjectName(u"guiDlg")
        guiDlg.resize(1280, 520)
        self.centralwidget = QWidget(guiDlg)
        self.horizontalLayout = QHBoxLayout(self.centralwidget)
        self.horizontalLayout.setSpacing(8)
        self.horizontalLayout.setContentsMargins(5, 5, 5, 5)
        self.color = QLabel(self.centralwidget)
        self.color.setObjectName(u"color")
        self.color.setMinimumSize(QSize(320, 240))
        self.color.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.horizontalLayout.addWidget(self.color)
        self.depth = QLabel(self.centralwidget)
        self.depth.setObjectName(u"depth")
        self.depth.setMinimumSize(QSize(320, 240))
        self.depth.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.horizontalLayout.addWidget(self.depth)
        guiDlg.setCentralWidget(self.centralwidget)

        self.retranslateUi(guiDlg)

        QMetaObject.connectSlotsByName(guiDlg)
    # setupUi

    def retranslateUi(self, guiDlg):
        guiDlg.setWindowTitle(QCoreApplication.translate("guiDlg", u"camera_kinova", None))
        self.color.setText(QCoreApplication.translate("guiDlg", u"Color space", None))
        self.depth.setText(QCoreApplication.translate("guiDlg", u"Depth space", None))
    # retranslateUi
