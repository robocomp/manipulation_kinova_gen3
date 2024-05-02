# -*- coding: utf-8 -*-

################################################################################
## Form generated from reading UI file 'mainUI.ui'
##
## Created by: Qt User Interface Compiler version 5.15.2
##
## WARNING! All changes made in this file will be lost when recompiling UI file!
################################################################################

from PySide2.QtCore import *
from PySide2.QtGui import *
from PySide2.QtWidgets import *


class Ui_guiDlg(object):
    def setupUi(self, guiDlg):
        if not guiDlg.objectName():
            guiDlg.setObjectName(u"guiDlg")
        guiDlg.resize(702, 877)
        self.color = QLabel(guiDlg)
        self.color.setObjectName(u"color")
        self.color.setGeometry(QRect(5, 5, 640, 480))
        self.depth = QLabel(guiDlg)
        self.depth.setObjectName(u"depth")
        self.depth.setGeometry(QRect(80, 550, 480, 270))

        self.retranslateUi(guiDlg)

        QMetaObject.connectSlotsByName(guiDlg)
    # setupUi

    def retranslateUi(self, guiDlg):
        guiDlg.setWindowTitle(QCoreApplication.translate("guiDlg", u"camera_kinova", None))
        self.color.setText(QCoreApplication.translate("guiDlg", u"Color space", None))
        self.depth.setText(QCoreApplication.translate("guiDlg", u"Depth space", None))
    # retranslateUi

