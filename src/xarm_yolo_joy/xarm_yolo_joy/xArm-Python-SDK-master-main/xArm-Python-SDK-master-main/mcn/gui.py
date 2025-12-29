# -*- coding: utf-8 -*-

################################################################################
## Form generated from reading UI file 'xfdngiyTb.ui'
##
## Created by: Qt User Interface Compiler version 5.15.2
##
## WARNING! All changes made in this file will be lost when recompiling UI file!
################################################################################

from PySide2.QtCore import *
from PySide2.QtGui import *
from PySide2.QtWidgets import *


class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        if not MainWindow.objectName():
            MainWindow.setObjectName(u"MainWindow")
        MainWindow.setEnabled(True)
        MainWindow.resize(1592, 990)
        self.centralwidget = QWidget(MainWindow)
        self.centralwidget.setObjectName(u"centralwidget")
        self.label = QLabel(self.centralwidget)
        self.label.setObjectName(u"label")
        self.label.setGeometry(QRect(0, 0, 641, 71))
        font = QFont()
        font.setFamily(u"Arial")
        font.setPointSize(21)
        self.label.setFont(font)
        self.groupBox = QGroupBox(self.centralwidget)
        self.groupBox.setObjectName(u"groupBox")
        self.groupBox.setGeometry(QRect(10, 150, 401, 121))
        font1 = QFont()
        font1.setFamily(u"Arial")
        font1.setPointSize(13)
        self.groupBox.setFont(font1)
        self.textEdit = QTextEdit(self.groupBox)
        self.textEdit.setObjectName(u"textEdit")
        self.textEdit.setGeometry(QRect(10, 30, 241, 71))
        self.textEdit.setFont(font1)
        self.pushButton = QPushButton(self.groupBox)
        self.pushButton.setObjectName(u"pushButton")
        self.pushButton.setGeometry(QRect(260, 30, 131, 71))
        self.groupBox_2 = QGroupBox(self.centralwidget)
        self.groupBox_2.setObjectName(u"groupBox_2")
        self.groupBox_2.setGeometry(QRect(20, 290, 391, 701))
        self.groupBox_2.setFont(font1)
        self.listView = QListView(self.groupBox_2)
        self.listView.setObjectName(u"listView")
        self.listView.setGeometry(QRect(10, 30, 371, 611))
        self.groupBox_4 = QGroupBox(self.centralwidget)
        self.groupBox_4.setObjectName(u"groupBox_4")
        self.groupBox_4.setGeometry(QRect(430, 810, 761, 121))
        self.groupBox_4.setFont(font1)
        self.layoutWidget_11 = QWidget(self.groupBox_4)
        self.layoutWidget_11.setObjectName(u"layoutWidget_11")
        self.layoutWidget_11.setGeometry(QRect(630, 20, 195, 101))
        self.horizontalLayout_14 = QHBoxLayout(self.layoutWidget_11)
        self.horizontalLayout_14.setObjectName(u"horizontalLayout_14")
        self.horizontalLayout_14.setContentsMargins(0, 0, 0, 0)
        self.pushButton_14 = QPushButton(self.layoutWidget_11)
        self.pushButton_14.setObjectName(u"pushButton_14")

        self.horizontalLayout_14.addWidget(self.pushButton_14)

        self.pushButton_15 = QPushButton(self.layoutWidget_11)
        self.pushButton_15.setObjectName(u"pushButton_15")

        self.horizontalLayout_14.addWidget(self.pushButton_15)

        self.layoutWidget = QWidget(self.groupBox_4)
        self.layoutWidget.setObjectName(u"layoutWidget")
        self.layoutWidget.setGeometry(QRect(10, 30, 611, 91))
        self.horizontalLayout_7 = QHBoxLayout(self.layoutWidget)
        self.horizontalLayout_7.setObjectName(u"horizontalLayout_7")
        self.horizontalLayout_7.setContentsMargins(0, 0, 0, 0)
        self.horizontalSlider_7 = QSlider(self.layoutWidget)
        self.horizontalSlider_7.setObjectName(u"horizontalSlider_7")
        self.horizontalSlider_7.setOrientation(Qt.Horizontal)

        self.horizontalLayout_7.addWidget(self.horizontalSlider_7)

        self.label_14 = QLabel(self.layoutWidget)
        self.label_14.setObjectName(u"label_14")

        self.horizontalLayout_7.addWidget(self.label_14)

        self.layoutWidget1 = QWidget(self.centralwidget)
        self.layoutWidget1.setObjectName(u"layoutWidget1")
        self.layoutWidget1.setGeometry(QRect(0, 60, 547, 51))
        self.verticalLayout_2 = QVBoxLayout(self.layoutWidget1)
        self.verticalLayout_2.setObjectName(u"verticalLayout_2")
        self.verticalLayout_2.setContentsMargins(0, 0, 0, 0)
        self.label_18 = QLabel(self.layoutWidget1)
        self.label_18.setObjectName(u"label_18")
        font2 = QFont()
        font2.setFamily(u"Arial")
        font2.setPointSize(11)
        self.label_18.setFont(font2)

        self.verticalLayout_2.addWidget(self.label_18)

        self.label_19 = QLabel(self.layoutWidget1)
        self.label_19.setObjectName(u"label_19")
        self.label_19.setFont(font2)

        self.verticalLayout_2.addWidget(self.label_19)

        self.groupBox_3 = QGroupBox(self.centralwidget)
        self.groupBox_3.setObjectName(u"groupBox_3")
        self.groupBox_3.setGeometry(QRect(430, 150, 221, 291))
        font3 = QFont()
        font3.setPointSize(13)
        self.groupBox_3.setFont(font3)
        self.textEdit_2 = QTextEdit(self.groupBox_3)
        self.textEdit_2.setObjectName(u"textEdit_2")
        self.textEdit_2.setGeometry(QRect(50, 60, 71, 41))
        self.textEdit_3 = QTextEdit(self.groupBox_3)
        self.textEdit_3.setObjectName(u"textEdit_3")
        self.textEdit_3.setGeometry(QRect(50, 140, 71, 41))
        self.textEdit_4 = QTextEdit(self.groupBox_3)
        self.textEdit_4.setObjectName(u"textEdit_4")
        self.textEdit_4.setGeometry(QRect(50, 220, 71, 41))
        self.layoutWidget2 = QWidget(self.groupBox_3)
        self.layoutWidget2.setObjectName(u"layoutWidget2")
        self.layoutWidget2.setGeometry(QRect(20, 40, 41, 231))
        self.verticalLayout = QVBoxLayout(self.layoutWidget2)
        self.verticalLayout.setObjectName(u"verticalLayout")
        self.verticalLayout.setContentsMargins(0, 0, 0, 0)
        self.label_2 = QLabel(self.layoutWidget2)
        self.label_2.setObjectName(u"label_2")

        self.verticalLayout.addWidget(self.label_2)

        self.label_3 = QLabel(self.layoutWidget2)
        self.label_3.setObjectName(u"label_3")

        self.verticalLayout.addWidget(self.label_3)

        self.label_4 = QLabel(self.layoutWidget2)
        self.label_4.setObjectName(u"label_4")

        self.verticalLayout.addWidget(self.label_4)

        self.layoutWidget3 = QWidget(self.groupBox_3)
        self.layoutWidget3.setObjectName(u"layoutWidget3")
        self.layoutWidget3.setGeometry(QRect(130, 40, 51, 231))
        self.verticalLayout_3 = QVBoxLayout(self.layoutWidget3)
        self.verticalLayout_3.setObjectName(u"verticalLayout_3")
        self.verticalLayout_3.setContentsMargins(0, 0, 0, 0)
        self.label_5 = QLabel(self.layoutWidget3)
        self.label_5.setObjectName(u"label_5")

        self.verticalLayout_3.addWidget(self.label_5)

        self.label_6 = QLabel(self.layoutWidget3)
        self.label_6.setObjectName(u"label_6")

        self.verticalLayout_3.addWidget(self.label_6)

        self.label_7 = QLabel(self.layoutWidget3)
        self.label_7.setObjectName(u"label_7")

        self.verticalLayout_3.addWidget(self.label_7)

        self.groupBox_8 = QGroupBox(self.centralwidget)
        self.groupBox_8.setObjectName(u"groupBox_8")
        self.groupBox_8.setGeometry(QRect(430, 460, 221, 341))
        self.groupBox_8.setFont(font3)
        self.textEdit_5 = QTextEdit(self.groupBox_8)
        self.textEdit_5.setObjectName(u"textEdit_5")
        self.textEdit_5.setGeometry(QRect(60, 80, 71, 41))
        self.textEdit_6 = QTextEdit(self.groupBox_8)
        self.textEdit_6.setObjectName(u"textEdit_6")
        self.textEdit_6.setGeometry(QRect(60, 160, 71, 41))
        self.textEdit_7 = QTextEdit(self.groupBox_8)
        self.textEdit_7.setObjectName(u"textEdit_7")
        self.textEdit_7.setGeometry(QRect(60, 240, 71, 41))
        self.layoutWidget_2 = QWidget(self.groupBox_8)
        self.layoutWidget_2.setObjectName(u"layoutWidget_2")
        self.layoutWidget_2.setGeometry(QRect(20, 60, 42, 231))
        self.verticalLayout_4 = QVBoxLayout(self.layoutWidget_2)
        self.verticalLayout_4.setObjectName(u"verticalLayout_4")
        self.verticalLayout_4.setContentsMargins(0, 0, 0, 0)
        self.label_8 = QLabel(self.layoutWidget_2)
        self.label_8.setObjectName(u"label_8")

        self.verticalLayout_4.addWidget(self.label_8)

        self.label_9 = QLabel(self.layoutWidget_2)
        self.label_9.setObjectName(u"label_9")

        self.verticalLayout_4.addWidget(self.label_9)

        self.label_10 = QLabel(self.layoutWidget_2)
        self.label_10.setObjectName(u"label_10")

        self.verticalLayout_4.addWidget(self.label_10)

        self.layoutWidget_3 = QWidget(self.groupBox_8)
        self.layoutWidget_3.setObjectName(u"layoutWidget_3")
        self.layoutWidget_3.setGeometry(QRect(140, 60, 51, 231))
        self.verticalLayout_5 = QVBoxLayout(self.layoutWidget_3)
        self.verticalLayout_5.setObjectName(u"verticalLayout_5")
        self.verticalLayout_5.setContentsMargins(0, 0, 0, 0)
        self.label_11 = QLabel(self.layoutWidget_3)
        self.label_11.setObjectName(u"label_11")

        self.verticalLayout_5.addWidget(self.label_11)

        self.label_12 = QLabel(self.layoutWidget_3)
        self.label_12.setObjectName(u"label_12")

        self.verticalLayout_5.addWidget(self.label_12)

        self.label_13 = QLabel(self.layoutWidget_3)
        self.label_13.setObjectName(u"label_13")

        self.verticalLayout_5.addWidget(self.label_13)

        self.groupBox_9 = QGroupBox(self.centralwidget)
        self.groupBox_9.setObjectName(u"groupBox_9")
        self.groupBox_9.setGeometry(QRect(670, 150, 521, 291))
        self.groupBox_9.setFont(font3)
        self.horizontalLayout = QHBoxLayout(self.groupBox_9)
        self.horizontalLayout.setObjectName(u"horizontalLayout")
        self.verticalLayout_6 = QVBoxLayout()
        self.verticalLayout_6.setObjectName(u"verticalLayout_6")
        self.pushButton_2 = QPushButton(self.groupBox_9)
        self.pushButton_2.setObjectName(u"pushButton_2")

        self.verticalLayout_6.addWidget(self.pushButton_2)

        self.pushButton_4 = QPushButton(self.groupBox_9)
        self.pushButton_4.setObjectName(u"pushButton_4")

        self.verticalLayout_6.addWidget(self.pushButton_4)

        self.pushButton_6 = QPushButton(self.groupBox_9)
        self.pushButton_6.setObjectName(u"pushButton_6")

        self.verticalLayout_6.addWidget(self.pushButton_6)


        self.horizontalLayout.addLayout(self.verticalLayout_6)

        self.verticalLayout_7 = QVBoxLayout()
        self.verticalLayout_7.setObjectName(u"verticalLayout_7")
        self.pushButton_3 = QPushButton(self.groupBox_9)
        self.pushButton_3.setObjectName(u"pushButton_3")

        self.verticalLayout_7.addWidget(self.pushButton_3)

        self.pushButton_5 = QPushButton(self.groupBox_9)
        self.pushButton_5.setObjectName(u"pushButton_5")

        self.verticalLayout_7.addWidget(self.pushButton_5)

        self.pushButton_7 = QPushButton(self.groupBox_9)
        self.pushButton_7.setObjectName(u"pushButton_7")

        self.verticalLayout_7.addWidget(self.pushButton_7)


        self.horizontalLayout.addLayout(self.verticalLayout_7)

        self.groupBox_10 = QGroupBox(self.centralwidget)
        self.groupBox_10.setObjectName(u"groupBox_10")
        self.groupBox_10.setGeometry(QRect(670, 460, 521, 181))
        self.groupBox_10.setFont(font3)
        self.horizontalLayout_2 = QHBoxLayout(self.groupBox_10)
        self.horizontalLayout_2.setObjectName(u"horizontalLayout_2")
        self.verticalLayout_8 = QVBoxLayout()
        self.verticalLayout_8.setObjectName(u"verticalLayout_8")
        self.pushButton_8 = QPushButton(self.groupBox_10)
        self.pushButton_8.setObjectName(u"pushButton_8")

        self.verticalLayout_8.addWidget(self.pushButton_8)

        self.pushButton_10 = QPushButton(self.groupBox_10)
        self.pushButton_10.setObjectName(u"pushButton_10")

        self.verticalLayout_8.addWidget(self.pushButton_10)


        self.horizontalLayout_2.addLayout(self.verticalLayout_8)

        self.verticalLayout_9 = QVBoxLayout()
        self.verticalLayout_9.setObjectName(u"verticalLayout_9")
        self.pushButton_9 = QPushButton(self.groupBox_10)
        self.pushButton_9.setObjectName(u"pushButton_9")

        self.verticalLayout_9.addWidget(self.pushButton_9)

        self.pushButton_11 = QPushButton(self.groupBox_10)
        self.pushButton_11.setObjectName(u"pushButton_11")

        self.verticalLayout_9.addWidget(self.pushButton_11)


        self.horizontalLayout_2.addLayout(self.verticalLayout_9)

        self.groupBox_11 = QGroupBox(self.centralwidget)
        self.groupBox_11.setObjectName(u"groupBox_11")
        self.groupBox_11.setGeometry(QRect(670, 660, 521, 141))
        self.groupBox_11.setFont(font3)
        self.layoutWidget4 = QWidget(self.groupBox_11)
        self.layoutWidget4.setObjectName(u"layoutWidget4")
        self.layoutWidget4.setGeometry(QRect(0, 0, 501, 111))
        self.horizontalLayout_3 = QHBoxLayout(self.layoutWidget4)
        self.horizontalLayout_3.setObjectName(u"horizontalLayout_3")
        self.horizontalLayout_3.setContentsMargins(0, 0, 0, 0)
        self.pushButton_12 = QPushButton(self.layoutWidget4)
        self.pushButton_12.setObjectName(u"pushButton_12")

        self.horizontalLayout_3.addWidget(self.pushButton_12)

        self.pushButton_16 = QPushButton(self.layoutWidget4)
        self.pushButton_16.setObjectName(u"pushButton_16")

        self.horizontalLayout_3.addWidget(self.pushButton_16)

        self.pushButton_13 = QPushButton(self.layoutWidget4)
        self.pushButton_13.setObjectName(u"pushButton_13")

        self.horizontalLayout_3.addWidget(self.pushButton_13)

        self.layoutWidget5 = QWidget(self.centralwidget)
        self.layoutWidget5.setObjectName(u"layoutWidget5")
        self.layoutWidget5.setGeometry(QRect(1220, 150, 361, 781))
        self.verticalLayout_10 = QVBoxLayout(self.layoutWidget5)
        self.verticalLayout_10.setObjectName(u"verticalLayout_10")
        self.verticalLayout_10.setContentsMargins(0, 0, 0, 0)
        self.groupBox_5 = QGroupBox(self.layoutWidget5)
        self.groupBox_5.setObjectName(u"groupBox_5")
        self.groupBox_5.setFont(font1)
        self.label_15 = QLabel(self.groupBox_5)
        self.label_15.setObjectName(u"label_15")
        self.label_15.setGeometry(QRect(10, 20, 341, 351))
        self.label_15.setAlignment(Qt.AlignCenter)

        self.verticalLayout_10.addWidget(self.groupBox_5)

        self.groupBox_6 = QGroupBox(self.layoutWidget5)
        self.groupBox_6.setObjectName(u"groupBox_6")
        self.groupBox_6.setFont(font1)
        self.label_16 = QLabel(self.groupBox_6)
        self.label_16.setObjectName(u"label_16")
        self.label_16.setGeometry(QRect(10, 20, 341, 351))
        self.label_16.setAlignment(Qt.AlignCenter)

        self.verticalLayout_10.addWidget(self.groupBox_6)

        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QMenuBar(MainWindow)
        self.menubar.setObjectName(u"menubar")
        self.menubar.setGeometry(QRect(0, 0, 1592, 26))
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QStatusBar(MainWindow)
        self.statusbar.setObjectName(u"statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)

        QMetaObject.connectSlotsByName(MainWindow)
    # setupUi

    def retranslateUi(self, MainWindow):
        MainWindow.setWindowTitle(QCoreApplication.translate("MainWindow", u"MainWindow", None))
        self.label.setText(QCoreApplication.translate("MainWindow", u"XARM LITE 6 MODBUS TCP CONTROL", None))
        self.groupBox.setTitle(QCoreApplication.translate("MainWindow", u"Conection Panel", None))
        self.textEdit.setHtml(QCoreApplication.translate("MainWindow", u"<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:'Arial','Arial'; font-size:13pt; font-weight:400; font-style:normal;\">\n"
"<p align=\"center\" style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-family:'MS Shell Dlg 2';\">192.158.1.165</span></p></body></html>", None))
        self.pushButton.setText(QCoreApplication.translate("MainWindow", u"Connect", None))
        self.groupBox_2.setTitle(QCoreApplication.translate("MainWindow", u"Log", None))
        self.groupBox_4.setTitle(QCoreApplication.translate("MainWindow", u"Speed", None))
        self.pushButton_14.setText(QCoreApplication.translate("MainWindow", u"-", None))
        self.pushButton_15.setText(QCoreApplication.translate("MainWindow", u"+", None))
        self.label_14.setText(QCoreApplication.translate("MainWindow", u"67%", None))
        self.label_18.setText(QCoreApplication.translate("MainWindow", u"Th\u00e0nh vi\u00ean: Nguy\u1ec5n Thanh Nh\u00e2n, H\u1ed3 \u0110\u1ee9c An, Nguy\u1ec5n V\u0169 Huy Kh\u00f4i ", None))
        self.label_19.setText(QCoreApplication.translate("MainWindow", u"GVHD: TS. Ph\u1ea1m Qu\u1ed1c Thi\u1ec7n", None))
        self.groupBox_3.setTitle(QCoreApplication.translate("MainWindow", u"Position", None))
        self.textEdit_2.setHtml(QCoreApplication.translate("MainWindow", u"<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:'MS Shell Dlg 2'; font-size:13pt; font-weight:400; font-style:normal;\">\n"
"<p align=\"center\" style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">22</p></body></html>", None))
        self.textEdit_3.setHtml(QCoreApplication.translate("MainWindow", u"<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:'MS Shell Dlg 2'; font-size:13pt; font-weight:400; font-style:normal;\">\n"
"<p align=\"center\" style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">22</p></body></html>", None))
        self.textEdit_4.setHtml(QCoreApplication.translate("MainWindow", u"<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:'MS Shell Dlg 2'; font-size:13pt; font-weight:400; font-style:normal;\">\n"
"<p align=\"center\" style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">22</p></body></html>", None))
        self.label_2.setText(QCoreApplication.translate("MainWindow", u"X: ", None))
        self.label_3.setText(QCoreApplication.translate("MainWindow", u"Y:", None))
        self.label_4.setText(QCoreApplication.translate("MainWindow", u"Z:", None))
        self.label_5.setText(QCoreApplication.translate("MainWindow", u"mm", None))
        self.label_6.setText(QCoreApplication.translate("MainWindow", u"mm", None))
        self.label_7.setText(QCoreApplication.translate("MainWindow", u"mm", None))
        self.groupBox_8.setTitle(QCoreApplication.translate("MainWindow", u"Orientattion", None))
        self.textEdit_5.setHtml(QCoreApplication.translate("MainWindow", u"<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:'MS Shell Dlg 2'; font-size:13pt; font-weight:400; font-style:normal;\">\n"
"<p align=\"center\" style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">22</p></body></html>", None))
        self.textEdit_6.setHtml(QCoreApplication.translate("MainWindow", u"<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:'MS Shell Dlg 2'; font-size:13pt; font-weight:400; font-style:normal;\">\n"
"<p align=\"center\" style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">22</p></body></html>", None))
        self.textEdit_7.setHtml(QCoreApplication.translate("MainWindow", u"<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:'MS Shell Dlg 2'; font-size:13pt; font-weight:400; font-style:normal;\">\n"
"<p align=\"center\" style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">22</p></body></html>", None))
        self.label_8.setText(QCoreApplication.translate("MainWindow", u"Rx: ", None))
        self.label_9.setText(QCoreApplication.translate("MainWindow", u"Ry:", None))
        self.label_10.setText(QCoreApplication.translate("MainWindow", u"Rz:", None))
        self.label_11.setText(QCoreApplication.translate("MainWindow", u"\u00b0", None))
        self.label_12.setText(QCoreApplication.translate("MainWindow", u"\u00b0", None))
        self.label_13.setText(QCoreApplication.translate("MainWindow", u"\u00b0", None))
        self.groupBox_9.setTitle(QCoreApplication.translate("MainWindow", u"Base Coordinate", None))
        self.pushButton_2.setText(QCoreApplication.translate("MainWindow", u"X+", None))
        self.pushButton_4.setText(QCoreApplication.translate("MainWindow", u"Y+", None))
        self.pushButton_6.setText(QCoreApplication.translate("MainWindow", u"Z+", None))
        self.pushButton_3.setText(QCoreApplication.translate("MainWindow", u"X-", None))
        self.pushButton_5.setText(QCoreApplication.translate("MainWindow", u"Y-", None))
        self.pushButton_7.setText(QCoreApplication.translate("MainWindow", u"Z-", None))
        self.groupBox_10.setTitle(QCoreApplication.translate("MainWindow", u"Function", None))
        self.pushButton_8.setText(QCoreApplication.translate("MainWindow", u"Clear Error", None))
        self.pushButton_10.setText(QCoreApplication.translate("MainWindow", u"Zero Position", None))
        self.pushButton_9.setText(QCoreApplication.translate("MainWindow", u"Enable Robot", None))
        self.pushButton_11.setText(QCoreApplication.translate("MainWindow", u"Initial Position", None))
        self.groupBox_11.setTitle(QCoreApplication.translate("MainWindow", u"Gripper", None))
        self.pushButton_12.setText(QCoreApplication.translate("MainWindow", u"OPEN", None))
        self.pushButton_16.setText(QCoreApplication.translate("MainWindow", u"STOP", None))
        self.pushButton_13.setText(QCoreApplication.translate("MainWindow", u"CLOSE", None))
        self.groupBox_5.setTitle(QCoreApplication.translate("MainWindow", u"Request", None))
        self.label_15.setText(QCoreApplication.translate("MainWindow", u"0x17|0x00|0x00|0x00|0x00|0x00", None))
        self.groupBox_6.setTitle(QCoreApplication.translate("MainWindow", u"Response", None))
        self.label_16.setText(QCoreApplication.translate("MainWindow", u"0x00|0x00|0x00|0x00|0x00|0x00", None))
    # retranslateUi

