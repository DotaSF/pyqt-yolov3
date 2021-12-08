# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'first.ui'
#
# Created by: PyQt5 UI code generator 5.11.3
#
# WARNING! All changes made in this file will be lost!

from PyQt5 import QtCore, QtGui, QtWidgets

class Ui_Form(object):
    def setupUi(self, Form):
        Form.setObjectName("Form")
        Form.resize(953, 697)
        self.label = QtWidgets.QLabel(Form)
        self.label.setGeometry(QtCore.QRect(20, 40, 431, 331))
        self.label.setText("")
        self.label.setObjectName("label")
        self.opencamera = QtWidgets.QPushButton(Form)
        self.opencamera.setGeometry(QtCore.QRect(530, 40, 93, 28))
        self.opencamera.setObjectName("opencamera")
        self.yolov3 = QtWidgets.QPushButton(Form)
        self.yolov3.setGeometry(QtCore.QRect(530, 90, 93, 28))
        self.yolov3.setObjectName("yolov3")

        self.retranslateUi(Form)
        #self.yolov3.clicked.connect(self.label_close)
        QtCore.QMetaObject.connectSlotsByName(Form)

    def label_close(self,Form):
        self.label.setScaledContents(True)
        self.label.setPixmap(QtGui.QPixmap("D:/dog.jpg"))

    def retranslateUi(self, Form):
        _translate = QtCore.QCoreApplication.translate
        Form.setWindowTitle(_translate("Form", "Form"))
        self.opencamera.setText(_translate("Form", "opencamera"))
        self.yolov3.setText(_translate("Form", "start_yolo"))

class myfirstwindows(QtWidgets.QWidget,Ui_Form):
    def __init__(self):
        super(myfirstwindows,self).__init__()
        self.setupUi(self)

def mainwindows():
    import sys
    app = QtWidgets.QApplication(sys.argv)
    new = myfirstwindows()
    new.show()
    sys.exit(app.exec_())

#if __name__ == "__main__":
 #   mainwindows()


