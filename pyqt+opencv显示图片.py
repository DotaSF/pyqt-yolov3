ע��pyqt����Ҫ��ͼƬת��ΪqImage��������ʾ


import sys
import cv2
import numpy as np
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtWidgets import QApplication, QDialog, QFileDialog, QGridLayout, QLabel, QPushButton


class win(QDialog):
    def __init__(self):
        # ��ʼ��һ��img��ndarry�����ڴ洢ͼ��
        self.img = np.ndarray(())
        super().__init__()
        self.initUI()

    def initUI(self):
        self.resize(400, 300)
        self.btnOpen = QPushButton('Open', self)
        self.btnSave = QPushButton('Save', self)
        self.btnProcess = QPushButton('Process', self)
        self.btnQuit = QPushButton('Quit', self)
        self.label = QLabel()

        # �����趨
        layout = QGridLayout(self)
        layout.addWidget(self.label, 0, 1, 3, 4)
        layout.addWidget(self.btnOpen, 4, 1, 1, 1)
        layout.addWidget(self.btnSave, 4, 2, 1, 1)
        layout.addWidget(self.btnProcess, 4, 3, 1, 1)
        layout.addWidget(self.btnQuit, 4, 4, 1, 1)

        # �ź���۽������ӣ��źſɰ���ͨ��Ա����
        self.btnOpen.clicked.connect(self.openSlot)
        self.btnSave.clicked.connect(self.saveSlot)
        self.btnProcess.clicked.connect(self.processSlot)
        self.btnQuit.clicked.connect(self.close)

    def openSlot(self):
        # ���ô洢�ļ�
        fileName, tmp = QFileDialog.getOpenFileName(self, 'Open Image', 'Image', '*.png *.jpg *.bmp')
        if fileName is '':
            return
        # ����OpenCV������ȡ����
        self.img = cv2.imread(fileName, -1)
        if self.img.size == 1:
            return
        self.refreshShow()

    def saveSlot(self):
        # ���ô洢�ļ�dialog
        fileName, tmp = QFileDialog.getSaveFileName(self, 'Save Image', 'Image', '*.png *.jpg *.bmp')
        if fileName is '':
            return
        if self.img.size == 1:
            return
        # ����OpenCVд�뺯��
        cv2.imwrite(fileName, self.img)

    def processSlot(self):
        if self.img.size == 1:
            return
        # ��ͼ����ģ�����������趨Ϊ5*5
        self.img = cv2.blur(self.img, (5, 5))
        self.refreshShow()

    def refreshShow(self):
        # ��ȡͼ���ͨ���ͳߴ磬���ڽ�OpenCV�µ�imageת����Qimage
        height, width, channel = self.img.shape
        bytesPerline = 3 * width
        self.qImg = QImage(self.img.data, width, height, bytesPerline, QImage.Format_RGB888).rgbSwapped()
        # ��QImage��ʾ����
        self.label.setPixmap(QPixmap.fromImage(self.qImg))


if __name__ == '__main__':
    a = QApplication(sys.argv)
    w = win()
    w.show()
    sys.exit(a.exec_())