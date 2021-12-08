https://blog.csdn.net/qq_34710142/article/details/80936986


import sys
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *

global sec
sec = 0

# ������һ���̳���QThread����࣬����д������run()����
# run()�����������߳���Ҫִ�еģ�ִ��һ��ѭ�������ͼ�����ɵ��źš�
class WorkThread(QThread):
    trigger = pyqtSignal()

    def __int__(self):
        super(WorkThread, self).__init__()

    def run(self):
        for i in range(2000000000):
            pass

        # ѭ����Ϻ󷢳��ź�
        self.trigger.emit()


def countTime():
    global sec
    sec += 1
    # LED��ʾ����+1
    lcdNumber.display(sec)


def work():
    # ��ʱ��ÿ�����
    timer.start(1000)
    # ��ʱ��ʼ
    workThread.start()
    # �����ѭ����ϵ��ź�ʱ��ֹͣ����
    workThread.trigger.connect(timeStop)


def timeStop():
    timer.stop()
    print("���н�����ʱ", lcdNumber.value())
    global sec
    sec = 0


if __name__ == "__main__":
    app = QApplication(sys.argv)
    top = QWidget()
    top.resize(300, 120)

    # ��ֱ������QVBoxLayout
    layout = QVBoxLayout(top)

    # �Ӹ���ʾ��
    lcdNumber = QLCDNumber()
    layout.addWidget(lcdNumber)
    button = QPushButton("����")
    layout.addWidget(button)

    timer = QTimer()
    workThread = WorkThread()

    button.clicked.connect(work)

    # ÿ�μ�ʱ���������� countTime
    timer.timeout.connect(countTime)

    top.show()
    sys.exit(app.exec_())