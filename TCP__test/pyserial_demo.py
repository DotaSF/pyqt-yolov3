# -*- coding: utf-8 -*-
"""
Created on Sun Dec 29 13:15:13 2019

@author: 张理官
"""
from ctypes import *
import math
import random
import os
import numpy as np
import time
from PyQt5 import QtWidgets
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *
import sys
import serial
import serial.tools.list_ports
from PyQt5 import QtWidgets
from PyQt5.QtWidgets import QMessageBox
from PyQt5.QtCore import *
from ui_demo_3 import Ui_Form
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *
import darknet
import cv2
from PyQt5.QtCore import QTimer
import socket
import threading
import stopThreading

#ÉãÏñÍ·µÄÏß³Ì 
class CWorkThread(QThread):
#³õÊ¼»¯£¬Ïß³Ìstartºó×Ôµ÷ÓÃ¸Ãº¯Êý
    def __int__(self):
        super(CWorkThread, self).__init__()
#ÔËÐÐº¯Êý
    def run(self):
        opencamera()
        


#´ò¿ªÉãÏñÍ·*************************************
def opencamera():
    cap = cv2.VideoCapture(0)
    global camera_flag
    while camera_flag:
        ret, frame_read = cap.read()
        image = cv2.resize(frame_read,
                                   (416,
                                    416),
                                   interpolation=cv2.INTER_LINEAR)
        height,width,channel = image.shape
        bytesPerline = 3*width
        qimage = QImage(image.data, width, height, bytesPerline, QImage.Format_RGB888).rgbSwapped()
        myshow.messi.setPixmap(QPixmap.fromImage(qimage))
    cap.release()


#Æô¶¯ÉãÏñÍ·º¯Êý
def cwork():
    global camera_flag
    camera_flag = 1
    myshow.close_camera.setEnabled(True)
    myshow.open_camera.setEnabled(False)
    cworkThread.start()

#ºÜÆø£¬ÕâÀïÓÐÒ»¸öÈ«¾Ö±äÁ¿µÄÖªÊ¶µã
def cdiswork():
    myshow.close_camera.setEnabled(False)
    myshow.open_camera.setEnabled(True)
    global camera_flag
    camera_flag = 0

#Ä¿±ê¼ì²âµÄÏß³Ì
class WorkThread(QThread):
#³õÊ¼»¯£¬Ïß³Ìstartºó×Ôµ÷ÓÃ¸Ãº¯Êý
    def __int__(self):
        super(WorkThread, self).__init__()
#ÔËÐÐº¯Êý
    def run(self):
        YOLO()


#Æô¶¯Ïß³Ìº¯Êý
def work():
    global yolov3_flag
    yolov3_flag = 1
    myshow.close_yolov3.setEnabled(True)
    myshow.yolov3.setEnabled(False)
    workThread.start()

#ºÜÆø£¬ÕâÀïÓÐÒ»¸öÈ«¾Ö±äÁ¿µÄÖªÊ¶µã
def diswork():
    myshow.close_yolov3.setEnabled(False)
    myshow.yolov3.setEnabled(True)
    global yolov3_flag
    yolov3_flag = 0


def convertBack(x, y, w, h):
    xmin = int(round(x - (w / 2)))
    xmax = int(round(x + (w / 2)))
    ymin = int(round(y - (h / 2)))
    ymax = int(round(y + (h / 2)))
    return xmin, ymin, xmax, ymax

#»æÖÆ·½¿ò
def cvDrawBoxes(detections, img):
    for detection in detections:
        x, y, w, h = detection[2][0],\
            detection[2][1],\
            detection[2][2],\
            detection[2][3]
        xmin, ymin, xmax, ymax = convertBack(
            float(x), float(y), float(w), float(h))
        pt1 = (xmin, ymin)
        pt2 = (xmax, ymax)
        cv2.rectangle(img, pt1, pt2, (0, 255, 0), 1)
        cv2.putText(img,
                    detection[0].decode() +
                    " [" + str(round(detection[1] * 100, 2)) + "]",
                    (pt1[0], pt1[1] - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    [0, 255, 0], 2)
    return img

netMain = None
metaMain = None
altNames = None

#Ä¿±ê¼ì²âº¯Êý
def YOLO():

    global metaMain, netMain, altNames
    configPath = "./yolo-obj.cfg"
    weightPath = "./yolo-obj_2000.weights"
    metaPath = "./obj.data"
    if not os.path.exists(configPath):
        raise ValueError("Invalid config path `" +
                         os.path.abspath(configPath)+"`")
    if not os.path.exists(weightPath):
        raise ValueError("Invalid weight path `" +
                         os.path.abspath(weightPath)+"`")
    if not os.path.exists(metaPath):
        raise ValueError("Invalid data file path `" +
                         os.path.abspath(metaPath)+"`")
    if netMain is None:
        netMain = darknet.load_net_custom(configPath.encode(
            "ascii"), weightPath.encode("ascii"), 0, 1)  # batch size = 1
    if metaMain is None:
        metaMain = darknet.load_meta(metaPath.encode("ascii"))
    if altNames is None:
        try:
            with open(metaPath) as metaFH:
                metaContents = metaFH.read()
                import re
                match = re.search("names *= *(.*)$", metaContents,
                                  re.IGNORECASE | re.MULTILINE)
                if match:
                    result = match.group(1)
                else:
                    result = None
                try:
                    if os.path.exists(result):
                        with open(result) as namesFH:
                            namesList = namesFH.read().strip().split("\n")
                            altNames = [x.strip() for x in namesList]
                except TypeError:
                    pass
        except Exception:
            pass
    cap = cv2.VideoCapture(0)
    #cap = cv2.VideoCapture("handou.mp4") 
    
    # Create an image we reuse for each detect
    darknet_image = darknet.make_image(darknet.network_width(netMain),
                                    darknet.network_height(netMain),3)
    global yolov3_flag
    while yolov3_flag:
        ret, frame_read = cap.read()
        frame_rgb = cv2.cvtColor(frame_read, cv2.COLOR_BGR2RGB)
        frame_resized = cv2.resize(frame_rgb,
                                   (darknet.network_width(netMain),
                                    darknet.network_height(netMain)),
                                   interpolation=cv2.INTER_LINEAR)

        darknet.copy_image_from_bytes(darknet_image,frame_resized.tobytes())

        detections = darknet.detect_image(netMain, metaMain, darknet_image, thresh=0.25)
        image = cvDrawBoxes(detections, frame_resized)
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        height,width,channel = image.shape
        bytesPerline = 3*width
        qimage = QImage(image.data, width, height, bytesPerline, QImage.Format_RGB888).rgbSwapped()
        myshow.messi.setPixmap(QPixmap.fromImage(qimage))
    cap.release()


class Pyqt5_Serial(QtWidgets.QWidget, Ui_Form):
    signal_write_msg = pyqtSignal(str)
    def __init__(self):
        super(Pyqt5_Serial, self).__init__()
        self.setupUi(self)
        self.init()
        self.setWindowTitle("º£²Î²¶ÀÌ»úÆ÷ÈËÉÏÎ»»ú")
        self.ser = serial.Serial()
        self.port_check()
        self.tcp_socket = None
        self.client_th = None
        self.client_socket_list = list()

        # ½ÓÊÕÊý¾ÝºÍ·¢ËÍÊý¾ÝÊýÄ¿ÖÃÁã
        self.link = False  # ÓÃÓÚ±ê¼ÇÊÇ·ñ¿ªÆôÁËÁ¬½Ó
        self.data_num_received = 0
        self.lineEdit.setText(str(self.data_num_received))

        # ´ò¿ªÈí¼þÊ±Ä¬ÈÏ»ñÈ¡±¾»úip
        self.click_get_ip()

    def connect(self):
        """
        ¿Ø¼þÐÅºÅ-²ÛµÄÉèÖÃ
        :param : QDialogÀà´´½¨µÄ¶ÔÏó
        :return: None
        """
        # ÈçÐè´«µÝ²ÎÊý¿ÉÒÔÐÞ¸ÄÎªconnect(lambda: self.click(²ÎÊý))
        super(MainWindow, self).connect()
        self.pushButton_link.clicked.connect(self.click_link)          #ÕâÊÇÁ¬½ÓÍøÂç°´Å¥
        self.pushButton_unlink.clicked.connect(self.click_unlink)#¶Ï¿ªÍøÂç°´Å¥
        self.pushButton_get_ip.clicked.connect(self.click_get_ip)#ÖØÐÂ»ñÈ¡±¾»úIP°´Å¥
        self.pushButton_send.clicked.connect(self.send)#·¢ËÍ°´Å¥


    def init(self):
        self.pushButton_link.clicked.connect(self.click_link)          #ÕâÊÇÁ¬½ÓÍøÂç°´Å¥
        self.pushButton_unlink.clicked.connect(self.click_unlink)#¶Ï¿ªÍøÂç°´Å¥
        self.pushButton_get_ip.clicked.connect(self.click_get_ip)#ÖØÐÂ»ñÈ¡±¾»úIP°´Å¥

        #tcpÐÅÏ¢ÏÔÊ¾
        self.signal_write_msg.connect(self.write_msg)

        # ´®¿Ú¼ì²â°´Å¥
        self.s1__box_1.clicked.connect(self.port_check)

        # ´®¿ÚÐÅÏ¢ÏÔÊ¾
        self.s1__box_2.currentTextChanged.connect(self.port_imf)

        # ´ò¿ª´®¿Ú°´Å¥
        self.open_button.clicked.connect(self.port_open)

        # ¹Ø±Õ´®¿Ú°´Å¥
        self.close_button.clicked.connect(self.port_close)

        #´ò¿ªÉãÏñÍ·°´Å¥***************************
        self.open_camera.clicked.connect(cwork)

        self.close_camera.clicked.connect(cdiswork)

        #¿ªÊ¼Ä¿±ê¼ì²â°´Å¥*************************
        self.yolov3.clicked.connect(work)

        #¹Ø±ÕÄ¿±ê¼ì²â°´Å¥
        self.close_yolov3.clicked.connect(diswork)

        # ¶¨Ê±Æ÷½ÓÊÕÊý¾Ý
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.data_receive)

        # Çå³ý½ÓÊÕ´°¿Ú
        self.s2__clear_button.clicked.connect(self.receive_data_clear)


    def write_msg(self, msg):#ÍøÂç½ÓÊÕÏÔÊ¾
        # signal_write_msgÐÅºÅ»á´¥·¢Õâ¸öº¯Êý
        """
        ¹¦ÄÜº¯Êý£¬Ïò½ÓÊÕÇøÐ´ÈëÊý¾ÝµÄ·½·¨
        ÐÅºÅ-²Û´¥·¢
        tip£ºPyQt³ÌÐòµÄ×ÓÏß³ÌÖÐ£¬Ö±½ÓÏòÖ÷Ïß³ÌµÄ½çÃæ´«Êä×Ö·ûÊÇ²»·ûºÏ°²È«Ô­ÔòµÄ
        :return: None
        """
        self.textBrowser_recv.insertPlainText(msg)
        # ¹ö¶¯ÌõÒÆ¶¯µ½½áÎ²
        self.textBrowser_recv.moveCursor(QTextCursor.End)
    


    # ´®¿Ú¼ì²â
    def port_check(self):
        # ¼ì²âËùÓÐ´æÔÚµÄ´®¿Ú£¬½«ÐÅÏ¢´æ´¢ÔÚ×ÖµäÖÐ
        self.Com_Dict = {}
        port_list = list(serial.tools.list_ports.comports())
        self.s1__box_2.clear()
        for port in port_list:
            self.Com_Dict["%s" % port[0]] = "%s" % port[1]
            self.s1__box_2.addItem(port[0])
        if len(self.Com_Dict) == 0:
            self.state_label.setText(" ÎÞ´®¿Ú")

    # ´®¿ÚÐÅÏ¢
    def port_imf(self):
        # ÏÔÊ¾Ñ¡¶¨µÄ´®¿ÚµÄÏêÏ¸ÐÅÏ¢
        imf_s = self.s1__box_2.currentText()
        if imf_s != "":
            self.state_label.setText(self.Com_Dict[self.s1__box_2.currentText()])

    # ´ò¿ª´®¿Ú
    def port_open(self):
        self.ser.port = self.s1__box_2.currentText()
        self.ser.baudrate = int(self.s1__box_3.currentText())
        self.ser.bytesize = int(self.s1__box_4.currentText())
        self.ser.stopbits = int(self.s1__box_6.currentText())
        self.ser.parity = self.s1__box_5.currentText()

        try:
            self.ser.open()
        except:
            QMessageBox.critical(self, "Port Error", "´Ë´®¿Ú²»ÄÜ±»´ò¿ª£¡")
            return None

        # ´ò¿ª´®¿Ú½ÓÊÕ¶¨Ê±Æ÷£¬ÖÜÆÚÎª2ms
        self.timer.start(2)

        if self.ser.isOpen():
            self.open_button.setEnabled(False)
            self.close_button.setEnabled(True)
            self.formGroupBox1.setTitle("´®¿Ú×´Ì¬£¨ÒÑ¿ªÆô£©")
    # ¹Ø±Õ´®¿Ú
    def port_close(self):
        self.timer.stop()
        try:
            self.ser.close()
        except:
            pass
        self.open_button.setEnabled(True)
        self.close_button.setEnabled(False)
        # ½ÓÊÕÊý¾ÝºÍ·¢ËÍÊý¾ÝÊýÄ¿ÖÃÁã
        self.data_num_received = 0
        self.lineEdit.setText(str(self.data_num_received))
        self.formGroupBox1.setTitle("´®¿Ú×´Ì¬£¨ÒÑ¹Ø±Õ£©")
    # ´®¿Ú½ÓÊÕÊý¾Ý
    def data_receive(self):
        try:
            num = self.ser.inWaiting()
        except:
            self.port_close()
            return None
        if num > 0:
            data = self.ser.read(num)
            num = len(data)
            # hexÏÔÊ¾
            if self.hex_receive.checkState():
                out_s = ''
                for i in range(0, len(data)):
                    out_s = out_s + '{:02X}'.format(data[i]) + ' '
                self.s2__receive_text.insertPlainText(out_s)
            else:
                # ´®¿Ú½ÓÊÕµ½µÄ×Ö·û´®Îªb'123',Òª×ª»¯³Éunicode×Ö·û´®²ÅÄÜÊä³öµ½´°¿ÚÖÐÈ¥
                self.s2__receive_text.insertPlainText(data.decode('iso-8859-1'))
                if self.link is False:
                    msg = 'ÇëÑ¡Ôñ·þÎñ£¬²¢µã»÷Á¬½ÓÍøÂç\n'
                    self.signal_write_msg.emit(msg)
                else:
                    try:
                        send_msg = data.decode().strip()
                        if send_msg == "1":
                            print("get 1")
                            self.tcp_socket.send(b"1")
                        elif send_msg == "0":
                            print("get 0")
                            self.tcp_socket.send(b"0")
                        msg = 'TCP¿Í»§¶ËÒÑ·¢ËÍ\n'
                        self.signal_write_msg.emit(msg)
                    except Exception as ret:
                        msg = '·¢ËÍÊ§°Ü\n'
                        self.signal_write_msg.emit(msg)

            # Í³¼Æ½ÓÊÕ×Ö·ûµÄÊýÁ¿
            self.data_num_received += num
            self.lineEdit.setText(str(self.data_num_received))

            # »ñÈ¡µ½text¹â±ê
            textCursor = self.s2__receive_text.textCursor()
            # ¹ö¶¯µ½µ×²¿
            textCursor.movePosition(textCursor.End)
            # ÉèÖÃ¹â±êµ½textÖÐÈ¥
            self.s2__receive_text.setTextCursor(textCursor)
        else:
            pass

    # ¶¨Ê±·¢ËÍÊý¾Ý
    def data_send_timer(self):
        if self.timer_send_cb.isChecked():
            self.timer_send.start(int(self.lineEdit_3.text()))
            self.lineEdit_3.setEnabled(False)
        else:
            self.timer_send.stop()
            self.lineEdit_3.setEnabled(True)

    # Çå³ýÏÔÊ¾
    def receive_data_clear(self):
        self.s2__receive_text.setText("")

    def tcp_client_start(self):
        """
        ¹¦ÄÜº¯Êý£¬TCP¿Í»§¶ËÁ¬½ÓÆäËû·þÎñ¶ËµÄ·½·¨
        :return:
        """
        self.tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            address = (str(self.lineEdit_ip.text()), int(self.lineEdit_port.text()))
        except Exception as ret:
            msg = 'Çë¼ì²éÄ¿±êIP£¬Ä¿±ê¶Ë¿Ú\n'
            self.signal_write_msg.emit(msg)
        else:
            try:
                msg = 'ÕýÔÚÁ¬½ÓÄ¿±ê·þÎñÆ÷\n'
                self.signal_write_msg.emit(msg)
                self.tcp_socket.connect(address)
            except Exception as ret:
                msg = 'ÎÞ·¨Á¬½ÓÄ¿±ê·þÎñÆ÷\n'
                self.signal_write_msg.emit(msg)
            else:
                self.client_th = threading.Thread(target=self.tcp_client_concurrency, args=(address,))
                self.client_th.start()
                msg = 'TCP¿Í»§¶ËÒÑÁ¬½ÓIP:%s¶Ë¿Ú:%s\n' % address
                self.signal_write_msg.emit(msg)

    def tcp_client_concurrency(self, address):
        """
        ¹¦ÄÜº¯Êý£¬ÓÃÓÚTCP¿Í»§¶Ë´´½¨×ÓÏß³ÌµÄ·½·¨£¬×èÈûÊ½½ÓÊÕ
        :return:
        """
        while True:
            recv_msg = self.tcp_socket.recv(1024)
            if recv_msg:
                msg = recv_msg.decode('utf-8')#½ÓÊÕÊý¾Ý
                msg = 'À´×ÔIP:{}¶Ë¿Ú:{}:\n{}\n'.format(address[0], address[1], msg)
                self.signal_write_msg.emit(msg)
            else:
                self.tcp_socket.close()
                self.reset()
                msg = '´Ó·þÎñÆ÷¶Ï¿ªÁ¬½Ó\n'
                self.signal_write_msg.emit(msg)
                break

    def tcp_close(self):
        """
        ¹¦ÄÜº¯Êý£¬¹Ø±ÕÍøÂçÁ¬½ÓµÄ·½·¨
        :return:
        """
        try:
            self.tcp_socket.close()
            if self.link is True:
                msg = 'ÒÑ¶Ï¿ªÍøÂç\n'
                self.signal_write_msg.emit(msg)
        except Exception as ret:
            pass
        try:
            stopThreading.stop_thread(self.client_th)
        except Exception:
            pass

    def click_link(self):
        """
        pushbutton_link¿Ø¼þµã»÷´¥·¢µÄ²Û
        :return: None
        """
        # Á¬½ÓÊ±¸ù¾ÝÓÃ»§Ñ¡ÔñµÄ¹¦ÄÜµ÷ÓÃº¯Êý
        self.tcp_client_start()
        self.link = True
        self.pushButton_unlink.setEnabled(True)
        self.pushButton_link.setEnabled(False)

    def click_unlink(self):
        """
        pushbutton_unlink¿Ø¼þµã»÷´¥·¢µÄ²Û
        :return: None
        """
        # ¹Ø±ÕÁ¬½Ó
        self.close_all()
        self.link = False
        self.pushButton_unlink.setEnabled(False)
        self.pushButton_link.setEnabled(True)

    def click_get_ip(self):#»ñÈ¡±¾µØip
        """
        pushbutton_get_ip¿Ø¼þµã»÷´¥·¢µÄ²Û
        :return: None
        """
        self.lineEdit_ip_local.clear()
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            s.connect(('8.8.8.8', 80))
            my_addr = s.getsockname()[0]
            self.lineEdit_ip_local.setText(str(my_addr))
        except Exception as ret:
            # ÈôÎÞ·¨Á¬½Ó»¥ÁªÍøÊ¹ÓÃ£¬»áµ÷ÓÃÒÔÏÂ·½·¨
            try:
                my_addr = socket.gethostbyname(socket.gethostname())
                self.lineEdit_ip_local.setText(str(my_addr))
            except Exception as ret_e:
                self.signal_write_msg.emit("ÎÞ·¨»ñÈ¡ip£¬ÇëÁ¬½ÓÍøÂç£¡\n")
        finally:
            s.close()

    def close_all(self):
        """
        ¹¦ÄÜº¯Êý£¬¹Ø±ÕÍøÂçÁ¬½ÓµÄ·½·¨
        :return:
        """
        # Á¬½ÓÊ±¸ù¾ÝÓÃ»§Ñ¡ÔñµÄ¹¦ÄÜµ÷ÓÃº¯Êý
        self.tcp_close()
        self.reset()

    def reset(self):
        """
        ¹¦ÄÜº¯Êý£¬½«°´Å¥ÖØÖÃÎª³õÊ¼×´Ì¬
        :return:None
        """
        self.link = False
        self.client_socket_list = list()
        self.pushButton_unlink.setEnabled(False)
        self.pushButton_link.setEnabled(True)

if __name__ == '__main__':
    app = QtWidgets.QApplication(sys.argv)
    workThread = WorkThread()
    cworkThread = CWorkThread()
    myshow = Pyqt5_Serial()
    myshow.show()
    sys.exit(app.exec_())