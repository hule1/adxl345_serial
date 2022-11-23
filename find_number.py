import re
import array
import serial
import threading
import numpy as np
import time
import pyqtgraph as pg


class Draw:
    def __init__(self):
        self.mSerial = None
        self.data_x = array.array('i')
        self.data_y = array.array('i')
        self.data_z = array.array('i')
        self.historyLength = 200
        self.idxPlot = 0
        self.win = None
        self.app = None
        self.p = None
        self.curve_z = None
        self.curve_y = None
        self.curve_x = None

    def main(self):

        th1 = threading.Thread(target=self.Serial)
        th1.start()

        self.data_processing()
        self.plot_init()
        timer = pg.QtCore.QTimer()
        timer.timeout.connect(self.plot_data)  # 定时刷新数据显示
        timer.start(1)  # 多少ms调用一次
        self.app.exec_()

    def Serial(self):
        portx = 'COM5'
        bps = 9600
        # 串口执行到这已经打开 再用open命令会报错
        self.mSerial = serial.Serial(portx, int(bps))
        if (self.mSerial.isOpen()):
            print("open success")
            self.mSerial.write("hello".encode())  # 向端口些数据 字符串必须译码
            self.mSerial.flushInput()  # 清空缓冲区
        else:
            print("open failed")
            serial.close()  # 关闭端口

    def data_processing(self):
        self.data_x = np.zeros(self.historyLength).__array__('d')  # 把数组长度定下来
        self.data_y = np.zeros(self.historyLength).__array__('d')
        self.data_z = np.zeros(self.historyLength).__array__('d')

    def plot_data(self):

        n = self.mSerial.inWaiting()
        if (n):
            list_line = self.mSerial.readline().decode()
            list_number = re.findall("\d+", list_line)
            list_number = list(map(int, list_number))
            print(list_number)
            if self.idxPlot < self.historyLength:
                self.data_x[self.idxPlot] = list_number[0]
                self.data_y[self.idxPlot] = list_number[1]
                self.data_z[self.idxPlot] = list_number[2]
                self.idxPlot += 1
            else:
                self.data_x[:-1] = self.data_x[1:]
                self.data_x[self.idxPlot - 1] = list_number[0]
                self.data_y[:-1] = self.data_y[1:]
                self.data_y[self.idxPlot - 1] = list_number[1]
                self.data_z[:-1] = self.data_z[1:]
                self.data_z[self.idxPlot - 1] = list_number[2]
        self.curve_x.setData(self.data_x)
        self.curve_y.setData(self.data_y)
        self.curve_z.setData(self.data_z)

    def plot_init(self):
        pg.setConfigOption('background', 'w')
        pg.setConfigOption('foreground', 'k')
        self.app = pg.mkQApp()  # 建立app
        self.win = pg.GraphicsWindow()  # 建立窗口
        self.win.setWindowTitle(u'pyqtgraph逐点画波形图')
        self.win.resize(800, 500)  # 小窗口大小
        self.p = self.win.addPlot()  # 把图p加入到窗口中
        self.p.showGrid(x=True, y=True)  # 把X和Y的表格打开
        self.p.setRange(xRange=[0, self.historyLength], yRange=[-1000, 1000], padding=0)
        self.p.setLabel(axis='left', text='y / V')  # 靠左
        self.p.setLabel(axis='bottom', text='x / point')
        self.p.setTitle('加速度波形图')  # 表格的名字

        self.curve_x = self.p.plot(pen=pg.mkPen(width=2, color='r'))  # 绘制一个图形
        self.curve_x.setData(self.data_x)
        self.curve_y = self.p.plot(pen=pg.mkPen(width=2, color='g'))  # 绘制一个图形
        self.curve_y.setData(self.data_y)
        self.curve_z = self.p.plot(pen=pg.mkPen(width=2, color='y'))  # 绘制一个图形
        self.curve_z.setData(self.data_z)


if __name__ == '__main__':
    draw = Draw()
    draw = draw.main()
