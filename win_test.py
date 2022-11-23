import array
import serial
import threading
import numpy as np
import time
import pyqtgraph as pg

def plotData():
    curve.setData(data)

data = array.array('i')  # 可动态改变数组的大小,double型数组
historyLength = 200  # 横坐标长度
data = np.zeros(historyLength).__array__('d')  # 把数组长度定下来
for i in range(200):
    data[i]=i

pg.setConfigOption('background', 'w')
pg.setConfigOption('foreground', 'k')
app = pg.mkQApp()  # 建立app
win = pg.GraphicsWindow()  # 建立窗口
win.setWindowTitle(u'pyqtgraph逐点画波形图')
win.resize(800, 500)  # 小窗口大小
# win.setBackground('w')
a = 0
p = win.addPlot()  # 把图p加入到窗口中
p.showGrid(x=True, y=True)  # 把X和Y的表格打开
p.setRange(xRange=[0, historyLength], yRange=[-4095, 4095], padding=0)
p.setLabel(axis='left', text='y / V')  # 靠左
p.setLabel(axis='bottom', text='x / point')
p.setTitle('电压波形图')  # 表格的名字

curve = p.plot(pen=pg.mkPen(width=3, color='g'))  # 绘制一个图形
curve.setData(data)
timer = pg.QtCore.QTimer()
timer.timeout.connect(plotData)  # 定时刷新数据显示
timer.start(1)  # 多少ms调用一次
app.exec_()
