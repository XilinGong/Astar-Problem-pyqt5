import sys
import time
import numpy as np
from PyQt5.QtWidgets import QApplication, QWidget, QPushButton, QMessageBox, QLabel, QLineEdit
from PyQt5.QtGui import QPainter, QColor, QPen
from PyQt5.QtCore import Qt, QThread, QMutex, pyqtSignal


class Map:
    def __init__(self, w, h, start, end, walllist):
        self.width = w
        self.heigth = h
        self.start = start
        self.end = end
        self.map = np.zeros((self.width, self.heigth))  # 没有访问过的普通位置是0
        for wall in walllist:
            self.map[wall] = -1  # 不能走过的墙是-1

    def isAvailable(self, x, y):
        if 0 <= x < self.width and 0 <= y < self.heigth and self.map[(x, y)] != -1:
            '''判断是墙或者在地图外面'''
            return True
        return False

    def arrivedEnd(self, x, y):
        if x == self.end[0] and y == self.end[1]:
            ''' 判断结束条件 到终点了？'''
            return True
        return False

    def drawMap(self):
        # 用来调试
        for i in range(self.heigth):
            for j in range(self.width):
                print(int(self.map[(j, i)]), end='')
            print('\n')


class AStar(QThread):
    # 设置若干输出信号
    signal_failed = pyqtSignal()  # 没有路
    signal_succeed = pyqtSignal()  # 找到路
    signal_step = pyqtSignal()  # 每次openlist和closelist更新

    def __init__(self, w, h, start, end, walllist):
        super(AStar, self).__init__()
        self.w = w
        self.h = h
        self.startpoint = start
        self.mapforastar = Map(w, h, start, end, walllist)

        self.openlist = []
        self.closelist = []
        self.eightneighbour = 1  # 邻接方式
        self.final_path = []  # 终点路径
        '''维护g h数组,其中g数组在放入open/closelist的时候计算值
            h数组在最开始就计算好'''
        self.hmap = np.zeros((self.w, self.h))
        self.gmap = -1 * np.ones((self.w, self.h))
        for i in range(self.w):
            for j in range(self.h):
                if self.mapforastar.map[(i, j)] == -1:
                    self.hmap[(i, j)] = -1
                else:
                    self.hmap[(i, j)] = self.distance((i, j), self.mapforastar.end)
        '''维护父节点数组，初始化为（-1，-1），每次扩展suclist的时候修改'''
        self.fatherlist = list()
        for i in range(self.w):
            self.fatherlist.append(list())
            for j in range(self.h):
                self.fatherlist[i].append((-1, -1))

        # muti thread
        self.mutex = QMutex()
        # 用于判断是否在找路 找路过程中不要更改地图
        self.isRunning = False

    def distance(self, p1, p2):
        dx = abs(p1[0] - p2[0])
        dy = abs(p1[1] - p2[1])
        if dx == dy and dx == 1:
            # 不可以穿墙
            if self.mapforastar.isAvailable(p1[0], p2[1]) is not True:
                return -1
            elif self.mapforastar.isAvailable(p2[0], p1[1]) is not True:
                return -1
        if self.eightneighbour is True:
            return 1.4 * min(dx, dy) + max(dx, dy) - min(dx, dy)
        return dx + dy

    def getNeighbour(self, x, y):
        suclist = list()
        if self.eightneighbour:
            for i in range(x - 1, x + 2):
                for j in range(y - 1, y + 2):
                    if i == x and j == y:
                        continue
                    # 满足不穿墙
                    if self.mapforastar.isAvailable(i, j) and self.distance((i, j), (x, y)) > 0:
                        suclist.append((i, j))
        else:
            if self.mapforastar.isAvailable(x - 1, y) and self.distance((x - 1, y), (x, y)) > 0:
                suclist.append((x - 1, y))
            if self.mapforastar.isAvailable(x, y - 1) and self.distance((x, y - 1), (x, y)) > 0:
                suclist.append((x, y - 1))
            if self.mapforastar.isAvailable(x + 1, y) and self.distance((x + 1, y), (x, y)) > 0:
                suclist.append((x + 1, y))
            if self.mapforastar.isAvailable(x, y + 1) and self.distance((x, y + 1), (x, y)) > 0:
                suclist.append((x, y + 1))
        return suclist

    def failed(self):
        print('failed')
        self.isRunning = False
        self.signal_failed.emit()

    def success(self):
        currnode = self.fatherlist[self.mapforastar.end[0]][self.mapforastar.end[1]]
        while currnode != self.startpoint:
            self.final_path.append(currnode)
            print(self.fatherlist[currnode[0]][currnode[1]])
            currnode = self.fatherlist[currnode[0]][currnode[1]]
        print('success')
        self.isRunning = False
        self.signal_succeed.emit()

    def findbest(self):
        best_node = self.openlist[0]
        best_f = self.gmap[best_node] + self.hmap[best_node]
        for node in self.openlist:
            f = self.gmap[node] + self.hmap[node]
            if f < best_f:
                best_node = node
                best_f = f
        return best_node

    def astar(self):
        self.isRunning = True
        '''把起点放到openlist里，维护gmap，然后开始循环'''
        self.openlist.append(self.mapforastar.start)
        self.gmap[self.mapforastar.start] = 0
        while True:
            if len(self.openlist) == 0:
                '''如果openlist是空的，那么已经没有可以去的节点了，找路失败'''
                self.failed()
                return
            '''找到openlist中f最小的节点，然后把它从openlist挪到closelist中'''
            bestnode = self.findbest()
            self.openlist.remove(bestnode)
            self.closelist.append(bestnode)
            if self.mapforastar.arrivedEnd(bestnode[0], bestnode[1]):
                '''如果找到的点是终点 那么就成功了'''
                self.success()
                return
            '''找出当前点的子节点，并算出从当前点走过去的g值，如果这个点之前就被发现过
            （指在open或close表中），那么根据g值大小决定是否更新他的父节点和gmap；如果没有被
            发现过，那么把它放到open表中待考察，更新他的父节点和gmap'''
            suclist = self.getNeighbour(bestnode[0], bestnode[1])
            for suc in suclist:
                currg = self.gmap[bestnode] + self.distance(suc, bestnode)
                if suc in self.openlist or suc in self.closelist:
                    if currg < self.gmap[suc]:
                        self.fatherlist[suc[0]][suc[1]] = bestnode
                        self.gmap[suc] = currg
                else:
                    self.openlist.append(suc)
                    self.gmap[suc] = currg
                    self.fatherlist[suc[0]][suc[1]] = bestnode
            self.signal_step.emit()
            time.sleep(0.05)

    def run(self):
        self.mutex.lock()
        self.astar()
        self.mutex.unlock()


class Window(QWidget):

    def __init__(self):
        super().__init__()
        self.graphLength = 800
        self.w = 8
        self.boxsize = int(self.graphLength / self.w)
        self.walllist = [(3, 1), (3, 2), (3, 3), (3, 4)]
        self.start = (0, 2)
        self.end = (6, 2)
        self.initUI1()
        self.curpos = (-1, -1)
        self.problem = None

    def curposAvailable(self):
        return 0 <= self.curpos[0] < self.w and 0 <= self.curpos[1] <= self.w * 3 / 4

    def mousePressEvent(self, e):
        if e.buttons() == Qt.LeftButton:
            pos = e.pos()
            self.curpos = (int(pos.x() / self.boxsize), int(pos.y() / self.boxsize))

    def keyPressEvent(self, e):
        if self.problem is None or self.problem.isRunning is False:
            # shift for start_point || alt for end point ||Capslock for wall
            if self.curposAvailable():
                if e.key() == Qt.Key_Shift:
                    if self.start == self.curpos:
                        self.start = (-1, -1)
                    else:
                        self.start = self.curpos
                        if self.curpos == self.end:
                            self.end = (-1, -1)
                        if self.curpos in self.walllist:
                            self.walllist.remove(self.curpos)
                    self.repaint()

                elif e.key() == Qt.Key_Alt:
                    if self.end == self.curpos:
                        self.end = (-1, -1)
                    else:
                        self.end = self.curpos
                        if self.start == self.curpos:
                            self.start = (-1, -1)
                        if self.curpos in self.walllist:
                            self.walllist.remove(self.curpos)
                    self.repaint()

                elif e.key() == Qt.Key_CapsLock:
                    if self.curpos in self.walllist:
                        self.walllist.remove(self.curpos)
                    else:
                        self.walllist.append(self.curpos)
                        if self.curpos == self.end:
                            self.end = (-1, -1)
                        if self.start == self.curpos:
                            self.start = (-1, -1)
                    self.repaint()

    def initUI1(self):
        self.lengthBox = QLabel('地图长度', self)
        self.numberBox = QLineEdit('8', self)
        # startButton = QPushButton('OK', self)
        startButton2 = QPushButton('START', self)
        resetButton = QPushButton('RESET', self)
        methodButton = QPushButton('HELP', self)

        self.w = int(self.numberBox.text())

        startButton2.clicked.connect(self.start2)
        resetButton.clicked.connect(self.restart)
        methodButton.clicked.connect(self.showHowToUse)

        self.numberBox.resize(100, 40)
        self.lengthBox.resize(100, 40)
        startButton2.resize(100, 40)
        resetButton.resize(100, 40)
        methodButton.resize(100, 40)

        self.lengthBox.move(int(self.graphLength + 50), 100)
        self.numberBox.move(int(self.graphLength + 50), 150)
        startButton2.move(int(self.graphLength + 50), 350)
        resetButton.move(int(self.graphLength + 50), 450)
        methodButton.move(int(self.graphLength + 50), 250)

        self.setGeometry(15, 150, int(self.graphLength + 200), int(self.graphLength * 3 / 4) + 20)
        self.setWindowTitle('AStar----请先点击HELP查看地图设置方法')
        self.show()

    def showHowToUse(self):
        QMessageBox.information(self, 'HELP',
                                "鼠标点击指定格子后\n按下键盘上对应按键\nshift设置红色起点\nalt设置绿色终点\nCapsLK设置蓝色墙" +
                                '\n再次设置可以取消')

    def start2(self):
        if self.end != (-1, -1) and self.start != (-1, -1):
            self.problem = AStar(self.w, int(self.w * 3 / 4), self.start, self.end, self.walllist)
            self.problem.signal_succeed.connect(self.repaint)
            self.problem.signal_failed.connect(self.showmessagebox)
            self.problem.signal_step.connect(self.repaint)
            print('prepared')
            self.problem.start()
        else:
            self.showmessagebox(2)

    def restart(self):
        self.end = (-1, -1)
        self.start = (-1, -1)
        self.walllist = []
        self.w = int(self.numberBox.text())
        self.boxsize = int(self.graphLength / self.w)
        if self.problem is not None:
            self.problem.openlist = []
            self.problem.closelist = []
            self.problem.final_path = []
        self.repaint()

    def showmessagebox(self, num=3):
        if num == 2:
            QMessageBox.information(self, 'WARNING', '请检查起点和终点')
        else:
            QMessageBox.information(self, 'FAILED', '无法到达')

    def paintEvent(self, e):
        self.paintInit()

    def paintInit(self):
        qp = QPainter()
        qp.begin(self)
        boxsize = self.boxsize
        # for board
        pen = QPen(Qt.black, 2, Qt.SolidLine)
        qp.setPen(pen)
        for i in range(int(self.w * 3 / 4) + 1):
            qp.drawLine(5, 5 + i * boxsize, 5 + boxsize * self.w, 5 + i * boxsize)
        for i in range(int(self.w) + 1):
            qp.drawLine(5 + i * boxsize, 5, 5 + i * boxsize, 5 + boxsize * int(self.w * 3 / 4))
        # for start node
        if self.start != (-1, -1):
            qp.setBrush(QColor(255, 0, 0))
            qp.drawRect(5 + self.start[0] * boxsize, 5 + self.start[1] * boxsize, boxsize, boxsize)

        # for end node
        if self.end != (-1, -1):
            qp.setBrush(QColor(0, 255, 0))
            qp.drawRect(5 + self.end[0] * boxsize, 5 + self.end[1] * boxsize, boxsize, boxsize)

        # for wall
        if len(self.walllist) > 0:
            for wall in self.walllist:
                qp.setBrush(QColor(0, 0, 255))
                qp.drawRect(5 + wall[0] * boxsize, 5 + wall[1] * boxsize, boxsize, boxsize)

        if self.problem is not None:
            if len(self.problem.final_path) == 0:
                for close in self.problem.closelist:
                    if close == self.end or close == self.start:
                        continue
                    qp.setBrush(QColor(128, 128, 255))
                    qp.drawRect(5 + close[0] * boxsize, 5 + close[1] * boxsize, boxsize, boxsize)
                for open in self.problem.openlist:
                    if open == self.end or open == self.start:
                        continue
                    qp.setBrush(QColor(200, 200, 255))
                    qp.drawRect(5 + open[0] * boxsize, 5 + open[1] * boxsize, boxsize, boxsize)
            else:
                for path in self.problem.final_path:
                    qp.setBrush(QColor(120, 0, 255))
                    qp.drawRect(5 + path[0] * boxsize, 5 + path[1] * boxsize, boxsize, boxsize)
        qp.end()


if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = Window()
    sys.exit(app.exec_())
