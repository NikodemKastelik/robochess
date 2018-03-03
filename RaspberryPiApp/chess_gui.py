import sys
import numpy as np
from PyQt4 import QtGui, QtCore
import threading
import multiprocessing
import copy
import time
import queue

from chess_bus import CyclicBus


wR, wN, wB, wQ, wK, wP = 1, 2, 3, 4, 5, 6
bR, bN, bB, bQ, bK, bP = 11, 12, 13, 14, 15, 16
basic_chessboard = np.zeros(shape=(8,8), dtype=int)
basic_chessboard[0] = [bR, bN, bB, bQ, bK, bB, bN, bR]
basic_chessboard[1] = [bP, bP, bP, bP, bP, bP, bP, bP]
basic_chessboard[6] = [wP, wP, wP, wP, wP, wP, wP, wP]
basic_chessboard[7] = [wR, wN, wB, wQ, wK, wB, wN, wR]

class Popup(QtGui.QWidget):
    popup_width = 200
    popup_height = 200

    def __init__(self, controller):
        super(Popup, self).__init__()
        self.controller = controller
        self.setWindowFlags(self.windowFlags() | QtCore.Qt.FramelessWindowHint | QtCore.Qt.WindowStaysOnTopHint)
        self.resize(self.popup_width, self.popup_height)
        self.followParent()
        self.show()


    def followParent(self):
        parent_width = self.controller.frameGeometry().width()
        parent_height = self.controller.frameGeometry().height()
        parent_x = self.controller.pos().x()
        parent_y = self.controller.pos().y()

        this_x = int(parent_x + parent_width/2 - self.popup_width/2)
        this_y = int(parent_y + parent_height/2 - self.popup_height/2)

        self.move(this_x, this_y)


class SliderMore(QtGui.QFrame):
    def __init__(self, parent, controller, lefttext, righttext, minval, maxval, defval):
        super(SliderMore, self).__init__(parent)
        self.parent = parent
        self.controller = controller

        grid = QtGui.QGridLayout()
        self.setLayout(grid)

        self.slider = QtGui.QSlider(QtCore.Qt.Horizontal, self)
        self.slider.setTickPosition(QtGui.QSlider.TicksBelow)
        self.slider.setTickInterval(1)
        self.slider.setRange(minval,maxval)
        self.slider.setValue(defval)
        self.slider.valueChanged.connect(lambda val: self.value_label.setText(str(val)))
        #self.slider.connect(self.slider, QtCore.SIGNAL("valueChanged(int)"),
        #                         lambda x=self.slider.value(): self.value_label.setText(str(x)))

        #self.value_label = QtGui.QLineEdit(str(self.slider.value()), self)
        #self.value_label.setReadOnly(True)
        #self.value_label.setStyleSheet("QLineEdit {background-color: transparent; border: 0px solid}")
        self.value_label = QtGui.QLabel(str(0), self)
        self.value_label.setAlignment(QtCore.Qt.AlignHCenter)

        self.minval_label = QtGui.QLabel(lefttext, self)
        self.minval_label.setAlignment(QtCore.Qt.AlignHCenter)

        self.maxval_label = QtGui.QLabel(righttext, self)
        self.maxval_label.setAlignment(QtCore.Qt.AlignHCenter)

        grid.addWidget(self.value_label, 0,0, 1, 3)
        grid.addWidget(self.minval_label, 1,0, 1, 1)
        grid.addWidget(self.slider, 1, 1, 1, 1)
        grid.addWidget(self.maxval_label, 1, 2, 1 ,1)

        self.slider.valueChanged.emit(self.slider.value())



class DragIcon(QtGui.QLabel):

    def __init__(self, text, parent, controller, row_col):
        super(DragIcon, self).__init__(text, parent)
        self.parent = parent
        self.controller = controller
        self.row_col = row_col
        self.prevpic = None
        self.piece_color = None

    #def dragMoveEvent(self, event):
    #    print("Draggin'")

    def dragEnterEvent(self, event):
        #print("Drag enter")
        event.accept()


    def dropEvent(self, event):
        print("Drop in: ", self.row_col)
        img = event.mimeData().imageData()
        arr = event.mimeData().text()
        row = int(arr[0])
        col = int(arr[1])
        #caller_row_col = [int(arr[0]), int(arr[1])]
        icon = QtGui.QPixmap.fromImage(img)
        self.controller.notifyCaller.emit(row, col)

        self.setPixmap(icon)


    def mousePressEvent(self, event):
        #print("Pressed from: ", self.row_col)
        icon = self.pixmap().copy()
        self.prevpic = icon.copy()
        self.clear()

        #arr = QtCore.QByteArray()
        #arr.append(bytearray(self.row_col))
        #arr.append(bytearray(self.row_col[1]))

        mimeData = QtCore.QMimeData()
        mimeData.setImageData(icon)
        mimeData.setText(str(self.row_col[0])+str(self.row_col[1]))

        qdrag = QtGui.QDrag(self)
        qdrag.setPixmap(icon)
        qdrag.setHotSpot(event.pos() - self.rect().topLeft())
        qdrag.setMimeData(mimeData)
        dropAction = qdrag.start(QtCore.Qt.MoveAction)


    def getNotification(self, row, col):
        print("Object {},{} got {},{}"
              .format(str(self.row_col[0]),str(self.row_col[1]),str(row),str(col)))



class ChessGUI(QtGui.QMainWindow):
    # signals
    notifyCaller = QtCore.pyqtSignal(int, int)

    startMovingIcon = QtCore.pyqtSignal(str)
    moveIcon = QtCore.pyqtSignal(int, int)
    finishMovingIcon = QtCore.pyqtSignal(int, int)

    openPopup = QtCore.pyqtSignal(str)
    changePopup = QtCore.pyqtSignal(str)
    stopPopup = QtCore.pyqtSignal()

    changeStatusBar = QtCore.pyqtSignal(str)

    def __init__(self, rawbus, buswindex, moduleid):
        self.app = QtGui.QApplication(sys.argv)
        super(ChessGUI, self).__init__()

        # store bus reference here
        self.moduleid = moduleid
        self.bus = CyclicBus(rawbus, buswindex, moduleid)

        # store reg values here
        regs_strings = ["player_colour", "difficulty"]
        self.regs = {}
        for str in regs_strings:
            self.regs[str] = None

        # prepare popup
        self.popup = None

        # initiate Qt app
        self.setWindowTitle('RoboChess')
        # self.main_window = QtGui.QWidget()
        # self.main_window.resize(250, 150)
        # self.main_window.move(300, 300)
        self.setWindowIcon(QtGui.QIcon('icons/chess.ico'))

        # create statusbar
        self.statbar = QtGui.QLabel("")
        self.statbar_lines = [" "] * 3
        self.statbar_index = 0

        self.statusbar = self.statusBar()
        self.statusbar.addWidget(self.statbar)
        #self.statusbar.showMessage('Ready')

        # set tooltip formatting
        QtGui.QToolTip.setFont(QtGui.QFont('SansSerif', 10))

        # create Qtabwidget
        self.tabwidget = QtGui.QTabWidget()
        #self.tabwidget.setTabShape(QtGui.QTabWidget.Triangular)
        #self.tabwidget.Triangular


        # set Qtabwidget as central widget
        self.setCentralWidget(self.tabwidget)

        # create 3 tabs
        self.tab1 = ChessboardTab(self) #QtGui.QWidget()
        self.tab2 = SettingsTab(self) #QtGui.QWidget()
        self.tab3 = ManualSteeringTab(self)

        # add tabs to tabwidget
        self.tabwidget.addTab(self.tab1, "Chessboard")
        self.tabwidget.addTab(self.tab2, "Settings")
        self.tabwidget.addTab(self.tab3, "Manual arm control")

        # self.ChessboardFrame = ChessboardTab(self.tab1, self)
        # self.setChessboardTab(self.tab1)
        # self.setSettingsTab(self.tab2)

        # start bus reader thread
        self.startThread(self.commWorkerLoop)

        # exit handler
        self.app.aboutToQuit.connect(self.exitHandler)

        # statusbar connect signal
        #self.changeStatusBar.connect(lambda string: self.statusbar.showMessage(string))
        self.changeStatusBar.connect(lambda string: self.addStatusMessage(string))

        self.show()

        #self.bus.put(("core", "guiReady"))

        #self.popup = Popup(self)


    def addStatusMessage(self, new_line):
        self.statbar_lines[2] = self.statbar_lines[1]
        self.statbar_lines[1] = self.statbar_lines[0]
        self.statbar_lines[0] = time.strftime("[%H:%M:%S] ") + new_line

        self.statbar.setText("\n".join(self.statbar_lines))


    def resizeEvent(self, QResizeEvent):
        if self.popup:
            self.popup.followParent()


    def moveEvent(self, QMoveEvent):
        if self.popup:
            self.popup.followParent()


    def commWorkerLoop(self):
        while True:
            msg = self.bus.get()
            if msg:
                print("Chess_GUI got: ", msg)
                msg = msg.split(";")
                action = msg[CyclicBus.ACTION_INDX]
                if action == "movePiece":
                    self.startMovingIcon.emit(msg[CyclicBus.ARG1_INDX])
                elif action == "startPopup":
                    pass
                elif action == "changeStatus":
                    self.changeStatusBar.emit(msg[CyclicBus.ARG1_INDX])
                elif action  == "changePopup":
                    pass
                elif action == "stopPopup":
                    pass
                else:
                    print("Chess_GUI: no such action: ", msg[CyclicBus.ACTION_INDX])
            time.sleep(0.01)


    def exitHandler(self):
        self.bus.put("cr","terminate")
        if self.popup:
            self.popup.close()
        #time.sleep(0.01)


    def startThread(self, function, *arguments):
        arg_tuple = ()
        for i in range(len(arguments)):
            arg_tuple += (arguments[i],)
        thread = threading.Thread(target=function, args=arg_tuple)
        thread.setDaemon(True)
        thread.start()
        return thread

    def start(self):
        # launch Qt app
        sys.exit(self.app.exec_())


class ChessboardTab(QtGui.QWidget):
    #notifyCaller = QtCore.pyqtSignal(int, int)
    # = QtCore.pyqtSignal(str)
    #moveIcon = QtCore.pyqtSignal(int, int)
    #finishMovingIcon = QtCore.pyqtSignal(int, int)

    def __init__(self, controller):
        super(ChessboardTab, self).__init__()

        self.controller = controller


        vbox = QtGui.QVBoxLayout()
        vbox.setAlignment(QtCore.Qt.AlignHCenter)
        # vbox.addStretch(1)
        self.setLayout(vbox)

        # create controls frame in parent
        self.control_frame = QtGui.QFrame(self)
        self.control_frame.setFrameStyle(QtGui.QFrame.Panel | QtGui.QFrame.Sunken)
        self.control_frame.setLineWidth(5)

        # embed control frame in parent
        vbox.addWidget(self.control_frame)

        # create frame containing chessboard
        self.chessb_frame = QtGui.QFrame(self)
        self.chessb_frame.setFrameStyle(QtGui.QFrame.Panel | QtGui.QFrame.Sunken)
        self.chessb_frame.setLineWidth(5)

        # embed chessb frame in parent
        vbox.addWidget(self.chessb_frame)

        # create button for user to indicate that move has been made
        self.nextmove_button = QtGui.QPushButton("Next!", self.chessb_frame)
        self.nextmove_button.setMinimumHeight(50)
        #self.nextmove_button.setStyleSheet("QPushButton {height: 50px;}")
        self.nextmove_button.clicked.connect(lambda : self.controller.bus.put("cv","makeTurn;"))
        #self.nextmove_button.connect(self.nextmove_button, QtCore.SIGNAL("clicked()"),
        #                             lambda : self.controller.bus.put(("cv","makeTurn","")))
        vbox.addWidget(self.nextmove_button)

        # create grid layout in control frame
        grid = QtGui.QGridLayout()
        self.control_frame.setLayout(grid)

        # create buttons in control frame
        self.startng_button = QtGui.QPushButton("Start!")
        self.startng_button.clicked.connect(lambda: self.startNewGame(None))
        #self.startng_button.connect(self.st  artng_button, QtCore.SIGNAL("clicked()"),
        #                            lambda: self.controller.bus.put(("cv","startNG","")))
        grid.addWidget(self.startng_button, 0, 0)

        # create grid layout in chessb frame
        grid = QtGui.QGridLayout()
        grid.setSpacing(0)
        grid.setMargin(3)
        self.chessb_frame.setLayout(grid)

        # create files labels

        for i in range(2):
            for j in range(8):
                for k in range(2):
                    filelabel = QtGui.QLabel(chr(i * (ord("a") + j) + (1 - i) * (ord("8") - j)), self.chessb_frame)
                    filelabel.setStyleSheet("QLabel {border: 3px solid}")
                    if i == 1:
                        filelabel.setAlignment(QtCore.Qt.AlignHCenter)
                    grid.addWidget(filelabel, 9 * k * i + (1 - i) * (j + 1), 9 * (1 - k) * (1 - i) + (i) * (j + 1))

        # create black and white files
        k = 0
        m = 0
        # whitenblack = ["white","black"]
        whitenblack = [[102, 51, 0], [255, 255, 153]]
        self.files = []
        arr = []
        for i in range(64):
            # button = QtGui.QLabel("", self.chessb_frame)
            button = DragIcon("", self.chessb_frame, self.controller, [m, k])
            button.setFixedSize(50, 50)
            # button.setToolTip('Button ' + str(i))
            # button.setStyleSheet("QLabel {background-color: "+whitenblack[m%2-i%2]+";}")
            button.setStyleSheet(
                "QLabel {{background-color:rgb({},{},{}); border: 3px double;}}"
                    .format(str(whitenblack[m % 2 - i % 2][0]), str(whitenblack[m % 2 - i % 2][1]),
                            str(whitenblack[m % 2 - i % 2][2]))
            )
            # button.connect(button, QtCore.SIGNAL("clicked()"), lambda string=str(i): print(string))
            button.setAcceptDrops(True)
            self.controller.notifyCaller.connect(button.getNotification)
            # button.connect(button, button.notifyCaller , button.getNotification)
            grid.addWidget(button, m + 1, k + 1)
            arr.append(button)
            k += 1
            if k == 8:
                m += 1
                k = 0
                self.files.append(arr)
                arr = []

        # random button
        # self.rand = QtGui.QPushButton("kek", self.chessb_frame)
        # self.rand.setPixmap(self.pieces[0][0])
        # self.rand.setStyleSheet("QLabel {background-color: red;}")
        # self.rand.move(50, 50)

        # create tmp label to move icons
        self.moving_icon = QtGui.QLabel("", self.chessb_frame)
        self.moving_icon.clear()
        self.moving_icon.move(0, 0)

        # show main window
        self.show()

        # disable chessb frame auto resize
        self.chessb_frame.setFixedSize(self.chessb_frame.size())

        # load pieces graphics:
        self.pieces = []
        # white:
        for i in range(2):
            arr = []
            for j in range(6):
                icon = QtGui.QPixmap("icons/{}.png".format(str(j + 1 + i * 10))).scaled(44, 44)
                arr.append(icon)
            self.pieces.append(arr)

        self.setChessboardPosition(basic_chessboard)

        # connect main thread with graphics moving signals
        self.controller.moveIcon.connect(self.performMove)
        self.controller.finishMovingIcon.connect(self.finishMoving)
        self.controller.startMovingIcon.connect(self.makeMove)

        #self.makeMove("a1h4")


    def startNewGame(self, initial_board):
        self.controller.bus.put("cv","startNG; ")
        if initial_board is None:
            self.setChessboardPosition(basic_chessboard)


    def setChessboardPosition(self, board_array):
        for row in range(8):
            for col in range(8):
                if board_array[row][col]:
                    # decode piece number into corresponding icon in pieces[]
                    code = board_array[row][col]
                    icon = self.pieces[int(code/10)][(code%10)-1]
                    #self.files.piece_color = int(code/10)+1
                    self.files[row][col].setPixmap(icon)
                else:
                    self.files[row][col].clear()


    def makeMove(self, string):
        if self.moving_icon.pixmap():
            print("ChessGUI: waiting for pixmap to be cleared")
            timer = QtCore.QTimer()
            timer.singleShot(500, lambda: self.controller.startMovingIcon.emit(string))
        else:
            moved_from_to = []
            for i in range(2):
                col = (ord(string[i*2]) - ord("a"))
                #col = int( str(chr(ord(string[i*2]) - ord("a")))[0])
                row = 7 - (int(string[(i*2)+1]) - 1)
                moved_from_to.append([row, col])

            filefrom = self.files[moved_from_to[0][0]][moved_from_to[0][1]]
            fileto = self.files[moved_from_to[1][0]][moved_from_to[1][1]]

            icon = filefrom.pixmap().copy()
            filefrom.clear()

            startpos = filefrom.pos()
            endpos = fileto.pos()

            #self.moving_icon = QtGui.QLabel("", self.chessb_frame)
            self.moving_icon.setPixmap(icon)

            mover = threading.Thread(target=self.animateMove,
                                     args=(moved_from_to[1][0],moved_from_to[1][1],startpos, endpos))
            mover.start()


    def performMove(self, newposx, newposy):
        #print("moving to", newposx, newposy)
        self.moving_icon.move(newposx, newposy)


    def finishMoving(self, row, col):
        icon = self.moving_icon.pixmap()
        self.files[row][col].setPixmap(icon)
        self.moving_icon.clear()
        self.moving_icon.move(0,0)


    def animateMove(self, endrow, endcol, startpos, endpos):
        iterations = 40
        lenx = endpos.x() - startpos.x()
        stepx = int(lenx / iterations)

        leny = endpos.y() - startpos.y()
        stepy = int(leny / iterations)

        for i in range(0, iterations):
            self.controller.moveIcon.emit(startpos.x() + i * stepx, startpos.y() + i * stepy)
            time.sleep(1/iterations)

        print("Finished moving")
        self.controller.finishMovingIcon.emit(endrow, endcol)

        #fileto.setPixmap(icon)
        #self.moving_icon = None


class SettingsTab(QtGui.QWidget):
    def __init__(self, controller):
        super(SettingsTab, self).__init__()
        self.controller = controller
        vbox = QtGui.QVBoxLayout()
        vbox.setAlignment(QtCore.Qt.AlignHCenter)
        self.setLayout(vbox)

        self.stockfish_group = QtGui.QGroupBox("Chess engine", self)
        self.stockfish_group.setStyleSheet(
            """QGroupBox {
                border: 1px solid gray;
                border-radius: 9px;
                margin-top: 0.5em;
            }

            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 3px 0 3px;
            }""")
        # self.chesscv_group = QtGui.QGroupBox("Image detection", parent)
        vbox.addWidget(self.stockfish_group)
        vbox.addStretch(1)

        grid = QtGui.QGridLayout()
        grid.setMargin(20)
        # grid.setAlignment(QtCore.Qt.AlignHCenter)
        # grid.setSizeConstraint(QtGui.QLayout.SetFixedSize)
        # grid.setAlignment(QtCore.Qt.AlignHCenter)
        # grid.setRowStretch(1, 0)
        # grid.setColumnStretch(1, 0)
        self.stockfish_group.setLayout(grid)

        labels = ["Player color", "Difficulty"]

        for i in range(len(labels)):
            label = QtGui.QLabel(labels[i], self)
            label.setFixedWidth(150)
            grid.addWidget(label, i, 0)

        self.color_combobox = QtGui.QComboBox(self)
        self.color_combobox.addItems(["white", "black"])
        self.color_combobox. currentIndexChanged.connect(
            lambda: self.controller.bus.put("cv","player_colour;" + self.color_combobox.currentText() ))
        grid.addWidget(self.color_combobox, 0, 1)
        self.color_combobox. currentIndexChanged.emit(0)

        self.diff_slider = SliderMore(self, self.controller, "Easy", "Hard",1, 20, 3)
        self.diff_slider.slider.sliderReleased.connect(
            lambda: self.controller.bus.put("cv","difficulty;" + self.diff_slider.value_label.text()))
        self.diff_slider.slider.sliderReleased.emit()

        grid.addWidget(self.diff_slider, 1, 1)


class ManualSteeringTab(QtGui.QWidget):
    def __init__(self, controller):
        super(ManualSteeringTab, self).__init__()
        self.controller = controller

        vbox = QtGui.QVBoxLayout()
        self.setLayout(vbox)

        self.tabwidget = QtGui.QTabWidget(self)
        self.tabwidget.setTabPosition(QtGui.QTabWidget.East)
        vbox.addWidget(self.tabwidget)

        # create button to open/close gripper
        self.gripper_button = QtGui.QPushButton("Gripper", self)
        self.gripper_button.clicked.connect(lambda: self.controller.bus.put("bt","gripperToggle"))
        vbox.addWidget(self.gripper_button)

        # create 3 tabs
        self.tab1 = SteeringControl(controller = self.controller,
                                    labels = [["X coordinate [mm]","Min","Max"],
                                              ["Y coordinate [mm]","Min","Max"],
                                              ["Z coordinate [mm]", "Min", "Max"],
                                              ["Orientation [\N{DEGREE SIGN}]", "Min", "Max"]],
                                    values = [[-300,300,0],
                                              [0,300,200],
                                              [0, 300, 100],
                                              [270,360,270]],
                                    steer_id = 0,
                                    )

        self.tab2 = SteeringControl(controller = self.controller,
                                    labels = [["Joint 1 (Base) [\N{DEGREE SIGN}]","Min","Max"],
                                              ["Joint 2 (Shoulder) [\N{DEGREE SIGN}]","Min","Max"],
                                              ["Joint 3 (Elbow) [\N{DEGREE SIGN}]", "Min", "Max"],
                                              ["Joint 4 (Pitch) [\N{DEGREE SIGN}]", "Min", "Max"],
                                              ["Joint 5 (Roll) [\N{DEGREE SIGN}]", "Min", "Max"]],
                                    values = [[-90,90,0],
                                              [0,180,90],
                                              [0, 180, 90],
                                              [0, 160, 0],
                                              [-90, 90, 0]],
                                    steer_id = 1,
                                    )

        self.tab3 = SteeringControl(controller = self.controller,
                                    labels = [["X","Min","Max"],
                                              ["Y","Min","Max"],
                                              ["Z", "Min", "Max"]],
                                    values = [[-30,30,0],
                                              [0,30,20],
                                              [0, 30, 15]],
                                    steer_id = 2,
                                    )

        # add tabs to tabwidget
        self.tabwidget.addTab(self.tab1, "Global coord")
        self.tabwidget.addTab(self.tab2, "Joint coord")
        #self.tabwidget.addTab(self.tab3, "Tool coord")


class SteeringControl(QtGui.QWidget):
    def __init__(self, controller, labels, values, steer_id):
        super(SteeringControl, self).__init__()
        self.controller = controller
        self.steer_id = steer_id
        #grid = QtGui.QGridLayout()
        #self.setLayout(grid)
        vbox = QtGui.QVBoxLayout()
        self.setLayout(vbox)

        self.tab_sliders = []
        for i in range(len(labels)):
            frame = QtGui.QGroupBox(labels[i][0], self)
            vbox.addWidget(frame)
            frame.setStyleSheet(
                """QGroupBox {
                    border: 1px solid gray;
                    border-radius: 9px;
                    margin-top: 0.5em;
                }

                QGroupBox::title {
                    subcontrol-origin: margin;
                    left: 10px;
                    padding: 0 3px 0 3px;
                }""")
            #label = QtGui.QLabel(labels[i][0], self)
            #grid.addWidget(label, i, 0)

            slid = SliderMore(frame, self.controller,
                                labels[i][1], labels[i][2], values[i][0], values[i][1], values[i][2] )
            slid.slider.sliderReleased.connect(self.toPosition)
            vboxtmp = QtGui.QVBoxLayout()
            vboxtmp.addWidget(slid)
            frame.setLayout(vboxtmp)
            self.tab_sliders.append(slid)
            #grid.addWidget(slid, i, 1)


    def toPosition(self):
        #print(self.steer_id, value)
        values = [str(self.tab_sliders[i].slider.value()) for i in range(len(self.tab_sliders))]
        self.controller.bus.put("bt","moveTo;{};{}".format(str(self.steer_id), ",".join(values)))



def ChessGUI_launcher(rawbus, buswindex, moduleid):
    chessgui = ChessGUI(rawbus, buswindex, moduleid)
    chessgui.start()

