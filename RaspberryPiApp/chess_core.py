# RoboChess ver. 1.3
#
# Changelog:
# ver 1.3:
# - changed bus from queue-based to shared array-based

import multiprocessing
import threading
import queue
import os
import time
import subprocess

from chess_bt import ChessBT_launcher
from chess_gui import ChessGUI_launcher
from chess_cv import ChessCV_launcher
from chess_bus import busCreate, CyclicBus

class RoboChessCore():
    def __init__(self):

        # init bus
        rawbus, buswindex = busCreate()
        self.moduleid = "cr"

        # init gui
        self.gui = self.startProcess(ChessGUI_launcher, rawbus, buswindex, "ui")

        # init chess cv
        self.cv = self.startProcess(ChessCV_launcher, rawbus, buswindex, "cv")

        # init bt reader
        self.bt = self.startThread(ChessBT_launcher, False, rawbus, buswindex, "bt")

        self.bus = CyclicBus(rawbus, buswindex, self.moduleid)


    def commWorker(self):
        while True:
            msg = self.bus.get()
            if msg:
                print("Chess_Core got: ", msg)
                msg = msg.split(";")
                if msg[CyclicBus.ACTION_INDX] == "terminate":
                    self.bus.put("cv", "terminate")
                    self.bus.put("bt", "terminate")
                    self.cv.join()
                    self.bt.join()
                    break
            time.sleep(0.01)


    def startProcess(self, function, *arguments):
        arg_tuple = ()
        for i in range(len(arguments)):
            arg_tuple += (arguments[i],)

        process = multiprocessing.Process(target=function, args=arg_tuple)
        process.start()
        return process


    def startThread(self, function, ifdaemon, *arguments):
        arg_tuple = ()
        for i in range(len(arguments)):
            arg_tuple += (arguments[i],)
        thread = threading.Thread(target=function, args=arg_tuple)
        thread.setDaemon(ifdaemon)
        thread.start()
        return thread


if __name__ == '__main__':
    inzynierka = RoboChessCore()
    inzynierka.commWorker()