import multiprocessing
import threading
import subprocess
import queue
import time
import os
import fcntl
import bluetooth

from chess_bus import CyclicBus

gettime = lambda: int(round(time.time() * 1000))

class ChessBT():

    def __init__(self, rawbus, buswindex, moduleid):
        self.devaddr = "20:17:01:10:26:87"
        self.devpath = "/dev/myrfcomm0"

        self.moduleid = moduleid
        self.bus = CyclicBus(rawbus, buswindex, moduleid)

        self.dev = None

        self.stop_threads = False
        self.connector = None
        #self.dev_mutex = threading.Lock()


    def connectToDongle(self, devpath, devaddr):
        while not self.stop_threads:
            btsock = bluetooth.BluetoothSocket( bluetooth.RFCOMM )
            try:
                btsock.connect((devaddr, 1))
                btsock.setblocking(False)
                self.bus.put("ui","changeStatus;Connected to bluetooth device")
                self.dev = btsock
                self.connector = None
                break
            except:
                self.bus.put("ui", "changeStatus;Could not connect to Bluetooth device")


    def connectToDongle_OLD(self, devpath, devaddr):
        while True:
            #self.bus.put(("ui", "changeStatus", "Trying to connect to Bluetooth device"))  # "Trying to connect to Bluetooth device"
            connecter = subprocess.Popen(["rfcomm", "-r", "connect", "hci0", devaddr, "1"],
                                         stdout=subprocess.PIPE, stderr=subprocess.STDOUT, bufsize=0)
            time1 = gettime()
            timeout = 7000

            timed_out = False
            while not os.path.exists(devpath):
                if gettime() - time1 > timeout:
                    print("Chess_BT: timed out waiting for connection")
                    self.bus.put(("ui","changeStatus","Could not connect to Bluetooth device"))
                    connecter.terminate()
                    timed_out = True
                    #time.sleep(1)
                    break


            if not timed_out:
                #self.dev_mutex.acquire()
                btdev = os.open(devpath, os.O_RDWR)
                flags = fcntl.fcntl(btdev, fcntl.F_GETFL)  # get current p.stdout flags
                fcntl.fcntl(btdev, fcntl.F_SETFL, flags | os.O_NONBLOCK)
                self.dev = btdev
                self.bus.put(("ui","changeStatus","Connected to bluetooth device"))
                break
                #return btdev


    # def bluetoothReader(self):
    #     while True:
    #         self.dev_mutex.acquire()
    #         self.dev = self.connectToDongle(self.devpath, self.devaddr)
    #         self.dev_mutex.release()
    #         self.bus.put(("ui","Connected to bluetooth device"))
    #
    #         while True:


    def commWorker(self):
        #bt_connecting = False
        line = b''
        while not self.stop_threads:
            msg = self.bus.get()
            if msg:
                print("Chess_BT got: ", msg)
                msg = msg.split(";")
                action = msg[CyclicBus.ACTION_INDX]
                if self.dev is not None:
                    if action == "movePieceFromTo":
                        pass
                    elif action == "moveTo":
                        id = msg[CyclicBus.ARG1_INDX]
                        values=msg[CyclicBus.ARG2_INDX]
                        cmd = "mov{}=".format(id) + ",".join(values)
                        self.dev.send((cmd+"\n").encode())
                        #os.write(self.dev, (cmd+"\n").encode())
                    elif action == "gripperToggle":
                        self.dev.send(("gripper\n").encode())
                        #os.write(self.dev, ("gripper\n").encode())
                if action == "terminate":
                    self.stop_threads = True
                    if self.connector:
                        self.connector.join()
                    break


            if self.dev is None:
                if self.connector is None:
                    #bt_connecting = True
                    self.connector = self.startThread(self.connectToDongle, False, self.devpath, self.devaddr)
            else:
                try:
                    char = self.dev.recv(1)
                    if char != b'\n':
                        line += char
                    else:
                        print("Chess_BT received from Bluetooth: ", line.decode())
                        line = b''
                except:
                    pass
            time.sleep(0.02)


    def startThread(self, function, ifdaemon, *arguments):
        arg_tuple = ()
        for i in range(len(arguments)):
            arg_tuple += (arguments[i],)
        thread = threading.Thread(target=function, args=arg_tuple)
        thread.setDaemon(ifdaemon)
        thread.start()
        return thread


def ChessBT_launcher(rawbus, buswindex, moduleid):
    chess_bt = ChessBT(rawbus, buswindex, moduleid)
    chess_bt.commWorker()

