# Chess bus ver 1.0
#
# @brief
# Implements message bus routines. Bus can be shared between processes. Note that bus is created only once, but has
# to be initialized in every process by using busInit routine.

import ctypes
import numpy as np
import multiprocessing

BUFSIZE1 = 20
BUFSIZE2 = 100

ID_1ST_CHAR = 0
ID_2ND_CHAR = 1
MSG_INDX = 2


def busCreate():
    # @brief
    # Routine used to create bus object, represented as c-style byte array.
    # @return bus - bus object
    # @return windex - pointer to next row to be filled with message. This get overflowed when reached BUFSIZE1 value to
    #  get cyclic buffer-like behaviur. Moreover, it is used to lock access to bus object
    bus = multiprocessing.Array(ctypes.c_byte, BUFSIZE1 * BUFSIZE2)
    windex = multiprocessing.Value(ctypes.c_short, lock=True)
    return bus, windex


class CyclicBus():
    # @brief
    # Class representing message bus.

    ACTION_INDX = 0
    ARG1_INDX = 1
    ARG2_INDX = 2
    ARG3_INDX = 3

    def __init__(self, bus, windex, processid):
        self.bus = self.init(bus)
        self.windex = windex
        self.rindex = 0
        self.processid = processid


    def init(self, bus2initialize):
        # @brief
        # Routine initializing bus object. Initialization creates two dimensional array from one dimensional array.
        # This has to be called in each process where bus object was passed.
        # @param bus2initialize - bus object to initialize
        # @return bus - initialized bus object
        bus = np.frombuffer(bus2initialize.get_obj(), dtype=ctypes.c_byte)
        bus = bus.reshape((BUFSIZE1, BUFSIZE2))
        return bus


    def put(self, receipentid, message):
        # @brief
        # Routine placing message in bus. All done under lock from windex, what assures thread and process safety.
        # @param message - message as string to be placed into bus
        # @param receipentid - id of process which should receive the message
        with self.windex.get_lock():
            tmpwindex = int(self.windex.value)
            self.bus[tmpwindex][ID_1ST_CHAR] = ord(receipentid[ID_1ST_CHAR])
            self.bus[tmpwindex][ID_2ND_CHAR] = ord(receipentid[ID_2ND_CHAR])
            i = MSG_INDX
            for character in message.encode():
               self.bus[tmpwindex][i] = character
               i += 1
            self.bus[tmpwindex][i] = 0
            tmpwindex += 1
            if tmpwindex == BUFSIZE1:
                tmpwindex = 0
            self.windex.value = tmpwindex


    def get(self):
        # @brief
        # Routine, which tries to get message from bus. Firstly, it compares if next message is addressed to this particular
        # process by checking receipent ID. If it does match, it gets the message. Otherwise the message is ignored, None
        # returned and message is waiting to be read by other process.
        # @return msg - message if ID match. Otherwise is None
        # @return rindex - incremented r(ead)index to try to get next message next time
        line = None
        msg = None

        if self.rindex != self.windex.value:
            #print(self.processid)
            with self.windex.get_lock():
                msg_id = chr((self.bus[self.rindex][ID_1ST_CHAR])) + chr((self.bus[self.rindex][ID_2ND_CHAR]))
                if msg_id == self.processid:
                    line = self.bus[self.rindex][MSG_INDX:]
                    self.bus[self.rindex][ID_1ST_CHAR] = 0
                    self.bus[self.rindex][ID_2ND_CHAR] = 0
            self.rindex += 1
            if self.rindex == BUFSIZE1:
                self.rindex = 0

            if line is not None:
                msg = []
                for byte in line:
                    if byte != 0:
                        msg.append(chr(byte))
                    else:
                        break
                        #msg = self.bus[self.rindex][MSG_INDX:].tostring().decode()
                        #msg = ''.join([c for c in msg if ord(c) > 31 or ord(c) == 9])
                msg = ''.join(msg)
        #         self.bus[self.rindex][ID_1ST_CHAR] = 0
        #         self.bus[self.rindex][ID_2ND_CHAR] = 0
        # self.rindex += 1
        # if self.rindex == BUFSIZE1:
        #     self.rindex = 0
        return msg

