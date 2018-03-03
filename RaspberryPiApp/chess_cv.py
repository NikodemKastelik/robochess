import cv2
import sys
import numpy as np
import subprocess
import time
import threading
import multiprocessing
import queue

from chess_engine import Stockfisher
from chess_bus import CyclicBus

wR,wN,wB,wQ,wK,wP = 1,2,3,4,5,6
bR,bN,bB,bQ,bK,bP = 11,12,13,14,15,16
piece_type = ["X","R","N","B","Q","K","P"]

first_pass = True

this_img = None
this_array = np.zeros(shape=(8,8), dtype=int)

# previous_img = None
# previous_array = np.zeros(shape=(8,8), dtype=int)
# previous_array[0] = [bR, bN, bB, bQ, bK, bB, bN, bR]
# previous_array[1] = [bP, bP, bP, bP, bP, bP, bP, bP]
# previous_array[6] = [wP, wP, wP, wP, wP, wP, wP, wP]
# previous_array[7] = [wR, wN, wB, wQ, wK, wB, wN, wR]

board = np.zeros(shape=(8,8), dtype=int)
board[0] = [bR, bN, bB, bQ, bK, bB, bN, bR]
board[1] = [bP, bP, bP, bP, bP, bP, bP, bP]
board[6] = [wP, wP, wP, wP, wP, wP, wP, wP]
board[7] = [wR, wN, wB, wQ, wK, wB, wN, wR]

DEBUG = 0

WIDTH=2592
HEIGHT=1944

DUPLICATES_ERROR = 3
RECTANGILITY_ERROR = 10
CONTOUR_MIN_LEN = 85
CONTOUR_MIN_AREA = 1000
CONTOUR_MAX_AREA = 30000
CONTOUR_EPSILON = 0.12
THRESH1 = 41
THRESH2 = 10

PLAYER_MOVE_ERR = 127
MOVE_RECOG_ERR =130
CHESSB_NOT_FOUND = 128
PIECES_NOT_FOUND = 129
SUCCESS = 126

# previous_num_of_white_pieces = 16
# previous_num_of_black_pieces = 16
# previous_num_of_pieces = [0]*2
# previous_num_of_pieces[0] = lambda: previous_num_of_white_pieces
# previous_num_of_pieces[1] = lambda: previous_num_of_black_pieces


class ChessCV():
    def __init__(self, rawbus, buswindex, moduleid):
        self.chess_engine = Stockfisher()

        self.moduleid = moduleid
        self.bus = CyclicBus(rawbus, buswindex, moduleid)

        regs_strings = ["player_colour", "difficulty"]
        self.regs = {}
        for str in regs_strings:
            self.regs[str] = None

        self.previous_num_of_white_pieces = 0
        self.previous_num_of_black_pieces = 0
        self.previous_num_of_pieces = [0] * 2
        self.previous_num_of_pieces[0] = lambda: self.previous_num_of_white_pieces
        self.previous_num_of_pieces[1] = lambda: self.previous_num_of_black_pieces

        self.possible_capture = False

        self.first_pass = True

        self.last_error = ""

        self.moves = []

        self.board = self.initChessboard(startpos=" ")

        #self.commWorker = self.startThread(self.commWorkerLoop, read_queue)


    def startThread(self, function, *arguments):
        arg_tuple = ()
        for i in range(len(arguments)):
            arg_tuple += (arguments[i],)
        thread = threading.Thread(target=function, args=arg_tuple)
        thread.setDaemon(True)
        thread.start()
        return thread


    def commWorkerLoop(self):
        while True:
            msg = self.bus.get()
            if msg:
                print("Chess_CV got: ", msg)
                msg = msg.split(";")
                if msg[CyclicBus.ACTION_INDX] == "makeTurn":
                    self.makeTurn()
                elif msg[CyclicBus.ACTION_INDX] == "startNG":
                    self.chess_engine.setDifficulty(self.regs["difficulty"])
                    self.chess_engine.resetGame()
                    self.board = self.initChessboard(msg[CyclicBus.ARG1_INDX])
                    self.makeTurn()
                elif msg[CyclicBus.ACTION_INDX] == "player_colour":
                    self.regs["player_colour"] = msg[CyclicBus.ARG1_INDX]
                elif msg[CyclicBus.ACTION_INDX] == "difficulty":
                    self.regs["difficulty"] = msg[CyclicBus.ARG1_INDX]
                elif msg[CyclicBus.ACTION_INDX] == "terminate":
                    self.chess_engine.terminate()
                    break
                else:
                    print("Chess_CV: no such action: ", msg[CyclicBus.ACTION_INDX])
            time.sleep(0.01)


    def makeTurn(self, *args):
        #print(self.regs)
        #while True:
        #print(self.first_pass)
        self.bus.put("ui","changeStatus;Taking photo...")
        img = self.takePhoto()

        if not self.first_pass:
            self.bus.put("ui", "changeStatus;Recognizing player move...")
        else:
            self.bus.put("ui", "changeStatus;Checking starting position...")

        status, mask, tile_positions = self.recognizeCurrentChessboard(img)


        print("This mask:\n", np.matrix(mask), "\n")

        if DEBUG == 1:
            imS = cv2.resize(img, (640,480))
            cv2.imshow('points', imS)
            cv2.waitKey(0)
            cv2.destroyAllWindows()

        #break

        if status != SUCCESS:
            #continue
            self.bus.put("ui", "changeStatus;ERROR: " + self.last_error)
            return

        this_img = img

        if not self.first_pass:
            tmp_board = np.copy(self.board)
            status, tmp_board, player_move = self.parsePlayerMove(tmp_board, mask)

            if status != SUCCESS:
                #continue
                self.bus.put("ui","changeStatus;ERROR: " + self.last_error)
                return

            self.bus.put("ui", "changeStatus;Player moved " + player_move)
            self.visualiseMove(player_move)
            self.moves.append(player_move)

            self.board = np.copy(tmp_board)

            computer_move = self.chess_engine.takeMove(self.moves)

            status, self.board = self.parseComputerMove(self.board, computer_move)

            if status != SUCCESS:
                self.bus.put("ui", "changeStatus;ERROR: " + self.last_error)
                self.visualiseMove(self.reverseMove(player_move))
                del self.moves[-1]
                return

            self.moveArm(tile_positions, computer_move)

            self.bus.put("ui", "changeStatus;Computer moved " + computer_move)
            self.visualiseMove(computer_move)
            self.moves.append(computer_move)
            #print("This move:\n", np.matrix(this_array), "\n")
            #print("Previous move:\n", np.matrix(previous_array), "\n")

            #if DEBUG == 1:
            #    cv2.imshow("this", this_img)
            #    cv2.imshow("previous", previous_img)
            #    cv2.waitKey(0)

            #for i in range(0, 8):
            #    for j in range(0, 8):
            #        previous_array[i][j] = this_array[i][j]

        else:
            # check if starting position is correct
            #self.bus.put(("ui", "changeStatus", "Checking "))
            self.first_pass = False
            for i in range(0, 8):
                for j in range(0, 8):
                    if (mask[i][j] != 0 and self.board[i][j] == 0) or (mask[i][j] == 0 and self.board[i][j] != 0):
                        self.last_error = "Wrong starting position"
                        print(self.last_error)
                        self.bus.put("ui", "changeStatus;ERROR: " + self.last_error)
                        self.first_pass = True
                        return

            #if chess engine is playing white let it make first move
            if self.regs["player_colour"] == "black": #chess_engine.colour == "white":
                computer_move = self.chess_engine.takeMove(self.moves)
                self.visualiseMove(computer_move)
                self.moves.append(computer_move)
                status, self.board = self.parseComputerMove(self.board, computer_move)
                self.moveArm(tile_positions, computer_move)
                #for i in range(0, 8):
                #    for j in range(0, 8):
                #        previous_array[i][j] = this_array[i][j]

        previous_img = this_img
        # cv2.imshow('original', img)
        # cv2.imshow('warped', img2)
        # cv2.waitKey(0)
        # cv2.destroyAllWindows()

        print("Turn complete\n")
        self.bus.put("ui", "changeStatus;Turn complete")
        #break


    def visualiseMove(self, move):
        self.bus.put("ui","movePiece;" + move)


    def moveArm(self,tile_positions, move):
        moved_from, moved_to, tile_pos_index = [0] * 2, [0] * 2, [0] * 2
        moved_from[0] = 7 - int(ord(move[1]) - ord('1'))
        moved_from[1] = int(ord(move[0]) - ord('a'))
        moved_to[0] = 7 - int(ord(move[3]) - ord('1'))
        moved_to[1] = int(ord(move[2]) - ord('a'))
        tile_pos_index[0] = moved_from[0] * 8 + moved_from[1]
        tile_pos_index[1] = moved_to[0] * 8 + moved_to[1]
        self.bus.put("bt", "movePieceFromTo;{},{};{},{}".format(
            str(tile_positions[0][tile_pos_index[0]]),
            str(tile_positions[1][tile_pos_index[0]]),
            str(tile_positions[0][tile_pos_index[1]]),
            str(tile_positions[1][tile_pos_index[1]])))


    def initChessboard(self, startpos):
        # moves format: "a2a4 d7d5 g8f6" etc..
        self.previous_num_of_white_pieces = 16
        self.previous_num_of_black_pieces = 16
        self.moves = []
        self.first_pass = True
        self.possible_capture = False

        board = np.zeros(shape=(8, 8), dtype=int)
        board[0] = [bR, bN, bB, bQ, bK, bB, bN, bR]
        board[1] = [bP, bP, bP, bP, bP, bP, bP, bP]
        board[6] = [wP, wP, wP, wP, wP, wP, wP, wP]
        board[7] = [wR, wN, wB, wQ, wK, wB, wN, wR]
        if startpos != " ":
            individual_moves = startpos.split(" ")
            for i in range(0, len(individual_moves)):
                board = self.parseComputerMove(board, individual_moves[i])  # WONT WORK NOW DUE TO PIECE CHECKING INCLUDED
            print("Initiated starting position. Current board:\n", np.matrix(board), "\n")
        return board


    def parsePlayerMove(self, previous_board, current_board_mask):
        #global previous_num_of_black_pieces
        #global previous_num_of_white_pieces
        #global possible_capture
        moved_to = None
        moved_from = None
        capture_confirmed = False
        this_board = np.zeros(shape=(8, 8), dtype=int)

        for i in range(0, 8):
            for j in range(0, 8):
                if current_board_mask[i][j]:
                    #  if there is a piece on i, j field
                    if previous_board[i][j] != 0:
                        # there was something here - now check if it was capture
                        piece_to_check = previous_board[i][j]
                        if (piece_to_check > 10 and current_board_mask[i][j] == 1) or \
                            (piece_to_check < 10 and current_board_mask[i][j] == 11):
                            # was A colour move but previously there was B colour piece - it is capture
                            if self.possible_capture:  # make sure camera found out that too
                                self.possible_capture = False
                            else:
                                print("Error recognizing move: Camera did not found possible capture")
                            if self.regs["player_colour"] == "white":  # if player is white then black piece was captured...
                                self.previous_num_of_black_pieces -= 1
                            else:  # else a white piece was captured
                                self.previous_num_of_white_pieces -= 1

                            print("Capture occured!")

                            if moved_to is None:
                                moved_to = [i, j]
                            else:
                                print("Error recognizing move: 'Moved to' found second time in capture recogition!")
                                return MOVE_RECOG_ERR, None, None
                            #else:
                            #    print("Error recognizing move: Camera did not found possible capture")

                        else:
                            this_board[i][j] = previous_board[i][j]
                    else:
                        if moved_to is None:
                            moved_to = [i, j]
                        else:
                            print("Error recognizing move: 'Moved to' found second time!")
                            return MOVE_RECOG_ERR, None, None
                else:
                    #  if no piece on i,j field
                    this_board[i][j] = 0
                    if previous_board[i][j] != 0:
                        if moved_from is None:
                            moved_from = [i, j]
                        else:
                            print("Error recognizing move: 'Moved from' found second time!")
                            return MOVE_RECOG_ERR, None, None

        if moved_to is None and moved_from is None:
            self.last_error = "Player move not found. Did you move any piece?"
            print("Player move not found. Did you move any piece?")
            return PLAYER_MOVE_ERR, None, None
        else:
            try:
                this_board[moved_to[0]][moved_to[1]] = previous_board[moved_from[0]][moved_from[1]]


                player_move = "{}{}{}{}".format(chr((moved_from[1]) + ord('a')),
                                                chr((7 - moved_from[0]) + ord('1')),
                                                chr((moved_to[1]) + ord('a')),
                                                chr((7 - moved_to[0]) + ord('1')))

                print("Player move: ", player_move)
                print("Previous board:\n", np.matrix(previous_board), "\n")
                print("This board:\n", np.matrix(this_board), "\n")

                return SUCCESS ,this_board, player_move
            except:
                self.last_error = "Could not recognise move"
                print("Error recognizing move")
                return MOVE_RECOG_ERR, None, None


    def parseComputerMove(self, previous_board, computer_move):
        # first check if computer has played his piece. If he has chosen player piece to play it means
        # that player move was not recognized -> it is not compilant with rules of chess game. It has to be undone.
        moved_from, moved_to = [0] * 2, [0] * 2
        moved_from[0] = 7 - int(ord(computer_move[1]) - ord('1'))
        moved_from[1] = int(ord(computer_move[0]) - ord('a'))

        # if self.regs["player_colour"] == "white" and previous_board[moved_from[0]][moved_from[1]] < 10 or \
        #     self.regs["player_colour"] == "black" and previous_board[moved_from[0]][moved_from[1]] > 10:
        #         self.last_error = "Player move is illegal"
        #         return PLAYER_MOVE_ERR, None

        print(">"+computer_move+"<")
        current_board = np.zeros(shape=(8,8))

        for i in range(0, 8):
            for j in range(0, 8):
                current_board[i][j] = previous_board[i][j]

        if len(computer_move) == 4:  # standard move
            #moved_from, moved_to = [0]*2, [0]*2
            #moved_from[0] = 7-int( ord(computer_move[1]) - ord('1'))
            #moved_from[1] = int( ord(computer_move[0]) - ord('a'))

            moved_to[0] = 7-int(ord(computer_move[3]) - ord('1'))
            moved_to[1] = int(ord(computer_move[2]) - ord('a'))

            # check if it capture:
            current_board[moved_to[0]][moved_to[1]] = previous_board[moved_from[0]][moved_from[1]]

            if previous_board[moved_to[0]][moved_to[1]]:  # did that field contained piece?
                print("Computer captures!")
                if previous_board[moved_to[0]][moved_to[1]] < 10 and self.regs["player_colour"] == "white": # means chess_engine_colour == "black":
                    #global previous_num_of_white_pieces
                    self.previous_num_of_white_pieces -= 1
                elif previous_board[moved_to[0]][moved_to[1]] > 10 and self.regs["player_colour"] == "black":#chess_engine_colour == "white":
                    #global previous_num_of_black_pieces
                    self.previous_num_of_black_pieces -= 1
                else:
                    print("Not supported capture move")

            current_board[moved_from[0]][moved_from[1]] = 0
        else:
            print("Error parsing computer move: Not yet supported")

        print("Previous board:\n", np.matrix(previous_board), "\n")
        print("This board:\n", np.matrix(current_board), "\n")

        return SUCCESS, current_board


    def findPointDuplicates(self, array):
        array.sort()
        duplicates = []
        i = 0
        while i < len(array) - 1:
            j = i
            area = array[j][3]
            while j<len(array)-1 and abs(array[j][0] - array[j + 1][0]) < DUPLICATES_ERROR and \
                            abs(array[j][1] - array[j + 1][1]) < DUPLICATES_ERROR:
                #print([array[i][0], array[i][1]])
                area += array[j+1][3]
                j += 1

            if j != i:
                if CONTOUR_MAX_AREA > area > CONTOUR_MIN_AREA:
                    duplicates.append([array[i][0], array[i][1], array[j][2]])
                    if DEBUG == 1:
                        color = (255 * (array[j][2] - 3)) ^ 255
                        #cv2.drawMarker(img, (array[i][0], array[i][1]), (color, color, color), cv2.MARKER_CROSS, 60, 20)
                        #cv2.circle(img, (array[i][0], array[i][1]), 20, (color, color, color), -1)
                #cv2.circle(image, (array[i][0], array[i][1]), 2, (0, 0, 255), -1)
                i = j
            i+=1

        return duplicates


    def detectMarkers(self, contours, contour_len, polygons_to_find):
        markers = []
        for c in contours:
            peri = cv2.arcLength(c, True)
            if peri > contour_len:
                approx = cv2.approxPolyDP(c, CONTOUR_EPSILON * peri, True)
                num_of_corners = len(approx)
                for i in range(0, len(polygons_to_find)):
                    if num_of_corners == polygons_to_find[i]:
                        M = cv2.moments(c)
                        try:
                            cX = int(M["m10"] / M["m00"])
                        except:
                            cX = 0
                        try:
                            cY = int(M["m01"] / M["m00"])
                        except:
                            cY = 0
                        area = cv2.contourArea(c, False)
                        #cv2.drawContours(image, [approx], -1, (0, 255, 0), 2)
                        if area > 0:
                            markers.append([cX, cY, num_of_corners, area])
                            #cv2.drawContours(image2, [approx], -1, (0, 255, 0), 1)

        return markers


    def recognizeCurrentChessboard(self, image):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        edges = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, THRESH1, THRESH2)
        im2, contours, hierarchy = cv2.findContours(edges, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        markers = self.detectMarkers(contours, CONTOUR_MIN_LEN, [3, 4])

        if DEBUG == 1:
            imS = cv2.resize(edges, (640,480))
            cv2.imshow('threshold', imS)

        possible_corners = self.findPointDuplicates(markers)

        # look for boundary markers
        corners = [0]*4

        distance_sum = []
        [distance_sum.append(possible_corners[i][0] + possible_corners[i][1]) for i in range(0, len(possible_corners))]

        min_index = distance_sum.index(min(distance_sum))
        min_x = possible_corners[min_index][0]
        min_y = possible_corners[min_index][1]
        corners[0] = ([min_x, min_y, possible_corners[min_index][2]])

        max_index = distance_sum.index(max(distance_sum))
        max_x = possible_corners[max_index][0]
        max_y = possible_corners[max_index][1]
        corners[2] = ([max_x, max_y, possible_corners[max_index][2]])

        center = [int((max_x + min_x) / 2), int((max_y + min_y) / 2)]

        left_index, right_index = 0, 0
        last_left_sum, last_right_sum = 0, 0
        for i in range(0, len(possible_corners)):
            this_x = possible_corners[i][0]
            this_y = possible_corners[i][1]
            if (this_x != max_x or this_y != max_y) and (this_x != min_x or this_y != min_y):
                summ = (this_x - center[0]) + (center[1] - this_y)
                if summ < 0 and summ < last_left_sum:
                    last_left_sum = summ
                    left_index = i
                elif summ > 0 and summ > last_right_sum:
                    last_right_sum = summ
                    right_index = i

        corners[3] = ([possible_corners[left_index][0], possible_corners[left_index][1], possible_corners[left_index][2]])
        corners[1] = ([possible_corners[right_index][0], possible_corners[right_index][1], possible_corners[right_index][2]])

        print("Chessboard corners: ", corners)

        num_of_rectangles = 0
        num_of_triangles = 0
        for i in range(0, 4):
            if corners[i][2] == 4:
                num_of_rectangles += 1
            elif corners[i][2] == 3:
                num_of_triangles += 1
                main_index = i

        if num_of_rectangles != 3 or num_of_triangles != 1:
            self.last_error = ("Corner detection error. Found {} rectangles and {} triangles".
                               format(str(num_of_rectangles),str(num_of_triangles)))
            print(self.last_error)
            return CHESSB_NOT_FOUND, None, None

        rect = np.zeros((4, 2), dtype="float32")
        rect[0][0] = corners[main_index][0]
        rect[0][1] = corners[main_index][1]
        index = main_index
        for i in range(1, 4):
            index += 1
            if index == 4:  # wrap quadrant value
                index = 0
            rect[i][0] = corners[index][0]
            rect[i][1] = corners[index][1]

        # check rectangility of figure constisted of corners:

        x1 = rect[0][0]
        y1 = rect[0][1]

        v1 = [ rect[0][0]-rect[1][0], rect[0][1]-rect[1][1] ]
        for i in range(0, 4):
            v2 = [ rect[(i+2)%4][0] - rect[(i+1)%4][0], rect[(i+2)%4][1] - rect[(i+1)%4][1]]
            angle = np.rad2deg(np.arccos( np.vdot(v1,v2) / (np.linalg.norm(v1) * np.linalg.norm(v2))))
            if not (90 - RECTANGILITY_ERROR < angle < 90 + RECTANGILITY_ERROR):
                self.last_error = "Found corners do not form chessboard shape"
                print(self.last_error)
                return CHESSB_NOT_FOUND, None, None
            v1 = v2


        corner1x=rect[0][0]
        corner1y=rect[0][1]
        corner2x=rect[1][0]
        corner2y=rect[1][1]
        border_len = np.sqrt( (corner2y-corner1y)**2 + (corner2x-corner1x)**2 )
        tile_len = (border_len / 8 )- 1

        dst = np.array([
            [0, 0],
            [border_len - 1, 0],
            [border_len - 1, border_len - 1],
            [0, border_len - 1]], dtype="float32")

        # compute the perspective transform matrix and then apply it
        markers2wrap = np.array([possible_corners], dtype='float32')
        markers2wrap = markers2wrap[:,:,:-1]
        #markers = np.array([markers])
        M = cv2.getPerspectiveTransform(rect, dst)
        warped_markers = cv2.perspectiveTransform(markers2wrap, M)

        warped_markers=warped_markers[0]

        pieces = [0]*2
        mask = np.zeros((8, 8))
        for i in range(0, len(warped_markers)):
            x = warped_markers[i][0] - 1
            y = warped_markers[i][1] - 1
            piece = possible_corners[i][2]-3
            #cv2.circle(img, (int(x), int(y)), 20, (0, 255, 0), -1)
            if x > 0 and y > 0:
                x_tile = int(x / tile_len)
                y_tile = int(y / tile_len)
                try:
                    mask[y_tile][x_tile] = 1 + 10 * (piece)
                    if possible_corners[i][2] == 3:
                        pieces[0] += 1
                    else:
                        pieces[1] += 1
                except:
                    pass

        #global possible_capture
        for i in range(0 ,2):
            if pieces[i] == self.previous_num_of_pieces[i]():  # found previous number of pieces? - just simple move
                self.possible_capture = False
            elif pieces[i] == self.previous_num_of_pieces[i]() - 1:  # found previous number of pieces - 1? possibly capture occured
                self.possible_capture = True
            else:
                self.last_error ="Error finding {}-colour pieces. Found: {}. Expected: {}".\
                                     format(str(chr( ord("W") - i*21)), str(pieces[i]), str(self.previous_num_of_pieces[i]()))
                print(self.last_error)
                return PIECES_NOT_FOUND, mask, None

        # calculate pieces real position

        pxtomm_scale = 230 / border_len

        # http://mathforum.org/mathimages/index.php/Transformation_Matrix

        v1 = [(rect[1][0] - rect[0][0]), (rect[1][1] - rect[0][1])]
        v2 = [(rect[2][0] - rect[3][0]), (rect[2][1] - rect[3][1])]
        angle1 = (np.arctan2(-v1[1], v1[0]) + np.arctan2(-v2[1], v2[0])) / 2

        v1 = [(rect[2][0] - rect[1][0]), (rect[2][1] - rect[1][1])]
        v2 = [(rect[3][0] - rect[0][0]), (rect[3][1] - rect[0][1])]
        angle2 = (np.arctan2(-v1[1], v1[0]) + np.arctan2(-v2[1], v2[0])) / 2

        angle = (angle1 + angle2) / 2 + (3.14 / 4)

        # rotmat = np.array([
        #     [np.cos(angle), -np.sin(angle)],
        #     [np.sin(angle), np.cos(angle)]])

        rotmat = np.array([
            [np.cos(angle), -np.sin(angle)],
            [np.sin(angle), np.cos(angle)]])

        a1 = (rect[2][1] - rect[0][1]) / (rect[2][0] - rect[0][0])
        b1 = rect[0][1] - rect[0][0] * a1

        a2 = (rect[3][1] - rect[1][1]) / (rect[3][0] - rect[1][0])
        b2 = rect[1][1] - rect[1][0] * a2

        chessb_center_x = (b2 - b1) / (a1 - a2)
        chessb_center_y = a1 * chessb_center_x + b1

        print("Chessb center: ", chessb_center_x, chessb_center_y)

        campos_x = 0
        campos_y = 250

        photo_center_x = WIDTH / 2
        photo_center_y = HEIGHT / 2

        vx = (chessb_center_x - photo_center_x) * pxtomm_scale
        vy = (chessb_center_y - photo_center_y) * pxtomm_scale

        tile_real = 25
        tile_pos = np.zeros(shape=(2, 64))

        for i in range(8):
            for j in range(8):
                tile_pos[0][i * 8 + j] = tile_real / 2 + j * tile_real - 100
                tile_pos[1][i * 8 + j] = tile_real / 2 + (7 - i) * tile_real - 100
        #for i in range(64):
        #    tile_pos[1][i] = (tile_real / 2 + int(i / 8) * tile_real) - 100  # j==0 => x,  j==1 -> y
        #    tile_pos[0][i] = (tile_real / 2 + i % 8 * tile_real) - 100

        # angle1 = (angles[0]+(angles[2]-3.14))/2
        # angle2 = (angles[1] + (angles[3] - 3.14)) / 2
        # print(np.rad2deg(angles))
        print(np.rad2deg(angle))

        tile_pos = np.dot(rotmat, tile_pos)

        tile_pos[0] += campos_x + vx
        tile_pos[1] += campos_y + vy

        # for i in range(len(tile_pos[0])):
        #     x = tile_pos[0][i]
        #     y = tile_pos[1][i]
        #     if i == 63:
        #         cv2.drawMarker(img, (int(x), int(y)), (0, 0, 255), cv2.MARKER_DIAMOND, 5, 5)
        #     else:
        #         cv2.drawMarker(img, (int(x), int(y)), (255, 0, 0), cv2.MARKER_DIAMOND, 5, 5)

        return SUCCESS, mask, tile_pos


    def takePhoto(self):
        #photographer = subprocess.Popen(["raspistill", "-t", "1000","-co", "100", "-o", "img.jpg"])
        #photographer.wait()
        image_taken = cv2.imread("img4.jpg")
        return image_taken


    def reverseMove(self, move):
        return move[2:4] + move[0:2]


#player_colour = "white"
#chess_engine_colour = "black"


# while True:
#
#     img = takePhoto()
#     status, mask = recognizeCurrentChessboard(img)
#
#     print("This mask:\n", np.matrix(mask), "\n")
#
#     if DEBUG == 1:
#         imS = cv2.resize(img, (640,480))
#         cv2.imshow('points', imS)
#         cv2.waitKey(0)
#         cv2.destroyAllWindows()
#
#     #break
#
#     if status != SUCCESS:
#         continue
#
#     this_img = img
#
#     if not first_pass:
#         tmp_board = np.copy(board)
#         status, tmp_board, player_move = parsePlayerMove(tmp_board, mask)
#
#         if status != SUCCESS:
#             continue
#
#         board = np.copy(tmp_board)
#
#         computer_move = chess_engine.respondToMove(player_move)
#         board = parseComputerMove(board, computer_move)
#
#         #print("This move:\n", np.matrix(this_array), "\n")
#         #print("Previous move:\n", np.matrix(previous_array), "\n")
#
#
#         #if DEBUG == 1:
#         #    cv2.imshow("this", this_img)
#         #    cv2.imshow("previous", previous_img)
#         #    cv2.waitKey(0)
#
#         #for i in range(0, 8):
#         #    for j in range(0, 8):
#         #        previous_array[i][j] = this_array[i][j]
#
#     else:
#         # check if starting position is correct
#         first_pass = False
#         for i in range(0, 8):
#             for j in range(0, 8):
#                 if (mask[i][j] != 0 and board[i][j] == 0) or (mask[i][j] == 0 and board[i][j] != 0):
#                     print("Wrong starting position")
#                     exit(0)
#
#         #if chess engine is playing white let it make first move
#         if chess_engine.colour == "white":
#             computer_move = chess_engine.takeMove()
#             board = parseComputerMove(board, computer_move)
#             #for i in range(0, 8):
#             #    for j in range(0, 8):
#             #        previous_array[i][j] = this_array[i][j]
#
#     previous_img = this_img
#     # cv2.imshow('original', img)
#     # cv2.imshow('warped', img2)
#     # cv2.waitKey(0)
#     # cv2.destroyAllWindows()
#
#     input("Take move!\n")

def ChessCV_launcher(rawbus, buswindex, moduleid):
    cv = ChessCV(rawbus, buswindex, moduleid)
    cv.commWorkerLoop()