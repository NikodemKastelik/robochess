import subprocess

class Stockfisher():
    def __init__(self):
        try:
            self.stockfisher = subprocess.Popen(["stockfish"], stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                           stderr=subprocess.DEVNULL, bufsize=0)
            #self.moves = []

            #self.waitForString("Stockfish")
            #self.writeString("uci")
            #self.waitForString("uciok")

            self.resetGame()

            #self.writeString("setoption name Skill Level value " + str(difficulty))
            #self.writeString("ucinewgame")

            print("Started Stockfish")

            #if "white" in colour:
            #    self.takeMove()

        except:
            print("Stockfish chess engine not found")



    def waitForString(self, string):
        while True:
            line = self.stockfisher.stdout.readline().decode()
            if string in line:
                return line


    def writeString(self, string):
        self.stockfisher.stdin.write("isready\n".encode())
        self.waitForString("readyok")
        self.stockfisher.stdin.write((string+"\n").encode())


    #def respondToMove(self, moves):
    #    self.moves.append(" " + player_move)
    #    stockfish_move = self.takeMove()
    #    return stockfish_move

    def takeMove(self, moves):
        command = "position startpos moves " + " ".join(moves)
        print("Command for stockfish: ",command)
        self.writeString(command)
        self.writeString("go")
        readout = self.waitForString("bestmove")
        try:
            ponder_string_location = readout.index("ponder")
            stockfish_move = readout[len("bestmove "):ponder_string_location - 1]
        except:
            stockfish_move = readout[len("bestmove "):]
        #self.moves.append(stockfish_move)
        print("Computer plays", stockfish_move)
        return stockfish_move

    def terminate(self):
        self.writeString("quit")


    def setDifficulty(self, difficulty):
        self.writeString("setoption name Skill Level value " + str(difficulty))


    def resetGame(self):
        self.writeString("ucinewgame")