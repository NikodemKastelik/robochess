# roboChess

Here you can find software controlling chess-playing articulated robot. It is composed of two applications - one running on STM32 microcontroller and second on Raspberry Pi. First one, written in C, is responsible for generating steering signals for robot actuators, taking position commands from RasPi as input to inverse kinematics equations. Second one, written in Python, acts as supervisor. It contains few modules - each one taking care of some part of whole system, splitted into several processes in order to utilize Raspi's quadcore CPU and to get around Python GIL. These tasks are: chessboard image processing (because robot recognize human moves by comparing camera snaphots), communication with STM32 motor driver, Qt-based user interface and chess engine i/o (Stockfish chess engine is used to act as virtual opponent).

Here you can see how this project evolved from this acrylic construction:

https://drive.google.com/open?id=1U5eVh4Rxq9fKQUv3RQS-O5vfSjTTu1Ak

into this full-fledged aluminium one:

https://drive.google.com/open?id=10SO0kZ9DUNW2njblFyZXr57TyfqdZTWr
