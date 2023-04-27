#!/usr/bin/env python

import chess_player.chessboard_echo as implementation
import chess_player.ChessEngine as ce
import chess as ch

import rospy
from std_msgs.msg import String



if __name__ == '__main__':
    newBoard= ch.Board()
    transmission = implementation.Echo(newBoard)
    bruh = transmission.startGame()
