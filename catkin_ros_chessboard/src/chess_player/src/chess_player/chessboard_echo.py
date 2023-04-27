#!/usr/bin/env python
#<description>Echo give insturction base on ChessEngine</description>
#<maintainer email="jcheng2020@my.fit.edu">Junfu Cheng</maintainer>
#<author email="jcheng2020@my.fit.edu">Junfu Cheng</author>

import rospy
from std_msgs.msg import String
import chess as ch
import chess_player.ChessEngine as ce

class Echo(object):

    def __init__(self, board=ch.Board):
        self.board=board
        '''ros initial'''
        self.value = ""
        self.whether_receiving_instruction = False
        rospy.init_node('chess_player')
        self.pub = rospy.Publisher('/instruction_from_chessEngine', String, queue_size=10)
        rospy.Subscriber('/interpretation_of_sensor_chessboard', String, self.callback)
        self.r = rospy.Rate(10)
        
    def callback(self, msg):
        self.value = msg.data
        self.whether_receiving_instruction = True

    #play human move
    def playHumanMove(self):
        try:
            print(self.board.legal_moves)
            print("uci legal moves:", end =" ")
            legal_moves = list(self.board.legal_moves)
            for legal_move in legal_moves:
            	print(legal_move.uci(), end =" ")
            print()
            print("""To undo your last move, type "undo".""")
            
            #get human move from ROS
            while not rospy.is_shutdown():
                if self.whether_receiving_instruction == True:
                    play = self.value
                    whetherMoveLegal = False   

                    	
                    for legal_move in legal_moves:
                    	if play == legal_move.uci():
                    		whetherMoveLegal = True
                    
                    if whetherMoveLegal == False and len(play) == 4 and play[0] >= 'a' and play[0] <= 'h' and play[1] >= '1' and play[1] <= '8' and play[2] >= 'a' and play[2] <= 'h' and play[3] >= '1' and play[3] <= '8':
                    	origin = play[0:2]
                    	target = play[2:4]
                    	instruction =  target + origin
                    	self.pub.publish(instruction)
                    self.whether_receiving_instruction = False
                    if whetherMoveLegal == True:
                    	print("legal input: " + play)
                    	break
                    else:
                    	print("illegal input: " + play)
                self.r.sleep()
            #get human move
            #play = input("Your move: ")
            if (play=="undo"):
                self.board.pop()
                self.board.pop()
                self.playHumanMove()
                return
            #self.board.push_san(play)
            self.board.push_uci(play)
        except:
            self.playHumanMove()

    #play engine move
    def playEngineMove(self, maxDepth, color):
        engine = ce.Engine(self.board, maxDepth, color)
        move = engine.getBestMove()
        print(self.board.san(move))
        san = self.board.san(move)
        print(self.board.uci(move))
        uci = self.board.uci(move)
        self.board.push(move)
        '''ros broadcast'''
        if san[1] == 'x':
        	instruction = "x" + san[2:4] + "i1" + uci
        	self.pub.publish(instruction)
        else:
        	self.pub.publish(uci)
        self.r.sleep()

    #start a game
    def startGame(self):
        #get human player's color
        color=None
        while(color!="b" and color!="w"):
            color = input("""Play as (type "b" or "w"): """)
        maxDepth=None
        while(isinstance(maxDepth, int)==False):
            maxDepth = int(input("""Choose depth: """))
        if color=="b":
            while (self.board.is_checkmate()==False):
                print("The engine is thinking...")
                self.playEngineMove(maxDepth, ch.WHITE)
                print(self.board)
                self.playHumanMove()
                print(self.board)
            print(self.board)
            print(self.board.outcome())    
        elif color=="w":
            while (self.board.is_checkmate()==False):
                print(self.board)
                self.playHumanMove()
                print(self.board)
                print("The engine is thinking...")
                self.playEngineMove(maxDepth, ch.BLACK)
            print(self.board)
            print(self.board.outcome())
        #reset the board
        self.board.reset
        #start another game
        self.startGame()


'''
class Echo(object):
    def __init__(self):
        self.value = ""
        self.permission_for_broadcast_instruction = False

        rospy.init_node('chess_player')

        self.pub = rospy.Publisher('/instruction_from_chessEngine', String, queue_size=10)
        rospy.Subscriber('/interpretation_of_sensor_chessboard', String, self.update_value)

    def update_value(self, msg):
        self.value = msg.data

    def run(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.permission_for_broadcast_instruction == True:
                self.pub.publish(self.value)
                self.permission_for_broadcast_instruction = False
            r.sleep()
'''
