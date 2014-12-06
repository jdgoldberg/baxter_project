#!/usr/bin/env python
import sys
import rospy
import moveit_commander
from moveit_msgs.msg import OrientationConstraint, Constraints
from geometry_msgs.msg import PoseStamped
from baxter_interface import gripper as baxter_gripper
from std_msgs.msg import String
import time
import connect4robot as ai

#rosrun rviz rviz
# roslaunch ar_track_alvar lefthand_track.launch
# rosrun baxter_interface joint_trajectory_action_server.py
# roslaunch baxter_moveit_config move_group.launch
# rosrun connect4 path_test.py

#ar_tag = "ar_marker_5"
ar_tag = "test1"
dx = -.035
dy = .0675
slotwidth = -0.035
x0 = 0.0
y0 = 0.0
z0 = 0.2
x1 = 0.0927 + .035
y1 = -0.0345 - .0675
z1 = 0.2
x2 = x1
y2 = y1
z2 = 0.09 #0.09
#pick up piece
x3 = x1
y3 = y1
z3 = 0.41
x4 = x1 + 0.01 - 0.035 #over slot 1
y4 = y1 + 0.155 + .0675
z4 = 0.39
#drop piece
x5 = x0
y5 = y0
z5 = z3

x = [x0,x1,x2,x3,x4,x5]
y = [y0,y1,y2,y3,y4,y5]
z = [z0,z1,z2,z3,z4,z5]
closegripper = 2
opengripper = 4

class game_play():
    def __init__(self):
        self.state = "INITIALIZING"
        moveit_commander.roscpp_initialize(sys.argv)

        #Start a node
        rospy.init_node('moveit_node')
        
        #Initialize subscriber
        self.sub = rospy.Subscriber("game_state", String, self.state_machine)
        
        #Initialize both arms
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.left_arm = moveit_commander.MoveGroupCommander('left_arm')
        self.right_arm = moveit_commander.MoveGroupCommander('right_arm')
        self.left_arm.set_planner_id('RRTConnectkConfigDefault')
        self.left_arm.set_planning_time(10)
        self.right_arm.set_planner_id('RRTConnectkConfigDefault')
        self.right_arm.set_planning_time(10)

        #Set up the grippers
        self.left_gripper = baxter_gripper.Gripper('left')
        self.right_gripper = baxter_gripper.Gripper('right')

        #Calibrate the gripper (other commands won't work unless you do this first)
        print('Calibrating...')
        #left_gripper.calibrate()
        self.right_gripper.calibrate()
        rospy.sleep(0.5)

        #Initialize class variables
        self.available_pieces = [0,1,2,3,4,5,6,7,8,9,13,14,15,16,20]

        self.state = "BEGIN"
    
    # Main run method for the ObjectTracker class
    def run(self):
        try:
            rospy.spin()
        except KeyboardInterrupt:
            cv2.destroyAllWindows()

    def state_machine(self, message):
        self.sub.unregister()
        print "in state machine"
        game_state = message.data
        if self.state == "INITIALIZING":
            print "initializing..."
        elif self.state == "BEGIN":
            #print "received board:",game_state
            for i in range(5,-1,-1):
                lineSoFar = ""
                for j in range(0,7):
                    lineSoFar += "|"
                    lineSoFar += game_state[i*7 + j]
                print "|" + lineSoFar + "||"
            valid_output = self.valid_board(game_state)
            if valid_output == "baxter":
                print "my turn!"
                self.baxter_move(game_state)
            elif valid_output == "human":
                print "waiting for human to move..."
            elif valid_output == "baxter wins":
                print "==GAME OVER=="
                print "I won, good game :)"
                self.victory_dance()
            elif valid_output == "human wins":
                print "==GAME OVER=="
                print "How did I lose?!? :("
            else:
                print "not valid"
        self.sub = rospy.Subscriber("game_state", String, self.state_machine)

    def valid_board(self, board):
        reds = 0
        yellows = 0
        for piece in board:
            if piece == "X":
                reds = reds + 1
            elif piece == "O":
                yellows = yellows + 1
        #print reds,yellows
        
        if reds - yellows == 0 or reds - yellows == 1:
            if ai.primitive(board):
                if reds > yellows:
                    return "human wins"
                return "baxter wins"
            if reds - yellows == 0:
                return "human"
            return "baxter"
        return "error"

    def baxter_move(self, board):
        next_piece = self.next_available_piece()
        if next_piece == "error":
            print "no more pieces"
            return
        print "next piece is:",next_piece

        for i in range(len(x)):
            print "Doing move " + str(i)
            goal = PoseStamped()
            goal.header.frame_id = ar_tag
            #x, y, and z position
            if i == 1 or i == 2 or i == 3:
                goal.pose.position.x = x[i] + dx * (next_piece % 7)
                goal.pose.position.y = y[i] + dy * (next_piece / 7)
                goal.pose.position.z = z[i]
            elif i == 4: # which slot to go to
                #slot_no = int(raw_input("Which slot? [1:7] "))
                moves = ai.board_to_response(board)
                print "---moves---"
                print moves
                print "-----------"
                nextMove = ai.best_move(moves)
                print nextMove
                print "-----------"
                #board = nextMove['board']
                slot_no = int(nextMove['move'])
                print "moving to slot " + str(slot_no)
                goal.pose.position.x = x[i] + slotwidth * slot_no
                goal.pose.position.y = y[i]
                goal.pose.position.z = z[i]
            else:
                goal.pose.position.x = x[i]
                goal.pose.position.y = y[i]
                goal.pose.position.z = z[i]
            #Orientation as a quaternion
            goal.pose.orientation.x = 0.0
            goal.pose.orientation.y = -1.0
            goal.pose.orientation.z = 0.0
            goal.pose.orientation.w = 0.0

            #Set the goal state to the pose you just defined
            #left_arm.set_pose_target(goal)
            self.right_arm.set_pose_target(goal)

            #Set the start state for the left arm
            #left_arm.set_start_state_to_current_state()
            self.right_arm.set_start_state_to_current_state()

            #Plan a path
            #left_plan = left_arm.plan()
            right_plan = self.right_arm.plan()
        
            #Execute the plan
            #raw_input('Press <Enter> to move the left arm to goal pose 1 (path constraints are never enforced during this motion): ')
            #left_arm.execute(left_plan)
            #raw_input("press enter to execute")
            self.right_arm.execute(right_plan)
            rospy.sleep(1.0)

            if i == closegripper:
                #left_gripper.close(block=True)
                self.right_gripper.close(block=True)
                rospy.sleep(0.5)
            elif i == opengripper:
                #left_gripper.open(block=True)
                self.right_gripper.open(block=True)
                rospy.sleep(1.0)
        rospy.sleep(10.0)


    def next_available_piece(self):
        if len(self.available_pieces) > 0:
            next_piece = self.available_pieces[0]
            self.available_pieces = self.available_pieces[1:]
            return next_piece
        return "error"

    def victory_dance(self):
        goal = PoseStamped()
        goal.header.frame_id = "base"
        goal.pose.position.x = .35
        goal.pose.position.y = -.74
        goal.pose.position.z = 1.2
        goal.pose.orientation.x = 0.393
        goal.pose.orientation.y = 0.101
        goal.pose.orientation.z = 0.238
        goal.pose.orientation.w = 0.882

        #Set the goal state to the pose you just defined
        self.right_arm.set_pose_target(goal)

        #Set the start state for the left arm
        self.right_arm.set_start_state_to_current_state()

        #Plan a path
        right_plan = self.right_arm.plan()
    
        #Execute the plan
        self.right_arm.execute(right_plan)


        goal = PoseStamped()
        goal.header.frame_id = "base"
        goal.pose.position.x = 0.901
        goal.pose.position.y = 0.202
        goal.pose.position.z = 0.971
        goal.pose.orientation.x = 0.227
        goal.pose.orientation.y = 0.428
        goal.pose.orientation.z = 0.317
        goal.pose.orientation.w = 0.815

        #Set the goal state to the pose you just defined
        self.left_arm.set_pose_target(goal)

        #Set the start state for the left arm
        self.left_arm.set_start_state_to_current_state()

        #Plan a path
        left_plan = self.left_arm.plan()
    
        #Execute the plan
        self.left_arm.execute(left_plan)
        self.left_gripper.calibrate()
        while True:
            self.right_gripper.close(block=True)
            self.right_gripper.open(block=True)
            self.left_gripper.close(block=True)
            self.left_gripper.open(block=True)


if __name__ == '__main__':
  node = game_play()
  node.run()
