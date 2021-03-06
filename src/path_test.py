#!/usr/bin/env python
import sys
import rospy
import moveit_commander
from moveit_msgs.msg import OrientationConstraint, Constraints
from geometry_msgs.msg import PoseStamped
from baxter_interface import gripper as baxter_gripper
from std_msgs.msg import String
import time

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
z2 = 0.08 #0.09
#pick up piece
x3 = x1
y3 = y1
z3 = 0.408
x4 = x1 + 0.01 - 0.035 #over slot 1
y4 = y1 + 0.15 + .0675
z4 = 0.38
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
        #self.left_gripper = baxter_gripper.Gripper('left')
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
            print "received board:",game_state
            if self.valid_board(game_state) == "baxter":
                print "my turn!"
                self.baxter_move(game_state)
            elif self.valid_board(game_state) == "human":
                print "waiting for human to move..."
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
        if reds - yellows == 0:
            return "human"
        if reds - yellows == 1:
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
                slot_no = int(raw_input("Which slot? [1:7] "))
                goal.pose.position.x = x[i] + slotwidth * (slot_no - 1)
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
            raw_input("press enter to execute")
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

'''
def main():
    #Initialize moveit_commander
    moveit_commander.roscpp_initialize(sys.argv)

    #Start a node
    rospy.init_node('moveit_node')
    
    #Initialize subscriber
    rospy.Subscriber("game_state", String, state_machine)
    rospy.spin()
    return
    #Initialize both arms
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    left_arm = moveit_commander.MoveGroupCommander('left_arm')
    right_arm = moveit_commander.MoveGroupCommander('right_arm')
    left_arm.set_planner_id('RRTConnectkConfigDefault')
    left_arm.set_planning_time(10)
    right_arm.set_planner_id('RRTConnectkConfigDefault')
    right_arm.set_planning_time(10)

    #Set up the grippers
    left_gripper = baxter_gripper.Gripper('left')
    right_gripper = baxter_gripper.Gripper('right')

    #Calibrate the gripper (other commands won't work unless you do this first)
    print('Calibrating...')
    #left_gripper.calibrate()
    right_gripper.calibrate()
    #rospy.sleep(0.5)

    piece_no = 0
    for i in range(len(x)):
        print "Doing move " + str(i)
        goal = PoseStamped()
        goal.header.frame_id = ar_tag
        #x, y, and z position
        if i == 1: # which piece to grab
            piece_no = int(raw_input("Which piece? "))
        if i == 1 or i == 2 or i == 3:
            goal.pose.position.x = x[i] + dx * (piece_no % 7)
            goal.pose.position.y = y[i] + dy * (piece_no / 7)
            goal.pose.position.z = z[i]
        elif i == 4: # which slot to go to
            slot_no = int(raw_input("Which slot? [1:7] "))
            goal.pose.position.x = x[i] + slotwidth * (slot_no - 1)
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
        right_arm.set_pose_target(goal)

        #Set the start state for the left arm
        #left_arm.set_start_state_to_current_state()
        right_arm.set_start_state_to_current_state()

        #Plan a path
        #left_plan = left_arm.plan()
        right_plan = right_arm.plan()
	
        #Execute the plan
        #raw_input('Press <Enter> to move the left arm to goal pose 1 (path constraints are never enforced during this motion): ')
        #left_arm.execute(left_plan)
        right_arm.execute(right_plan)
        rospy.sleep(1.0)

        if i == closegripper:
            #left_gripper.close(block=True)
            right_gripper.close(block=True)
            rospy.sleep(0.5)
        elif i == opengripper:
            #left_gripper.open(block=True)
            right_gripper.open(block=True)
            rospy.sleep(1.0)
	
    return

    

    #First goal pose ------------------------------------------------------
    goal_1 = PoseStamped()
    goal_1.header.frame_id = ar_tag

    #x, y, and z position
    goal_1.pose.position.x = x1
    goal_1.pose.position.y = y1
    goal_1.pose.position.z = z1
    
    #Orientation as a quaternion
    goal_1.pose.orientation.x = 0.0
    goal_1.pose.orientation.y = -1.0
    goal_1.pose.orientation.z = 0.0
    goal_1.pose.orientation.w = 0.0

    #Set the goal state to the pose you just defined
    left_arm.set_pose_target(goal_1)

    #Set the start state for the left arm
    left_arm.set_start_state_to_current_state()

    #Plan a path
    left_plan = left_arm.plan()

    #Execute the plan
    #raw_input('Press <Enter> to move the left arm to goal pose 1 (path constraints are never enforced during this motion): ')
    left_arm.execute(left_plan)

    #Second goal pose -----------------------------------------------------
    rospy.sleep(2.0)
    goal_2 = PoseStamped()
    goal_2.header.frame_id = ar_tag

    #x, y, and z position
    goal_2.pose.position.x = x2
    goal_2.pose.position.y = y2
    goal_2.pose.position.z = z2
    
    #Orientation as a quaternion
    goal_2.pose.orientation.x = 0.0
    goal_2.pose.orientation.y = -1.0
    goal_2.pose.orientation.z = 0.0
    goal_2.pose.orientation.w = 0.0

    #Set the goal state to the pose you just defined
    left_arm.set_pose_target(goal_2)

    #Set the start state for the left arm
    left_arm.set_start_state_to_current_state()

    # #Create a path constraint for the arm
    # #UNCOMMENT TO ENABLE ORIENTATION CONSTRAINTS
    # orien_const = OrientationConstraint()
    # orien_const.link_name = "left_gripper";
    # orien_const.header.frame_id = "base";
    # orien_const.orientation.y = -1.0;
    # orien_const.absolute_x_axis_tolerance = 0.1;
    # orien_const.absolute_y_axis_tolerance = 0.1;
    # orien_const.absolute_z_axis_tolerance = 0.1;
    # orien_const.weight = 1.0;
    # consts = Constraints()
    # consts.orientation_constraints = [orien_const]
    # left_arm.set_path_constraints(consts)

    #Plan a path
    left_plan = left_arm.plan()

    #Execute the plan
    #raw_input('Press <Enter> to move the left arm to goal pose 2: ')
    left_arm.execute(left_plan)


    #Third goal pose -----------------------------------------------------
    rospy.sleep(2.0)
    goal_3 = PoseStamped()
    goal_3.header.frame_id = ar_tag

    #x, y, and z position
    goal_3.pose.position.x = x3
    goal_3.pose.position.y = y3
    goal_3.pose.position.z = z3
    
    #Orientation as a quaternion
    goal_3.pose.orientation.x = 0.0
    goal_3.pose.orientation.y = -1.0
    goal_3.pose.orientation.z = 0.0
    goal_3.pose.orientation.w = 0.0

    #Set the goal state to the pose you just defined
    left_arm.set_pose_target(goal_3)

    #Set the start state for the left arm
    left_arm.set_start_state_to_current_state()

    #Create a path constraint for the arm
    # #UNCOMMENT TO ENABLE ORIENTATION CONSTRAINTS
    # orien_const = OrientationConstraint()
    # orien_const.link_name = "left_gripper";
    # orien_const.header.frame_id = "base";
    # orien_const.orientation.y = -1.0;
    # orien_const.absolute_x_axis_tolerance = 0.1;
    # orien_const.absolute_y_axis_tolerance = 0.1;
    # orien_const.absolute_z_axis_tolerance = 0.1;
    # orien_const.weight = 1.0;
    # consts = Constraints()
    # consts.orientation_constraints = [orien_const]
    # left_arm.set_path_constraints(consts)

    #Plan a path
    left_plan = left_arm.plan()

    #Execute the plan
    #raw_input('Press <Enter> to move the left arm to goal pose 3: ')
    left_arm.execute(left_plan)

    #Close the left gripper
    #raw_input('Press <Enter> to close the gripper: ')
    rospy.sleep(1.0)
    print('Closing...')
    left_gripper.close(block=True)
    rospy.sleep(0.5)

    #Fourth goal pose -----------------------------------------------------
    rospy.sleep(2.0)
    goal_4 = PoseStamped()
    goal_4.header.frame_id = ar_tag

    #x, y, and z position
    goal_4.pose.position.x = x4
    goal_4.pose.position.y = y4
    goal_4.pose.position.z = z4
    
    #Orientation as a quaternion
    goal_4.pose.orientation.x = 0.0
    goal_4.pose.orientation.y = -1.0
    goal_4.pose.orientation.z = 0.0
    goal_4.pose.orientation.w = 0.0

    #Set the goal state to the pose you just defined
    left_arm.set_pose_target(goal_4)

    #Set the start state for the left arm
    left_arm.set_start_state_to_current_state()

    #Create a path constraint for the arm
    # #UNCOMMENT TO ENABLE ORIENTATION CONSTRAINTS
    # orien_const = OrientationConstraint()
    # orien_const.link_name = "left_gripper";
    # orien_const.header.frame_id = "base";
    # orien_const.orientation.y = -1.0;
    # orien_const.absolute_x_axis_tolerance = 0.1;
    # orien_const.absolute_y_axis_tolerance = 0.1;
    # orien_const.absolute_z_axis_tolerance = 0.1;
    # orien_const.weight = 1.0;
    # consts = Constraints()
    # consts.orientation_constraints = [orien_const]
    # left_arm.set_path_constraints(consts)

    #Plan a path
    left_plan = left_arm.plan()

    #Execute the plan
    #raw_input('Press <Enter> to move the left arm to goal pose 4: ')
    left_arm.execute(left_plan)

    #Fifth goal pose -----------------------------------------------------
    rospy.sleep(2.0)
    goal_5 = PoseStamped()
    goal_5.header.frame_id = ar_tag

    #x, y, and z position
    goal_5.pose.position.x = x5
    goal_5.pose.position.y = y5
    goal_5.pose.position.z = z5
    
    #Orientation as a quaternion
    goal_5.pose.orientation.x = 0.0
    goal_5.pose.orientation.y = -1.0
    goal_5.pose.orientation.z = 0.0
    goal_5.pose.orientation.w = 0.0

    #Set the goal state to the pose you just defined
    left_arm.set_pose_target(goal_5)

    #Set the start state for the left arm
    left_arm.set_start_state_to_current_state()

    #Create a path constraint for the arm
    # #UNCOMMENT TO ENABLE ORIENTATION CONSTRAINTS
    # orien_const = OrientationConstraint()
    # orien_const.link_name = "left_gripper";
    # orien_const.header.frame_id = "base";
    # orien_const.orientation.y = -1.0;
    # orien_const.absolute_x_axis_tolerance = 0.1;
    # orien_const.absolute_y_axis_tolerance = 0.1;
    # orien_const.absolute_z_axis_tolerance = 0.1;
    # orien_const.weight = 1.0;
    # consts = Constraints()
    # consts.orientation_constraints = [orien_const]
    # left_arm.set_path_constraints(consts)

    #Plan a path
    left_plan = left_arm.plan()

    #Execute the plan
    #raw_input('Press <Enter> to move the left arm to goal pose 5: ')
    left_arm.execute(left_plan)

    #Open the left gripper
    #raw_input('Press <Enter> to open the gripper: ')
    rospy.sleep(1.5)
    print('Opening...')
    left_gripper.open(block=True)
    rospy.sleep(1.0)

    
    #Sixth goal pose -----------------------------------------------------
    rospy.sleep(2.0)
    goal_6 = PoseStamped()
    goal_6.header.frame_id = ar_tag

    #x, y, and z position
    goal_6.pose.position.x = x6
    goal_6.pose.position.y = y6
    goal_6.pose.position.z = z6
    
    #Orientation as a quaternion
    goal_6.pose.orientation.x = 0.0
    goal_6.pose.orientation.y = -1.0
    goal_6.pose.orientation.z = 0.0
    goal_6.pose.orientation.w = 0.0

    #Set the goal state to the pose you just defined
    left_arm.set_pose_target(goal_6)

    #Set the start state for the left arm
    left_arm.set_start_state_to_current_state()

    #Create a path constraint for the arm
    # #UNCOMMENT TO ENABLE ORIENTATION CONSTRAINTS
    # orien_const = OrientationConstraint()
    # orien_const.link_name = "left_gripper";
    # orien_const.header.frame_id = "base";
    # orien_const.orientation.y = -1.0;
    # orien_const.absolute_x_axis_tolerance = 0.1;
    # orien_const.absolute_y_axis_tolerance = 0.1;
    # orien_const.absolute_z_axis_tolerance = 0.1;
    # orien_const.weight = 1.0;
    # consts = Constraints()
    # consts.orientation_constraints = [orien_const]
    # left_arm.set_path_constraints(consts)

    #Plan a path
    left_plan = left_arm.plan()

    #Execute the plan
    #raw_input('Press <Enter> to move the left arm to goal pose 6: ')
    left_arm.execute(left_plan)
    
    
    print('Done!')


if __name__ == '__main__':
    main()
'''

if __name__ == '__main__':
  node = game_play()
  node.run()
