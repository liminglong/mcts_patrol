#!/usr/bin/env python
# coding=utf-8

__author__ = 'Minglong Li'
__date__ = 'Nov. 2017'

import numpy as np
import random
import math
import copy
import sys
''' A Python program using MCTS-UC (Monte Carlo Tree Search with useful cycles) for multi-robot patrol.'''
''' This version revised the following aspects:
    1. Old: expand all children of a node at a time, set initial reward to 1.0. New: expand one child at a time.如果被检查的局面依然存在没有被拓展的子节点(例如说某节点有20个可行动作，但是在搜索树中才创建了19个子节点)，那么我们认为这个节点就是本次迭代的的目标节点N，并找出N还未被拓展的动作A。
    2. 保持A和B的统一性，以维持收敛性。
'''



ROBOT_NUM = 2;
#MCTS scalar.  Larger scalar will increase exploitation, smaller will increase exploration. 
#SCALAR= math.sqrt(2.0)
SCALAR = math.sqrt(2.0)
GLOBAL_ID = 0 #维护一个全局ＩＤ，每个ｎｏｄｅ有且仅有一个独立ＩＤ，包括cyclic_node.


NODE_DICT = {}#key is node_ID, value is node name.
CHILDREN_DICT = {}#key is node_ID, value is a list.

CYCLIC_NUM = 0
NOT_CYCLIC_NUM = 0
NEG_INFINITY = float('-Inf')
BONUS  = 1
PENALTY = -1
#robot_init_pose: robot_0:[1, 0]; roobt_1:[2, 2]
#create a map, 0 represents 'free', 1 represents 'obstacle'.  
class Map():
    
    def __init__(self):
        self.map = [[0, 0, 1, 0, 0],
                    [0, 0, 1, 0, 0],
                    [1, 0, 0, 0, 1],
                    [0, 1, 0, 1, 0],
                    [0, 0, 0, 0, 0]]
        self.horizontal_length = 5
        self.vertical_length = 5
    '''
    def __init__(self):
        self.map = [[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]]
        self.horizontal_length = 1#1行
        self.vertical_length = 12#12列
    ''' 
    def obstacles_in_map(self):
        temp_obstacles = []
        for i in range(self.horizontal_length):
            for j in range(self.vertical_length):
                if(self.map[i][j] == 1):
                    temp_obstacles.append([i,j])
        return temp_obstacles
        
    def random_pose_without_obstacles(self):
        obstacles = self.obstacles_in_map()
        while(True):
            temp_x = random.randint(0, self.horizontal_length-1)
            temp_y = random.randint(0, self.vertical_length-1)
            obstacle_flag = False
            for i in range(len(obstacles)):
                if(obstacles[i] == [temp_x, temp_y]):
                    obstacle_flag = True
                    break
            if(obstacle_flag == False):
                return [temp_x, temp_y]    
               
    def change_scale(self):
        pass
    
    def add_obstacle(self):
        pass

class DynamicIntruder():
    def __init__(self):
        self.time_e = 0
        self.time_step = 0
        pass    
    
    def update_time_step(self):
        pass
        
    def time_e(self):
        pass
        
    def time_e(self):
        pass
        
    def time_p(self):
        pass
        
    def pose(self, time):
        pass 
    
class StationaryIntruder():
    def __init__(self):
        self.time_e = random.randint(6,10)#intruder出现的时刻，从第3个时间步开始计算，０，１时刻，ｉｎｔｒｕｄｅｒ机器人肯定不会出现。
        #从直觉上讲，time_p越大，在一定budget内，捕捉到intruder的概率越大，作者最小设置为９
        self.time_p = 30#intruder停留的时间步。
        self.time_step = self.time_e
        self.map = Map()
        self.pose = self.map.random_pose_without_obstacles()
        
    def update_time_step(self):
        self.time_step = self.time_step + 1
    
    def update_initial_pose(self):
        self.pose = self.map.random_pose_without_obstacles()
        
    def get_time_e(self):
        return self.time_e
        
    def get_time_p(self):
        return self.time_p

    def get_pose(self):
        return self.pose

class Node():
    def __init__(self, actions = None, poses = None, parent = None, time_step = None, map = None, node_ID = None):
        self.visits = 1#初始化的时候就是为１的，只要ｃｈｉｌｄ被添加上，ｖｉｓｉｔｓ就是１．
        self.reward = 0.0	
        self.children = []
        self.actions = actions
        self.poses = poses
        self.parent = parent	
        self.time_step = time_step
        self.is_cyclic_arm = False
        self.cyclic_head_ID = None
        self.node_ID = node_ID
        self.first_backup_flag = True
        self.is_added_as_child = False
        self.is_fully_expanded = False#TODO:fully_expanded 初始化设置为False
        if(time_step == None):
            time_step = 0#default parameter
        self.map = map
        if(self.map == None):
            self.map = Map()#default parameter      
    
    #已经检查过的child
    def add_child(self, child):
        global GLOBAL_ID
        global NODE_DICT
        temp_child = copy.deepcopy(child)
        temp_child.node_ID = GLOBAL_ID
        self.children.append(temp_child) 
        NODE_DICT[GLOBAL_ID] = temp_child
        GLOBAL_ID += 1    
    
    def backup_child(self, child):
        global GLOBAL_ID
        global NODE_DICT
        global CHILDREN_DICT
        temp_child = copy.deepcopy(child)
        temp_child.node_ID = GLOBAL_ID
        NODE_DICT[GLOBAL_ID] = temp_child
        GLOBAL_ID += 1                
        self_ID = self.node_ID                
        if self.first_backup_flag == True:
            CHILDREN_DICT[self_ID] = [temp_child]
            self.first_backup_flag = False
        elif self.first_backup_flag == False:
            CHILDREN_DICT[self_ID].append(temp_child)
    
    #根据当前节点的actions和poses算出child_poses,　child_actions经过传参得到,再算出grand_son，检查合法性。
    def check_and_add_child_actions(self,child_actions):
        global ROBOT_NUM
        global GLOBAL_ID
        #temp_actions = []#temp_actions是个一维数组，维度是robot_num
        child_poses = []#child_poses是个二维数组,第一个维度是robot_num
        #子节点的位置是由当前节点的动作和位置决定的，子节点对应的各个机器人动作可以指定。           
        child_poses = NEXT_POSES(copy.deepcopy(self.actions), copy.deepcopy(self.poses))       
        #因为self节点的上一层节点在add_child的时候，会检测碰撞，所以，当前节点的poses不会有碰撞。  
        #但是，根据child_poses和指定的child_actions加入grandson节点的poses，就会可能碰撞
        grandson_poses = []        
        grandson_poses = NEXT_POSES(copy.deepcopy(child_actions), copy.deepcopy(child_poses))
        #child节点的位置已经是无碰撞的了，需要用grandson的位置判断是否加入child节点。        
        bump_flag = self.check_bump(copy.deepcopy(grandson_poses))  
        if(not bump_flag):
            temp_child_actions = copy.deepcopy(child_actions)
            temp_child_poses = copy.deepcopy(child_poses)
            child = Node(actions = temp_child_actions, poses = temp_child_poses, parent = self, time_step = self.time_step + 1)
            child.node_ID = GLOBAL_ID
            self.children.append(child)
            NODE_DICT[GLOBAL_ID] = child
            GLOBAL_ID +=1

            

    def check_and_backup_child_actions(self, child_actions):
        global ROBOT_NUM
        global GLOBAL_ID
        #temp_actions = []#temp_actions是个一维数组，维度是robot_num
        child_poses = []#child_poses是个二维数组,第一个维度是robot_num
        #子节点的位置是由当前节点的动作和位置决定的，子节点对应的各个机器人动作可以指定。           
        child_poses = NEXT_POSES(copy.deepcopy(self.actions), copy.deepcopy(self.poses))       
        #因为self节点的上一层节点在add_child的时候，会检测碰撞，所以，当前节点的poses不会有碰撞。  
        #但是，根据child_poses和指定的child_actions加入grandson节点的poses，就会可能碰撞
        grandson_poses = []        
        grandson_poses = NEXT_POSES(copy.deepcopy(child_actions), copy.deepcopy(child_poses))
        #child节点的位置已经是无碰撞的了，需要用grandson的位置判断是否加入child节点。        
        bump_flag = self.check_bump(copy.deepcopy(grandson_poses))  
        if(not bump_flag):
            temp_child_actions = copy.deepcopy(child_actions)
            temp_child_poses = copy.deepcopy(child_poses)
            child = Node(actions = temp_child_actions, poses = temp_child_poses, parent = self, time_step = self.time_step + 1)
            child.node_ID = GLOBAL_ID
            NODE_DICT[GLOBAL_ID] = child            
            GLOBAL_ID +=1        
            self_ID = self.node_ID                
            if self.first_backup_flag == True:
                CHILDREN_DICT[self_ID] = [temp_child]
                self.first_backup_flag = False
            elif self.first_backup_flag == False:
                CHILDREN_DICT[self_ID].append(temp_child)
                                
    def check_and_add_child_actions_poses(self, child_actions, child_poses):
        global GLOBAL_ID
        grandson_poses = NEXT_POSES(child_actions, child_poses)
        bump_flag = self.check_bump(grandson_poses)
        if(not bump_flag):
            temp_child_actions = copy.deepcopy(child_actions)
            temp_child_poses = copy.deepcopy(child_poses)
            child = Node(actions = temp_child_actions, poses = temp_child_poses, parent = self, time_step = self.time_step + 1)
            child.node_ID = GLOBAL_ID
            self.children.append(child)
            NODE_DICT[GLOBAL_ID] = child
            GLOBAL_ID += 1

    def check_and_backup_child_actions_poses(self, child_actions, child_poses):
        global GLOBAL_ID
        grandson_poses = NEXT_POSES(child_actions, child_poses)
        bump_flag = self.check_bump(grandson_poses)
        if(not bump_flag):
            temp_child_actions = copy.deepcopy(child_actions)
            temp_child_poses = copy.deepcopy(child_poses)
            child = Node(actions = temp_child_actions, poses = temp_child_poses, parent = self, time_step = self.time_step + 1)
            child.node_ID = GLOBAL_ID
            NODE_DICT[GLOBAL_ID] = child
            GLOBAL_ID += 1       
            self_ID = self.node_ID                
            if self.first_backup_flag == True:
                CHILDREN_DICT[self_ID] = [child]
                self.first_backup_flag = False
            elif self.first_backup_flag == False:
                CHILDREN_DICT[self_ID].append(child) 
    
    
    def check_bump(self, poses):
        global ROBOT_NUM
        #bump_flag = False#bump into obstacles, or bump into boundings
        for i in range(ROBOT_NUM):
            if(poses[i][0] == -1 or poses[i][0] == self.map.horizontal_length):#bump into left or right boundings
                #bump_flag = True
                #break
                return True
            if(poses[i][1] == -1 or poses[i][1] == self.map.vertical_length):#bump into up or down boundings
                #bump_flag = True
                #break
                return True
            else:
                obstacles = self.map.obstacles_in_map()#a list storing the coordinates of the obstacles
                for j in range(len(obstacles)):
                    if poses[i] == obstacles[j]:
                    #if(poses[i][0] == obstacles[j][0] and poses[i][1] == obstacles[j][1]):
                        #bump_flag == True
                        #break
                        return True
                #break
        #return bump_flag
        return False
        
    def update(self,reward):
        self.reward+=reward
        self.visits+=1
    
    def ucb(self):
        visits_parent = self.parent.visits
        visits_child = self.visits
        w_pi = self.reward
        return w_pi + SCALAR * math.sqrt((math.log(visits_parent))/visits_child)
    
    def create_cyclic_sibling_arm(self, cyclic_head_ID):
        global GLOBAL_ID
        #print "create 1"
        temp_parent = self.parent
        #print "create 6"
        new_node = Node()
        new_node.parent = self.parent
        new_node.poses = copy.deepcopy(self.poses)
        new_node.actions = copy.deepcopy(self.actions)
        new_node.reward = copy.deepcopy(self.reward)
        new_node.time_step = self.time_step
        #print "create 4"
        new_node.cyclic_head_ID = cyclic_head_ID
        #print "create 5"
        new_node.is_cyclic_arm = True
        new_node.visits = 1
        #print "create 2"
        new_node.node_ID = GLOBAL_ID
        NODE_DICT[GLOBAL_ID] = new_node
        temp_parent.children.append(new_node)
        GLOBAL_ID += 1
        #print "create 3"
    
    def get_is_cyclic_arm(self):
        return self.is_cyclic_arm
           
        
    def node_equal(self, other):
        self_node_ID = self.node_ID
        other_node_ID = other.node_ID
        
        if(self_node_ID == other_node_ID):
            return True
        else:
            return False  

    def node_ID_equal(self, other):
        self_node_ID = self.node_ID
        other_node_ID = other
        
        if(self_node_ID == other_node_ID):
            return True
        else:
            return False

    def set_is_added_as_child(self, value):
        self.is_added_as_child = value

    def get_is_added_as_child(self):
        return self.is_added_as_child


    def check_is_fully_expanded(self):
        #To be verified: Version 0 of mucts_patrol, all of the nodes are fully expanded
        if(self.first_backup_flag == True):
            print "Error! Check whether fully expanded before backup."
            sys.exit(1)#异常退出
        else:
            expanded_children_len = len(self.children)
            all_children_len = len(CHILDREN_DICT[self.node_ID])
            if(expanded_children_len < all_children_len):
                self.is_fully_expanded = False
            elif(expanded_children_len == all_children_len):
                self.is_fully_expanded = True
            elif(expanded_children_len > all_children_len):
                print "Error! expanded_children_len should be smaller than all_children_len."
                sys.exit(1)#异常退出
            return self.is_fully_expanded

#global functions
#UCT search
#def SELECT(budget, root):
def SELECT(root):
    #pass
    global NEG_INFINITY
    selected_node = Node()
    global_ucb = NEG_INFINITY
    temp_ucb = NEG_INFINITY
    temp_children = root.children #TODO:root has already been fully expanded already.
    #print "244: SELECT 1"
    while(len(temp_children) != 0):
        #print "246: SELECT while"
        for i in range(len(temp_children)):   
            temp_ucb = temp_children[i].ucb()
            #print temp_ucb
            if(global_ucb < temp_ucb or global_ucb == temp_ucb):
                #print "SELECT if"
                global_ucb = temp_ucb

                selected_node = temp_children[i]
        print "SELECT 2"    
        print "每一层被选中的global_ucb: "
        print global_ucb    
        temp_children = selected_node.children
        global_ucb = NEG_INFINITY
        #print "len(temp_children):"
        #print len(temp_children)
    return selected_node   
  
      
      
      
      
      
  
def ITERATIVE_LOOP(temp_node, cycles, temp_actions):
    global ROBOT_NUM
    cycles -= 1
    index = ROBOT_NUM - cycles -1
 
    if(cycles==-1):
        return 
    else:
        if(cycles == 0):
            for i in range(4):
                temp_actions[index] = i
                temp_node.check_and_backup_child_actions(child_actions = temp_actions);
        else:
            for i in range(4):
                temp_actions[index] = i
                ITERATIVE_LOOP(temp_node, cycles, temp_actions)
                              
#fully expand the node
'''
def EXPAND(node):
    global ROBOT_NUM
    temp_actions = []
    for i in range(ROBOT_NUM):
        temp_actions.append(-1)   
    ITERATIVE_LOOP(node, ROBOT_NUM, temp_actions)
    i = random.randint(0, len(node.children)-1)
    return node.children[i]
'''

def BACKUP(node):
    global ROBOT_NUM
    temp_actions = []
    for i in range(ROBOT_NUM):
        temp_actions.append(-1)
    ITERATIVE_LOOP(node, ROBOT_NUM, temp_actions)

def NEXT_POSES(actions, poses):
    child_poses = []
    for index in range(ROBOT_NUM):
        child_poses.append([0,0])
    #print child_poses
    #print child_poses[0][0]
    #print actions[0]
    #print poses[0]
    for i in range(ROBOT_NUM):
        j = actions[i]#up, down, left, right
        if(j == 0):#up
            child_poses[i][0] = poses[i][0] 
            child_poses[i][1] = poses[i][1] + 1
        if(j == 1):#down
            child_poses[i][0] = poses[i][0] 
            child_poses[i][1] = poses[i][1] - 1
        if(j == 2):#left
            child_poses[i][0] = poses[i][0] - 1
            child_poses[i][1] = poses[i][1] 
        if(j == 3):#right
            child_poses[i][0] = poses[i][0] + 1
            child_poses[i][1] = poses[i][1]  
    return child_poses 

def ITERATIVE_LOOP_ROOT_CHILD_ACTION(root, cycles, child_poses, child_actions):
    global ROBOT_NUM
    cycles -= 1
    index = ROBOT_NUM - cycles - 1

    if cycles == -1:
        return 
    else:
        if cycles == 0:
            for i in range(4):
                child_actions[index] = i
                #if bump, not add. if not bump, add.
                root.check_and_backup_child_actions_poses(child_actions, child_poses)            
        else:
            for i in range(4):
                child_actions[index] = i
                ITERATIVE_LOOP_ROOT_CHILD_ACTION(root, cycles, child_poses, child_actions)

#root节点的初始化当中，action为空，poses是初始化指定的值。
def ITERATIVE_LOOP_ROOT_ACTION(root, cycles, root_actions):
    global ROBOT_NUM
    cycles -= 1
    index = ROBOT_NUM - cycles - 1
    count_num = 0
    #print "cycles in ITERATIVE_LOOP_ROOT_ACTION: "
    #print cycles
    #print "index in ITERATIVE_LOOP_ROOT_ACTION: "
    #print index
    if(cycles == -1):
        return
    else:
        if(cycles == 0):
            for i in range(4):
                #print "count_num:"
                #print count_num
                #print "index in if part: "
                #print index
                count_num +=1
                root_actions[index] = i
                #print "root_actions in if part:"
                print root_actions
                child_poses = NEXT_POSES(root_actions, root.poses)
                #print "child_poses"
                #print child_poses
                bump_flag = root.check_bump(child_poses)
                #print "bump_flag"
                #print bump_flag
                if bump_flag == True:
                    pass#检查合法性，如果不合法，continue。
                elif(bump_flag == False):
                    child_actions = []
                    for i in range(ROBOT_NUM):
                        child_actions.append(-1)
                    ITERATIVE_LOOP_ROOT_CHILD_ACTION(root=root, cycles=ROBOT_NUM, child_poses=child_poses,child_actions = child_actions)
        else:
            for i in range(4):
                #print "index in else part: "
                #print index
                root_actions[index] = i
                #print "root_actions in else part: "
                #print root_actions
                ITERATIVE_LOOP_ROOT_ACTION(root, cycles, root_actions)
'''
def EXPAND_ROOT(root):
    global ROBOT_NUM
    root_actions = []
    for i in range(ROBOT_NUM):
        root_actions.append(-1)
    ITERATIVE_LOOP_ROOT_ACTION(root, ROBOT_NUM, root_actions)
    i = random.randint(0, len(root.children)-1)
    return root.children[i]
'''

def EXPAND(node):
    if node.first_backup_flag == True:
        print "Error! Expand before backup."
        sys.exit(1)#异常退出
    if node.check_is_fully_expanded() == True:
        print "Error! Expand the fulle-expanded node."
        sys.exit(1)
    else:
        for i in range(len(CHILDREN_DICT[node.node_ID])):
            if CHILDREN_DICT[node.node_ID][i].get_is_added_as_child() == False:
                node.children.append(CHILDREN_DICT[node.node_ID][i])
                CHILDREN_DICT[node.node_ID][i].set_is_added_as_child(True)
                return CHILDREN_DICT[node.node_ID][i]

def BACKUP_ROOT(root):
    global ROBOT_NUM
    root_actions = []
    for i in range(ROBOT_NUM):
        root_actions.append(-1)
    ITERATIVE_LOOP_ROOT_ACTION(root, ROBOT_NUM, root_actions)

'''
#TODO:根据loop的逻辑写两个ITERATIVE LOOP
def loop(cycles):
    for a0 in range(4):
        for a1 in range(4):
            for a2 in range(4):
   　    　　　　    root_actions[0] = a0
   　    　　　　    root_actions[1] = a1
   　    　　　　    root_actions[2] = a2
   　    　　　　    root_poses = [[1,1],[2,2]]   　    　　　　    
   　    　　　　    child_poses = function(root_actions, root_poses)
                #对于root的话，CHILD_POSES在这里也要检查吗？
   　    　　　　    
                for b0 in range(4):
                    for b1 in range(4):
                        for b2 in range(4):   　    　　　　    
                       　    child_actions[0] = a0
               　    　　　　    child_actions[1] = a1
               　    　　　　    child_actions[2] = a2 　    　　　　    
               　    　　　　    
               　    　　　　       　    　　　　    
               　    　　　　    grandson_poses = function2(child_actions, child_poses);
               　    　　　　    bump_flag = check_bump(grandson_poses)
               　    　　　　    
               　    　　　　    if bump_flag == False
               　    　　　　        add_child(child_poses, child_actions)
   　
    for a0 in range(4):
       　root_actions[0] = a0
        for a1 in range(4):
           　root_actions[1] = a1
            for a2 in range(4):
                root_actions[2] = a2  
                add_child(root_actions)
                
                print root_actions 　
''' 

def RANDOM_ROLLOUT_FOR_ONE_STEP(actions, poses):
    global ROBOT_NUM
    map = Map()
    current_actions = copy.deepcopy(actions)#把函数的传引用改为了传值
    current_poses = copy.deepcopy(poses)#把函数的传引用改为了传值
    
    next_actions = []
    next_poses = []
    
    obstacles =  map.obstacles_in_map()
        
    for i in range(ROBOT_NUM):
        if(current_actions[i] == 0):#up
            next_poses.append( [current_poses[i][0], current_poses[i][1]+1] )
        if(current_actions[i] == 1):#down
            next_poses.append( [current_poses[i][0], current_poses[i][1]-1] )                  
        if(current_actions[i] == 2):#left
            next_poses.append( [current_poses[i][0]-1, current_poses[i][1]] )               
        if(current_actions[i] == 3):#right
            next_poses.append( [current_poses[i][0]+1, current_poses[i][1]] )        
                   
    for i in range(ROBOT_NUM):
        while(True):
            temp_poses = [0,0]
            random_action = random.randint(0, 3)#TODO:一个可以优化的地方，因为这次random的action，下次可以排除。
            if(random_action == 0):#up
                temp_poses[0] = next_poses[i][0]
                temp_poses[1] = next_poses[i][1] + 1
            if(random_action == 1):#down
                temp_poses[0] = next_poses[i][0]
                temp_poses[1] = next_poses[i][1] - 1   
            if(random_action == 2):#left
                temp_poses[0] = next_poses[i][0] - 1
                temp_poses[1] = next_poses[i][1]         
            if(random_action == 3):#right
                temp_poses[0] = next_poses[i][0] + 1
                temp_poses[1] = next_poses[i][1]
            bump_flag = False
            for j in range(len(obstacles)):
                if(temp_poses == obstacles[j]):
                    bump_flag = True
                    break
            if temp_poses[0]==-1 or temp_poses[0]== map.horizontal_length or temp_poses[1] == -1 or temp_poses[1] == map.vertical_length:
                bump_flag = True
            if(bump_flag == False):#TODO:只能跳过内层循环，跳不过外层循环。
                temp_random_action = copy.deepcopy(random_action)
                next_actions.append(temp_random_action)
                break
    return next_actions, next_poses                        

#node is a cyclic leaf, find its cyclic head by its cyclic head ID.
def PERFORM_CYCLIC_ACTIONS(node, intruder):
    global ROBOT_NUM
    global BONUS
    global PENALTY
    time_e = intruder.get_time_e()
    time_p = intruder.get_time_p()
    intruder_pose = intruder.get_pose()
    
    #status of the node to be evaluated
    temp_time_step = node.time_step
    temp_poses = node.poses 
    
    temp_node = node    
    '''
    print "node.cyclic_head_ID: "
    print node.cyclic_head_ID
    print "node.node_ID:"
    print node.node_ID
    print "node.parent.node_ID:"
    print node.parent.node_ID
    '''
    head_mark_node_ID = node.cyclic_head_ID   
    head_node = NODE_DICT[head_mark_node_ID]#利用字典查找找到这个ｎｏｄｅ
    #print head_node.poses
    #print node.poses        
            
    poses_buffer = []#创建一个缓冲池，用来存储需要走过的ｐｏｓｅｓ，缓冲池当中没有重复位置（不对应的重复也算是重复）的节点。
    #poses_buffer.append(temp_poses)
 
    while(not temp_node.node_ID_equal(head_mark_node_ID)):
        poses_buffer.append(temp_node.poses)        
        temp_node = temp_node.parent    
    index = 0    
    index_increase_flag = True
    print "len(poses_buffer):"
    print len(poses_buffer)
    
    #只需在下面考虑，怎么把周期n-1考虑进来，即把周期为1的情况也要考虑进来。
    
    #表示偶数个机器人位置两两交换
    if len(poses_buffer) == 1:
        if poses_buffer[0] == intruder_pose:
            return BONUS
        else:
            return PENALTY       
    else:
        while(True):
            win_flag = False
            for i in range(ROBOT_NUM):
                #print "index:"
                #print index
                if poses_buffer[index][i] == intruder_pose:
                    win_flag = True
                    break
            if temp_time_step < time_e:
                pass                    
            elif ((temp_time_step > time_e or temp_time_step == time_e) and temp_time_step < time_e + time_p):
                if win_flag == True:
                    print "win"
                    return BONUS
            elif temp_time_step == time_e + time_p:        
                if win_flag == True:
                    print "win"
                    return BONUS
                elif win_flag == False:
                    print "loose"
                    return PENALTY
            
            if(index_increase_flag):
                index +=1
                temp_time_step+=1
                if index == len(poses_buffer):
                    index-=1
                    temp_time_step-=1
                    index_increase_flag = False
            elif(not index_increase_flag):
                index -= 1
                temp_time_step+=1
                if index == -1:
                    index+=2
                    index_increase_flag = True
            
def ROLLOUT(node, intruder):
    #有两个前提：
    #第一个前提是当前ｎｏｄｅ上层的ｎｏｄｅ一定不是终止节点，ｓｅｌｅｃｔ和ｒｏｌｌｏｕｔ的都不是终止节点，只有当前ｎｏｄｅ和之后的ｎｏｄｅ可能是终止节点。
    #第二个前提是当前ｎｏｄｅ是经过ｅｘｐａｎｄ选出来的唯一一个待评估节点。
    #status of the intruder
    time_e = intruder.get_time_e()
    time_p = intruder.get_time_p()
    intruder_pose = intruder.get_pose()
    #status of the node to be evaluated
    temp_time_step = node.time_step
    temp_actions = node.actions
    temp_poses = node.poses    
    
    next_actions = temp_actions
    next_poses = temp_poses    
    #print "rollout 1"
    if(node.get_is_cyclic_arm() == True):
        #print "rollout 2"
        temp_reward_cyclic = PERFORM_CYCLIC_ACTIONS(node, intruder)
        print "PERFORM_CYCLIC_ACTIONS(node, intruder): "
        print temp_reward_cyclic
        return temp_reward_cyclic
    else:
        #print "rollout 3"
        while(True):
            #print "rollout 4"
            win_flag = False
            for i in range(len(next_poses)):
                if(next_poses[i] == intruder_pose):
                    win_flag = True
                    break
            #if the intruder completing intruding or capture the intruder
            if(temp_time_step < time_e):
                temp_time_step += 1
                next_actions, next_poses = RANDOM_ROLLOUT_FOR_ONE_STEP(next_actions, next_poses)
            elif((temp_time_step == time_e or temp_time_step > time_e) and temp_time_step < time_e + time_p):
                if win_flag == True:
                    return BONUS
                elif win_flag == False:
                    #侵入完成时间也没到，机器人也没找着，继续搜索,TODO:一个idea，是否把找到的时间也算在内呢？
                    temp_time_step += 1
                    next_actions, next_poses = RANDOM_ROLLOUT_FOR_ONE_STEP(next_actions, next_poses)
            elif(temp_time_step == time_e + time_p):#入侵完成时间到了
                if(win_flag == True):
                    return PENALTY
                elif(win_flag == False):
                    return PENALTY

#TODO:这里肯定也是需要的。
def ONE_EQUAL_IN_LIST_DEL(list_0, list_1):
    equal_flag = False
    for i in range(len(list_0)):
        for j in range(len(list_1)):
            if(list_0[i] == list_1[j]):
                del list_0[i]
                del list_1[j]
                equal_flag = True
                return list_0, list_1                
    if(equal_flag == False):
        list_0[0] = "not_equal"
        return list_0, list_1
        
#只需要node_0和node_1当中的所有的机器人的位置都有一一相等的，不用顺序相等。    　　　
def STATUS_EQUAL(node_0, node_1):
    global ROBOT_NUM
    poses_0 = []
    poses_1 = []
    for i in range(ROBOT_NUM):
        temp_0 = copy.deepcopy(node_0.poses[i])
        temp_1 = copy.deepcopy(node_1.poses[i])     
        poses_0.append(temp_0)
        poses_1.append(temp_1)
    #poses_0 = node_0.poses
    #poses_1 = node_1.poses
    if(len(poses_0) != len(poses_1)):
        return False
    else:    
        equal_flag = False
        while(True):
            poses_0, poses_1 = ONE_EQUAL_IN_LIST_DEL(poses_0, poses_1)
            if(len(poses_0) == 0 and len(poses_1) == 0):
                return True
            if(poses_0[0] == "not_equal"):
                return False
                
def BACK_PROPAGATION(leaf_node, reward):
    leaf_node.update(reward)
    temp_node = leaf_node.parent

    while(True):
        #print "while 0"
        if(STATUS_EQUAL(temp_node, leaf_node)):
            #print "while 1"
            if (not leaf_node.get_is_cyclic_arm()):
                temp_node_ID = temp_node.node_ID
                leaf_node.create_cyclic_sibling_arm(cyclic_head_ID = temp_node_ID)#it is an arm
            #TODO:如果temp_node本身就是一个cyclicarm，不再向上创建cyclic arm
        #print "while 2"
        temp_node.update(reward)
        temp_node = temp_node.parent
        if temp_node == None:
            break

def IS_INTRUDER_CAPTURED(node, intruder):
    temp_poses = copy.deepcopy(node.poses)
    intruder_pose = intruder.pose
    win_flag = False
    for i in range(len(temp_poses)):
        if(temp_poses[i] == intruder_pose):
            win_flag = True
            break
    if win_flag:
        return True
    else:
        return False

def OUTPUT_ROBOT_TRACK(root):
    global ROBOT_NUM
    global NEG_INFINITY
    #print root.poses
    OUTPUT_ROBOT_IN_MAP(root.poses)
    
    selected_node = Node()
    global_ucb = NEG_INFINITY
    temp_ucb = NEG_INFINITY
    temp_children = root.children #'children' is a list
    while(len(temp_children) != 0):
        for i in range(len(temp_children)):   
            temp_ucb = temp_children[i].ucb()            
            if(global_ucb < temp_ucb or global_ucb == temp_ucb):
                global_ucb = temp_ucb
                selected_node = temp_children[i]
        if selected_node.get_is_cyclic_arm():
            print "cycle断点"
            temp_node = selected_node
            head_mark_node_ID = temp_node.cyclic_head_ID
            head_node = NODE_DICT[head_mark_node_ID]                    
            while (not temp_node.node_ID_equal(head_mark_node_ID)):
                #print temp_node.poses
                OUTPUT_ROBOT_IN_MAP(temp_node.poses)
                temp_node = temp_node.parent
            #print temp_node.poses
            OUTPUT_ROBOT_IN_MAP(temp_node.poses)
            return
        OUTPUT_ROBOT_IN_MAP(selected_node.poses)
        #print selected_node.poses
        temp_children = selected_node.children
        global_ucb = NEG_INFINITY

    return selected_node       

def OUTPUT_ROBOT_IN_MAP(robot_poses):
    map = Map()
    temp_map = map.map
    for i in range(len(robot_poses)):
        temp_map[robot_poses[i][0]][robot_poses[i][1]] = "R" + str(i)
    print temp_map

#重新写一个main函数。
if __name__ == "__main__":
    map = Map()
    intruder = StationaryIntruder()
    BUDGET = 1000000
    reward = 0.0
    #初始的机器人的位置是(1, 0), (1, 1)
    root = Node(actions = None, poses = [[1,0],[1,1]], parent = None, time_step = 0, map = map)
    #root = Node(actions = None, poses = [[0,0],[0,24]], parent = None, time_step = 0, map = map)
    root.node_ID = -1    
    NODE_DICT[-1] = root
    #print "Hello 1"    
    BACKUP_ROOT(root)
    print "len(CHILDREN_DICT[root.node_ID]): "
    print len(CHILDREN_DICT[root.node_ID])
    
    cycle_count = 0    
    
    selected_node = EXPAND(root)
    reward = ROLLOUT(root, intruder)
    BACK_PROPAGATION(selected_node, reward)
    
    #TODO: 应该在这里只需要重写SELECT方法就可以了。    
    
    while(cycle_count < BUDGET):
        intruder.update_initial_pose()
        leaf_node = SELECT(root)


'''        
if __name__=="__main__":
    map = Map()
    intruder = StationaryIntruder()
    BUDGET = 1000000
    reward = 0.0
    #初始的机器人的位置是(1, 0), (1, 1)
    root = Node(actions = None, poses = [[1,0],[1,1]], parent = None, time_step = 0, map = map)
    #root = Node(actions = None, poses = [[0,0],[0,24]], parent = None, time_step = 0, map = map)
    root.node_ID = -1    
    NODE_DICT[-1] = root
    #print "Hello 1"
    EXPAND_ROOT(root)   
    
    #print len(root.children)
    #print len(root.children)  
    
    cycle_count = 0
    #print "Hello 2"
    while(cycle_count < BUDGET):
        print "Hello 3"
        intruder.update_initial_pose()
        print "intruder.pose"
        print intruder.pose
        leaf_node = SELECT(root)#TODO:这里需要判断node是不是时间终止节点。
        print "Hello 4"
        if leaf_node.time_step == intruder.time_e + intruder.time_p:
            print "Hello 5"
            if IS_INTRUDER_CAPTURED(leaf_node, intruder) == True:
                reward = BONUS
            else:
                reward = PENALTY
            BACK_PROPAGATION(leaf_node, reward)    
        elif leaf_node.time_step < intruder.time_e + intruder.time_p:    
            print "Hello 6"
            if leaf_node.get_is_cyclic_arm():
                print "cyclic"
                CYCLIC_NUM += 1 
                print "Hello 7"
                reward = ROLLOUT(leaf_node, intruder)
                print "Hello 7 middle"
                BACK_PROPAGATION(leaf_node, reward)
                print "Hello 7 end"
            else:
                NOT_CYCLIC_NUM += 1 
                added_node = EXPAND(leaf_node)#TODO:测试过程中，这一步极其花费时间。？？？？？？？？
                #TODO：leaf_node是不是一定不是cyclic arm？？？？？？？？？？？
                print len(leaf_node.children)#只有12的是肯定不对的。
                print "Hello 9"
                reward = ROLLOUT(added_node, intruder)
                print "Hello 10"
                BACK_PROPAGATION(added_node, reward)
                print "Hello 11"
        elif leaf_node.time_step > intruder.time_e + intruder.time_p:
            print "程序异常，异常点标号为１"           

        print "cycle_count :"
        print cycle_count
        cycle_count += 1
    
    
    OUTPUT_ROBOT_TRACK(root)
        
    print "CYCLIC_NUM: "    
    print CYCLIC_NUM
    print "NOT_CYCLIC_NUM: "
    print NOT_CYCLIC_NUM
'''
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
