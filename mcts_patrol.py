#!/usr/bin/env python
# coding=utf-8

__author__ = 'Minglong Li'
__date__ = 'Nov. 2017'

import numpy as np
import random
import math

''' A Python program using MCTS-UC (Monte Carlo Tree Search with useful cycles) for multi-robot patrol.'''

ROBOT_NUM = 2;
#MCTS scalar.  Larger scalar will increase exploitation, smaller will increase exploration. 
SCALAR= math.sqrt(2.0)

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
     
    def obstacles_in_map():
        temp_obstacles = []
        for(i in range(self.horizontal_length)):
            for(j in range(self.vertical_length)):
                if(self.map[i, j] == 1):
                    temp_obstacles.append([i,j])
        return temp_obstacles
        
    def random_pose_without_obstacles():
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
        pass #TODO: version 0.0 of mcts_patrol, we use a stationary intruder.    
    
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
        self.time_e = random.randint(1,10)
        #从直觉上讲，time_p越大，在一定budget内，捕捉到intruder的概率越大，作者最小设置为９
        self.time_p = 20 
        self.time_step = self.time_e
        self.map = Map()
        self.pose = self.map.random_pose_without_obstacles()
        
    def update_time_step(self):
        self.time_step = self.time_step + 1
        
    def time_e(self):
        return self.time_e
        
    def time_p(self):
        return self.time_p
        
   　def pose(self):
   　    return self.pose
   　    
    

class Node():
    def __init__(self, actions = None, poses = None, parent = None, time_step = None, map = None):
        self.visits = 0
        self.reward = 0.0	
        self.children = []
        self.actions = actions
        self.poses = poses
        self.parent = parent	
        self.time_step = time_step
        self.is_cyclic_arm = False
        self.cyclic_head = Node()
        if(time_step == None):
            time_step = 0#default parameter
        if(map == None):
            map = Map()#default parameter      
    
    #已经检查过的ｃｈｉｌｄ
    def add_child(self, child):
        temp_child = child
        self.children.apend(temp_child)     
    
    #根据当前节点的actions和poses算出child_poses,　child_actions经过传参得到,再算出grand_son，检查合法性。
    def check_and_add_child(self,child_actions=[]):
        #temp_actions = []#temp_actions是个一维数组，维度是robot_num
        child_poses = []#child_poses是个二维数组,第一个维度是robot_num
        #子节点的位置是由当前节点的动作和位置决定的，子节点对应的各个机器人动作可以指定。    
        for(i = 0; i < ROBOT_NUM; i+=1):
            j = self.actions[i]#up, down, left, right
            if(j == 0):#up
                child_poses[i][0] = self.poses[0] 
                child_poses[i][1] = self.poses[1] - 1
            if(j == 1):#down
                child_poses[i][0] = self.poses[0] 
                child_poses[i][1] = self.poses[1] + 1
            if(j == 2):#left
                child_poses[i][0] = self.poses[0] - 1
                child_poses[i][1] = self.poses[1] 
            if(j == 3):#right
                child_poses[i][0] = self.poses[0] + 1
                child_poses[i][1] = self.poses[1]                 
        #因为self节点的上一层节点在add_child的时候，会检测碰撞，所以，当前节点的poses不会有碰撞。  
        #但是，根据child_poses和指定的child_actions加入grandson节点的poses，就会可能碰撞
        grandson_poses = []
        for(i = 0; i < ROBOT_NUM; i+=1):
            j = child_actions[i]
            if(j == 0):#up
                grandson_poses[i][0] = child.poses[0] 
                grandson_poses[i][1] = child_poses[1] - 1
            if(j == 1):#down
                grandson_poses[i][0] = child_poses[0] 
                grandson_poses[i][1] = child_poses[1] + 1
            if(j == 2):#left
                grandson_poses[i][0] = child_poses[0] - 1
                grandson_poses[i][1] = child_poses[1] 
            if(j == 3):#right
                grandson_poses[i][0] = child_poses[0] + 1
                grandson_poses[i][1] = child_poses[1]         
        
        #child节点的位置已经是无碰撞的了，需要用grandson的位置判断是否加入child节点。        
        bump_flag = self.check_bump(grandson_poses)  
                                  

        if(not bump_flag):
            child = Node(actions = child_actions, poses = child_poses, parent = self, time_step = self.time_step + 1)
            self.children.apend(child)       

　　　　def check_and_add_child(self, child_poses=[], child_poses=[]):
　　　　    #TODO
　　　　    grandson_poses = []
        for(i = 0; i < ROBOT_NUM; i+=1):
            j = child_actions[i]
            if(j == 0):#up
                grandson_poses[i][0] = child.poses[0] 
                grandson_poses[i][1] = child_poses[1] - 1
            if(j == 1):#down
                grandson_poses[i][0] = child_poses[0] 
                grandson_poses[i][1] = child_poses[1] + 1
            if(j == 2):#left
                grandson_poses[i][0] = child_poses[0] - 1
                grandson_poses[i][1] = child_poses[1] 
            if(j == 3):#right
                grandson_poses[i][0] = child_poses[0] + 1
                grandson_poses[i][1] = child_poses[1]     
　　　　    bump_flag = self.check_bump(grandson_poses)
　　　　    if(not bump_flag):
　　　　        child = Node(actions = child_actions, poses = child_poses, parent = self, time_step = self.time_step + 1)
　　　　        self.children.apend(child)
　　　　    
　　　　    

    def check_bump(self, poses):
        bool bump_flag = False#bump into obstacles, or bump into boundings
        for(i = 0; i < ROBOT_NUM; i+=1):                  
            if(poses[i][0] == -1 or poses[i][0] == self.map.horizontal_length):#bump into left or right boundings
                bump_flag = True
                break
            if(poses[i][1] == -1 or poses[i][1] == self.map.vertical_length):#bump into up or down boundings
                bump_flag = True
                break
            else:
                obstacles = self.map.obstacles_in_map()#a list storing the coordinates of the obstacles
                for(j in range(len(obstacles))):
                    if(poses[i][0] == obstacles[j][0] and poses[i][1] == obstacles[j][1]):
                        bump_flag == True
                        break
                break
         return bump_flag        

        
    def update(self,reward):
        self.reward+=reward
        self.visits+=1
    
    def ucb(self):
        visits_parent = self.parent.visits
        visits_child = self.visits
        w_pi = self.reward
        return w_pi + SCALAR * math.sqrt(math.log(visits_parent/visits_children))
    
    def create_cyclic_sibling_arm(self, temp_cyclic_head):
        temp_parent = self.parent
        temp_node = self
        temp_node.cyclic_head = temp_cyclic_head
        temp_node.is_cyclic_arm = True
        temp_node.visits = 1
        temp_parent.children.append(temp_node)
    
    def is_cyclic_arm(self):
        return self.is_cyclic_arm
        
    def node_equal(self, other):
        temp_poses = self.poses
        temp_actions = self.actions
        temp_time_step = self.time_step
        
        other_poses = other.poses
        other_actions = other.actions
        other_time_step = other.time_step
        
        if(temp_poses==other_poses and temp_actions==other_actions and temp_time_step==other_time_step):
            return True
        else: 
            return False        
    
    def is_fully_expanded(self):
        #To be verified: Version 0 of mucts_patrol, all of the nodes are fully expanded
        pass       

#global functions
#UCT search
#def SELECT(budget, root):
def SELECT(root):
    #pass
    temp_node = Node
    temp_ucb = 0
    temp_children = root.children #'children' is a list
    while(len(temp_children) != 0):
        for(i = 0; i < len(temp_children); i+=1):
            if(temp_ucb < temp_chiledren[i].ucb):
                temp_ucb = temp_children[i].ucb
                temp_node = temp_children[i]
        temp_children = temp_node.children
    #TODO:在这里少了一步，就是判断temp_node是不是终止节点，如果是终止节点，就不需要ｅｘｐａｎｄ了，直接去ｒｏｌｌｏｕｔ了，直接去ｂａｃｋｐｒｏｐａｇａｔｉｏｎ就行了，这个在主逻辑当中去判断。
    return temp_node   
  
def ITERATIVE_LOOP(temp_node, cycles):
    cycles -= 1
    index = ROBOT_NUM - cycles -1
    if(cycles==-1):
        return 
    else:
        if(cycles == 0):
            for i in range(4):
                temp_actions[index] = i
                node.check_and_add_child(child_actions = temp_actions);
        else:
            for i in range(4):
                temp_actions[index] = i
                ITERATIVE_LOOP(temp_node, cycles)
              
                

                
#fully expand the node        
def EXPAND(node):
　　　　#TODO:扩展的时候，根据当前节点扩展，在当前节点动作已经确定的情况之下，下一个节点的位置已经被确定，只是把无碰撞的动作添加进去就好，只有在ｒｏｏｔ的时候可以随便加，因为ｒｏｏｔ本身没有ａｃｔｉｏｎｓ。
    ITERATIVE_LOOP(node, ROBOT_NUM)
    #choose the first child to return    
    return node.children[0]

def EXPAND_ROOT(root):
    root_poses = root.poses
    root_actions = []#root_actions没有保存任何东西

def ITERATIVE_LOOP_ROOT(root, cycles):
    cycles -= 1
    index = ROBOT_NUM - cycles - 1
    if(cycles == -1):
        return
    else:
        if(cycles == 0):
            for i in range(4)
                temp_actions[index] = i
                node.add_
#TODO:根据loop的逻辑鞋两个ITERATIVE LOOP
def loop(cycles):
    for a0 in range(4):
        for a1 in range(4):
            for a2 in range(4):
   　    　　　　    root_actions[0] = a0
   　    　　　　    root_actions[1] = a1
   　    　　　　    root_actions[2] = a2
   　    　　　　    root_poses = [[1,1],[2,2]]   　    　　　　    
   　    　　　　    child_poses = function(root_actions, root_poses);
   　    　　　　    
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
    

def RANDOM_ROLLOUT_FOR_ONE_STEP(actions, poses):
    map = Map()
    current_actions = actions
    current_poses = poses
    
    next_actions = []
    next_poses = []
    
    obstacles =  map.obstacles_in_map()
        
    for i in range(ROBOT_NUM):
        if(current_actions[i] == 0):#up
            next_poses.append( [current_poses[i][0], current_poses[i][1]-1] )
        if(current_actions[i] == 1):#down
            next_poses.append( [current_poses[i][0], current_poses[i][1]+1] )                  
        if(current_actions[i] == 2):#left
            next_poses.append( [current_poses[i][0]-1, current_poses[i][1]] )               
        if(current_actions[i] == 3):#right
            next_poses.append( [current_poses[i][0]+1, current_poses[i][1]] )        
                   
    for i in range(ROBOT_NUM):
        while(True):
            temp_poses = [0,0]
            random_action = random.randint(0, 3)
            if(random_action == 0):#up
                temp_poses[0] = next_poses[i][0]
                temp_poses[1] = next_poses[i][1] - 1
            if(random_action == 1):#down
                temp_poses[0] = next_poses[i][0]
                temp_poses[1] = next_poses[i][1] + 1   
            if(random_action == 2):#left
                temp_poses[0] = next_poses[i][0] - 1
                temp_poses[1] = next_poses[i][1]         
            if(random_action == 3):#right
                temp_poses[0] = next_poses[i][0] + 1
                temp_poses[1] = next_poses[i][1]         
            obstacle_flag == False
            for j in range(len(obstacles)):
                if(temp_poses == obstacles[j]):
                    obstacle_flag = True
                    break
            if(obstacles_flag == False):
                next_actions.append(random_action)
                break
    return next_actions, next_poses                        

def PERFORM_CYCLIC_ACTIONS(node, intruder):
    time_e = intruder.time_e()
    time_p = intruder.time_p()
    intruder_pose = intruder.pose()
    
    #status of the node to be evaluated
    temp_time_step = node.time_step
    #temp_poses = node.poses 
    
    #temp_node = node.parent
    temp_node = node    
    head_mark_node = node.cyclic_head
    #arm_mark_node = node
    
    poses_buffer = []
    poses_buffer.append(temp_poses)
    while(not temp_node.node_equal(head_mark_node)):
        poses_buffer.apend(temp_node.poses)        
        temp_node = temp_node.parent
        
    index = 0    
    index_increase_flag = True
    while(True):
        win_flag = False
        for i in range(len(poses_buffer[index])):
            if poses_buffer[index][i] == intruder_pose:
                win_flag = True
                break
        if temp_time_step < time_e + time_p:
            if win_flag == True:
                return 1
        elif temp_time_step == time_e + time_p:        
            if win_flag == True:
                return 1
            elif win_flag == False:
                return 0
        
        if(index_increase_flag):
            index +=1
            temp_time_step+=1
            if index == len(poses_buffer):
                index-=1
                temp_time_step-=1
                index_increase_flag = False
        else if(not index_increase_flag):
            index -= 1
            temp_time_step+=1
            if index == -1
                index+=2
                index_increase_flag = True
    #TODO:下午先看这段代码正确性，１２月５日１０点１２分
            
def ROLLOUT(node, intruder):
    #有两个前提：
    #第一个前提是当前ｎｏｄｅ上层的ｎｏｄｅ一定不是终止节点，ｓｅｌｅｃｔ和ｒｏｌｌｏｕｔ的都不是终止节点，只有当前ｎｏｄｅ和之后的ｎｏｄｅ可能是终止节点。
    #第二个前提是当前ｎｏｄｅ是经过ｅｘｐａｎｄ选出来的唯一一个待评估节点。
    #status of the intruder
    time_e = intruder.time_e()
    time_p = intruder.time_p()
    intruder_pose = intruder.pose()
    #status of the node to be evaluated
    temp_time_step = node.time_step
    temp_actions = node.actions
    temp_poses = node.poses    
    
    next_actions = temp_actions
    next_poses = temp_poses
    
    if(node.is_cyclic_arm() == True):
　　　　　　　　return PERFORM_CYCLIC_ACTIONS(node, intruder)
    else:
        while(True):
            win_flag = False
    　　　　    for i in range(len(next_poses)):
    　　　　        if(next_poses[i] == intruder_pose):
    　　　　            win_flag = True
    　　　　            break　　　     
    　　　　    #if the intruder completing intruding or capture the intruder
    　　　　    if(temp_time_step < time_e + time_p):
    　　　　        if win_flag == True:
    　　　　            return 1
    　　　　    elif((temp_time_step == time_e + time_p):
    　　　　        if(win_flag == True):
    　　　　            return 1
    　　　　        else if(win_flag == False):
    　　　　            return 0
    　　　　    else:        
    　　　　        temp_time_step += 1
    　　　　        next_actions, next_poses = RANDOM_ROLLOUT_FOR_ONE_STEP(next_actions, next_poses)

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
        list_0[0] = "none"
        return list_0, list_1
        
#只需要node_0和node_1当中的所有的机器人的位置都有一一相等的，不用顺序相等。    　　　
def STATUS_EQUAL(node_0, node_1):
    poses_0 = node_0.poses
    poses_1 = node_1.poses
    if(len(poses_0) != len(poses_1)):
        return False
    else:    
        equal_flag = False
        while(True):
            poses_0, poses_1 = ONE_EQUAL_IN_LIST_DEL(poses_0, poses_1)
            if(len(poses_0) == 0 and len(poses_1) == 0):
                return True
            if(poses_0[0] == "none"):
                return False
    　　　　        
def BACKPROPAGATION(leaf_node, reward):
    leaf_node.update(reward)
    temp_node = leaf_node.parent
    while(temp_node != None):
        if(STATUS_EQUAL(temp_node, leaf_node)):
            leaf_node.create_cyclic_sibling_arm(cyclic_head = temp_node)#it is an arm
        else:
            temp_node = temp_node.parent
            temp_node.update(reward)   
        
if __name__=="__main__":
    map = Map()
    intruder = StationaryIntruder()
    #TODO:初始的机器人的位置是(1, 0), (1, 1)
    root = Node()
    #TODO:在这里少了一步，就是判断temp_node是不是终止节点，如果是终止节点，就不需要ｅｘｐａｎｄ了，直接去ｒｏｌｌｏｕｔ了，直接去ｂａｃｋｐｒｏｐａｇａｔｉｏｎ就行了，这个在主逻辑当中去判断。select和ｒｏｌｌｏｕｔ都要判断是不是终止节点。
    #TODO:2017年１５点４５分，明天开始缕清上面的各个函数，然后开始看主逻辑的代码。
    #node = Node([0,0])
    #child = node.add_child([1,1])
    #print child.parent.actions        
    #print map.map
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
