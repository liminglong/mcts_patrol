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
        for(i in range(self.horizontal_length))
            for(j in range(self.vertical_length))
                if(self.map[i, j] == 1)
                    temp_obstacles.append([i,j])
        return temp_obstacles
        
    def random_pose_without_obstacles():
        obstacles = self.obstacles_in_map()
        while(True)
            temp_x = random.randint(0, self.horizontal_length-1)
            temp_y = random.randint(0, self.vertical_length-1)
            obstacle_flag = False
            for i in range(len(obstacles))
                if(obstacles[i] == [temp_x, temp_y])
                    obstacle_flag = True
                    break
            if(obstacle_flag == False)
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
        if(time_step == None)
            time_step = 0#default parameter
        if(map == None)
            map = Map()#default parameter      
    
    def check_and_add_child(self,child_actions=[]):
        #temp_actions = []#temp_actions是个一维数组，维度是robot_num
        temp_poses = []#temp_poses是个二维数组,第一个维度是robot_num    
        for(i = 0; i < ROBOT_NUM; i+=1)
            for(j = 0; j < 4; j+=1)#up, down, left, right
                if(j == 0)#up
                    temp_poses[i][0] = self.poses[0] 
                    temp_poses[i][1] = self.poses[1] - 1
                if(j == 1)#down
                    temp_poses[i][0] = self.poses[0] 
                    temp_poses[i][1] = self.poses[1] + 1
                if(j == 2)#left
                    temp_poses[i][0] = self.poses[0] - 1
                    temp_poses[i][1] = self.poses[1] 
                if(j == 3)#right
                    temp_poses[i][0] = self.poses[0] + 1
                    temp_poses[i][1] = self.poses[1]                                   
        bool bump_flag = False#bump into obstacles, or bump into boundings
        for(i = 0; i < ROBOT_NUM; i+=1)                  
            if(temp_poses[i][0] == -1 or temp_poses[i][0] == self.map.horizontal_length)#bump into left or right boundings
                bump_flag = True
                break
            if(temp_poses[i][1] == -1 or temp_poses[i][1] == self.map.vertical_length)#bump into up or down boundings
                bump_flag = True
                break
            else
                obstacles = self.map.obstacles_in_map()#a list storing the coordinates of the obstacles
                for(j in range(len(obstacles)))
                    if(temp_poses[i][0] == obstacles[j][0] and temp_poses[i][1] == obstacles[j][1])
                        bump_flag == True
                        break
                break
        if(not bump_flag)
            child.time_step = self.time_step + 1
            child = Node(actions = child_actions, poses = temp_poses, parent = self, time_step = self.time_step + 1)
            self.children.apend(child)       
        
    def update(self,reward):
        self.reward+=reward
        self.visits+=1
    
    def ucb(self):
        visits_parent = self.parent.visits
        visits_child = self.visits
        w_pi = self.reward
        return w_pi + SCALAR * math.sqrt(math.log(visits_parent/visits_children))
    
    def create_cyclic_sibling_arm(self):
        pass
    
    def is_cyclic_arm(self):
        return self.is_cyclic_arm
    
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
    while(len(temp_children) != 0)
        for(i = 0; i < len(temp_children); i+=1)
            if(temp_ucb < temp_chiledren[i].ucb)
                temp_ucb = temp_children[i].ucb
                temp_node = temp_children[i]
        temp_children = temp_node.children
    #TODO:在这里少了一步，就是判断temp_node是不是叶子节点，如果是叶子节点，就不需要ｅｘｐａｎｄ了，直接去ｒｏｌｌｏｕｔ了，直接去ｂａｃｋｐｒｏｐａｇａｔｉｏｎ就行了。
    return temp_node   
  
def ITERATIVE_LOOP(temp_node, cycles):
    cycles -= 1
    index = ROBOT_NUM - cycles -1
    if(cycles==-1)
        return 
    else
        if(cycles == 0)
            for i in range(4):
                temp_actions[index] = i
                node.add_child(child_actions = temp_actions);
        else
            for i in range(4):
                temp_actions[index] = i
                ITERATIVE_LOOP(temp_node, cycles)
#fully expand the node        
def EXPAND(node):
    ITERATIVE_LOOP(node, ROBOT_NUM)    
    #TODO:这里应该少一个步骤，ｅｘｐａｎｄ了很多节点之后，要选择这些中的一个返回，作为一个待评估节点。
　　　　#expand之后是怎么选择的一个呢？  
　　　　#node = Node(), intruder = StationaryIntruder    

def RANDOM_ROLLOUT_FOR_ONE_STEP(actions, poses):
    map = Map()
    current_actions = actions
    current_poses = poses
    
    next_actions = []
    next_poses = []
    
    obstacles =  map.obstacles_in_map()
        
    for i in range(ROBOT_NUM)
        if(current_actions[i] == 0)#up
            next_poses.append( [current_poses[i][0], current_poses[i][1]-1] )
        if(current_actions[i] == 1)#down
            next_poses.append( [current_poses[i][0], current_poses[i][1]+1] )                  
        if(current_actions[i] == 2)#left
            next_poses.append( [current_poses[i][0]-1, current_poses[i][1]] )               
        if(current_actions[i] == 3)#right
            next_poses.append( [current_poses[i][0]+1, current_poses[i][1]] )        
                   
    for i in range(ROBOT_NUM)
        while(True)
            temp_poses = [0,0]
            random_action = random.randint(0, 3)
            if(random_action == 0)#up
                temp_poses[0] = next_poses[i][0]
                temp_poses[1] = next_poses[i][1] - 1
            if(random_action == 1)#down
                temp_poses[0] = next_poses[i][0]
                temp_poses[1] = next_poses[i][1] + 1   
            if(random_action == 2)#left
                temp_poses[0] = next_poses[i][0] - 1
                temp_poses[1] = next_poses[i][1]         
            if(random_action == 3)#right
                temp_poses[0] = next_poses[i][0] + 1
                temp_poses[1] = next_poses[i][1]         
            obstacle_flag == False
            for j in range(len(obstacles))
                if(temp_poses == obstacles[j])
                    obstacle_flag = True
                    break
            if(obstacles_flag == False)
                next_actions.append(random_action)
                break
    return next_actions, next_poses                        
            
def ROLLOUT(node, intruder):
    #有两个前提：
    #第一个前提是当前ｎｏｄｅ上层的ｎｏｄｅ一定不是终止节点，只有当前ｎｏｄｅ和之后的ｎｏｄｅ可能是终止节点。
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
    
    if(node.is_cyclic_arm() == True)
        #TODO:perform_cyclic_actions() 
        #TODO:如果在ｒｏｌｌｏｕｔ当中构造了这个方法，那么就直接调用就好了。
        win_flag = node.perform_cyclic_actions()
        if(win_flag)
            return 1
        else
            return 0
    else
        while(True)
            win_flag = False
    　　　　    for i in range(len(next_poses))
    　　　　        if(next_poses[i] == intruder_pose)
    　　　　            win_flag = True
    　　　　            break　　　     
    　　　　    #if the intruder completing intruding or capture the intruder
    　　　　    if((temp_time_step == time_e + time_p)or(win_flag = True))
    　　　　        if(win_flag)
    　　　　            return 1
    　　　　        else 
    　　　　            return 0
    　　　　    else        
    　　　　        temp_time_step += 1
    　　　　        next_actions, next_poses = RANDOM_ROLLOUT_FOR_ONE_STEP(next_actions, next_poses)

#只需要node_0和node_1当中的所有的机器人的位置都有一一相等的，不用顺序相等。    　　　　        
def STATUS_EQUAL(node_0, node_1):
    pass
        　　　　        
    　　　　        
def BACKPROPAGATION(leaf_node, reward):
    leaf_node.update(reward)
    temp_node = leaf_node.parent
    while(temp_node != None)
        if(STATUS_EQUAL(temp_node, leaf_node))
            leaf_node.create_cyclic_sibling_arm()#TODO: to be complished
        else
            temp_node = temp_node.parent
            temp_node.update(reward)   
        
if __name__=="__main__":
    map = Map()
    #node = Node([0,0])
    #child = node.add_child([1,1])
    #print child.parent.actions        
    #print map.map
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
