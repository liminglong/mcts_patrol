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

    def change_scale(self):
        pass
    
    def add_obstacle(self):
        pass
    
#actions = [up, down, left, right ... ... ], the lenth of the actions is the robot number, 0 represents 'can't move', 1 represents 'can move'.    
class Node():
    def __init__(self, actions=None, poses = None, parent=None):
        self.visits = 1
        self.reward = 0.0	
        self.children = []
        self.parent = parent	
        self.actions = actions
        self.poses = poses
        self.time_step = 0
    
    def add_child(self,child_actions=[], child_poses = [], parent = None):
        child=Node(actions = child_actions, poses = child_poses, parent = self)
        child.parent = self#TODO:这里可以加可以不加，因为后面在ｅｘｐａｎｄ的时候可以加入ｐａｒｅｎｔ
        self.children.append(child)
        return child
        
    def update(self,reward):
        self.reward+=reward
        self.visits+=1
    
    def is_fully_expanded(self):
        pass
        '''
        if len(self.children)==self.state.num_moves:
            return True
        return False
        '''        
    
    def ucb(self):
        visits_parent = self.parent.visits
        visits_child = self.visits
        w_pi = self.reward
        return w_pi + SCALAR * math.sqrt(math.log(visits_parent/visits_children))
    
    def create_cyblic_arm(self):
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
    return temp_node   
    
    
#全局变量 
temp_actions = []   
#最大为一个robot_num,cycles = robot_num, index = -1

cycles = ROBOT_NUM    
    
def ITERATIVE_LOOP(temp_node, cycles):
    cycles -= 1
    index = ROBOT_NUM - cycles -1
    if(cycles==-1)
        return 
    else
        if(cycles == 0)
            for i in range(4):
                temp_actions[index] = i
                node.add_child(child_actions = temp_actions);#TODO:限制和ｐｏｓｅ的加入，都放在add_child里面
        else
            for i in range(4):
                temp_actions[index] = i
                ITERATIVE_LOOP(temp_node, cycles)
        
def                
#循环展开,假设机器人数目为三个,cycles=3, index = 0
'''

    for i in range(4)
        temp_actions[0] = i
        for j in range(4)
            temp_actions[1] = j
            for k in range(4)
                temp_actions[2] = k
                node.add_child(actions = temp_actions)
                
   
                 
    
'''        

def EXPAND(node):
    ITERATIVE_LOOP(node, ROBOT_NUM)
    
    
#expand based on leaf node    
def EXPAND(node):
    #pass
    temp_node = node

    #先看完全没有障碍物没有边界，也不考虑多机器人之间的碰撞。
    for(i = 0; i < robot_num; i+=1)#robot_0, robot_1
        temp_actions = []#temp_actions是个一维数组，维度是robot_num
        temp_poses = []#temp_poses是个二维数组,第一个维度是robot_num    
        for(j = 0; j < 4; j+=1)#up, down, left, right                     
            temp_actions[i] = j             
            if(j == 0)#up
                temp_poses[i][0] = node.poses[0] 
                temp_poses[i][1] = node.poses[1] - 1
            if(j == 1)#down
                temp_poses[i][0] = node.poses[0] 
                temp_poses[i][1] = node.poses[1] + 1
            if(j == 2)#left
                temp_poses[i][0] = node.poses[0] - 1
                temp_poses[i][1] = node.poses[1] 
            if(j == 3)#right
                temp_poses[i][0] = node.poses[0] + 1
                temp_poses[i][1] = node.poses[1]                     
            #TODO:暂时不考虑碰撞，如果考虑碰撞，就在这里加上。
            if(not(temp))
            temp_node.add_child(actions = temp_actions, poses = temp_poses, parent = node)         
            
   temp_node = node         
   for(i = 0; i < 4; i+=1)#up, down, left, right
       temp_actions = []
       for(j = 0; j < robot_num; j+=1)
           temp_actions[j] = i       
       if(not(碰撞边界或者障碍物))
           temp_node.add_child(actions = temp_actions, parent = node)
                 
   
    
def ROLLOUT(node):
    pass
    
def BACKPROP(node):
    pass
        
if __name__=="__main__":
    map = Map()
    #node = Node([0,0])
    #child = node.add_child([1,1])
    print child.parent.actions    
    #print map.map
