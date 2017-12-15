#!/usr/bin/env python
# coding=utf-8
import numpy as np
import random
import math
ROBOT_NUM = 2
GLOBAL_COUNT = 0
COUNT = 0
CHILDREN = []
import copy
class Map():
    def __init__(self):
        self.map = [[0, 0, 1, 0, 0],
                    [0, 0, 1, 0, 0],
                    [1, 0, 0, 0, 1],
                    [0, 1, 0, 1, 0],
                    [0, 0, 0, 0, 0]]
        self.horizontal_length = 5
        self.vertical_length = 5
     
    def obstacles_in_map(self):
        temp_obstacles = []
        for i in range(self.horizontal_length):
            for j in range(self.vertical_length):
                if(self.map[i][j] == 1):
                    temp_obstacles.append([i,j])
        return temp_obstacles


class Node():
    def __init__(self, actions = None, poses = None, parent = None, time_step = None, map = None):
        self.visits = 1
        self.reward = 0.0
        self.children = []
        self.actions = actions
        self.poses = poses
        self.parent = parent	
        self.time_step = time_step
        self.is_cyclic_arm = False
        self.cyclic_head = None
        if(time_step == None):
            time_step = 0#default parameter
        self.map = map
        if(self.map == None):
            self.map = Map()#default parameter      
    
    def add_child(self, child):
        self.children.append(child)     

    def check_and_add_child_actions_poses(self, child_actions, child_poses):
        global COUNT
        global CHILDREN
        grandson_poses = NEXT_POSES(child_actions, child_poses)
        bump_flag = self.check_bump(grandson_poses)
        if(not bump_flag):
            #print child_actions
            #print child_poses
            temp_child_actions = copy.deepcopy(child_actions)
            temp_child_poses = copy.deepcopy(child_poses)
            child = Node(actions = temp_child_actions, poses = temp_child_poses, parent = self, time_step = self.time_step + 1)
            '''
            print "child.poses"
            print child.poses
            print "child.actions"
            print child.actions
            '''
            '''
            print id(child)
            temp_child = copy.deepcopy(child)
            print id(temp_child)
            CHILDREN.append(temp_child)
            '''
            CHILDREN.append(child)
            print "CHILDREN poses: "
            print CHILDREN[COUNT].poses
            print "CHILDREN actions"
            print CHILDREN[COUNT].actions
            
            #self.children[COUNT] = child
            #CHILDREN.insert(COUNT, child)
            #self.children.append(child)
            COUNT+=1

    
    def check_bump(self, poses):
        #bump_flag = False#bump into obstacles, or bump into boundings
        for i in range(ROBOT_NUM):
            if(poses[i][0] == -1 or poses[i][0] == self.map.horizontal_length):#bump into left or right boundings
                return True
            if(poses[i][1] == -1 or poses[i][1] == self.map.vertical_length):#bump into up or down boundings
                return True
            else:
                obstacles = self.map.obstacles_in_map()#a list storing the coordinates of the obstacles
                for j in range(len(obstacles)):
                    if poses[i] == obstacles[j]:
                        return True
        return False


     

#root节点的初始化当中，action为空，poses是初始化指定的值。
def ITERATIVE_LOOP_ROOT_ACTION(root, cycles, root_actions):
    cycles -= 1
    index = ROBOT_NUM - cycles - 1
    if(cycles == -1):
        return
    else:
        if(cycles == 0):
            for i in range(4):
                root_actions[index] = i
                child_poses = NEXT_POSES(root_actions, root.poses)
                bump_flag = root.check_bump(child_poses)
                #print "bump_flag"
                #print bump_flag
                if bump_flag == True:
                    pass#检查合法性，如果不合法，continue。
                elif(bump_flag == False):
                    child_actions = []
                    for i in range(ROBOT_NUM):
                        child_actions.append(-1)
                    temp_child_actions = copy.deepcopy(child_actions)
                    ITERATIVE_LOOP_ROOT_CHILD_ACTION(root=root, cycles=ROBOT_NUM, child_poses=child_poses,child_actions = temp_child_actions)
        else:
            for i in range(4):
                root_actions[index] = i
                ITERATIVE_LOOP_ROOT_ACTION(root, cycles, root_actions)

def ITERATIVE_LOOP_ROOT_CHILD_ACTION(root, cycles, child_poses, child_actions):
    cycles -= 1
    index = ROBOT_NUM - cycles - 1

    if cycles == -1:
        return 
    else:
        if cycles == 0:
            for i in range(4):
                child_actions[index] = i
                '''
                print "child_poses: "
                print child_poses
                print "child_actions:"
                print child_actions
                '''
                root.check_and_add_child_actions_poses(child_actions, child_poses)#####TODO：这一句肯定不对。            
        else:
            for i in range(4):
                child_actions[index] = i
                ITERATIVE_LOOP_ROOT_CHILD_ACTION(root, cycles, child_poses, child_actions)

def EXPAND_ROOT(root):
    root_actions = []
    for i in range(ROBOT_NUM):
        root_actions.append(-1)
    temp_root_actions = copy.deepcopy(root_actions)
    ITERATIVE_LOOP_ROOT_ACTION(root, ROBOT_NUM, temp_root_actions)
    #i = random.randint(0, len(root.children)-1)
    #return root.children[i]            
    
def NEXT_POSES(actions, poses):
    child_poses = []
    for index in range(ROBOT_NUM):
        child_poses.append([0,0])
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

if __name__=="__main__":
    map = Map()
    root = Node(poses = [[1,0],[1,1]], parent = None, time_step = 0, map = map)
    #root.children = []
    print "root.poses: "
    print root.poses
    print "len(root.children):"
    print len(root.children)
    print "Hello 1"
    EXPAND_ROOT(root)   
    print len(root.children)
    print len(CHILDREN)
    #print GLOBAL_COUNT
    for i in range(len(CHILDREN)):
        print CHILDREN[i].poses  
        print CHILDREN[i].actions    
    '''
    for i in range(len(root.children)):
        print root.children[i].poses  
        print root.children[i].actions
    '''

   
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
