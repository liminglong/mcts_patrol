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

ROBOT_NUM = 2;
#MCTS scalar.  Larger scalar will increase exploitation, smaller will increase exploration. 
SCALAR= math.sqrt(2.0)
GLOBAL_ID = 0 #维护一个全局ＩＤ，每个ｎｏｄｅ有且仅有一个独立ＩＤ，包括cyclic_node.

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
        self.time_e = random.randint(2,10)#intruder出现的时刻，从第3个时间步开始计算，０，１时刻，ｉｎｔｒｕｄｅｒ机器人肯定不会出现。
        #从直觉上讲，time_p越大，在一定budget内，捕捉到intruder的概率越大，作者最小设置为９
        self.time_p = 9#intruder停留的时间为10个时间步。
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
        
        if(time_step == None):
            time_step = 0#default parameter
        self.map = map
        if(self.map == None):
            self.map = Map()#default parameter      
    
    #已经检查过的child
    def add_child(self, child):
        global GLOBAL_ID
        temp_child = copy.deepcopy(child)
        temp_child.node_ID = GLOBAL_ID
        self.children.append(temp_child) 
        GLOBAL_ID += 1    
    
    #根据当前节点的actions和poses算出child_poses,　child_actions经过传参得到,再算出grand_son，检查合法性。
    def check_and_add_child_actions(self,child_actions):
        global ROBOT_NUM
        global GLOBAL_ID
        #temp_actions = []#temp_actions是个一维数组，维度是robot_num
        child_poses = []#child_poses是个二维数组,第一个维度是robot_num
        #子节点的位置是由当前节点的动作和位置决定的，子节点对应的各个机器人动作可以指定。    
        
        child_poses = NEXT_POSES(copy.deepcopy(self.actions), copy.deepcopy(self.poses))
        '''
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
        '''             
        #因为self节点的上一层节点在add_child的时候，会检测碰撞，所以，当前节点的poses不会有碰撞。  
        #但是，根据child_poses和指定的child_actions加入grandson节点的poses，就会可能碰撞
        grandson_poses = []
        
        grandson_poses = NEXT_POSES(copy.deepcopy(child_actions), copy.deepcopy(child_poses))
        '''
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
        '''
        #child节点的位置已经是无碰撞的了，需要用grandson的位置判断是否加入child节点。        
        bump_flag = self.check_bump(copy.deepcopy(grandson_poses))  
        if(not bump_flag):
            temp_child_actions = copy.deepcopy(child_actions)
            temp_child_poses = copy.deepcopy(child_poses)
            child = Node(actions = temp_child_actions, poses = temp_child_poses, parent = self, time_step = self.time_step + 1)
            child.node_ID = GLOBAL_ID
            self.children.append(child)
            GLOBAL_ID +=1

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
            GLOBAL_ID += 1
    
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
        return w_pi + SCALAR * math.sqrt(math.log(visits_parent/visits_child))
    
    def create_cyclic_sibling_arm(self, cyclic_head_ID):
        global GLOBAL_ID
        temp_parent = self.parent
        new_node = copy.deepcopy(self)
        new_node.cyclic_head_ID = cyclic_head_ID
        new_node.is_cyclic_arm = True
        new_node.visits = 1
        new_node.node_ID = GLOBAL_ID
        temp_parent.children.append(new_node)
        GLOBAL_ID += 1
    
    def get_is_cyclic_arm(self):
        return self.is_cyclic_arm
        
    def node_equal(self, other):
        self_node_ID = self.node_ID
        other_node_ID = other.node_ID
        
        if(self_node_ID == other_node_ID):
            return True
        else:
            return False
  
    
    def is_fully_expanded(self):
        #To be verified: Version 0 of mucts_patrol, all of the nodes are fully expanded
        pass       

#global functions
#UCT search
#def SELECT(budget, root):
#TODO:关于深度拷贝的检查，看到了这里。
def SELECT(root):
    #pass
    selected_node = Node()
    global_ucb = 0
    temp_ucb = 0
    temp_children = root.children #'children' is a list
    #print "244: SELECT 1"
    while(len(temp_children) != 0):
        #print "246: SELECT while"
        for i in range(len(temp_children)):
            '''
            print "248: SELECT for"
            print "temp_ucb: "
            print temp_children[i].parent.visits
            print temp_children[i].visits
            print temp_children[i].reward
            '''            
            temp_ucb = temp_children[i].ucb()
            #print temp_ucb
            if(global_ucb < temp_ucb or global_ucb == temp_ucb):
                #print "SELECT if"
                global_ucb = temp_ucb
                selected_node = temp_children[i]
        #如果始终找不到比当前ｕｃｂ还小的，那么temp_children应该就是不发生变化的？
        #print "SELECT 2"        
        temp_children = selected_node.children#TODO:还是这里存在问题，一直下去肯定是一个Ｎｏｎｅ。２１点４２分。
        #len(temp_children)　等于０的时候不就跳出了吗？此时Ｓｅｌｅｃｔｅｄ应该不是０啊。
        global_ucb = 0#TODO:这里对吗？很有可能是个雷啊。
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
                temp_node.check_and_add_child_actions(child_actions = temp_actions);
        else:
            for i in range(4):
                temp_actions[index] = i
                ITERATIVE_LOOP(temp_node, cycles, temp_actions)
                              
#fully expand the node
def EXPAND(node):
    global ROBOT_NUM
    temp_actions = []
    for i in range(ROBOT_NUM):
        temp_actions.append(-1)   
    ITERATIVE_LOOP(node, ROBOT_NUM, temp_actions)
    i = random.randint(0, len(node.children)-1)
    return node.children[i]

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
                root.check_and_add_child_actions_poses(child_actions, child_poses)            
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

def EXPAND_ROOT(root):
    global ROBOT_NUM
    root_actions = []
    for i in range(ROBOT_NUM):
        root_actions.append(-1)
    ITERATIVE_LOOP_ROOT_ACTION(root, ROBOT_NUM, root_actions)
    i = random.randint(0, len(root.children)-1)
    return root.children[i]

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
            obstacle_flag = False
            for j in range(len(obstacles)):
                if(temp_poses == obstacles[j]):
                    obstacle_flag = True
                    break
            if(obstacle_flag == False):
                next_actions.append(random_action)
                break
    return next_actions, next_poses                        

def PERFORM_CYCLIC_ACTIONS(node, intruder):
    time_e = intruder.get_time_e()
    time_p = intruder.get_time_p()
    intruder_pose = intruder.get_pose()
    
    #status of the node to be evaluated
    temp_time_step = node.time_step
    #temp_poses = node.poses 
    
    #temp_node = node.parent
    temp_node = node    
    head_mark_node_ID = node.cyclic_head_ID
    #指针赋值＃TODO: TO BE REVISED>
    head_mark_node = node
    #arm_mark_node = node
    
    poses_buffer = []
    poses_buffer.append(temp_poses)
    while(not temp_node.node_equal(head_mark_node)):
        poses_buffer.append(temp_node.poses)        
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
        return PERFORM_CYCLIC_ACTIONS(node, intruder)
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
            if(temp_time_step < time_e + time_p):
                if win_flag == True:
                    return 1
                elif win_flag == False:
                    #侵入完成时间也没到，机器人也没找着，继续搜索,TODO:一个idea，是否把找到的时间也算在内呢？
                    temp_time_step += 1
                    next_actions, next_poses = RANDOM_ROLLOUT_FOR_ONE_STEP(next_actions, next_poses)
            elif(temp_time_step == time_e + time_p):#入侵完成时间到了
                if(win_flag == True):
                    return 1
                elif(win_flag == False):
                    return 0


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
    global ROBOT_NUM
    poses_0 = []
    poses_1 = []
    for i in range(ROBOT_NUM):
        poses_0.append(node_0.poses[i])
        poses_1.append(node_1.poses[i])
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
            if(poses_0[0] == "none"):
                return False
                #TODO:对应的ｉ个机器人的概念在这里没有考虑，索引号没有加进来。
                
def BACK_PROPAGATION(leaf_node, reward):
    leaf_node.update(reward)
    temp_node = leaf_node.parent

    while(True):
        if(STATUS_EQUAL(temp_node, leaf_node)):
            temp_node_ID = temp_node.node_ID
            leaf_node.create_cyclic_sibling_arm(cyclic_head_ID = temp_node_ID)#it is an arm
        temp_node.update(reward)
        temp_node = temp_node.parent
        if temp_node == None:
            break

def IS_INTRUDER_CAPTURED(node, intruder):
    temp_poses = node.poses;
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

        
if __name__=="__main__":
    map = Map()
    intruder = StationaryIntruder()
    BUDGET = 100000;
    reward = 0.0;
    #初始的机器人的位置是(1, 0), (1, 1)
    root = Node(actions = None, poses = [[1,0],[1,1]], parent = None, time_step = 0, map = map)
    root.node_ID = -1    
    print "Hello 1"
    EXPAND_ROOT(root)   
    
    #print len(root.children)
    print len(root.children)  
    
    cycle_count = 0
    print "Hello 2"
    while(cycle_count < BUDGET):
        print "Hello 3"
        leaf_node = SELECT(root)#TODO:这里需要判断node是不是时间终止节点。
        '''
        print "639, leaf_node.actions: "
        print leaf_node.actions
        print "641, leaf_node.poses: "
        print leaf_node.poses
        '''    
        print "Hello 4"
        if leaf_node.time_step == intruder.time_e + intruder.time_p:
            print "Hello 5"
            if IS_INTRUDER_CAPTURED(leaf_node, intruder) == True:
                reward = 1
            else:
                reward = 0
            BACK_PROPAGATION(leaf_node, reward)    
        elif leaf_node.time_step < intruder.time_e + intruder.time_p:    
            print "Hello 6"
            if leaf_node.get_is_cyclic_arm():
                print "Hello 7"
                reward = ROLLOUT(leaf_node, intruder)
                BACK_PROPAGATION(leaf_node, reward)
            else:
                
                print "Hello 8"
                
                print "664, leaf_node.actions: "
                print leaf_node.actions
                print "666, leaf_node.poses: "
                print leaf_node.poses
                

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
        intruder.update_initial_pose()
        print "cycle_count :"
        print cycle_count
        cycle_count += 1
        

    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
