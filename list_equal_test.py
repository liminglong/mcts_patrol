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
        #print "hello none"
        list_0[0] = "none"
        #list_1[1] = "none"
        return list_0, list_1

def STATUS_EQUAL(list_0, list_1):
    poses_0 = list_0
    poses_1 = list_1
    if(len(poses_0) != len(poses_1)):
        return False
    else:    
        equal_flag = False
        while(True):
            #print "hello1"
            #print poses_0
            poses_0, poses_1 = ONE_EQUAL_IN_LIST_DEL(poses_0, poses_1)
            #print "hello2"
            #print poses_0
            if(len(poses_0) == 0 and len(poses_1) == 0):
                return True
            if(poses_0[0] == "none"):
                return False

            
    

if __name__=="__main__":
    list_0 = [1, 1, 0]
    list_1 = [1, 1, 1]
    print STATUS_EQUAL(list_0, list_1)
    
