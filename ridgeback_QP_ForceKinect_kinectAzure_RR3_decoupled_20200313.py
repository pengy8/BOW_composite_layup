# -*- coding: utf-8 -*-
"""
Created on Fri Mar 13 10:44:52 2020

@author: d0v0b
"""
#!/usr/bin/env python
import rospy
import numpy as np
import scipy.io as sio
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
#from scipy.spatial.transform import Rotation
from tf.transformations import quaternion_matrix
import sys
sys.path.append('/home/cats/rr/robotraconteur/build/out/Python')
from RobotRaconteur.Client import *
from QuadProg_BOW import QP_bow
#from QuadProg_SingleArm import QP_BaxterArm
from baxter_info import *
import time
import timeit


ridgeback_state = 0
pose = None
joint_state = None



class ridgeback():

    def __init__(self):
        self.velocity_publisher = rospy.Publisher('/ridgeback_velocity_controller/cmd_vel', Twist, queue_size=50)
        self.odom_subscriber = rospy.Subscriber('/ridgeback_velocity_controller/odom',Odometry,self.odometryCb)#odometry/filtered
        self.rate = rospy.Rate(50)
        
    def unpack_odom(self, msg):
        """
        Converts an Odometry message into a state vector.
        from https://www.programcreek.com/python/example/95996/nav_msgs.msg.Odometry
        """
        q = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        twist = [msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.angular.z]
        return np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
                         
    def odometryCb(self,msg):
        global ridgeback_state
        ridgeback_state = self.unpack_odom(msg)


def bodytracker(pipe_ep):
    global pose 
    global joint_state
    
    data=pipe_ep.ReceivePacket()
    joint_state = data.joint_tracking_state
    P=[]
    for p in (data.joint_poses):
        P.append([p[1][0],p[1][1],p[1][2]])

    pose = np.array(P)/1000

        
def collision_check(pp_L,pp_R):
    
    body_radius = 0.35
    Closest_Pt_l = np.array([1000,1000,1000])
    Closest_Pt_env_l = np.array([0,0,0])
    Closest_Pt_r = np.array([0,0,0]) 
    Closest_Pt_env_r = np.array([1000,1000,1000])
    
    d_min = 1000
        
    for l in pp_L.T:
        for r in pp_R.T:
            dist = np.linalg.norm(l-r)
            if dist < d_min:
                d_min = dist
                Closest_Pt_l = l
                Closest_Pt_env_l = r
                Closest_Pt_r = r
                Closest_Pt_env_r = l     
                
    if (np.linalg.norm(pp_L[0:2,-1])-body_radius) < d_min:
        Closest_Pt_l = pp_L[:,-1]
        scale = body_radius/np.linalg.norm(pp_L[0:2,-1])
        Closest_Pt_env_l = np.array([scale*pp_L[0,-1],scale*pp_L[1,-1],pp_L[2,-1]])       

    if (np.linalg.norm(pp_R[0:2,-1])-body_radius) < d_min:
        Closest_Pt_r = pp_R[:,-1]
        scale = body_radius/np.linalg.norm(pp_R[0:2,-1])
        Closest_Pt_env_r = np.array([scale*pp_R[0,-1],scale*pp_R[1,-1],pp_R[2,-1]])
        
    return Closest_Pt_l , Closest_Pt_env_l, Closest_Pt_r , Closest_Pt_env_r



def stage0():
    
    print ("stage 0: ")
    claps = 0    
    while not claps:     

        if pose is not None:
            human_R = pose[iR,:]
            human_L = pose[iL,:]            
            claps_norm = np.linalg.norm(human_R-human_L)
            print ('Claps Norm:', claps_norm)           
            if claps_norm <0.1: 
                claps = 1
    print ('3')
    time.sleep(1)
    print ('2')
    time.sleep(1)
    print ('1')
    time.sleep(1)
    print ('Start!')
    
    return 0
 
def stageI(baxter):
    print ("Stage 1: ")
    return 1
 
def stageII():
    print ("Stage 2")
    return 2
 
switcher = {
        0: stage0,
        1: stageI,
        2: stageII
    }
 
 


if __name__ == '__main__': 
    
    baxter = RRN.ConnectService('tcp://localhost:1111/BaxterJointServer/Baxter')
    gripper = RRN.ConnectService('tcp://localhost:2222/BaxterPeripheralServer/BaxterPeripherals')
    kinect = RRN.ConnectService('tcp://192.168.131.151:8888/sensors.kinect2/KinectTracker')
    
    
    kinect.active_body_tracking=True   
    K=kinect.kinect_body_sensor_data.Connect(-1)
    K.PacketReceivedEvent+=bodytracker

    
#    gripper.calibrateGripper('left')
#    gripper.calibrateGripper('right')
    gripper.closeGripper('Left')
    gripper.closeGripper('Right')
#    gripper.openGripper('Left');
#    gripper.openGripper('Right');
    
    Bdef = defineBaxter()    
    q0 = np.array([0.77350981,  0.26461169, -1.40589339,  2.07777698,  0.33594179, -0.49547579, -0.03298059]) 
    q0L = q0
    q0R = q0*[-1,1,-1,1,-1,1,-1]
    qL = q0L
    qR = q0R
    
    baxter.setControlMode(0)
    baxter.setJointCommand('left',q0L)
    baxter.setJointCommand('right',q0R)  

    iL = 14
    iR = 7 

    try:
        rospy.init_node('ROS_ridgeback_and_kinect', anonymous=True)
        ridgeback = ridgeback()
        print (rospy.is_shutdown())
        print (ridgeback_state)
        ridgeback_vel_msg = Twist()
        ridgeback_vel_msg.linear.x = 0
        ridgeback_vel_msg.linear.y = 0
        ridgeback_vel_msg.linear.z = 0
        ridgeback_vel_msg.angular.x = 0
        ridgeback_vel_msg.angular.y = 0
        ridgeback_vel_msg.angular.z = 0 
        
        rospy.sleep(0.025)

        
    except rospy.ROSInterruptException: pass

    Select = input("Press 1 to continue...")  


    # ============================== Program Starts ==============================
    if Select == 3:
        
        
        process = stage0()#switcher.get(0, "nothing")

        baxter.setControlMode(1)
        
#        er = 1
#        ep = 1
        
        K_com = 0.015
        GraspForce = np.array([0,0,0,6,-6,0,0,0,0,6,6,0])
        ez = np.array([0,0,1])
        dt = 0.05
               
        
        # Kalman Filter Parameters
        R_K2B = np.array([[0,0,1],[-1,0,0],[0,-1,0]])
        Rk = np.eye(3)
        pk = np.array([0,0,0])
        r = 3*np.array([0.0137331,  0.02264715, 0.01016529, 0.00852387, 0.02615293, 0.00723866])          
        Phi = np.vstack([np.hstack([np.eye(6), dt*np.eye(6)]), np.hstack([np.zeros([6,6]),np.eye(6)])])  
        P = np.eye(12)
        Q = np.vstack([np.hstack([dt**2*np.diag(r**2),np.zeros([6,6])]), np.hstack([np.zeros([6,6]),dt**2*np.diag(r**2)])])
        M = np.hstack([np.eye(6),np.zeros([6,6])])
        R = dt*np.diag(r**2)    
      
        dqB_pre = np.array([0,0,0])
        dq_pre = np.zeros([17,1]) 
        Lambda = 50      # Lambda: weighting on dual arm motion
        Epsilon1 = 1    # Epsilon1: alpha close to 1
        Epsilon2 = 50   # Epsilon2: q close to previous     
        
        r_ini = quaternion_matrix(ridgeback_state[-4:])[0:3,0:3]
        p_ini = ridgeback_state[0:3]
        human_head0 = pose[26,:]/np.linalg.norm(pose[26,:])
                
        command_claps = 0
        i = 0
        
        p_desired = np.array([2.1, 1.1, 0.])        
        
        t_all = []
        VT_all = []
        wr_all = []
        q_all = []
        pose_all = []
        Xk_all = []
        Z_all = []
        Z = np.zeros([6,1])
        Xk = np.zeros([12,1])       
        qB = np.zeros([1,3])
        while not command_claps:

            if not rospy.is_shutdown() :#and (pose is not None):
                
                tic = timeit.default_timer()
                i +=1
                print ('iteration:', i)
                
                VT = np.zeros([12,1])
                q_read = baxter.joint_positions
                qL = q_read[0:7]
                qR = q_read[7:14]                
                q = np.hstack([qL,qR,np.array([0,0,0])]).reshape([17,1])
                
                t0 = timeit.default_timer()-tic
                
                JL = robotjacobian(Bdef.left.H, Bdef.left.P, Bdef.left.joint_type, qL)
                JR = robotjacobian(Bdef.right.H, Bdef.right.P, Bdef.right.joint_type, qR)
                
                pp_L,RR = fwdkin_alljoints(qL, Bdef.left.joint_type, Bdef.left.H, Bdef.left.P, 7)
                p_BTL = pp_L[:, -1]
                pp_R,RR = fwdkin_alljoints(qR, Bdef.right.joint_type, Bdef.right.H, Bdef.right.P, 7)
                p_BTR = pp_R[:, -1]
                JT = getJT(JL,JR,p_BTL,p_BTR)              

           
                ### Force Control
                wr_read = baxter.endeffector_wrenches
                wrL = wr_read[0:6]
                wrR = wr_read[6:12]

                h_LR = p_BTR-p_BTL
                h_LR_norm = h_LR/np.linalg.norm(h_LR)
                h_LRv = np.cross(h_LR_norm,ez)
                vL_force = -K_com*(np.dot(h_LR_norm[0:2],wrL[3:5])-GraspForce[4])*h_LR_norm-K_com*(np.dot(h_LRv[0:2],wrL[3:5])-GraspForce[3])*h_LRv
                vR_force = -K_com*(np.dot(h_LR_norm[0:2],wrR[3:5])-GraspForce[10])*h_LR_norm-K_com*(np.dot(h_LRv[0:2],wrR[3:5])-GraspForce[9])*h_LRv

                VT[3:6] = vL_force.reshape([3,1])
                VT[9:12] = vR_force.reshape([3,1])
                    

                Closest_Pt_L , Closest_Pt_env_L, Closest_Pt_R , Closest_Pt_env_R = collision_check(pp_L,pp_R)
                human_R = pose[iR,:]
                human_L = pose[iL,:]
                Human_LR_norm = np.linalg.norm(human_R-human_L)
#                print ('Human_LR_norm: ',Human_LR_norm)
                if process == 0:
                    
#                    print ("VT norm:", np.linalg.norm(VT))    
                    if np.linalg.norm(VT)>0.01:
#                        dqL = QP_BaxterArm(JL,qL,VT[0:6],er,ep, Closest_Pt_L , Closest_Pt_env_L)
#                        dqR = QP_BaxterArm(JR,qL,VT[6:12],er,ep, Closest_Pt_R , Closest_Pt_env_R)
#                        baxter.setJointCommand('left',dqL)
#                        baxter.setJointCommand('right',dqR)  
                        
                        dq_sln = QP_bow(JT,VT,Lambda,Epsilon1,q,Epsilon2,dq_pre, Closest_Pt_L , Closest_Pt_env_L, Closest_Pt_R , Closest_Pt_env_R)    
                            
                        dq_pre = dq_sln[0:17].reshape([17,1])
                        dqL = dq_sln[0:7]
                        dqR = dq_sln[7:14]   
                        dqB = 0.2*dq_sln[14:17]+0.8*dqB_pre
                        dqB_pre = dqB
                        ridgeback_vel_msg.linear.x = dqB[1]
                        ridgeback_vel_msg.linear.y = dqB[2]
                        ridgeback_vel_msg.angular.z = dqB[0] 
                        ridgeback.velocity_publisher.publish(ridgeback_vel_msg)
                        ridgeback.rate.sleep() 
                        baxter.setJointCommand('left',dqL)
                        baxter.setJointCommand('right',dqR)                        
                        
                    else:
                        print('================ Stage I ================')
                        process = 1
                        command_spread = 0
 
                        p_L = np.dot(Rk,np.dot(R_K2B,human_L))+pk
                        p_R = np.dot(Rk,np.dot(R_K2B,human_R))+pk
                        Z = np.vstack([p_L,p_R]).reshape([6,1])
                        Xk_prev = np.vstack([Z, np.zeros([6,1])]).reshape([12,1])
                        p_LR_prev = (p_R-p_L).reshape([3,])
                        p_LR_prev = p_LR_prev/np.linalg.norm(p_LR_prev)
                        pose0 = pose
                        r0 = quaternion_matrix(ridgeback_state[-4:])[0:3,0:3]
                        p0 = ridgeback_state[0:3]
                        baxter.setJointCommand('left',np.zeros([7,1]))
                        baxter.setJointCommand('right',np.zeros([7,1])) 
                        dqB_pre = np.array([0,0,0])
                        dq_pre = np.zeros([17,1]) 
                        Lambda = 0      # Lambda: weighting on dual arm motion
                        Epsilon1 = 1    # Epsilon1: alpha close to 1
                        Epsilon2 = 0.5   # Epsilon2: q close to previous                        
                        
                if process == 1:             
                    ### Kinect Pose Control
                    if not command_spread: #cnt_small_motion < Small_Motion_TH:#(joint_state[iL]==1) and (joint_state[iR]==1):
                    
                        ## Kalman Iteration    
                        P = np.dot(np.dot(Phi,P),Phi.T) + Q
                        S = np.dot(np.dot(M,P),M.T) + R
                        K = np.dot(np.dot(P,M.T),np.linalg.inv(S))
                        P = P - np.dot(np.dot(K,M),P)    
                        Xk = np.dot(Phi,Xk_prev) + np.dot(K,(Z-np.dot(np.dot(M,Phi),Xk_prev)))
                        #Xk = np.vstack([Z, np.zeros([6,1])]).reshape([12,1])    
                        
                        Vk = Xk - Xk_prev
                        Xk_prev = Xk
                                               
                        # For the next iteration                                                                 
                        p_LR = (Xk[3:6,:]-Xk[0:3,:]).reshape([3,])
                        p_LR = p_LR/np.linalg.norm(p_LR)
                        vc = 1*(Vk[3:6,:]+Vk[0:3,:]).reshape([3,1])
#                        print ('theta_hand: ',np.dot(p_LR_prev,p_LR),np.arccos(np.dot(p_LR_prev,p_LR)))
                        wc = 10*(np.cross(p_LR_prev,p_LR)*np.arccos(np.clip(np.dot(p_LR_prev,p_LR),-1,1))).reshape([3,1])                        
                        
                        Vc = np.vstack([wc,vc])
                        
                        AL = np.vstack([np.hstack([np.eye(3),np.zeros([3,3])]),np.hstack([-hat(-h_LR*0.5),np.eye(3)])])
                        AR = np.vstack([np.hstack([np.eye(3),np.zeros([3,3])]),np.hstack([-hat(h_LR*0.5),np.eye(3)])])
                        
                        VT[0:6] += 1*np.dot(AL,Vc)
                        VT[6:12] += 1*np.dot(AR,Vc)

                        p_LR_prev = p_LR
#                        print ('test:',VT,VT[6:12]+VT[0:6])
                        if not any(np.isnan(VT)):
                            
#                            VB = VT[6:12]#+VT[0:6]
#                            print ('VB: ',VB)
#                            dqB = 0.3*np.array([VB[2][0],VB[3][0],VB[4][0]])
#                            
#                            dqB = dqB_pre*0.8 + dqB*0.2
#                            
#                            dqB_pre = dqB
#                            dqB = np.clip(dqB, [-1,-5,-5],[1,5,5])#*(abs(dqB)>0.1)
#                            print ('dqB1: ',dqB)
#
##                            print ('dqB2: ',dqB)
#                            VT[[2,3,4,8,9,10]] = 0
##                            dqL = QP_BaxterArm(JL,qL,VT[0:6],er,ep, Closest_Pt_L , Closest_Pt_env_L)
##                            dqR = QP_BaxterArm(JR,qL,VT[6:12],er,ep, Closest_Pt_R , Closest_Pt_env_R)
##                            baxter.setJointCommand('left',dqL)
##                            baxter.setJointCommand('right',dqR) 

                            dq_sln = QP_bow(JT,VT,Lambda,Epsilon1,q,Epsilon2,dq_pre, Closest_Pt_L , Closest_Pt_env_L, Closest_Pt_R , Closest_Pt_env_R)    
                            
                            human_headc = pose[26,:]/np.linalg.norm(pose[26,:])
                            v_head = np.cross(human_headc,human_head0)
#                            print ('theta_head: ',np.dot(human_headc,human_head0),v_head.dot(ez))
                            theta_head = np.arccos(np.dot(human_headc,human_head0))
                            
                            dq_pre = dq_sln[0:17].reshape([17,1])
                            dqL = dq_sln[0:7]
                            dqR = dq_sln[7:14]
                            
                            dq_sln[14] = dq_sln[14] + np.sign(v_head.dot(ez))*theta_head*0.008*(theta_head<0.95)
                            dqB = dq_sln[14:17]
                            
                            #dqB[0] = dqB[0] + np.sign(v_head.dot(ez))*theta_head*0.01*(theta_head<0.95)
                            dqB = 0.18*dqB + 0.82*dqB_pre
                            dqB_pre = dqB
                            qB += dqB
                            ridgeback_vel_msg.linear.x = dqB[1]
                            ridgeback_vel_msg.linear.y = dqB[2]
                            ridgeback_vel_msg.angular.z = dqB[0] 
                            ridgeback.velocity_publisher.publish(ridgeback_vel_msg)
                            ridgeback.rate.sleep() 
                            baxter.setJointCommand('left',dqL)
                            baxter.setJointCommand('right',dqR)     
    
                            rc = quaternion_matrix(ridgeback_state[-4:])[0:3,0:3]
                            pc = ridgeback_state[0:3]
                            Rk = np.dot(r0, rc.T)
                            pk = pc - p0        
                            
                        print ("P_dist,theta_head,qB: ",np.linalg.norm(p_desired-pc), theta_head,qB)
                        if np.linalg.norm(p_desired-pc)<0.35 and theta_head<0.2:#Human_LR_norm>1.15:#np.linalg.norm(pk-p_desired)<0.5:#and theta_head>0.99:#>1.15:
                            command_spread += 1
                            print('Stage II: ',command_spread)
                            Vc = np.array([0,0,0,0,0,0]).reshape([6,1])   
                            
                    else:
                  
#                        baxter.setControlMode(0)
#                        qL = q0L
#                        qR = q0R
#                        baxter.setJointCommand('left',q0L)
#                        baxter.setJointCommand('right',q0R) 
#                        time.sleep(1)
#                        baxter.setControlMode(1)
#                        JL = robotjacobian(Bdef.left.H, Bdef.left.P, Bdef.left.joint_type, qL)
#                        JR = robotjacobian(Bdef.right.H, Bdef.right.P, Bdef.right.joint_type, qR)                        
#                        pp_L,RR = fwdkin_alljoints(qL, Bdef.left.joint_type, Bdef.left.H, Bdef.left.P, 7)
#                        p_BTL = pp_L[:, -1]
#                        pp_R,RR = fwdkin_alljoints(qR, Bdef.right.joint_type, Bdef.right.H, Bdef.right.P, 7)
#                        p_BTR = pp_R[:, -1]
#                        JT = getJT(JL,JR,p_BTL,p_BTR) 
                        
                        
                        
                        print('================ Stage II ================')
                        process = 2
                        K_com = 0.01
                        dqB_pre = np.array([0,0,0])
                        dq_pre = np.zeros([17,1]) 
                        Lambda = 50      # Lambda: weighting on dual arm motion
                        Epsilon1 = 1    # Epsilon1: alpha close to 1
                        Epsilon2 = 50   # Epsilon2: q close to previous                               
                        
                        
                if process == 2:        
#                    dqL = QP_BaxterArm(JL,qL,VT[0:6],er,ep, Closest_Pt_L , Closest_Pt_env_L)
#                    dqR = QP_BaxterArm(JR,qL,VT[6:12],er,ep, Closest_Pt_R , Closest_Pt_env_R)
#                    baxter.setJointCommand('left',dqL)
#                    baxter.setJointCommand('right',dqR) 
                    dq_sln = QP_bow(JT,VT,Lambda,Epsilon1,q,Epsilon2,dq_pre, Closest_Pt_L , Closest_Pt_env_L, Closest_Pt_R , Closest_Pt_env_R)    
                        
                    dq_pre = dq_sln[0:17].reshape([17,1])
                    dqL = dq_sln[0:7]
                    dqR = dq_sln[7:14]   
#                    dqB = 0.2*dq_sln[14:17]+0.8*dqB_pre
#                    dqB_pre = dqB
#                    ridgeback_vel_msg.linear.x = dqB[1]
#                    ridgeback_vel_msg.linear.y = dqB[2]
#                    ridgeback_vel_msg.angular.z = dqB[0] 
#                    ridgeback.velocity_publisher.publish(ridgeback_vel_msg)
#                    ridgeback.rate.sleep() 
                    baxter.setJointCommand('left',dqL)
                    baxter.setJointCommand('right',dqR)                
#                Closest_Pt_l , Closest_Pt_env_l, Closest_Pt_r , Closest_Pt_env_r = collision_check(pp_L,pp_R)
#                dq_sln = QP_bow(JT,VT,Lambda,Epsilon1,q,Epsilon2,dq_pre, Closest_Pt_l , Closest_Pt_env_l, Closest_Pt_r , Closest_Pt_env_r)
#
#                if any(np.isnan(dq_sln)):
#                    dq_sln = np.zeros([18,])
#                    
#                dq_pre = dq_sln[0:17].reshape([17,1]) 
#                
#                t4 = timeit.default_timer()-tic
#                ### Commnad BOW Robot
#                dqL = dq_sln[0:7]
#                dqR = dq_sln[7:14]   
#                dqB = dq_sln[14:17]
#                baxter.setJointCommand('left',dqL)
#                baxter.setJointCommand('right',dqR) 
#                
#                t5 = timeit.default_timer()-tic
#                
#                
#                ridgeback_vel_msg.linear.x = dqB[1]
#                ridgeback_vel_msg.linear.y = dqB[2]
#                ridgeback_vel_msg.angular.z = dqB[0] 
#                ridgeback.velocity_publisher.publish(ridgeback_vel_msg)
#                ridgeback.rate.sleep()  
#                
#                t6 = timeit.default_timer()-tic
#                
#                

                

#                t_all.append([t05,t075,t1,t2,t25, t3,t4,t5,t6,t7])
                

#                print ('Claps Norm:', claps_norm)           
                if Human_LR_norm <0.1: 
                    command_claps = 1

                t1 = timeit.default_timer()-tic
                
                VT_all.append(VT)
                wr_all.append(wr_read)
                q_all.append(q_read)
                pose_all.append(pose)    
                Xk_all.append(Xk)
                Z_all.append(Z) 
                    
                toc = timeit.default_timer()-tic
                if toc< dt:
                    time.sleep(dt-toc)
                else:
                    print ('exceed sampling time !!', toc)    
                    
                t_all.append([t1, toc, timeit.default_timer()-tic])
                
        sio.savemat('Data20200316_1.mat', {'t_all':t_all,'VT_all':VT_all,'Xk_all':Xk_all,'Z_all':Z_all, 'wr_all':wr_all, 'q_all':q_all,'pose_all':pose_all})    
        print ('Time Mean: ',np.mean(t_all,0))
        print ('Time Max:  ',np.max(t_all,0))
        print (np.std(Z_all, axis=0))
        
        print ('r_ini,p_ini',r_ini,p_ini)
        print ('r0,p0',r0,p0)
        print ('rc,pc',rc,pc)
        print ('Rk,pk',Rk,pk)
               
    elif Select == 1:
        print ('end')
        
        
    gripper.openGripper('Left');
    gripper.openGripper('Right');        
    baxter.setControlMode(0)
#    baxter.setJointCommand('left',q0L)
#    baxter.setJointCommand('right',q0R)     
#    Select = input("Press 1 to continue...")  
#    func = switcher.get(Select, "nothing")
#    print func()