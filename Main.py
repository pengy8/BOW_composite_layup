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
#from QuadProg_BOW import QP_bow
from QuadProg_BOW_wHead import QP_bow_head
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



def stage0(voice_control):
    
    print ("stage 0: ")
    claps = 0    
    while voice_control.read<0:# and not claps:     

        if pose is not None:
            human_R = pose[iR,:]
            human_L = pose[iL,:]            
            claps_norm = np.linalg.norm(human_R-human_L)
            print ('Claps Norm:', claps_norm)           
            if claps_norm <0.1: 
                claps = 1
        print ('voice_control: ',voice_control.read)
            
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
    voice_control = RRN.ConnectService('rr+tcp://192.168.131.151:9999/?service=Create')
    
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
#    q0 = np.array([0.38963112,  0.42491268, -1.41816524,  2.02101969,  0.45175734, -0.85826225,  0.19328158]) # changed on 0815
    q0 = np.array([0.34054373,  0.29605829, -1.52132545,  2.23117506,  0.23048061, -1.07685451,  0.10852914]) # move higher
    
    q0L = q0
    q0R = q0*[-1,1,-1,1,-1,1,-1]
    qL = q0L
    qR = q0R
    
    baxter.setControlMode(0)
    baxter.setJointCommand('left',q0L)
    baxter.setJointCommand('right',q0R)  
    
#    ### set position ##
#    baxter.setControlMode(1)
#    q_read = baxter.joint_positions
#    print q_read

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
    
    voice_control.StartStreaming()
    print "voice_control: ",voice_control.read
    
    Select = input("Press 1 to continue...")  


    # ============================== Program Starts ==============================
    
    if Select == 3:
        
        
        process = stage0(voice_control)#switcher.get(0, "nothing")

        baxter.setControlMode(1)

        # Parameter setting        
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
        q_read_pre = baxter.joint_positions
        wr_read_pre = baxter.endeffector_wrenches          
#        Lambda = 50      # Lambda: weighting on dual arm motion
#        Epsilon1 = 1    # Epsilon1: alpha close to 1
#        Epsilon2 = 50   # Epsilon2: q close to previous     
        Epsilon1 = 1     # Epsilon1: alpha close to 1
        Epsilon2 = 50     # Epsilon2: weighting on dual arm motion
        Epsilon3 = 50   # Epsilon3: q close to previous      
        Epsilon4 = 0    # Epsilon4: BOW facing the human head 

        
        r_ini = quaternion_matrix(ridgeback_state[-4:])[0:3,0:3]
        p_ini = ridgeback_state[0:3]
        human_head0 = pose[26,:]/np.linalg.norm(pose[26,:])
                
        command_claps = 0
        i = 0
        
        p_desired = np.array([1.54, 0.67, 0.])-np.array([0.56, -1.15, 0.])      
        
        t_all = []
        VT_all = []
        wr_control_all = []
        wr_all = []
        wr_ori_all = []
        q_all = []
        pose_all = []
        Xk_all = []
        Z_all = []
        dq_all = []
        theta_head_all = []
        Z = np.zeros([6,1])
        Xk = np.zeros([12,1])       
        qB = np.zeros([1,3])
        Ktheta_h = 0
        theta_head = 0
        
        command_wait = 0
        print ('voice_control: ',voice_control.read)
        
        while (not command_claps) and voice_control.read:
            print ('voice_control: ',voice_control.read)
            
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
                wr_read_ori = baxter.endeffector_wrenches
                wr_read = 0.2*wr_read_ori + 0.8*wr_read_pre
                wrL = wr_read_ori[0:6]
                wrR = wr_read_ori[6:12]

                h_LR = p_BTR-p_BTL
                h_LR_norm = h_LR/np.linalg.norm(h_LR)
                h_LRv = np.cross(h_LR_norm,ez)
                
                wr_control = [np.dot(h_LR_norm[0:2],wrL[3:5]), np.dot(h_LRv[0:2],wrL[3:5]), np.dot(h_LR_norm[0:2],wrR[3:5]), np.dot(h_LRv[0:2],wrR[3:5])]
                vL_force = -K_com*(wr_control[0]-GraspForce[4])*h_LR_norm-K_com*(wr_control[1]-GraspForce[3])*h_LRv
                vR_force = -K_com*(wr_control[2]-GraspForce[10])*h_LR_norm-K_com*(wr_control[3]-GraspForce[9])*h_LRv
                
                VT[3:6] = vL_force.reshape([3,1])
                VT[9:12] = vR_force.reshape([3,1])
                
                Closest_Pt_L , Closest_Pt_env_L, Closest_Pt_R , Closest_Pt_env_R = collision_check(pp_L,pp_R)
                human_R = pose[iR,:]
                human_L = pose[iL,:]
                Human_LR_norm = np.linalg.norm(human_R-human_L)
                
                if command_wait:
                    
                    if voice_control.read ==5:
                        command_wait = 0
                        
                    
                elif process == 0: # Comliance control only until steady-state 
                    
#                   # Extend Sheet Until stable
                    if np.linalg.norm(VT)>0.025:

#                        dq_sln = QP_bow(JT,VT,Lambda,Epsilon1,q,Epsilon2,dq_pre, Closest_Pt_L , Closest_Pt_env_L, Closest_Pt_R , Closest_Pt_env_R)    
                        dq_sln = QP_bow_head(JT,VT,Epsilon1,Epsilon2,q,Epsilon3, Epsilon4, Ktheta_h, dq_pre, Closest_Pt_L , Closest_Pt_env_L, Closest_Pt_R , Closest_Pt_env_R)
    
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
                        command_layup = 0
 
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
                        Epsilon1 = 1     # Epsilon1: alpha close to 1
                        Epsilon2 = 0     # Epsilon2: weighting on dual arm motion
                        Epsilon3 = 0.3   # Epsilon3: q close to previous      
                        Epsilon4 = 3    # Epsilon4: BOW facing the human head 
                        
                elif process == 1:  # Transport stage until arriving the predefined location
                    
                    ### Transport with Kinect Pose Control Until reaching a predefined position
                    if not command_layup: #cnt_small_motion < Small_Motion_TH:#(joint_state[iL]==1) and (joint_state[iR]==1):
                    
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
                        wc = 10*(np.cross(p_LR_prev,p_LR)*np.arccos(np.clip(np.dot(p_LR_prev,p_LR),-1,1))).reshape([3,1])                                                
                        Vc = np.vstack([wc,vc])
                        
                        AL = np.vstack([np.hstack([np.eye(3),np.zeros([3,3])]),np.hstack([-hat(-h_LR*0.5),np.eye(3)])])
                        AR = np.vstack([np.hstack([np.eye(3),np.zeros([3,3])]),np.hstack([-hat(h_LR*0.5),np.eye(3)])])                 
                        VT[0:6] += 1*np.dot(AL,Vc)
                        VT[6:12] += 1*np.dot(AR,Vc)

                        p_LR_prev = p_LR

                        if not any(np.isnan(VT)):


                            human_headc = pose[26,:]/np.linalg.norm(pose[26,:])
                            v_head = np.cross(human_headc,human_head0)
                            theta_head = np.arccos(np.dot(human_headc,human_head0))
                            Ktheta_h = np.sign(v_head.dot(ez))*theta_head*0.1*(theta_head<0.95)
                            
                            
                            dq_sln = QP_bow_head(JT,VT,Epsilon1,Epsilon2,q,Epsilon3, Epsilon4, Ktheta_h, dq_pre, Closest_Pt_L , Closest_Pt_env_L, Closest_Pt_R , Closest_Pt_env_R)

                            dq_pre = dq_sln[0:17].reshape([17,1])
                            dqL = dq_sln[0:7]
                            dqR = dq_sln[7:14]
                            dqB = dq_sln[14:17]
                            
                            dqB = 0.15*dqB + 0.85*dqB_pre
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
                        if not command_layup and (np.linalg.norm(p_desired-pk)<0.3):# and theta_head<0.2:
                            command_layup += 1
                            print('Stage II: ',command_layup)
                            Vc = np.array([0,0,0,0,0,0]).reshape([6,1])   
                            
                    else:
                  
#                        baxter.setControlMode(0)
#                        qL = q0L
#                        qR = q0R
#                        baxter.setJointCommand('left',q0L)
#                        baxter.setJointCommand('right',q0R) 
#                        time.sleep(4)
#                        
#                        JL = robotjacobian(Bdef.left.H, Bdef.left.P, Bdef.left.joint_type, qL)
#                        JR = robotjacobian(Bdef.right.H, Bdef.right.P, Bdef.right.joint_type, qR)                        
#                        pp_L,RR = fwdkin_alljoints(qL, Bdef.left.joint_type, Bdef.left.H, Bdef.left.P, 7)
#                        p_BTL = pp_L[:, -1]
#                        pp_R,RR = fwdkin_alljoints(qR, Bdef.right.joint_type, Bdef.right.H, Bdef.right.P, 7)
#                        p_BTR = pp_R[:, -1]
#                        JT = getJT(JL,JR,p_BTL,p_BTR) 
#                        baxter.setControlMode(1)
#                        
#                        p_BTL_desired = p_BTL + np.array([0.25,0,0])
#                        p_BTR_desired = p_BTR + np.array([0.25,0,0])
                        
                        
                        human_R_record = human_R
                        human_L_record = human_L
                        move_forward = 0
                        print('================ Stage II ================')
                        process = 2
                        K_com = 0.01
                        dqB_pre = np.array([0,0,0])
                        dq_pre = np.zeros([17,1]) 
                        Epsilon1 = 1     # Epsilon1: alpha close to 1
                        Epsilon2 = 50     # Epsilon2: weighting on dual arm motion
                        Epsilon3 = 50   # Epsilon3: q close to previous      
                        Epsilon4 = 0    # Epsilon4: BOW facing the human head  
                        Ktheta_h = 0
                        
                elif process == 2:   ## Hold stage. Hold the sheet like a damping fixture     
                    
#                    print ("hands down: ",np.linalg.norm((human_R+human_L)-(human_R_record+human_L_record)))
#                    print (p_BTL_desired)
#                    if  not move_forward and np.linalg.norm((human_R+human_L)-(human_R_record+human_L_record))>0.2:
#                        print ("========================= Move Forwad =========================")
#                        move_forward = 1
#                    
#                    if move_forward:
#                        VT[3:6] += 0.03*(p_BTL_desired-p_BTL_desired).reshape([3,1])
#                        VT[9:12] += 0.03*(p_BTR_desired-p_BTR_desired).reshape([3,1])
                    # Layup Process
                    dq_sln = QP_bow_head(JT,VT,Epsilon1,Epsilon2,q,Epsilon3, Epsilon4, Ktheta_h, dq_pre, Closest_Pt_L , Closest_Pt_env_L, Closest_Pt_R , Closest_Pt_env_R)
    
                    dq_pre = dq_sln[0:17].reshape([17,1])
                    dqL = dq_sln[0:7]
                    dqR = dq_sln[7:14]   
                    
                    baxter.setJointCommand('left',dqL)
                    baxter.setJointCommand('right',dqR)                

                    if voice_control.read == 3 or (gripper.getNavigatorState('left').ok_button == 1 or gripper.getNavigatorState('right').ok_button == 1):
                        
#                        process = 3
#                        dq_sln = np.zeros([18,])
#                        dq_pre = dq_sln[0:17].reshape([17,1])                
#                        dqB = dq_sln[14:17]
#                        dqB = 0.15*dqB + 0.85*dqB_pre
#                        dqB_pre = dqB
#                        qB += dqB
#                        ridgeback_vel_msg.linear.x = dqB[1]
#                        ridgeback_vel_msg.linear.y = dqB[2]
#                        ridgeback_vel_msg.angular.z = dqB[0] 
#                        ridgeback.velocity_publisher.publish(ridgeback_vel_msg)
#                        ridgeback.rate.sleep()  
#                        baxter.setControlMode(0) 
#                        baxter.setJointCommand('left',qL)
#                        baxter.setJointCommand('right',qR) 
                        gripper.openGripper('Left')
                        gripper.openGripper('Right')
                         
#                        baxter.setControlMode(1)
#                    
#                elif process == 3:
#                    
#                    if gripper.getNavigatorState('left').ok_button == 1 or gripper.getNavigatorState('right').ok_button == 1:
                        process = 4
                        baxter.setControlMode(1)
                        
                        Epsilon1 = 1     # Epsilon1: alpha close to 1
                        Epsilon2 = 0     # Epsilon2: weighting on dual arm motion
                        Epsilon3 = 0.01   # Epsilon3: qB close to previous      
                        Epsilon4 = 0    # Epsilon4: BOW facing the human head 
                        GraspForce = np.array([0,0,0,0,0,0,0,0,0,0,0,0])
                        dq_pre = np.zeros([17,1])
                        dqB_pre = np.array([0,0,0])
                        Ktheta_h = 0
                        time.sleep(1.5)  
                        
                      
                         
                elif process == 4:
                    
#                    print ("sum: ",sum(abs(q_read-q_read_pre)),sum(abs(wr_read-wr_read_pre)))
                    if (sum(abs(q_read-q_read_pre))<0.1) and (sum(abs(wr_read-wr_read_pre)))>2 and (gripper.getNavigatorState('left').ok_button != 1) and (gripper.getNavigatorState('right').ok_button != 1):
                        h_LR = p_BTR-p_BTL
                        h_LR_norm = h_LR/np.linalg.norm(h_LR)
                        h_LRv = np.cross(h_LR_norm,ez)
                        
                        wr_control = [np.dot(h_LR_norm[0:2],wrL[3:5]), np.dot(h_LRv[0:2],wrL[3:5]), np.dot(h_LR_norm[0:2],wrR[3:5]), np.dot(h_LRv[0:2],wrR[3:5])]
                        vL_force = -K_com*(wr_control[0]-GraspForce[4])*h_LR_norm-K_com*(wr_control[1]-GraspForce[3])*h_LRv
                        vR_force = -K_com*(wr_control[2]-GraspForce[10])*h_LR_norm-K_com*(wr_control[3]-GraspForce[9])*h_LRv
                        
                        VT[3:6] = vL_force.reshape([3,1])
                        VT[9:12] = vR_force.reshape([3,1])
                        
                        dq_sln = QP_bow_head(JT,VT,Epsilon1,Epsilon2,q,Epsilon3, Epsilon4, Ktheta_h, dq_pre, Closest_Pt_L , Closest_Pt_env_L, Closest_Pt_R , Closest_Pt_env_R)
                        
                    else:
                        dq_sln = np.zeros([18,])
                        
                        
                    dq_pre = dq_sln[0:17].reshape([17,1])                
                    dqB = dq_sln[14:17]
                    dqB = 0.15*dqB + 0.85*dqB_pre
                    dqB_pre = dqB
                    qB += dqB
                    ridgeback_vel_msg.linear.x = dqB[1]
                    ridgeback_vel_msg.linear.y = dqB[2]
                    ridgeback_vel_msg.angular.z = dqB[0] 
                    ridgeback.velocity_publisher.publish(ridgeback_vel_msg)
                    ridgeback.rate.sleep()
                    
                    if (gripper.getNavigatorState('left').ok_button == 1 or gripper.getNavigatorState('right').ok_button == 1):
                        
                        print('================ Stage II ================')
                        process = 2
                        K_com = 0.01
                        dqB_pre = np.array([0,0,0])
                        dq_pre = np.zeros([17,1]) 
                        Epsilon1 = 1     # Epsilon1: alpha close to 1
                        Epsilon2 = 50     # Epsilon2: weighting on dual arm motion
                        Epsilon3 = 50   # Epsilon3: q close to previous      
                        Epsilon4 = 0    # Epsilon4: BOW facing the human head  
                        Ktheta_h = 0
                        gripper.closeGripper('Left')
                        gripper.closeGripper('Right')
                        time.sleep(1.5) 
                    
                # Gripper Conrtol
                if gripper.getNavigatorState('left').show_button == 1:
                    gripper.openGripper('Left')
                if gripper.getNavigatorState('right').show_button == 1:
                    gripper.openGripper('Right')
                if gripper.getNavigatorState('left').cancel_button == 1:
                    gripper.closeGripper('Left')
                if gripper.getNavigatorState('right').cancel_button == 1:
                    gripper.closeGripper('Right')    
                    
                if voice_control.read > 12:

                    dv = 0.025    
                    while (voice_control.read>12):
                        dq_sln = np.zeros([18,])
                        
                        if voice_control.read == 13:
                            dq_sln[15] = -dv
                        elif voice_control.read == 14:
                            dq_sln[15] = dv                        
                        dqB = 0.2*dq_sln[14:17]+0.8*dqB_pre
                        dqB_pre = dqB
                        ridgeback_vel_msg.linear.x = dqB[1]
                        ridgeback_vel_msg.linear.y = dqB[2]
                        ridgeback_vel_msg.angular.z = dqB[0] 
                        ridgeback.velocity_publisher.publish(ridgeback_vel_msg)
                        ridgeback.rate.sleep() 
                    print("Stop moving base.")   
#                    ridgeback_vel_msg.linear.x = 0
#                    ridgeback_vel_msg.linear.y = 0
#                    ridgeback_vel_msg.angular.z = 0 
#                    ridgeback.velocity_publisher.publish(ridgeback_vel_msg)
#                    ridgeback.rate.sleep()
                    
                elif voice_control.read > 6:
                    
                    dv = 0.025
                    Ktheta_h = 0
                    p_BTL_tmp = p_BTL
                    p_BTR_tmp = p_BTR
#                    move_hand = 1
                    while (voice_control.read>6): 
                        VT = np.zeros([12,1])
                        if voice_control.read == 7:
                            VT[5] = dv
                        elif voice_control.read == 8:
                            VT[11] = dv
                        elif voice_control.read == 9:
                            VT[5] = dv
                            VT[11] = dv
                        elif voice_control.read == 10:
                            VT[5] = -dv
                        elif voice_control.read == 11:
                            VT[11] = -dv
                        elif voice_control.read == 12:
                            VT[5] = -dv
                            VT[11] = -dv
                    
                       
                        q_read = baxter.joint_positions
                        qL = q_read[0:7]
                        qR = q_read[7:14]                
                        q = np.hstack([qL,qR,np.array([0,0,0])]).reshape([17,1])
                        
                        JL = robotjacobian(Bdef.left.H, Bdef.left.P, Bdef.left.joint_type, qL)
                        JR = robotjacobian(Bdef.right.H, Bdef.right.P, Bdef.right.joint_type, qR)
                        
                        pp_L,RR = fwdkin_alljoints(qL, Bdef.left.joint_type, Bdef.left.H, Bdef.left.P, 7)
                        p_BTL = pp_L[:, -1]
                        pp_R,RR = fwdkin_alljoints(qR, Bdef.right.joint_type, Bdef.right.H, Bdef.right.P, 7)
                        p_BTR = pp_R[:, -1]
                        JT = getJT(JL,JR,p_BTL,p_BTR)
                        Closest_Pt_L , Closest_Pt_env_L, Closest_Pt_R , Closest_Pt_env_R = collision_check(pp_L,pp_R)
                        dq_sln = QP_bow_head(JT,VT,Epsilon1,Epsilon2,q,Epsilon3, Epsilon4, Ktheta_h, dq_pre, Closest_Pt_L , Closest_Pt_env_L, Closest_Pt_R , Closest_Pt_env_R)
                        dq_pre = dq_sln[0:17].reshape([17,1])
                        dqL = dq_sln[0:7]
                        dqR = dq_sln[7:14]
                        baxter.setJointCommand('left',dqL)
                        baxter.setJointCommand('right',dqR)
                        
#                        if abs(p_BTL_tmp-p_BTL)>0.2 or abs(p_BTR_tmp-p_BTR)>0.2:
#                            move_hand = 0
                        
                    dq_sln = np.zeros([18,])    
                    dqL = dq_sln[0:7]
                    dqR = dq_sln[7:14]
                    baxter.setJointCommand('left',dqL)
                    baxter.setJointCommand('right',dqR)
#                        dqB = dq_sln[14:17]
#                        dqB = 0.15*dqB + 0.85*dqB_pre
#                        dqB_pre = dqB
#                        qB += dqB
#                        ridgeback_vel_msg.linear.x = dqB[1]
#                        ridgeback_vel_msg.linear.y = dqB[2]
#                        ridgeback_vel_msg.angular.z = dqB[0] 
#                        ridgeback.velocity_publisher.publish(ridgeback_vel_msg)
#                        ridgeback.rate.sleep() 
                                                   

                if voice_control.read == 4 and command_wait !=1:
                    command_wait = 1
                    dq_sln = np.zeros([18,])
                    dq_pre = dq_sln[0:17].reshape([17,1])                
                    dqB = dq_sln[14:17]
                    dqB_pre = dqB
                    qB += dqB
                    ridgeback_vel_msg.linear.x = dqB[1]
                    ridgeback_vel_msg.linear.y = dqB[2]
                    ridgeback_vel_msg.angular.z = dqB[0] 
                    ridgeback.velocity_publisher.publish(ridgeback_vel_msg)
                    ridgeback.rate.sleep()  
                    baxter.setControlMode(0) 
                    baxter.setJointCommand('left',qL)
                    baxter.setJointCommand('right',qR) 
                    time.sleep(1)           
                    baxter.setControlMode(1)


                wr_read_pre = wr_read
                q_read_pre = q_read
                    
#                if process<2 and Human_LR_norm <0.1: 
#                    command_claps = 1

                t1 = timeit.default_timer()-tic
                
                dq_all.append(dq_sln)
                VT_all.append(VT)
                wr_control_all.append(wr_control)
                wr_ori_all.append(wr_read_ori)
                wr_all.append(wr_read)
                q_all.append([q_read, qB])
                pose_all.append(pose)    
                Xk_all.append(Xk)
                Z_all.append(Z) 
                theta_head_all.append([theta_head])   
                
                toc = timeit.default_timer()-tic
                if toc< dt:
                    time.sleep(dt-toc)
                else:
                    print ('exceed sampling time !!', toc)    
                    
                t_all.append([t1, toc, timeit.default_timer()-tic,timeit.default_timer()])
                
                
                
        sio.savemat('Data20200922_1_'+str(timeit.default_timer())+'.mat', {'t_all':t_all,
        'dq_all':dq_all,
        'VT_all':VT_all,
        'wr_control_all':wr_control_all,
        'wr_ori_all':wr_ori_all,
        'wr_all':wr_all,
        'q_all':q_all,
        'pose_all':pose_all,
        'Xk_all':Xk_all,
        'Z_all':Z_all,          
        'theta_head_all':theta_head_all})
        
        print ('Time Mean: ',np.mean(t_all,0))
        print ('Time Max:  ',np.max(t_all,0))
#        print (np.std(Z_all, axis=0))
#        
#        print ('r_ini,p_ini',r_ini,p_ini)
#        print ('r0,p0',r0,p0)
#        print ('rc,pc',rc,pc)
#        print ('Rk,pk',Rk,pk)
               
    elif Select == 1:

        Epsilon1 = 1     # Epsilon1: alpha close to 1
        Epsilon2 = 0     # Epsilon2: weighting on dual arm motion
        Epsilon3 = 0.01   # Epsilon3: qB close to previous      
        Epsilon4 = 0    # Epsilon4: BOW facing the human head 
        GraspForce = np.array([0,0,0,0,0,0,0,0,0,0,0,0])
        K_com = 0.01
        
#        stage4(Epsilon1,Epsilon2,Epsilon3,Epsilon4,GraspForce,K_com)
        
        Ktheta_h = 0
        dq_pre = np.zeros([17,1])
        dqB_pre = np.array([0,0,0])
        ez = np.array([0,0,1])             
        qB = np.zeros([1,3])
        baxter.setControlMode(1)
        
        claps = 0    
        q_read_pre=baxter.joint_positions
        wr_read_pre = baxter.endeffector_wrenches        
        
        while not claps:     
    
            if pose is not None:
                
                human_R = pose[iR,:]
                human_L = pose[iL,:]            
                claps_norm = np.linalg.norm(human_R-human_L)
    #            print ('Claps Norm:', claps_norm)           
                if claps_norm <0.1: 
                    claps = 1
                
                
            VT = np.zeros([12,1])
            q_read = baxter.joint_positions
            qL = q_read[0:7]
            qR = q_read[7:14]                
            q = np.hstack([qL,qR,np.array([0,0,0])]).reshape([17,1])
            
            JL = robotjacobian(Bdef.left.H, Bdef.left.P, Bdef.left.joint_type, qL)
            JR = robotjacobian(Bdef.right.H, Bdef.right.P, Bdef.right.joint_type, qR)
            
            pp_L,RR = fwdkin_alljoints(qL, Bdef.left.joint_type, Bdef.left.H, Bdef.left.P, 7)
            p_BTL = pp_L[:, -1]
            pp_R,RR = fwdkin_alljoints(qR, Bdef.right.joint_type, Bdef.right.H, Bdef.right.P, 7)
            p_BTR = pp_R[:, -1]
            JT = getJT(JL,JR,p_BTL,p_BTR)              
       
            ### Force Control
            wr_read = 0.2*baxter.endeffector_wrenches+0.8*wr_read_pre
            
            wrL = wr_read[0:6]
            wrR = wr_read[6:12]
            
            print ("sum: ",sum(abs(q_read-q_read_pre)),sum(abs(wr_read-wr_read_pre)))
            if (sum(abs(q_read-q_read_pre))<0.1) and (sum(abs(wr_read-wr_read_pre)))>1:
                print("Run!!")
                h_LR = p_BTR-p_BTL
                h_LR_norm = h_LR/np.linalg.norm(h_LR)
                h_LRv = np.cross(h_LR_norm,ez)
                
                wr_control = [np.dot(h_LR_norm[0:2],wrL[3:5]), np.dot(h_LRv[0:2],wrL[3:5]), np.dot(h_LR_norm[0:2],wrR[3:5]), np.dot(h_LRv[0:2],wrR[3:5])]
                vL_force = -K_com*(wr_control[0]-GraspForce[4])*h_LR_norm-K_com*(wr_control[1]-GraspForce[3])*h_LRv
                vR_force = -K_com*(wr_control[2]-GraspForce[10])*h_LR_norm-K_com*(wr_control[3]-GraspForce[9])*h_LRv
                
                VT[3:6] = vL_force.reshape([3,1])
                VT[9:12] = vR_force.reshape([3,1])
                
#                print ("V force:",VT[3:6],VT[9:12])
#                VT[3:6] = VT[3:6]*(abs(VT[3:6])>0.025)
#                VT[9:12] = VT[9:12]*(abs(VT[9:12])>0.025)
                
                Closest_Pt_L , Closest_Pt_env_L, Closest_Pt_R , Closest_Pt_env_R = collision_check(pp_L,pp_R)
    
                dq_sln = QP_bow_head(JT,VT,Epsilon1,Epsilon2,q,Epsilon3, Epsilon4, Ktheta_h, dq_pre, Closest_Pt_L , Closest_Pt_env_L, Closest_Pt_R , Closest_Pt_env_R)
                dq_pre = dq_sln[0:17].reshape([17,1])
                dqB = dq_sln[14:17]
                
                dqB = 0.15*dqB + 0.85*dqB_pre
                dqB_pre = dqB
                qB += dqB
                ridgeback_vel_msg.linear.x = dqB[1]
                ridgeback_vel_msg.linear.y = dqB[2]
                ridgeback_vel_msg.angular.z = dqB[0] 
                ridgeback.velocity_publisher.publish(ridgeback_vel_msg)
                ridgeback.rate.sleep() 
#                baxter.setJointCommand('left',dqL)
#                baxter.setJointCommand('right',dqR)    
            else:
                dq_sln =np.zeros([17,1])
                dq_pre = dq_sln[0:17].reshape([17,1])                
                dqB = np.array([0,0,0])
                
                dqB = 0.15*dqB + 0.85*dqB_pre
                dqB_pre = dqB
                qB += dqB
                ridgeback_vel_msg.linear.x = dqB[1]
                ridgeback_vel_msg.linear.y = dqB[2]
                ridgeback_vel_msg.angular.z = dqB[0] 
                ridgeback.velocity_publisher.publish(ridgeback_vel_msg)
                ridgeback.rate.sleep()
            
                
            wr_read_pre = wr_read
            q_read_pre = q_read
            
            
            if gripper.getNavigatorState('left').show_button == 1:
                gripper.openGripper('Left')
            if gripper.getNavigatorState('right').show_button == 1:
                gripper.openGripper('Right')
            if gripper.getNavigatorState('left').cancel_button == 1:
                gripper.closeGripper('Left')
            if gripper.getNavigatorState('right').cancel_button == 1:
                gripper.closeGripper('Right')                    
                  
#                print ("Buttons: ",gripper.getNavigatorState('left').ok_button,gripper.getNavigatorState('left').show_button,gripper.getNavigatorState('left').cancel_button)
        
        
    gripper.openGripper('Left');
    gripper.openGripper('Right');        
    baxter.setControlMode(0)
#    baxter.setJointCommand('left',q0L)
#    baxter.setJointCommand('right',q0R)     
#    Select = input("Press 1 to continue...")  
#    func = switcher.get(Select, "nothing")
#    print func()