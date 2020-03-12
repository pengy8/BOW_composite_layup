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
from QuadProg_SingleArm import *
from baxter_info import *

import time
import timeit

ridgeback_state = 0
pose = None
joint_state = None
          
def bodytracker(pipe_ep):
    global pose 
    global joint_state
    
    data=pipe_ep.ReceivePacket()
    #print(data.tracking_id)
    #print(data.joint_tracking_state[1])
    joint_state = data.joint_tracking_state
    P=[]
    for p in (data.joint_poses):
        #print(pose[1])
        P.append([p[1][0],p[1][1],p[1][2]])

    pose = np.array(P)/1000
    #print (np.array(P))    
    #print('========================================')	
    
    
class ridgeback():

    def __init__(self):
        #Creating our node,publisher and subscriber
#        rospy.init_node('ridgeback_run', anonymous=True)
        self.velocity_publisher = rospy.Publisher('/ridgeback_velocity_controller/cmd_vel', Twist, queue_size=50)
        self.odom_subscriber = rospy.Subscriber('odometry/filtered',Odometry,self.odometryCb)
        self.rate = rospy.Rate(50)
        #self.state = 0
        #rospy.spin()
        
    def unpack_odom(self, msg):
        """
        Converts an Odometry message into a state vector.
        from https://www.programcreek.com/python/example/95996/nav_msgs.msg.Odometry
        """
        q = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        twist = [msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.angular.z]
#        return np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, trns.euler_from_quaternion(q)[2],
#                         msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.angular.z]) 
        return np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
                         
    def odometryCb(self,msg):
        global ridgeback_state
        ridgeback_state = self.unpack_odom(msg)
        #print (msg)
        
def collision_check(pp_L,pp_R):
    
    body_radius = 0.35
    Closest_Pt_l = np.array([1000,1000,1000])
    Closest_Pt_env_l = np.array([0,0,0])
    Closest_Pt_r = np.array([0,0,0]) 
    Closest_Pt_env_r = np.array([1000,1000,1000])
    
    d_min = 1000
        
    for l in pp_L.T:
        #l = pp_L[0:2,il+1]
        for r in pp_R.T:
            #r = pp_R[0:2,ir+1]
            dist = np.linalg.norm(l-r)
            if dist < d_min:
                d_min = dist
                Closest_Pt_l = l
                Closest_Pt_env_l = r
                Closest_Pt_r = r
                Closest_Pt_env_r = l     
                
    #print (np.linalg.norm(pp_L[0:2,-1])-body_radius)
    
    if (np.linalg.norm(pp_L[0:2,-1])-body_radius) < d_min:
        #print ('(left) colliding body')
        Closest_Pt_l = pp_L[:,-1]
        scale = body_radius/np.linalg.norm(pp_L[0:2,-1])
        Closest_Pt_env_l = np.array([scale*pp_L[0,-1],scale*pp_L[1,-1],pp_L[2,-1]])       

    if (np.linalg.norm(pp_R[0:2,-1])-body_radius) < d_min:
        #print ('(right) colliding body')
        Closest_Pt_r = pp_R[:,-1]
        scale = body_radius/np.linalg.norm(pp_R[0:2,-1])
        Closest_Pt_env_r = np.array([scale*pp_R[0,-1],scale*pp_R[1,-1],pp_R[2,-1]])
        
    #print ('d_min: ', d_min)             
    #print ('closest points: ', Closest_Pt_l , Closest_Pt_env_l, Closest_Pt_r , Closest_Pt_env_r)  
    
    return Closest_Pt_l , Closest_Pt_env_l, Closest_Pt_r , Closest_Pt_env_r
    
    
if __name__ == '__main__':
    
    baxter = RRN.ConnectService('tcp://localhost:1111/BaxterJointServer/Baxter')
    gripper = RRN.ConnectService('tcp://localhost:2222/BaxterPeripheralServer/BaxterPeripherals')
    kinect = RRN.ConnectService('tcp://192.168.131.151:8888/sensors.kinect2/KinectTracker')
    
    
    kinect.active_body_tracking=True   
    K=kinect.kinect_body_sensor_data.Connect(-1)
    K.PacketReceivedEvent+=bodytracker

    
#    gripper.calibrateGripper('left')
#    gripper.calibrateGripper('right')
#    gripper.closeGripper('Left')
#    gripper.closeGripper('Right')
    gripper.openGripper('Left');
    gripper.openGripper('Right');
    
    Bdef = defineBaxter()    
    #q0 = np.array([0.3804,0.0579,-1.6341,1.2805,0.1603,1.3948,0.0667])
    q0 = np.array([0.77350981,  0.26461169, -1.40589339,  2.07777698,  0.33594179, -0.49547579, -0.03298059])    
    #q0 = np.array([0.27995149,  0.19711653, -1.43235456,  2.20739835,  0.1836942 , -1.16160695, -0.06596117 ]) 
    q0L = q0
    q0R = q0*[-1,1,-1,1,-1,1,-1]
    qL = q0L
    qR = q0R
    dq_pre = np.zeros([17,1])
    baxter.setControlMode(0)
    baxter.setJointCommand('left',q0L)
    baxter.setJointCommand('right',q0R)  
    print('Joint Positions: ', baxter.joint_positions)
    
    iL = 14
    iR = 7    
    
    elbow_L = 13
    elbow_R = 6
    
    try:
        rospy.init_node('ROS_ridgeback_and_kinect', anonymous=True)
        t_ros = 0.025
        #Testing our function
        ridgeback = ridgeback()
        print (rospy.is_shutdown())
        print (ridgeback_state)
        ridgeback_vel_msg = Twist()
        #linear velocity:
        ridgeback_vel_msg.linear.x = 0
        ridgeback_vel_msg.linear.y = 0
        ridgeback_vel_msg.linear.z = 0
        #angular velocity in the z-axis:
        ridgeback_vel_msg.angular.x = 0
        ridgeback_vel_msg.angular.y = 0
        ridgeback_vel_msg.angular.z = 0 
        
        print (rospy.is_shutdown())
        rospy.sleep(t_ros)

        
    except rospy.ROSInterruptException: pass

    Select = input("Press 1 to continue...")  


    if Select == 3:
        
        claps = 0
        
        while not claps:     
            
            #print (joint_state, pose)
            if pose is not None:
                human_R = pose[iR,:]
                human_L = pose[iL,:]
    
                if 1:#(joint_state[iL]==1) and (joint_state[iR]==1):#all(pose) != 0:
    
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

        ini=0

        R_K2B = np.array([[0,0,1],[-1,0,0],[0,-1,0]])
        
        Rk = np.eye(3)
        pk = np.array([0,0,0])
        
        while(ini==0):
            #print (joint_state, pose)
            human_R = pose[iR,:]
            human_L = pose[iL,:]
            
            print ('joint_state ini: ',joint_state[iL],joint_state[iR])
            if 1:#(joint_state[iL]==1) and (joint_state[iR]==1):#all(pose) != 0:
                
                #joint_positions_prev = np.reshape(person1.joint_positions,[25,3])  
                
                p_L = np.dot(R_K2B,human_L)
                p_R = np.dot(R_K2B,human_R)
                Z = np.vstack([p_L,p_R]).reshape([6,1])         
                
                p_LR_prev = (p_R-p_L).reshape([3,])
                p_LR_prev = p_LR_prev/np.linalg.norm(p_LR_prev)
                ini=1
                pose0 = pose
                p_elbow_L_prev = np.dot(R_K2B,pose[elbow_L,:])
                p_elbow_R_prev = np.dot(R_K2B,pose[elbow_R,:])

        baxter.setControlMode(1)
        Lambda = 0     # Lambda: weighting on dual arm motion
        Epsilon1 = 1    # Epsilon1: alpha close to 1
        Epsilon2 = 12   # Epsilon2: q close to previous
    
        VT = np.zeros([12,1])
        K_com = 0.015
        GraspForce = np.array([0,0,0,6,-6,0,0,0,0,6,6,0])
        t_all = []
        VT_all = []
        ez = np.array([0,0,1])
        dt = 0.05
        Small_Motion_TH = 1
        #Ts = 0.045
        
        ## Kalman Filter initialization
        #r = 3*np.array([0.00976878, 0.04394652, 0.00280894, 0.00929264, 0.05048317, 0.00397884])
        r = 3*np.array([0.0137331,  0.02264715, 0.01016529, 0.00852387, 0.02615293, 0.00723866])
        Xk_prev = np.vstack([Z, np.zeros([6,1])]).reshape([12,1])
        Phi = np.vstack([np.hstack([np.eye(6), dt*np.eye(6)]), np.hstack([np.zeros([6,6]),np.eye(6)])])
        P = np.eye(12)
        Q = np.vstack([np.hstack([dt**2*np.diag(r**2),np.zeros([6,6])]), np.hstack([np.zeros([6,6]),dt**2*np.diag(r**2)])])
        M = np.hstack([np.eye(6),np.zeros([6,6])])
        R = dt*np.diag(r**2)
        Xk_all = []
        Xk_all.append(Xk_prev.T)
        Z_all = []
        
        print ('State: ',ridgeback_state)
        
        r0 = quaternion_matrix(ridgeback_state[-4:])[0:3,0:3]
        p0 = ridgeback_state[0:3]

        dqL = np.zeros([7,])
        dqR = np.zeros([7,])   
        dqB = np.zeros([3,])
        dq_pre = np.zeros([17,1]) 
        
        cnt_small_motion = 0
        
        
        claps = 0
        i = 0
        while not claps:
        #for i in range(750):
            if not rospy.is_shutdown():
                i +=1
                print ('iteration:', i)
                tic = timeit.default_timer()
                VT = np.zeros([12,1])
                q_read = baxter.joint_positions
                qL = q_read[0:7]
                qR = q_read[7:14]                
                q = np.hstack([qL,qR,np.array([0,0,0])]).reshape([17,1])
                
                t05 = timeit.default_timer()-tic
                
                JL = robotjacobian(Bdef.left.H, Bdef.left.P, Bdef.left.joint_type, qL)
                JR = robotjacobian(Bdef.right.H, Bdef.right.P, Bdef.right.joint_type, qR)
                t075 = timeit.default_timer()-tic
                
                pp_L,RR = fwdkin_alljoints(qL, Bdef.left.joint_type, Bdef.left.H, Bdef.left.P, 7)
                p_BTL = pp_L[:, -1]
                pp_R,RR = fwdkin_alljoints(qR, Bdef.right.joint_type, Bdef.right.H, Bdef.right.P, 7)
                p_BTR = pp_R[:, -1]
                JT = getJT(JL,JR,p_BTL,p_BTR)
                

                t1 = timeit.default_timer()-tic
                
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
                    
                t2 = timeit.default_timer()-tic
                ### Kinect Pose Control
                #print (joint_state, pose)
                #print ('joint_state: ',joint_state[iL],joint_state[iR])

                human_R = pose[iR,:]
                human_L = pose[iL,:]                 
                #print human_R,human_L
                if cnt_small_motion<Small_Motion_TH:#(joint_state[iL]==1) and (joint_state[iR]==1):
                   
                    p_L = np.dot(Rk,np.dot(R_K2B,human_L))+pk
                    p_R = np.dot(Rk,np.dot(R_K2B,human_R))+pk
#                    p_L = np.dot(R_K2B,human_L))
#                    p_R = np.dot(R_K2B,human_R)
                    
                    t25 = timeit.default_timer()-tic
                    ## Kalman Iteration
                    Z = np.vstack([p_L,p_R]).reshape([6,1])
                    P = np.dot(np.dot(Phi,P),Phi.T) + Q
                    S = np.dot(np.dot(M,P),M.T) + R
                    K = np.dot(np.dot(P,M.T),np.linalg.inv(S))
                    P = P - np.dot(np.dot(K,M),P)    
                    Xk = np.dot(Phi,Xk_prev) + np.dot(K,(Z-np.dot(np.dot(M,Phi),Xk_prev)))
                    
                    
                    #Xk = np.vstack([Z, np.zeros([6,1])]).reshape([12,1])
                    # For the next iteration                  
                    joint_velocity = Xk-Xk_prev

                    p_LR = (Xk[3:6,:]-Xk[0:3,:]).reshape([3,])
                    p_LR = p_LR/np.linalg.norm(p_LR)
                    
                    vc = 1*(joint_velocity[3:6,:]+joint_velocity[0:3,:]).reshape([3,1])
                    wc = 10*(np.cross(p_LR_prev,p_LR)*np.arccos(np.dot(p_LR_prev,p_LR))).reshape([3,1])
                    Vc = np.vstack([wc,vc])
                    #print wc
                    p_elbow_L = np.dot(R_K2B,pose[elbow_L,:])
                    p_elbow_R = np.dot(R_K2B,pose[elbow_R,:])
                    
#                    print ('p_elbow_L,p_elbow_R: ',((p_elbow_L-p_elbow_L_prev)[-1]),((p_elbow_R-p_elbow_R_prev)[-1]))
#                    print ('extend: ',np.linalg.norm(human_R-human_L))
                   
                    if np.linalg.norm(human_R-human_L)>1.2:#(((p_elbow_L-p_elbow_L_prev)[-1]) > 0.1) and (((p_elbow_R-p_elbow_R_prev)[-1]) > 0.1):
                        #print ('small motion **********************')
                        cnt_small_motion += 1
                        print('Stage II: ',np.linalg.norm(human_R-human_L))
                        Vc = np.array([0,0,0,0,0,0]).reshape([6,1])
                        q_read = baxter.joint_positions
#                    else:
#                        if cnt_small_motion<5:
#                            cnt_small_motion = 0
                    
                    AL = np.vstack([np.hstack([np.eye(3),np.zeros([3,3])]),np.hstack([-hat(-h_LR*0.5),np.eye(3)])])
                    AR = np.vstack([np.hstack([np.eye(3),np.zeros([3,3])]),np.hstack([-hat(h_LR*0.5),np.eye(3)])])
                    
                    VT[0:6] += 5*np.dot(AL,Vc)
                    VT[6:12] += 5*np.dot(AR,Vc)
#                    VT[3:6] = vL_force.reshape([3,1])+VT[3:6]   
#                    VT[9:12] = vR_force.reshape([3,1])+VT[9:12]
                    #print 'VT: ',VT

                                       
                    #joint_positions_prev = joint_positions
                    p_LR_prev = p_LR
                    Xk_prev = Xk 
                    
                    p_elbow_L_prev = p_elbow_L
                    p_elbow_R_prev = p_elbow_R


                    
                else:
                    t25 = timeit.default_timer()-tic

                    if cnt_small_motion < Small_Motion_TH+1:               
                        baxter.setControlMode(0)
                        qL = q_read[0:7]
                        qR = q_read[7:14]
                        baxter.setJointCommand('left',qL)
                        baxter.setJointCommand('right',qR) 
                        time.sleep(2)
                        JL = robotjacobian(Bdef.left.H, Bdef.left.P, Bdef.left.joint_type, qL)
                        JR = robotjacobian(Bdef.right.H, Bdef.right.P, Bdef.right.joint_type, qR)
                        
                        pp_L,RR = fwdkin_alljoints(qL, Bdef.left.joint_type, Bdef.left.H, Bdef.left.P, 7)
                        p_BTL = pp_L[:, -1]
                        pp_R,RR = fwdkin_alljoints(qR, Bdef.right.joint_type, Bdef.right.H, Bdef.right.P, 7)
                        p_BTR = pp_R[:, -1]
                        JT = getJT(JL,JR,p_BTL,p_BTR) 
                        cnt_small_motion +=1
                        
                        K_com = 0.006
                        print ('====================== Stop ======================')
                        
                    else:
                        baxter.setControlMode(1)
                        
                        
                    
                    ### Commnad BOW Robot
                        
#                    print ('====================== Stop ======================')    
                    dq_pre = np.zeros([17,1]) 
#                    dqL = 0.5*dqL#np.zeros([7,])
#                    dqR = 0.5*dqR#np.zeros([7,])   
#                    dqB = 0.5*dqB#np.zeros([3,])
#                    baxter.setJointCommand('left',dqL)
#                    baxter.setJointCommand('right',dqR)   
#                    ridgeback_vel_msg.linear.x = dqB[1]
#                    ridgeback_vel_msg.linear.y = dqB[2]
#                    ridgeback_vel_msg.angular.z = dqB[0] 
#                    ridgeback.velocity_publisher.publish(ridgeback_vel_msg)   
#                    dq_pre = 0.5*dq_pre#np.zeros([17,1]) 
                    
                Xk_all.append(Xk.T)
                Z_all.append(Z.T)                     
                VT_all.append(VT)  
                
                t3 = timeit.default_timer()-tic
                
                Closest_Pt_l , Closest_Pt_env_l, Closest_Pt_r , Closest_Pt_env_r = collision_check(pp_L,pp_R)
                dq_sln = QP_bow(JT,VT,Lambda,Epsilon1,q,Epsilon2,dq_pre, Closest_Pt_l , Closest_Pt_env_l, Closest_Pt_r , Closest_Pt_env_r)
                #print dq_sln
                #print np.dot(np.linalg.pinv(JT),Vd)
                #print 'dq_sln: ',dq_sln
                if any(np.isnan(dq_sln)):
                    dq_sln = np.zeros([18,])
                    
                dq_pre = dq_sln[0:17].reshape([17,1]) 
                
                t4 = timeit.default_timer()-tic
                ### Commnad BOW Robot
                dqL = dq_sln[0:7]
                dqR = dq_sln[7:14]   
                dqB = dq_sln[14:17]
                baxter.setJointCommand('left',dqL)
                baxter.setJointCommand('right',dqR) 
                
                t5 = timeit.default_timer()-tic
                
                
                ridgeback_vel_msg.linear.x = dqB[1]
                ridgeback_vel_msg.linear.y = dqB[2]
                ridgeback_vel_msg.angular.z = dqB[0] 
                ridgeback.velocity_publisher.publish(ridgeback_vel_msg)
                ridgeback.rate.sleep()  
                
                t6 = timeit.default_timer()-tic
                
                rc = quaternion_matrix(ridgeback_state[-4:])[0:3,0:3]
                pc = ridgeback_state[0:3]
                
                print('odem: ',pc-p0)
                
                #Rk = np.dot(rc, r0.T)[0:3,0:3]
                Rk = np.dot(r0, rc.T)
                pk = pc - p0
                
                claps_norm = np.linalg.norm(human_R-human_L)
#                print ('Claps Norm:', claps_norm)           
                if claps_norm <0.1: 
                    claps = 1
                    
                toc = timeit.default_timer()-tic
                if toc< dt:
                    time.sleep(dt-toc)
                else:
                    print ('exceed sampling time !!', toc)     
                t7 = timeit.default_timer()-tic
                t_all.append([t05,t075,t1,t2,t25, t3,t4,t5,t6,t7])
                
    
        sio.savemat('Data20200309_1.mat', {'t_all':t_all,'VT_all':VT_all,'Xk_all':Xk_all,'Z_all':Z_all})    
        print ('Time Mean: ',np.mean(t_all,0))
        print ('Time Max:  ',np.max(t_all,0))
        print (np.std(Z_all, axis=0))


               
    elif Select == 1:
        print ('end')
        
    baxter.setControlMode(0)
    baxter.setJointCommand('left',q0L)
    baxter.setJointCommand('right',q0R) 
