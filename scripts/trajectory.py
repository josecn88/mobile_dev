#!/usr/bin/env python
import math
import numpy as np
import rospy 
import tf
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry

 ###################################################
 # trajectory.py
 # Created in: April 3rd, 2021 by josecn
 ###################################################

class Mobile:
    #mobile controller parameters
    x1_cir = 0.00
    Kx     = 0.0
    Ky     = 0
    Kpsi   = 0
    T      = 0
    
    #trajectory parameters
    ti = 0.0 #initial and final time
    tf = 0.0
    t  = 0
    to = 0 

    xi  = 0 # initial and final position
    yi  = 0
    psi = 0
    xf  = 0
    yf  = 0
    ax = 0
    ay = 0 
     
    newTarget = 0
    newTraj = 0
    vr = 0.5 #reference velocity 0.1 ok
    
    # reference trajectory Parametes
    xRef = np.array([0,0,0])
    dxRef = np.array([0,0])
    d2xRef = np.array([0,0])

    # stores actual state got from odometry 
    # [pose.x, pose.y, yaw_angle, linear.x, linear.y, angular.z]
    stateActual = np.array([0.0,0.0,0.0,0.0,0.0])

    #messages parameters
    odomMsg = Odometry()
    joyMsg = Twist()
    cmd_vel = Twist()
    poseMsg = Pose()

    # ROS variables
    #### Mudar aqui para Publisher Simulador ou Robo Real ################
    #pub = rospy.Publisher('/sim_p3at/cmd_vel', Twist, queue_size=100)
    pub = rospy.Publisher('/lynxMotion/cmd_vel', Twist, queue_size=100)
    ###################################################################
    #now = rospy.Time.now()

    #CSV files
    desiredTrajFile = 0
    referenceTrajFile = 0

    def __init__(self):
        self.x1_cir = 0.008
        self.Kx     = 0.5 #0,5 3,0 1,0
        self.Ky     = 3.0
        self.Kpsi   = 1.0
        self.T      = 0.01

        # generates reference trajectory based on initial and final positions

        #self.traj_gen()
        rospy.init_node('trajectoryNode')

        #### Mudar aqui para Subscriber Simulador e Robo Real #####
        #rospy.Subscriber('/sim_p3at/odom', Odometry, callback=self.odom_callback , callback_args='/sim_p3at/odom')
        rospy.Subscriber('/lynxMotion/odom', Odometry, callback=self.odom_callback , callback_args='/lynxMotion/odom')
        rospy.Subscriber('/target/pose', Odometry, callback=self.pose_callback , callback_args='/target/pose')

    def getFrequency(self):
        return self.T

    def setOdomMsg(self, msg):
        self.odomMsg = msg
        position_list = self.odomMsg.pose.pose.position
        orientation_q = self.odomMsg.pose.pose.orientation
        linear_vel = self.odomMsg.twist.twist.linear
        angular_vel = self.odomMsg.twist.twist.angular
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(orientation_list)
        self.stateActual = [position_list.x, position_list.y, yaw,linear_vel.x, linear_vel.y,angular_vel.z]
        
        #rospy.loginfo("X, Y, Yaw, Linear Velocities[x,y], Angular Velocities[z] : %s", self.stateActual)
        #data = str(self.t)+','+str(position_list.x)+','+str(position_list.y)+','+str(yaw)+','+str(linear_vel.x)+','+str(linear_vel.y)+','+str(angular_vel.z)+'\n'
        #self.desiredTrajFile.write(data)
    
    def setJoyMsg(self, msg):
        self.joyMsg = msg
        
    def traj_pol (self, a, tf, ti):
        delta_t = tf - ti
        
        q    =   a[0] +    a[1]*delta_t +    a[2]*delta_t**2 +    a[3]*delta_t**3 +   a[4]*delta_t**4 + a[5]*delta_t**5
        dq   =   a[1] +  2*a[2]*delta_t +  3*a[3]*delta_t**2 +  4*a[4]*delta_t**3 + 5*a[5]*delta_t**4
        ddq  = 2*a[2] +  6*a[3]*delta_t + 12*a[4]*delta_t**2 + 20*a[5]*delta_t**3
        dddq = 6*a[3] + 24*a[4]*delta_t + 60*a[5]*delta_t**2
        result = np.array([q,dq,ddq,dddq])
        return result

    def par_pol(self,to, tf, qo, dqo, d2qo, qf, dqf, d2qf):
        # performs 5th dedree polinomial matrix multiplication a = A^-1 * q
        q_v = np.array([qo, dqo, d2qo, qf, dqf, d2qf])
        delta_t = tf-to
        A = np.array([[1, 0, 0, 0, 0, 0],
                    [0, 1, 0, 0, 0, 0],
                    [0, 0, 2, 0, 0, 0],
                    [1, delta_t, delta_t**2, delta_t**3, delta_t**4, delta_t**5],
                    [0, 1, 2*delta_t, 3*delta_t**2, 4*delta_t**3, 5*delta_t**4],
                    [0, 0, 2, 6*delta_t, 12*delta_t**2, 20*delta_t**3]])
        Ai = np.linalg.inv(A)
        a = Ai.dot(q_v)
        return a

    def normalize_angle(self,ang, low):
        return ang - 2*math.pi*math.floor((ang-low)/(2*math.pi))

    def traj_desired(self):
        # used to generate desired velocities in order to keep robot on reference trajectory
        
        #T = self.T
       
        x1_cir = self.x1_cir
        Kx = self.Kx
        Ky = self.Ky
        Kpsi = self.Kpsi

        # xRef (xr, yr, psi_r)
        xr = self.xRef[0]
        yr = self.xRef[1]
        psir = self.xRef[2]

        # vRef (vr, dpsi_r)
        vr = self.dxRef[0]
        dpsir = self.dxRef[1]
     

        # dvRef (dv_r, d2psi_r)
        dvr = self.d2xRef[0]
        d2psir = self.d2xRef[1]

        # X_actual(x, y, psi)
        x = self.stateActual[0]
        y = self.stateActual[1]
        psi = self.normalize_angle(self.stateActual[2], math.pi*(-1.0))

        # position error calculation (xe, ye, psie)
        xe = math.cos(psi)*(xr-x) + math.sin(psi)*(yr-y)
        ye = math.sin(psi)*(xr-x)*(-1) + math.cos(psi)*(yr-y)
        psie = self.normalize_angle((psir - psi), (-1.0)*math.pi)
        #rospy.loginfo("erro     : %s",[xe,ye,psie])
        #rospy.loginfo("ref      : %s",[xr,yr,psir])
        #rospy.loginfo("vr       : %s",vr)
        
        v_d = vr*math.cos(psie) + self.Kx*xe
        #rospy.loginfo(dpsir)
        #rospy.loginfo("Ky*vr*ye :%s",Ky*vr*ye)
        #rospy.loginfo("kpsi     : %s" , Kpsi*vr*math.sin(psie))

        w_d = dpsir + Ky*vr*ye + Kpsi*vr*math.sin(psie)

        #rospy.loginfo("vd,omegad: %s",[v_d,w_d])
        V_desired = [
            v_d, w_d
        ]
        # Setting Twist Message in order to send to CMD_VEL
        self.cmd_vel.linear.x = v_d
        self.cmd_vel.angular.z = w_d
        #rospy.loginfo(V_desired)

        #publishing Velocities
        self.pub.publish(self.cmd_vel)

    def odom_callback(self, data, topic):
        self.setOdomMsg(data)


    # get Pose mesage to update x,y reference point and reference velocity vr
    def pose_callback(self, data, topic):
        if self.newTarget == 0 and self.newTraj == 0:
            self.newTarget = 1
            self.poseMsg = data
            self.xf = self.poseMsg.pose.pose.position.x
            self.yf = self.poseMsg.pose.pose.position.y
            self.vr = self.poseMsg.twist.twist.linear.x
            #rospy.loginfo(self.poseMsg)


    def setTime(self):
        while True:
            time = rospy.get_time()
            if time > 0:
                break
        return time
        


    def joy_callback(self, data, topic):
        #rospy.loginfo(data)
        self.setJoyMsg(data)
        self.t = rospy.Time.now
        joyX = self.joyMsg.linear.x
        joyZ = self.joyMsg.angular.z
        self.cmd_vel.linear.x = joyX
        self.cmd_vel.angular.z = joyZ
        #rospy.loginfo(self.cmd_vel)
        self.pub.publish(self.cmd_vel)

    def cmdPublisher(self):
        #rospy.loginfo("entrei no cmd publish")
        self.pub.publish(self.cmd_vel)

    def control(self):
        
        #self.t = self.t + self.T
        #rospy.loginfo( "ti     : %s", self.ti )
        self.t = self.setTime()
        #rospy.loginfo( "t      : %s", self.t )
        self.t = self.t - self.ti
        #rospy.loginfo( "t - ti : %s", t )
        if self.newTarget == 1 :
            # INITIAL CONDITIONS
            self.to = self.t
            xo = self.stateActual[0]
            dxo = self.stateActual[3]
            d2xo = 0
            yo = self.stateActual[1]
            dyo = self.stateActual[4]
            d2yo = 0

            # FINAL CONDITION
            #tf = self.tf
            xf = self.xf
            dxf = 0
            d2xf = 0
            yf = self.yf
            dyf = 0
            d2yf = 0

            vr = self.vr

            if vr==0:
                return -1
            
            if (xf-xo) == 0 and (yf-yo):
                return -1

            delta_d = math.sqrt((xf-xo)**2 + (yf-yo)**2)
            delta_t = delta_d/vr # delta_t = (tf - to)
            self.tf = delta_t + self.to

            self.ax = self.par_pol(self.to,self.tf,xo,dxo,d2xo,xf,dxf,d2xf)
            self.ay = self.par_pol(self.to,self.tf,yo,dyo,d2yo,yf,dyf,d2yf)   
            self.newTarget = 0
            self.newTraj = 1
        
        if self.newTraj == 1:
            refX =  self.traj_pol(self.ax, self.t, self.to) #[qx, dqx, ddqx, dddqx]
            refY =  self.traj_pol(self.ay, self.t, self.to) #[qy, dqy, ddqy, dddqy]    
            rospy.loginfo("Ref x  : %s", refX[0])
            rospy.loginfo("Ref y  : %s", refY[0])
            rospy.loginfo("tf     : %s", self.tf)
            rospy.loginfo("t      : %s", self.t)
            #psi = atan2 Refy[1]/refX[1]
            self.psi = math.atan2(refY[1],refX[1])        
            self.xRef = [refX[0],refY[0],self.psi]
            
            #dxREF - cmd_ref
            #w_ref= atan2 Refy[2]/refX[2]
            #refW = (1/(((refY[1]/refX[1])**2)+1)) + (refY[2]/refX[2])
            #df = -(y)/(y^2 + x^2)
            #refW = ()
            refW = 0
            refV = math.sqrt(refX[1]**2 + refY[1]**2)
            self.dxRef = [refV,refW]
            rospy.loginfo(self.dxRef)

            #d2xREF
            #dwref = (((refY[1]/refX[1])*(-2.0) * (refY[2]**2/refX[2]**2))/(1+((refY[1]/refX[1])**2)))+ ((refY[3]/refX[3])* (1+(1+(refY[1]/refX[1])**2)))
            #dvref = math.sqrt(refX[2]**2 + refY[2]**2)
            self.d2xRef = [0,0]
            #data = str(self.t)+','+str(self.xRef[0]) +','+ str(self.xRef[1])+','+str(self.xRef[2])+','+str(self.dxRef[0])+','+str(self.dxRef[1])+','+str(self.d2xRef[0])+','+str(self.d2xRef[1])+'\n'
            #self.referenceTrajFile.write(data)
            self.traj_desired()
            
            if self.t > self.tf:
            #if math.sqrt((self.xf - self.stateActual[0])**2 + (self.yf - self.stateActual[1])**2) < 2:
                self.newTraj = 0
            
            
        else:
            self.cmd_vel.linear.x = 0
            self.cmd_vel.angular.z = 0
            #rospy.loginfo("cmd_vel msg: %s", self.cmd_vel)
            self.pub.publish(self.cmd_vel)



    def run(self):
        #setting initial time
        self.ti = self.setTime()
        rate=rospy.Rate(100)
        #trajReference = "refTraj"+str(int(self.ti))+".csv"
        #trajDesired   = "desTraj"+str(int(self.ti))+".csv"
        #self.referenceTrajFile = open(trajReference,"w")
        #self.desiredTrajFile = open(trajDesired, "w")
        while not rospy.is_shutdown():
            self.control()
            rate.sleep()
        rospy.spin()

        #self.desiredTrajFile.close()
        #self.referenceTrajFile.close()

if __name__ ==  '__main__':

    rospy.loginfo("Starting trajectoryNode")
    pioneer = Mobile()
    pioneer.run()
