#!/usr/bin/env python
import math
import numpy as np
import rospy 
import tf
from geometry_msgs.msg import Twist
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
    ti = 0 #initial and final time
    tf = 0

    xi  = 0 # initial and final position
    yi  = 0
    psi = 0
    xf  = 5
    yf  = 5

    # Quadrante com erro xf  = -5 yf  = 5


    vr = 1 #reference velocity
    
    # reference trajectory Parametes
    xRef = np.array([0,0,0])
    dxRef = np.array([0,0])
    d2xRef = np.array([0,0])

    xActual = np.array([0,0,0])

    #messages parameters
    odomMsg = Odometry()
    joyMsg = Twist()

    cmd_vel = Twist()

    # ROS variables
    pub = rospy.Publisher('/sim_p3at/cmd_vel', Twist, queue_size=100)

    def __init__(self):
        self.x1_cir = 0.008
        self.Kx     = 0.2
        self.Ky     = 3.0
        self.Kpsi   = 1.0
        self.T      = 1/100
        #self.odomMsg = None
        #self.joyMsg = None
        #self.to = rospy.Time.now
        # generates reference trajectory based on initial and final positions
        self.traj_gen()

    def getFrequency(self):
        return self.T

    def setOdomMsg(self, msg):
        self.odomMsg = msg
        x = self.odomMsg.pose.pose.orientation.x
        y = self.odomMsg.pose.pose.orientation.y
        z = self.odomMsg.pose.pose.orientation.z
        w = self.odomMsg.pose.pose.orientation.w
        angles = tf.transformations.euler_from_quaternion([x,y,z,w])
        #rospy.loginfo(angles)
        self.xActual[0] = self.odomMsg.pose.pose.position.x
        self.xActual[1] = self.odomMsg.pose.pose.position.y
        self.xActual[2] = angles[2]

        #rospy.loginfo(self.xActual)

    
    def setJoyMsg(self, msg):
        self.joyMsg = msg
        
    def traj_pol (self, a):
        delta_t = self.tf - self.ti
        
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


    #def traj_gen(vr, traj_origin, traj_final):
    def traj_gen(self):
        # defined by a fifth degree polynomial function
        # q = a0 + a1*(t-t0) + a2*(t-t0)^2 + a3*(t-t0)^3 + a4*(t-t0)^4 +a5*(t-t0)^5
        # from initial and final conditions (x,y,t) its possible to get reference trajectory 
        # and your derivates in each timestamp t
        
        # INITIAL CONDITIONS
        to = self.ti
        xo = self.xi
        dxo = 0
        d2xo = 0
        yo = self.yi
        dyo = 0
        d2yo = 0

        # FINAL CONDITION
        tf = self.tf
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
        tf = delta_t + to

        ax = self.par_pol(to,tf,xo,dxo,d2xo,xf,dxf,d2xf)
        ay = self.par_pol(to,tf,yo,dyo,d2yo,yf,dyf,d2yf)
        
        #after clculation update final time with
        self.tf = tf

        refX = self.traj_pol(ax) #[qx, dqx, ddqx, dddqx]
        refY = self.traj_pol(ay) #[qy, dqy, ddqy, dddqy]
        
        #psi = atan2 Refy[1]/refX[1]
        self.psi = math.atan2(refY[1],refX[1])        
        self.xRef = [refX[0],refY[0],self.psi]
        
        #dxREF
        #w_ref= atan2 Refy[2]/refX[2]
        refW = (1/(((refY[1]/refX[1])**2)+1)) + (refY[2]/refX[2])
        self.dxRef = [refX[1],refW]
        
        #d2xREF
        #dwref = atan2 Refy[2]/refX[2] ddd
        dwref = (((refY[1]/refX[1])*(-2.0) * (refY[2]**2/refX[2]**2))/(1+((refY[1]/refX[1])**2)))+ ((refY[3]/refX[3])* (1+(1+(refY[1]/refX[1])**2)))
        self.d2xRef = [refX[2],dwref]

    def normalize_angle(self,ang, low):
        return ang - 2*math.pi*math.floor((ang-low)/(2*math.pi))

    def traj_desired(self):
        # used to generate desired velocities in order to keep robot on reference trajectory
        
        T = self.T
       
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
        #vr = self.vr
        dpsir = self.dxRef[1]

        # dvRef (dv_r, d2psi_r)
        dvr = self.d2xRef[0]
        d2psir = self.d2xRef[1]

        # X_actual(x, y, psi)
        x = self.xActual[0]
        y = self.xActual[1]
        psi = self.normalize_angle(self.xActual[2], math.pi*(-1.0))

        # position error calculation (xe, ye, psie)
        xe = math.cos(psi)*(xr-x) + math.sin(psi)*(yr-y)
        ye = math.sin(psi)*(xr-x)*(-1) + math.cos(psi)*(yr-y)
        psie = self.normalize_angle((psir - psi), (-1.0)*math.pi)
        
        #psie = self.subtract_angle(psir,psi)

        v_d = vr*math.cos(psie) + self.Kx*xe
        w_d = dpsir + Ky*vr*ye + Kpsi*vr*math.sin(psie)
############################
        #dxe = ye*w_d - v_d + vr*math.cos(psie)
        #dye = -1.0 * (xe*w_d + vr*math.sin(psie))
        #dpsie = dpsir - w_d

        #dv_d = dvr*math.cos(psie) - vr*math.sin(psie) * dpsie +Kx*dxe
        #dw_d = d2psir + Ky* (dvr*ye + vr*dye) + Kpsi * (dvr*math.sin(psie) + vr*math.cos(psie) * dpsie)

        #knematic model

        #S = np.array([
        #    [math.cos(psi), x1_cir*math.sin(psi)],
        #    [math.sin(psi), x1_cir*math.cos(psi) *(-1.0)],
        #    [0, 1]
        #])

        #dq_d = S.dot(np.array([v_d, w_d]))

        #dx = dq_d[0]
        #y = dq_d[1]
        #dpsi = dq_d[2]

        #X_desired = [
            #dx*T + self.xActual[0],
            #dy*T + self.xActual[1],
            #dpsi*T + self.xActual[2]
        #]

        V_desired = [
            v_d, w_d
        ]

        #dV_desired = [
        #    dv_d, dw_d
        #]

        #result = [X_desired, V_desired, dV_desired]

        # Setting Twist Message in order to send to CMD_VEL TOPIC
        self.cmd_vel.linear.x = V_desired[0]
        self.cmd_vel.angular.z = w_d
        rospy.loginfo(V_desired)

        #publishing Velocities
        self.pub.publish(self.cmd_vel)
        #rospy.loginfo(result[1])

    def odom_callback(self, data, topic):
        #rospy.loginfo( data.pose.pose )
        self.setOdomMsg(data)
        self.traj_desired()
        


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

    def run(self):
        # subscribe odometry topic from robot
        rospy.Subscriber('/sim_p3at/odom', Odometry, callback=self.odom_callback , callback_args='/sim_p3at/odom')
        
        # subscribe joy topic from joystic 
        #rospy.Subscriber('/mobile/joy', Twist,callback=self.joy_callback,callback_args='/mobile/joy')
        
        rospy.spin()

if __name__ ==  '__main__':
    rospy.loginfo("Starting trajectoryNode...")
    pioneer = Mobile()
    rospy.init_node('trajectoryNode')
    pioneer.run()
    rate=rospy.Rate(100)
    while not rospy.is_shutdown():
        rate.sleep()