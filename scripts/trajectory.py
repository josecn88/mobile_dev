#!/usr/bin/env python
import math
import numpy as np
import rospy 
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

 ###################################################
 # trajectory.py
 # Created in: April 3rd, 2021 by josecn
 ###################################################

class Mobile:
    #mobile controller parameters
    x1_cir = 0.00
    Kx = 0.0
    Ky = 0
    Kpsi = 0
    T = 0
    
    #trajectory parameters
    to = 0
    t = 0

    xo = 0
    yo = 0

    xf = 0
    yf = 0
    
    #messages parameters
    odomMsg = Odometry()
    joyMsg = Twist()

    nextPoint = Twist()

    # ROS variables
    pub = rospy.Publisher('/sim_p3at/cmd_vel', Twist, queue_size=10)

    def __init__(self):
        self.x1_cir = 0.008
        self.Kx = 0.5
        self.Ky = 3
        self.Kpsi = 1
        self.T = 1/25
        #self.odomMsg = None
        #self.joyMsg = None
        self.to = rospy.Time.now
        

    def setOdomMsg(self, msg):
        self.odomMsg = msg
    
    def setJoyMsg(self, msg):
        self.joyMsg = msg
        
    def traj_pol (self, a):
        delta_t = self.t - self.to
        
        q    =   a[0] +    a[1]*delta_t +    a[2]*delta_t**2 +    a[3]*delta_t**3 +   a[4]*delta_t**4 + a[5]*delta_t**5
        dq   =   a[1] +  2*a[2]*delta_t +  3*a[3]*delta_t**2 +  4*a[4]*delta_t**3 + 5*a[5]*delta_t**4
        ddq  = 2*a[2] +  6*a[3]*delta_t + 12*a[4]*delta_t**2 + 20*a[5]*delta_t**3
        dddq = 6*a[3] + 24*a[4]*delta_t + 60*a[5]*delta_t**2
        result = np.array([q,dq,ddq,dddq])
        return result

    def par_pol(to, tf, qo, dqo, d2qo, qf, dqf, d2qf):
        # performs 5th dedree polinomial matrix multiplication a = A^-1 * q
        q_v = np.array(qo, dqo, d2qo, qf, dqf, d2qf)
        delta_t = tf-to
        A = np.array([[1, 0, 0, 0, 0, 0],
                    [0, 1, 0, 0, 0, 0],
                    [0, 0, 2, 0, 0, 0],
                    [1, delta_t, delta_t**2, delta_t**3, delta_t**4, delta_t**5],
                    [0, 1, 2*delta_t, 3*delta_t**2, 4*delta_t**3, 5*delta_t**4],
                    [0, 0, 2, 6*delta_t, 12*delta_t**2, 20*delta_t**3]])
        Ai = np.linalg.inv(A)
        a = np.array(Ai * q_v)
        return a


    def traj_gen(vr, traj_origin, traj_final):
        # defined by a fifth degree polynomial function
        # q = a0 + a1*(t-t0) + a2*(t-t0)^2 + a3*(t-t0)^3 + a4*(t-t0)^4 +a5*(t-t0)^5
        # from initial and final conditions (x,y,t) its possible to get reference trajectory 
        # and your derivates in each timestamp t
        
        # INITIAL CONDITIONS
        to = traj_origin[0]
        xo = traj_origin[1]
        dxo = traj_origin[2]
        d2xo = traj_origin[3]
        yo = traj_origin[4]
        dyo = traj_origin[5]
        d2yo = traj_origin[6]

        # FINAL CONDITION
        tf = traj_final[0]
        xf = traj_final[1]
        dxf = traj_final[2]
        d2xf = traj_final[3]
        yf = traj_final[4]
        dyf = traj_final[5]
        d2yf = traj_final[6]

        if vr==0:
            return -1
        
        if (xf-xo) == 0 and (yf-y):
            return -1

        delta_d = math.sqrt((xf-xo)**2 + (yf-yo)**2)
        delta_t = delta_d/vr # delta_t = (tf - to)
        tf = delta_t + to

        ax = par_pol (to,tf,xo,dxo,d2xo,xf,dxf,d2xf)
        ay = par_pol (to,tf,yo,dyo,d2yo,yf,dyf,d2yf)
        
        result[0] = ax
        result[1] = ay
        result[2] = tf

        return result
    def normalize_angle(self,ang, low):
        return ang - 2*math.pi*math.floor((ang-low)/(2*math.pi))

    def subtract_angle (self,ang1,ang2):
        temp1 = ang1
        if temp1 > 0:
            temp1 = temp1 - 2.0*math.pi*math.floor(temp1/(2.0*math.pi))
        
        if temp1 < 0:
            temp1 = 2*math.pi - ((-1.0 *temp1) - (2*math.pi*math.floor(-1.0*temp1/(2.0*math.pi))))

        temp2 = ang2
        if temp2 > 0:
            temp2 = temp2 - 2.0*math.pi*math.floor(temp2/(2.0*math.pi))
        
        if temp2 < 0:
            temp2 = 2*math.pi - ((-1.0 *temp2) - (2*math.pi*math.floor(-1.0*temp2/(2.0*math.pi))))

        diff = temp1-temp2

        if diff > math.pi:
            diff = diff - 2.0*math.pi

        if diff < (-1.0*math.pi):
            diff = diff + 2.0*math.pi
        
        return diff

    def traj_desired(T,X_ref, V_ref, dV_ref, X_actual, wmr):
        # used to generate desired velocities in order to keep robot on reference trajectory
        
        # load wmr params - ATENTION get from ROS param after testing ********************
        #x1_cir = wmr[0]
        #Kx = wmr[1]
        #Ky = wmr[2]
        #Kpsi = wmr[3]

        # X_ref (xr, yr, psi_r)
        xr = X_ref[0]
        yr = X_ref[1]
        psir = X_ref[2]

        # V_ref (vr, dpsi_r)
        vr = V_ref[0]
        dpsir = V_ref[2]

        # dV_ref (dv_r, d2psi_r)
        dvr = dV_ref[0]
        d2psir = dV_ref[1]

        # X_actual(x, y, psi)
        x = X_actual[0]
        y = X_actual[1]
        psi = X_actual[2]

        # position error calculation (xe, ye, psie)
        xe = math.cos(psi)*(xr-x) + math.sin(psi)*(yr-y)
        ye = math.sin(psi)*(xr-x)*(-1) + math.cos(psi)*(yr-y)
        psie = subtract_angle(pisr,psi)

        v_d = vr*math.cos(psie) + Kx*xe
        w_d = dpsir = Ky*vr*ye + Kpsi*vr*math.sin(psie)

        dxe = ye*w_d - v_d + vr*math.cos(psie)
        dye = -1.0 * (xe*w_d + vr*math.sin(psie))
        dpsie = dpsir - w_d

        dv_d = dvr*math.cos(psie) - vr*math.sin(psie) * dpsie +Kx*dxe
        dw_d = d2psir + Ky* (dvr*ye + vr*dye) + Kpsi * (dvr*math.sin(psie) + vr*math.cos(psie) * dpsie)

        #knematic model

        S = np.array([
            [math.cos(psi), x1_cir*math.sin(psi)*-1.0],
            [math.sin(psi), x1_cir*math.cos(psi)],
            [0, 1]
        ])

        dq_d = S* np.array([v_d, w_d])

        dx = dq_d[0]
        dy = dq_d[1]
        dpsi = dq_d[2]

        X_desired = np.array([
            dx*T + X_actual[0],
            dy*T + X_actual[1],
            dpsi*T + X_actual[2]
        ])

        V_desired = np.array([
            v_d, w_d
        ])

        dV_desired = np.array([
            dv_d, dw_d
        ])

        result = [X_desired, V_desired, dV_desired]

        return result

    def odom_callback(self, data, topic):
        #rospy.loginfo( data.pose.pose )
        self.setOdomMsg(data)


    def joy_callback(self, data, topic):
        #rospy.loginfo(data)
        self.setJoyMsg(data)
        self.t = rospy.Time.now
        joyX = self.joyMsg.linear.x
        joyZ = self.joyMsg.angular.z
        self.nextPoint.linear.x = joyX
        self.nextPoint.angular.z = joyZ
        #rospy.loginfo(self.nextPoint)
        self.pub.publish(self.nextPoint)

    def cmdPublisher(self):
        rospy.loginfo("entrei no cmd publish")
        self.pub.publish(self.nextPoint)

    def run(self):
        # subscribe odometry topic from robot
        rospy.Subscriber('/sim_p3at/odom', Odometry, callback=self.odom_callback , callback_args='/sim_p3at/odom')
        
        # subscribe joy topic from joystic 
        rospy.Subscriber('/mobile/joy', Twist,callback=self.joy_callback,callback_args='/mobile/joy')
        
        # update trajectory parameters based on odom and joystick

        # generate reference trajectory 

        # generate desired trajectory

        # publish trajectory
        
        rospy.spin()

    

    
        

if __name__ ==  '__main__':
    rospy.loginfo("Starting trajectoryNode...")
    pioneer = Mobile()
    rospy.init_node('trajectoryNode')
    pioneer.run()
    rate=rospy.Rate(25)
    while not rospy.is_shutdown():
        rate.sleep()