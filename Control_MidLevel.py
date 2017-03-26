'''
    Control_MidLevel.py
    
    Description: Library containing the mid level controllers
                 of the autopilot.
    
    Revision History
    3 Feb 2017 - Created and debugged
    
    Author: Lars Soltmann
    
    References: None
    
    Calls: PID.py
    
    Notes:
    - Written for Python3
    
    Inputs: psi_target - commanded heading (deg)
            psi_meas - measured heading (deg)
            h_target - commanded altitude (ft)
            h_meas - measured altitude (ft)
            V_target - commanded velocity (ft/s)
            V_meas - measured velocity (ft/s)
            
    Outputs: phi - commanded roll angle (deg)
             theta - commanded pitch angle (deg)
             d_T - throttle (%)
             
    
    OPEN ITEMS:
    1. Heading - problem when passing 360 with PID controller
    
'''

from PID import PID

class ML_controls:
    def __init__(self):
        ## Aircraft state limits
        self.max_roll_angle= 40 #deg
        self.max_pitch_angle=25 #deg
        self.min_airspeed=   22 #ft/s
        self.max_throttle=  100 #%
        
        ## Airspeed/altitude controller switch limit
        self.h_V_switch=20 #ft
        
        ## Controller gains
        phi_kp=    1
        phi_kd=    0
        phi_ki=    0.1
        theta_kp_1=1
        theta_kd_1=0
        theta_ki_1=0.1
        theta_kp_2=1
        theta_kd_2=0
        theta_ki_2=0.1
        d_T_kp_1=  1
        d_T_kd_1=  0
        d_T_ki_1=  0.1
        d_T_kp_2=  1
        d_T_kd_2=  0
        d_T_ki_2=  0.1
        
        ## Integrator limits
        phi_int_limit=  5
        theta_int_limit=5
        d_T_int_limit=  5
        
        ## Controller initialization
        self.roll_PID=PID(phi_kp,phi_kd,phi_ki,phi_int_limit)
        # Altitude to pitch / airspeed to throttle controllers
        self.pitch_PID_1=PID(theta_kp_1,theta_kd_1,theta_ki_1,theta_int_limit)
        self.throttle_PID_1=PID(d_T_kp_1,d_T_kd_1,d_T_ki_1,d_T_int_limit)
        # Airspeed to pitch / altitude to throttle controllers
        self.pitch_PID_2=PID(theta_kp_2,theta_kd_2,theta_ki_2,theta_int_limit)
        self.throttle_PID_2=PID(d_T_kp_2,d_T_kd_2,d_T_ki_2,d_T_int_limit)

    def controllers(self,psi_target,psi_meas,h_target,h_meas,V_target,V_meas):
        ## Heading to roll angle [psi to phi]
        ## 360 deg probelm?
        phi=self.roll_PID.control(psi_target,psi_meas)
        # Limit roll angle
        if phi > self.max_roll_angle:
            phi=self.max_roll_angle
        elif phi < -self.max_roll_angle:
            phi=-self.max_roll_angle
        else:
            pass
        
        ## Determine which set of controllers to use based on altitude error
        h_error=h_target-h_meas
        if abs(h_error)<=self.h_V_switch:
            ## Altitude to pitch [h to theta]
            theta=self.pitch_PID_1.control(h_target,h_meas)
            # Limit pitch angle
            if theta > self.max_pitch_angle:
                theta=self.max_pitch_angle
            elif theta < -self.max_pitch_angle:
                theta=-self.max_pitch_angle
            else:
                pass
        
            ## Airspeed to throttle [V to d_T]
            d_T=self.throttle_PID_1.control(V_target,V_meas)
            # Limit throttle
            if d_T > self.max_throttle:
                d_T=self.max_throttle
            elif d_T < 0:
                d_T=0
            else:
                pass
        
        else:
            ## Airspeed to pitch [V to theta]
            theta=self.pitch_PID_2.control(V_target,V_meas)
            # Limit pitch angle
            if theta > self.max_pitch_angle:
                theta=self.max_pitch_angle
            elif theta < -self.max_pitch_angle:
                theta=-self.max_pitch_angle
            else:
                pass
        
            ## Altitude to throttle [h to d_T]
            d_T=self.throttle_PID_2.control(h_target,h_meas)
            # Limit throttle
            if d_T > self.max_throttle:
                d_T=self.max_throttle
            elif d_T < -self.max_throttle:
                d_T=-self.max_throttle
            else:
                pass
        
        return [phi,theta,d_T]
