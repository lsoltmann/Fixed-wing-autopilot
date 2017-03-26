'''
    Control_LowLevel.py
    
    Description: Library containing the low level controllers
                 of the autopilot.
    
    Revision History
    3 Feb 2017 - Created and debugged
    
    Author: Lars Soltmann
    
    References: None
    
    Calls: PID.py
    
    Notes:
    - Written for Python3
    
    Inputs: phi_target - commanded roll angle (deg)
            phi_meas - measured roll angle (deg)
            theta_target - commanded pitch angle (deg)
            theta_meas - measured pitch angle (deg)
            r - current yaw rate (deg/s)
            V - airspeed (ft/s)
            
    Outputs: d_a - aileron deflection (deg)
             d_e - elevator deflection (deg)
             d_r - rudder deflection (deg)
    
    OPEN ITEMS:
    1. gain and limit list definitions
    
'''

from PID import PID

class LL_controls:
    def __init__(self):
        ## Control surface deflection limits
        self.max_ail_def= 20 #deg
        self.max_elev_def=20 #deg
        self.max_rud_def= 20 #deg
        
        ## Initial controller gains
        self.rudder_gain=0.1
        elev_kp= 1
        elev_kd= 0
        elev_ki= 0.1
        ail_kp=  1
        ail_kd=  0
        ail_ki=  0.1
        self.lift_compensation_gain=0.1
        
        ## Integrator limits
        ail_int_limit= 5
        elev_int_limit=5
        
        ## Controller initialization
        self.ail_PID=PID(ail_kp,ail_kd,ail_ki,ail_int_limit)
        self.elev_PID=PID(elev_kp,elev_kd,elev_ki,elev_int_limit)

    def controllers(self,phi_target,phi_meas,theta_target,theta_meas,r,V):
        ## Roll angle to aileron [phi to d_a]
        # Gain scheduling as a function of airspeed
        # ail_PID.set_kp=
        # ail_PID.set_kd=
        # ail_PID.set_ki=
        d_a=self.ail_PID.control(phi_target,phi_meas)
        # Limit aileron deflection
        if d_a > self.max_ail_def:
            d_a=self.max_ail_def
        elif d_a < -self.max_ail_def:
            d_a=-self.max_ail_def
        else:
            pass

        ## Pitch angle to elevator [theta to d_e]
        # Gain scheduling as a function of airspeed
        # elev_PID.set_kp=
        # elev_PID.set_kd=
        # elev_PID.set_ki=
        # self.lift_compensation_gain=
        d_e1=self.elev_PID.control(theta_target,theta_meas)
        d_e2=phi_meas*self.lift_compensation_gain
        d_e=d_e1+d_e2
        # Limit elevator deflection
        if d_e > self.max_elev_def:
            d_e=self.max_elev_def
        elif d_e < -self.max_elev_def:
            d_e=-self.max_elev_def
        else:
            pass

        ## Yaw rate to rudder [r to d_r]
        # Gain scheduling as a function of airspeed
        #self.rudder_gain=
        d_r=self.rudder_gain*r
        # Limit rudder deflection
        if d_a > self.max_ail_def:
            d_a=self.max_ail_def
        elif d_a < -self.max_ail_def:
            d_a=-self.max_ail_def
        else:
            pass

        return [d_a,d_e,d_r] #deg
