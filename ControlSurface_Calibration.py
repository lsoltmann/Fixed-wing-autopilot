'''
    ControlSurface_Calibration.py
    
    Description: Library containing the control surface calibration
                 for servo limiting.
    
    Revision History
    15 Feb 2017 - Created and debugged
    
    Author: Lars Soltmann
    
    References: None
    
    Calls: None
    
    Notes:
    - Written for Python3
    
    Inputs: d_a - aileron deflection (deg)
            d_e - elevator deflection (deg)
            d_r - rudder deflection (deg)
    
    Outputs: d_a_pwm - PWM command for aileron servo (u_sec)
             d_e_pwm - PWM command for elevator servo (u_sec)
             d_r_pwm - PWM command for rudder servo (u_sec)
             d_T_pwm - PWM command for throttle (u_sec)
    
    
    OPEN ITEMS:
    1. 
    
    '''

class CS_cal:
    def __init__(self):
        #Calibration functions are measured and will generally follow the form
        # slope=((pwm_1-pwm_2)/(deflection_1-deflection_2))
        # intercept=center
        
        #Zero position PWM (u_sec) of each control surface
        self.da_center=1504
        self.de_center=1488
        self.dr_center=1523
        self.dT_off=900
    
        #Slope of calibration function
        self.da_slope=-23.328
        self.de_slope=17.316
        self.dr_slope=-19.786
        self.dT_slope=(2100-900)/(100)

    def delta_to_pwm(self,d_a,d_e,d_r,d_T):
        ##Apply control surface calibrations
        #Aileron
        d_a_pwm=round(self.da_slope*d_a+self.da_center)
        
        #Elevator
        d_e_pwm=round(self.de_slope*d_e+self.de_center)
        
        #Rudder
        d_r_pwm=round(self.dr_slope*d_r+self.dr_center)
        
        #Throttle
        d_T_pwm=round(self.dT_slope*d_T+self.dT_off)
        
        return d_a_pwm,d_e_pwm,d_r_pwm,d_T_pwm

    def pwm_to_delta(self,d_a_pwm,d_e_pwm,d_r_pwm,d_T_pwm):
        ##Apply inverse of control surface calibration
        #Aileron
        d_a=-(self.da_center-d_a_pwm)/self.da_slope

        #Elevator
        d_e=-(self.de_center-d_e_pwm)/self.de_slope

        #Rudder
        d_r=-(self.dr_center-d_r_pwm)/self.dr_slope

        #Throttle
        d_T=-(self.dT_off-d_T_pwm)/self.dT_slope

        return d_a,d_e,d_r,d_T

