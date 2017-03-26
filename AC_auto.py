'''
    AC_auto.py
    
    Description: Autopilot control for fixed wing aicraft for flight dynamics analysis
    
    Revision History
    14 Feb 2016 - Created and debugged
 
    Author: Lars Soltmann
    
    Notes:
    Written for Python3 using the RPi3+NAVDAQ+NAVIO2(by Emlid)
    
    *RC INPUT
    Ch1 - Roll       [0]
    Ch2 - Pitch      [1]
    Ch3 - Throttle   [2]
    Ch4 - Yaw        [3]
    Ch5 - Mode       [4]
    
    *AXIS SYSTEM
    
    Forward
       x
       |
       0-y
      /
     z (into page)
    
    Aircraft body-axis coordinate system, right hand rule
    
    LED Guide:
    Yellow = Initialization - do not move vehicle
    Green = Ready to fly, 3D GPS fix
    Cyan = Ready to fly, no GPS fix
    Red (flashing) = throttle not set at zero, move throttle to zero
    Red (solid) = Error
    
    
    '''

from multiprocessing import Process, Array, Value
import time
import sys
import os
import math

#sys.path.append('/home/pi/AirLink')
sys.path.append('/home/pi/Python3_AP_Library')
sys.path.append('/home/pi/Navio2/Python/navio')

#import AirLink_serial
import mpu9250
import Complementary_Filter2
import ControlSurface_Calibration
import Controls_LowLevel
import Controls_MidLevel
import PID
import rcinput
import pwm
import leds
#import ms5611
#import UbloxGPS

# Exit_flag=0 during normal run and 1 when ready to exit
exit_flag=0
# Used to capture toggle time of gear swtich to exit program
prev_tgear=0
tgear=0
gearflag=0

## VARIABLE DEFINITIONS ##
hix=-13.696
hiy=34.376
hiz=-10.118



def AHRS_process(processEXIT,output_array):
    # output_array[0]=AHRS roll angle (deg)
    # output_array[1]=ARHS pitch angle (deg)
    # output_array[2]=AHRS yaw angle (deg)
    # output_array[3]=AHRS psi dot (deg/s)
    # output_array[4]=AHRS phi dot (deg/s)
    # output_array[5]=AHRS theta dot (deg/s)
    # output_array[6]=hard iron offest x
    # output_array[7]=hard iron offset y
    # output_array[8]=hard iron offset z
    # output_array[9]=magnetometer function check
    
    print('Starting AHRS process.')
    
    att=Complementary_Filter2.comp_filt(output_array[6],output_array[7],output_array[8])
    imu=mpu9250.MPU9250()
    imu.initialize()
    time.sleep(1)
    g_offset=[0,0,0]

    # Low pass filter setup
    fc=15; #Hz
    dt_f=0.0011 #measured average (sec)
    pie=3.14159265
    a_f=2*pie*dt_f*fc/(2*pie*dt_f*fc+1)
    b_f=1-a_f
    first_time=1

    # Loop to determine gyroscope offsets
    for x in range(0, 100):
        m9a, m9g, m9m = imu.getMotion9()
        g_offset[0]=g_offset[0]+m9g[1]/100
        g_offset[1]=g_offset[1]+m9g[0]/100
        g_offset[2]=g_offset[2]+m9g[2]/100    
    g_offset[2]=-g_offset[2] #because z-axis is oriented 180deg

    if (m9m[0]==0.0 or m9m[1]==0.0 or m9m[2]==0.0):
        output_array[9]=1.0

    while processEXIT.value==0:
        m9a, m9g, m9m = imu.getMotion9()
        gx=(m9g[1]-g_offset[0])*57.2958 #Convert to deg/s
        gy=(m9g[0]-g_offset[1])*57.2958
        gz=(-m9g[2]-g_offset[2])*57.2958
        ax=m9a[1]*0.10197 #Convert to g-force (1/9.81)
        ay=m9a[0]*0.10197
        az=-m9a[2]*0.10197
        att.attitude3(ax,ay,az,gx,gy,gz,m9m[0],m9m[1],m9m[2])

        # Apply LPF (only for ouput data)
        if first_time==1:
            theta_dot_d=att.thetad_d
            phi_dot_d=att.phid_d
            psi_dot_d=att.psid_d
            first_time=0
        else:
            theta_dot_d=a_f*att.thetad_d+b_f*theta_dot_d
            phi_dot_d=a_f*att.phid_d+b_f*phi_dot_d
            psi_dot_d=a_f*att.psid_d+b_f*psi_dot_d
     
        # Set outputs
        output_array[0]=att.roll_d-output_array[10]  #Subtract offsets due to orientation error
        output_array[1]=att.pitch_d-output_array[11] #Subtract offsets due to orientation error
        output_array[2]=att.yaw_d
        output_array[3]=psi_dot_d
        output_array[4]=phi_dot_d
        output_array[5]=theta_dot_d
    
    return None

def check_CLI_inputs():
    #Modes:
    # 1 = Normal operation mode
    # 2 = Calibration mode, straight pass through from RCINPUT
    # 3 = Normal operation mode with data logging

    if len(sys.argv)==1:
        mode=1
        print('No command line inputs found ... entering normal operation mode.')
    elif len(sys.argv)>1:
        if sys.argv[1]=='1':
            mode=1
            print('Entering normal operation.')
        elif sys.argv[1]=='2':
            mode=2
            print('Entering calibration mode.')
        elif sys.argv[1]=='3':
            mode=3
            print('Entering normal operation with data logging.')
        else:
            sys.exit('Unknown input argument!')
    return mode



##### MAIN PROGRAM #####
# Read command line inputs during program excecution and direct program accordingly
mode=check_CLI_inputs()

# Setup LED
led=leds.Led()
led.setColor('Yellow')

# Setup RCinput and RCoutput
rcin=rcinput.RCInput()
rcou1=pwm.PWM(0)
rcou2=pwm.PWM(1)
rcou3=pwm.PWM(2)
rcou4=pwm.PWM(3)
rcou1.set_period(50) #Hz
rcou2.set_period(50)
rcou3.set_period(50)
rcou4.set_period(50)

# NORMAL OPERATION
if (mode==1 or mode==4):

    ## Subprocesses
    # Subprocess array for AHRS data
    AHRS_data=Array('d', [0.0,0.0,0.0,0.0,0.0,0.0,hix,hiy,hiz,0.0,0,0])
    # Subprocess value that sets exit flag
    process_EXIT=Value('i', 0)
    # Define the subprocesses
    AHRS_proc=Process(target=AHRS_process, args=(process_EXIT,AHRS_data))
    # Start the subprocesses
    AHRS_proc.start()
                          
    # Initialize controllers
    CsCal=ControlSurface_Calibration.CS_cal()
    LowLevel=Controls_LowLevel.LL_controls()
    MidLevel=Controls_MidLevel.ML_controls()
                          
    time.sleep(3)

    # Check to make sure magnetometer is functioning correctly and not reporting all zeros
    if AHRS_data[9]==1.0:
        led.setColor('Red')
        exit_flag=1
        process_EXIT.value=1
        AHRS_proc.join()
        sys.exit('Magnetometer reading zero!')

    # Setup up data log if mode=4
    if mode==4:
        print('Setting up flight log.')
        flt_log=open('flight_log.txt', 'w')
        flt_log.write('t dt phi theta psi phi_dot theta_dot psi_dot elev ail thr rudd\n') #alt d_alt vel\n')

    print('Initialization complete. Starting control loops...')    
    t_start=time.time()

    count=0 # Used for reduced frame rate output to screen
    auto_at_auto_flag=1
    led.setColor('Cyan')

    ##FOR DEBUG
    d_a=0
    d_e=0
    d_r=0

    while exit_flag==0:
        t_1=time.time()
        
        # Check to see if mode is manual or auto
        gear_switch=float(rcin.read(4)) #Mode
    
        # ---- Manual control mode
        if gear_switch<=1500:
            # Definition is based of Assan X8R6 receiver
            # Straight pass through
            d_a_pwm=float(rcin.read(1)) #Aileron
            d_e_pwm=float(rcin.read(2)) #Elevator
            d_T_pwm=float(rcin.read(0)) #Throttle
            d_r_pwm=float(rcin.read(3)) #Rudder
            auto_at_auto_flag=1
        
        # ---- Automatic control mode
        elif gear_switch>1500:
            if auto_at_auto_flag==1:
                #Get the states when the autopilot was activated
                phi_input_at_auto=AHRS_data[0] #roll
                theta_input_at_auto=AHRS_data[1] #pitch
                psi_input_at_auto=AHRS_data[2] #heading
                #V_input_at_auto=V #velocity
                #alt_input_at_auto=h #altitude
                #Get current control surface commands
                d_a_pwm=float(rcin.read(1)) #Aileron
                d_e_pwm=float(rcin.read(2)) #Elevator
                d_T_pwm=float(rcin.read(0)) #Throttle
                d_r_pwm=float(rcin.read(3)) #Rudder
                d_a_input_at_auto,d_e_input_at_auto,d_r_input_at_auto,d_T_input_at_auto=CsCal.pwm_to_delta(d_a_pwm,d_e_pwm,d_r_pwm,d_T_pwm) #control surfaces
                
                #Seed controllers
                LowLevel.ail_PID.integral_term=d_a_input_at_auto
                LowLevel.elev_PID.integral_term=d_e_input_at_auto
                
                #Change flag value (only needs to be one at moment controller is activated
                auto_at_auto_flag=0
                          
            #MidLevel.controllers(psi_input_at_auto,AHRS_data[0],alt_input_at_auto,h_meas,V_input_at_auto,V_meas)
            d_a,d_e,d_r=LowLevel.controllers(phi_input_at_auto,AHRS_data[0],theta_input_at_auto,AHRS_data[1],AHRS_data[3],0)
            d_a_pwm,d_e_pwm,d_r_pwm,d_T_pwm=CsCal.delta_to_pwm(d_a,d_e,d_r,0) #Throttle hard coded to zero
            d_T_pwm=float(rcin.read(0)) #Throttle
        
        # Send servo commands to RCoutput --- CHECK THIS MAPPING
        #rcou1.set_duty_cycle(d_T_pwm*0.001) # u_sec to m_sec
        #rcou2.set_duty_cycle(d_a_pwm*0.001)
        #rcou3.set_duty_cycle(d_e_pwm*0.001)
        #rcou4.set_duty_cycle(d_r_pwm*0.001)
        rcou1.set_duty_cycle(d_a_pwm*0.001) # u_sec to m_sec        

        # Determine if gear switch has been toggled rapidly (less than 0.5sec
        # between ON and ON) to exit program
        prev_tgear=tgear
        if gear_switch<1500:
            if gearflag==0:
                tgear=time.time()
                gearflag=1
        else:
            gearflag=0
        if ((tgear-prev_tgear)<0.5 and (tgear-prev_tgear)>0) and prev_tgear != 0:
            exit_flag=1
            process_EXIT.value=1

        t_2=time.time()
        dt=t_2-t_1
        
        if mode==4:
            t_elapsed=t_2-t_start
            flt_log.write('%.3f %.4f %.2f %.2f %.2f %.2f %.2f %.2f %d %d %d %d\n' % (t_elapsed,dt,AHRS_data[0],AHRS_data[1],AHRS_data[2],AHRS_data[4],AHRS_data[5],AHRS_data[3],int(motor[0]),int(motor[1]),int(motor[2]),int(motor[3])))

        #DEBUG - output to screen
        count=count+1
        if count==200:
            print('%.2f %.2f %.2f' % (d_a,d_e,d_r))
        #    print('%.2f %.2f %.2f %.1f' % (targets[0],targets[1],targets[2],1/dt))
            print('%.2f %.2f %.2f %.1f' % (AHRS_data[0],AHRS_data[1],AHRS_data[2],1/dt))
            count=0



















#------------------------------------- CALIBRATION SCRIPTS --------------------------------

# ESC CALIBRATION MODE
# in this mode all channels are slaved to throttle channel directly with PWM limiting
if mode==2:
    led.setColor('Black')
    print('ESC CALIBRATION MODE')
    print(' ')
    dummy=input('Press any key to start')
    print(' ')
    while exit_flag==0:
        # Get inputs from the receiver and slave them to throttle channel
        RCinput[0]=1500
        RCinput[1]=1500
        RCinput[2]=float(rcin.read(0)) #Throttle
        RCinput[3]=1500
        gear_switch=float(rcin.read(4)) #Mode
        # Limit PWM output between MIN MAX values
        if RCinput[2]>config.PWM_MAX:
            servo_out[2]=config.PWM_MAX
        elif RCinput[2]<config.PWM_MIN:
            servo_out[2]=config.PWM_MIN
        else:
            servo_out[2]=RCinput[2]
            
        # Send motor commands to RCoutput
        rcou1.set_duty_cycle(servo_out[0]*0.001) # u_sec to m_sec
        rcou2.set_duty_cycle(servo_out[1]*0.001)
        rcou3.set_duty_cycle(servo_out[2]*0.001)
        rcou4.set_duty_cycle(servo_out[3]*0.001)

        # Output to console
        print('PWM out = %d uS' % servo_out[2])
        time.sleep(0.02)

        # Determine if gear switch has been toggled rapidly (less than 0.5sec
        # between ON and ON) to exit program
        prev_tgear=tgear
        if gear_switch<1500:
            if gearflag==0:
                tgear=time.time()
                gearflag=1
        else:
            gearflag=0
        if ((tgear-prev_tgear)<0.5 and (tgear-prev_tgear)>0) and prev_tgear != 0:
            exit_flag=1


# ---------- Exit Sequence ---------- #
if (mode==1 or mode==4):
    if mode==4:
        flt_log.close()

    led.setColor('Black')
    AHRS_proc.join()
    #ALT_proc.join()
    #GPS_proc.join()

    led.setColor('Red')
    time.sleep(0.1)
    led.setColor('Blue')
    time.sleep(0.1)
    led.setColor('Red')
    time.sleep(0.1)
    led.setColor('Blue')
    time.sleep(0.1)
    led.setColor('Red')
    time.sleep(0.1)
    led.setColor('Blue')
    time.sleep(0.1)
    led.setColor('Red')
    time.sleep(0.1)
    led.setColor('Blue')
    time.sleep(0.1)
    led.setColor('Red')
    time.sleep(0.1)
    led.setColor('Blue')
    time.sleep(0.1)
    led.setColor('Red')
    time.sleep(0.1)
    led.setColor('Blue')
    time.sleep(0.1)
    led.setColor('Red')
    time.sleep(0.1)
    led.setColor('Blue')
    time.sleep(0.1)
    led.setColor('Red')
    time.sleep(0.1)
    led.setColor('Blue')
    time.sleep(0.1)
    led.setColor('Black')
    print('Done.')
