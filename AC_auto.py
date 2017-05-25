'''
    AC_auto.py
    
    Description: Autopilot control for fixed wing aicraft for flight dynamics analysis
    
    Revision History
    05 May 2016 - Created and debugged
    
    Author: Lars Soltmann
    
    Notes:
    Written for Python3 using the RPi3+NAVDAQ+NAVIO2(by Emlid)
    
    RC Ch Mapping
    Ch1 - Throttle  [0]
    Ch2 - Aileron   [1]
    Ch3 - Elevator  [2]
    Ch4 - Rudder    [3]
    Ch5 - Mode      [4]
    
    AXIS SYSTEM
    
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

    Orientation definitions: [when viewed from above]
    1 = right side up, USB ports facing forward
    2 = right side up, USB facing to the right
    3 = right side up, USB facing backwards
    4 = right side up, USB facing to the left
    5 = upside down, USB ports facing forward
    6 = upside down, USB facing to the right
    7 = upside down, USB facing backwards
    8 = upside down, USB facing to the left        
'''

from multiprocessing import Process, Array, Value
import time
import sys
import os
import math
import RPi.GPIO as GPIO

#sys.path.append('/home/pi/AirLink')
sys.path.append('/home/pi/Python3_AP_Library')
sys.path.append('/home/pi/Navio2/Python/navio')

#import AirLink_serial
import mpu9250
import Complementary_Filter2
import ControlSurface_Calibration
import Control_LowLevel
import Control_MidLevel
import read_preprogrammed_maneuver
import PID
import rcinput
import pwm
import leds
import SSC005D
import MS5805
#import UbloxGPS

# Exit_flag=0 during normal run and 1 when ready to exit
exit_flag=0
# Used to capture toggle time of gear swtich to exit program
prev_tgear=0
tgear=0
gearflag=0

## VARIABLE DEFINITIONS ##
ORIENTATION=5
hix=-13.696 #Hard iron offset, x (from calibration)
hiy=34.376 #Hard iron offset, y (from calibration)
hiz=-10.118 #Hard iron offset, z (from calibration)
steady_state_time=1 #Number of seconds that the aircraft must be at the initial condition (only for IC_TYPE=2) before the preprogrammed maneuver starts
loop_dt=0.04 #Autopilot loop frequency (sec)
steady_state_pitch_range=1 #+/-deg that pitch should be from initial condition before maneuver can start
steady_state_roll_range=1 #+/-deg that roll should be from initial condition before maneuver can start
steady_state_vel_range=3 #+/-deg that velocity should be from initial condition before maneuver can start

count_at_steady_state_pitch=0
count_at_steady_state_roll=0
count_at_steady_state_vel=0
steady_state_condition_achieved_pitch=0
steady_state_condition_achieved_roll=0
steady_state_condition_achieved_vel=0
man_flag=0
maneuver_count=1

## Subprocess for attitude heading reference system
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
    # output_array[10]=roll orientation offset due to installation
    # output_array[11]=pitch orientation offset due to installation
    # output_array[12]=ax
    # output_array[13]=ay
    # output_array[14]=az
    # output_array[15]=p
    # output_array[16]=q
    # output_array[17]=r  
    # output_array[18]=orientation
    
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

        if (m9m[0]==0.0 and m9m[1]==0.0 and m9m[2]==0.0):
            print(m9m)
            output_array[9]=1.0

    while processEXIT.value==0:
        m9a, m9g, m9m = imu.getMotion9()
        if output_array[18]==1:
            gx=(m9g[1]-g_offset[0])*57.2958 #Convert to deg/s
            gy=(m9g[0]-g_offset[1])*57.2958
            gz=(-m9g[2]-g_offset[2])*57.2958
            ax=m9a[1]*0.10197 #Convert to g-force (1/9.81)
            ay=m9a[0]*0.10197
            az=-m9a[2]*0.10197
        elif output_array[18]==5:## NEEDS TO BE UPDATED
            gx=(m9g[1]-g_offset[0])*57.2958 #Convert to deg/s
            gy=(-m9g[0]-g_offset[1])*57.2958
            gz=(m9g[2]-g_offset[2])*57.2958
            ax=m9a[1]*0.10197 #Convert to g-force (1/9.81)
            ay=-m9a[0]*0.10197
            az=m9a[2]*0.10197
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
        output_array[12]=ax
        output_array[13]=ay
        output_array[14]=az
        output_array[15]=gx
        output_array[16]=gy
        output_array[17]=gz
    
    return None

# Subprocess for pressure transducers
def PRESS_process(processEXIT,output_array):
    # output_array[0]=Velocity (ft/s)
    # output_array[1]=Altitude (ft)
    # output_array[2]=Velocity offset (ft/s)
    # output_array[3]=Altitude offset (ft) - Pressure altitude at initialization location

    # Create and initialize pressure transducers
    pt=SSC005D.HWSSC(0x28) #Pitot-static, differential
    st=MS5805.MS5805(0x76) #Static, absolute
    st.initialize()

    #Setup the GPIO and set GPIO 17 and 24 low. The NAVDAQ shields contains a 74HC4052DIP multiplexer and both pins low access the pitot-static tube pressure transducer
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(17, GPIO.OUT)
    GPIO.setup(24, GPIO.OUT)
    GPIO.output(17, GPIO.LOW)
    GPIO.output(24, GPIO.LOW)
    
    output_array[2]=0
    output_array[3]=0
    INIT_SAMP=10
    COUNT=0

    while processEXIT.value==0:
        pt.readPressure_raw()
        pt_press=pt.convertPressure(1,5)*5.2023300231 #inH2O to PSF
        if pt_press>0:
            # If averaging loop has finished, subtracted the average
            if COUNT==INIT_SAMP:
                output_array[0]=(math.sqrt(pt_press*841.4321175))-output_array[2] #Indicated velocity in ft/s (841.43=2/0.0023769)
            # If averaging loop hasn't finished, don't subtract
            else:
                output_array[0]=(math.sqrt(pt_press*841.4321175)) #Indicated velocity in ft/s (841.43=2/0.0023769) 
        else:
            output_array[0]=0
        
        st.read_pressure_temperature()
        st_temp=st.getTemperature_degF()
        st_press=st.getPressure_mbar()
        # If averaging loop has finished, subtracted the average
        if COUNT==INIT_SAMP:
            output_array[1]=(1.4536645e5-38951.51955*st_press**0.190284)-output_array[3] #Pressure altitude in ft
        # If averaging loop hasn't finished, don't subtract
        else:
            output_array[1]=(1.4536645e5-38951.51955*st_press**0.190284) #Pressure altitude in ft   
            
        ## Tracking filter goes here
    
        # Find the average velocity and altitude during the first INIT_SAMP samples
        if COUNT<(INIT_SAMP):
            output_array[2]=output_array[2]+output_array[0]/INIT_SAMP #Average velocity offset
            output_array[3]=output_array[3]+output_array[1]/INIT_SAMP #Average altitude offset
            COUNT=COUNT+1

    # Clean up the GPIO before ending the process
    GPIO.cleanup()


def check_CLI_inputs():
    #Modes:
    # 1 = Pass through
    # 2 = Pass through with data logging
    # 3 = Preprogrammed maneuver mode [with data logging]
    
    if len(sys.argv)==1:
        mode=1
        print('No command line inputs found ... entering pass through mode with data logging.')
    elif len(sys.argv)>1:
        if sys.argv[1]=='1':
            mode=1
            print('Entering pass through mode with data logging.')
        
        elif sys.argv[1]=='2':
            mode=2
            print('<Undefined as of now.')
        
        elif sys.argv[1]=='3':
            if len(sys.argv)<3:
                sys.exit('Maneuver file not given! Enter maneuver file after mode value.')
            mode=3
            print('Entering maneuver mode.')
        
        elif sys.argv[1]=='-1':
            mode=-1
            print('Entering calibration mode.')
        
        else:
            led.setColor('Red')
            sys.exit('Unknown input argument!')
    return mode

def get_current_RCinputs():
    # Definition is based of Assan X8R6 receiver
    d_a_pwm=float(rcin.read(1)) #Aileron
    d_e_pwm=float(rcin.read(2)) #Elevator
    d_T_pwm=float(rcin.read(0)) #Throttle
    d_r_pwm=float(rcin.read(3)) #Rudder
    return d_a_pwm,d_e_pwm,d_T_pwm,d_r_pwm

def set_initial_cmds(PM,AHRS_data,d_T_cmd,PRESS_data):
    #If mode=3 (preprogrammed maneuver) set some variables
    if mode==3:
        #Set the initial conditions equal to the aircraft states at activation
        if PM.ic_type==1:
            phi_cmd=AHRS_data[0] #roll
            theta_cmd=AHRS_data[1] #pitch
            psi_cmd=AHRS_data[2] #heading, not used ... just for reference
            #V_cmd=V #velocity
            #alt_cmd=h #altitude
        elif (PM.ic_type==3 or PM.ic_type==4):
            phi_cmd=PM.man_phi[0] #roll
            theta_cmd=PM.man_theta[0] #pitch
            psi_cmd=AHRS_data[2] #heading , not used ... just for reference
            if PM.ic_type==3:
                V_cmd=PM.man_vel[0]
            elif PM.ic_type==4:
                d_T_cmd=PM.man_thr[0]

    #If mode=1,2 save the current aircraft states
    else:
        #Get the states when the autopilot was activated
        phi_cmd=AHRS_data[0] #roll
        theta_cmd=AHRS_data[1] #pitch
        psi_cmd=AHRS_data[2] #heading
        V_cmd=PRESS_data[0]
    
    return phi_cmd,theta_cmd,psi_cmd,d_T_cmd,V_cmd


##### MAIN PROGRAM #####
# Setup LED
led=leds.Led()
led.setColor('Yellow')
# Read command line inputs during program excecution and direct program accordingly
mode=check_CLI_inputs()

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
if (mode>0):
    
    ## Subprocesses
    # Subprocess array for AHRS and pressure transducer data
    AHRS_data=Array('d', [0.0,0.0,0.0,0.0,0.0,0.0,hix,hiy,hiz,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,ORIENTATION])
    PRESS_data=Array('d',[0.0,0.0,0.0,0.0])
    
    # Subprocess value that sets exit flag
    process_EXIT=Value('i', 0)
    
    # Define the subprocesses
    AHRS_proc=Process(target=AHRS_process, args=(process_EXIT,AHRS_data))
    PRESS_proc=Process(target=PRESS_process, args=(process_EXIT,PRESS_data))
    
    # Start the subprocesses
    AHRS_proc.start()
    PRESS_proc.start()
    
    # Initialize controllers
    CsCal=ControlSurface_Calibration.CS_cal()
    LowLevel=Control_LowLevel.LL_controls()
    MidLevel=Control_MidLevel.ML_controls()
    
    time.sleep(3)
    
    # Check to make sure magnetometer is functioning correctly and not reporting all zeros
    if AHRS_data[9]==1.0:
        led.setColor('Red')
        exit_flag=1
        process_EXIT.value=1
        AHRS_proc.join()
        sys.exit('Magnetometer reading zero!')

    # Setup up data log
    print('Setting up flight log.')
    try:
        log_timestr = time.strftime("%d%m%Y-%H%M")
        #log_timestr2= time.strftime("%d-%m-%Y--%H:%M")
        log_timestr2= time.asctime()
        flt_log=open('flight_log_'+log_timestr+'.txt', 'w')
        #flt_log.write('Date[d-m-y] -- Time[H:M]\n')
        #flt_log.write(log_timestr2)
        flt_log.write(log_timestr2+' local time')
        flt_log.write('\n\n')
        flt_log.write('Velocity/Altitude Offsets\n')
        flt_log.write('%.1f %.0f\n' % (PRESS_data[2],PRESS_data[3]))
        flt_log.write('\n')
        #flt_log.write('T DT PHI THETA PSI P Q R AX AY AZ VIAS ALT ELEV AIL THR RUDD PHI_CMD THETA_CMD PSI_CMD VEL_CMD\n')
        #flt_log.write('sec sec deg deg deg deg/s deg/s deg/s g g g ft/s ft deg deg % deg deg deg deg ft/s\n')
        flt_log.write('T DT PHI THETA PSI PHI_DOT THETA_DOT PSI_DOT P Q R AX AY AZ VIAS ALT ELEV AIL THR RUDD\n')
        flt_log.write('sec sec deg deg deg deg/s deg/s deg/s deg/s deg/s deg/s g g g ft/s ft PWM PWM PWM PWM\n')
    except:
        led.setColor('Red')
        sys.exit('Error creating log file!')

    if mode==3:
        try:
            PM=read_preprogrammed_maneuver.read_maneuver_file(sys.argv[2])
            PM.read_file()
        except:
            sys.exit('Error processing maneuver file!')


    print('Initialization complete.')
    print('Starting autopilot loop...')
    t_start=time.time()
    
    count=0 # Used for reduced frame rate output to screen
    auto_at_auto_flag=1
    led.setColor('Cyan')
    dt3=0
    
    while exit_flag==0:
        t_1=time.time()
        
        # Check to see if mode is manual or auto
        gear_switch=float(rcin.read(4)) #Mode
        
        # ---- Pass through mode
        if gear_switch<=1500:
            # Straight pass through since saftey switch has taken FCS out of the loop
            d_a_pwm,d_e_pwm,d_T_pwm,d_r_pwm=get_current_RCinputs()
            auto_at_auto_flag=1

        # ---- Automatic control mode
        elif gear_switch>1500:
            #Get/set initial conditions when switching into auto mode
            if (auto_at_auto_flag==1 and mode != 1):
                #Initialize variables
                count_at_steady_state_pitch=0
                count_at_steady_state_roll=0
                count_at_steady_state_vel=0
                steady_state_condition_achieved_pitch=0
                steady_state_condition_achieved_roll=0
                steady_state_condition_achieved_vel=0
                man_flag=0
                maneuver_count=1
                #Get current control surface commands
                d_a_pwm,d_e_pwm,d_T_pwm,d_r_pwm=get_current_RCinputs()
                #Convert control surface commands to angles
                d_a_cmd,d_e_cmd,d_r_cmd,d_T_cmd=CsCal.pwm_to_delta(d_a_pwm,d_e_pwm,d_r_pwm,d_T_pwm) #control surfaces
                
                phi_cmd,theta_cmd,psi_cmd,d_T_cmd,V_cmd=set_initial_cmds(PM,AHRS_data,d_T_cmd,PRESS_data[0])
                
                #Seed controllers
                LowLevel.ail_PID.integral_term=d_a_cmd
                LowLevel.elev_PID.integral_term=d_e_cmd
                
                #Change flag value (only needs to be one at moment controller is activated
                auto_at_auto_flag=0
            
            # PASS THROUGH MODE
            if (mode==1 or mode==2):
                d_a_pwm,d_e_pwm,d_T_pwm,d_r_pwm=get_current_RCinputs()
                d_e_cmd=0
                d_a_cmd=0
                d_T_cmd=0
                d_r_cmd=0
                phi_cmd=0
                theta_cmd=0
                psi_cmd=0
                V_cmd=0
            
            # <Currently not defined> pass through for now
            #elif mode==2:
            
            # PREPROGRAMMED MANEUVER MODE
            elif mode==3:
                #Check to make sure that conditions are 'close' before activating maneuver
                #If conditions are close, start maneuver
                if (steady_state_condition_achieved_pitch==1 and steady_state_condition_achieved_roll==1 and steady_state_condition_achieved_vel==1):
                    if man_flag==0:
                        t_man_start=time.time()
                        man_flag=1
                    else:
                        t_current=time.time()
                        if (t_current-t_man_start)>=PM.man_time[maneuver_count] and man_flag<2:
                            #If the current time is equal to (or just barely greater than) the maneuver time, change the target values
                            phi_cmd=PM.man_phi[maneuver_count] #roll
                            theta_cmd=PM.man_theta[maneuver_count] #pitch
                            if PM.ic_type==3:
                                V_cmd=PM.man_vel[maneuver_count] #velocity
                            #Increment count to set next maneuver time
                            maneuver_count+=1
                            if maneuver_count>=len(PM.man_time):
                                man_flag=2
                        else:
                            pass
                
                #If conditions aren't close, let the controller continue until the conditions are within the specified tolerances
                else:
                    #Check to make sure pitch angle is within +/- 'steady_state_pitch_range' for a certain amount of time before starting maneuver
                    if abs(AHRS_data[1]-theta_cmd)<=steady_state_pitch_range and count_at_steady_state_pitch<(steady_state_time/loop_dt):
                        count_at_steady_state_pitch+=1
                    elif abs(AHRS_data[1]-theta_cmd)>1:
                        count_at_steady_state_pitch=0
                    else:
                        steady_state_condition_achieved_pitch=1
                    
                    #Check to make sure roll angle is within +/- 'steady_state_roll_range' for a certain amount of time before starting maneuver
                    if abs(AHRS_data[0]-phi_cmd)<steady_state_roll_range and count_at_steady_state_roll<(steady_state_time/loop_dt):
                        count_at_steady_state_roll+=1
                    elif abs(AHRS_data[0]-phi_cmd)>steady_state_roll_range:
                        count_at_steady_state_roll=0
                    else:
                        steady_state_condition_achieved_roll=1
                    
                    #Check to make sure velocity is within +/- 'steady_state_vel_range' for a certain amount of time before starting maneuver
                    if PM.ic_type==3:
                        if abs(PRESS_data[0]-V_cmd)<steady_state_vel_range and count_at_steady_state_vel<(steady_state_time/loop_dt):
                            count_at_steady_state_vel+=1
                        elif abs(PRESS_data[0]-V_cmd)>steady_state_vel_range:
                            count_at_steady_state_vel=0
                        else:
                            steady_state_condition_achieved_vel=1
                    #Auto set flag when throttle is set as the initial condition
                    elif PM.ic_type==4:
                        steady_state_condition_achieved_vel=1
        
            if mode!=1:
                #MidLevel.controllers(psi_cmd,AHRS_data[0],alt_cmd,h_meas,V_cmd,V_meas)
                d_a_cmd,d_e_cmd,d_r_cmd=LowLevel.controllers(phi_cmd,AHRS_data[0],theta_cmd,AHRS_data[1],AHRS_data[3],0)
                d_a_pwm,d_e_pwm,d_r_pwm,d_T_pwm=CsCal.delta_to_pwm(d_a_cmd,d_e_cmd,d_r_cmd,d_T_cmd)
                d_T_pwm=float(rcin.read(0)) ##Throttle hard coded to manual
        
        # Send servo commands to RCoutput --- CHECK THIS MAPPING
        rcou1.set_duty_cycle(d_T_pwm*0.001) # u_sec to m_sec
        rcou2.set_duty_cycle(d_a_pwm*0.001)
        rcou3.set_duty_cycle(d_e_pwm*0.001)
        rcou4.set_duty_cycle(d_r_pwm*0.001)
        
        # Determine if gear switch has been toggled rapidly (less than 0.5sec
        # between ON and ON) to exit program
        prev_tgear=tgear
        if gear_switch>1500:
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
        
        t_elapsed=t_2-t_start
        
        if gear_switch>1500:
            #                                                                                                        THETA_CMD VEL_CMD
            #              T    DT   PHI THETA PSI  P    Q    R    AX   AY   AZ   IAS  ALT  ELEV AIL  THR  RUDD PHI_CMD   PSI_CMD 
            #flt_log.write('%.3f %.4f %.2f %.2f %.2f %.2f %.2f %.2f %.3f %.3f %.3f %.1f %.0f %.1f %.1f %.1f %.1f %.1f %.1f %.1f %.1f\n' % (t_elapsed,dt3,AHRS_data[0],AHRS_data[1],AHRS_data[2],AHRS_data[4],AHRS_data[5],AHRS_data[3],AHRS_data[12],AHRS_data[13],AHRS_data[14],PRESS_data[0],PRESS_data[1],d_e_cmd,d_a_cmd,d_T_cmd,d_r_cmd, phi_cmd, theta_cmd, psi_cmd, V_cmd))
            #                                                                                                 
            #              T    DT   PHI THETA PSI  PHID THTD PSID  P    Q    R    AX   AY   AZ   IAS  ALT  ELEV AIL  THR  RUDD [PWM]            
            flt_log.write('%.3f %.4f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.3f %.3f %.3f %.1f %.0f %d %d %d %d\n' % (t_elapsed,dt3,AHRS_data[0],AHRS_data[1],AHRS_data[2],AHRS_data[4],AHRS_data[5],AHRS_data[3],AHRS_data[15],AHRS_data[16],AHRS_data[17],AHRS_data[12],AHRS_data[13],AHRS_data[14],PRESS_data[0],PRESS_data[1],d_e_pwm,d_a_pwm,d_T_pwm,d_r_pwm))

        #If loop time was less than autopilot loop time, sleep the remaining time
        t_3=time.time()
        dt2=t_3-t_1
        if loop_dt>dt2:
            time.sleep(loop_dt-dt2)
        else:
            print('Loop time exceeded!')
        t_4=time.time()
        dt3=t_4-t_1

        #DEBUG - output to screen
        #count=count+1
        #if count==25:
            #print('%.2f %.2f %.2f' % (d_a,d_e,d_r))
            #print('%.2f %.2f %.2f %.1f' % (targets[0],targets[1],targets[2],1/dt))
            #print('%.2f %.2f %.2f %.2f %.2f %.2f %.1f' % (AHRS_data[0],AHRS_data[1],AHRS_data[2],AHRS_data[4],AHRS_data[5],AHRS_data[3],1/dt))
            #count=0
        #print(PRESS_data[0])
        #print(PRESS_data[1])


















#------------------------------------- CALIBRATION SCRIPTS --------------------------------
'''
    # ESC CALIBRATION MODE
    # in this mode all channels are slaved to throttle channel directly with PWM limiting
    if mode==-1:
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
    
'''
# ---------- Exit Sequence ---------- #
if (mode>0):
    if mode==2:
        flt_log.close()

    led.setColor('Black')
    AHRS_proc.join()
    PRESS_proc.join()
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
