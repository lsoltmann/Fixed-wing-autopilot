'''
    AC_auto.py
    
    Description: Autopilot control for fixed wing aicraft for flight dynamics analysis
    
    Revision History
    05 May 2016 - Created
    
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
import subprocess
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
import Alpha_Beta_Filter
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

# Create flight log (1) or not (0)
FLTLOG=0

## VARIABLE DEFINITIONS ##
ORIENTATION=5

# Hard iron offsets measured on 27May2017 on Firstar V2
hix=-18.117 #Hard iron offset, x (from calibration)
hiy=27.974 #Hard iron offset, y (from calibration)
hiz=-17.841 #Hard iron offset, z (from calibration)

theta_offset=0.0 #deg
phi_offset=-0.6

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
    
    '''
    # Low pass filter setup
    fc=15; #Hz
    dt_f=0.0011 #measured average (sec)
    pie=3.14159265
    a_f=2*pie*dt_f*fc/(2*pie*dt_f*fc+1)
    b_f=1-a_f
    first_time=1
    '''
    
    # Loop to determine gyroscope offsets
    for x in range(0, 100):
        m9a, m9g, m9m = imu.getMotion9()
        g_offset[0]=g_offset[0]+m9g[1]/100
        g_offset[1]=g_offset[1]+m9g[0]/100
        g_offset[2]=g_offset[2]+m9g[2]/100

        if (m9m[0]==0.0 and m9m[1]==0.0 and m9m[2]==0.0):
            output_array[9]=1.0
            
    # Main AHRS loop
    while processEXIT.value==0:
        m9a, m9g, m9m = imu.getMotion9()
        if output_array[18]==1:
            gx=(m9g[1]-g_offset[0])*57.2958 #Convert to deg/s
            gy=(m9g[0]-g_offset[1])*57.2958
            gz=-(m9g[2]-g_offset[2])*57.2958
            ax=m9a[1]*0.10197 #Convert to g-force (1/9.81)
            ay=m9a[0]*0.10197
            az=-m9a[2]*0.10197
        elif output_array[18]==5:
            gx=(m9g[1]-g_offset[0])*57.2958 #Convert to deg/s
            gy=-(m9g[0]-g_offset[1])*57.2958
            gz=(m9g[2]-g_offset[2])*57.2958
            ax=m9a[1]*0.10197 #Convert to g-force (1/9.81)
            ay=-m9a[0]*0.10197
            az=m9a[2]*0.10197
        att.attitude3(ax,ay,az,gx,gy,gz,m9m[0],m9m[1],m9m[2])
        
        '''
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
        '''
        
        # Set outputs
        output_array[0]=att.roll_d-output_array[10]  #Subtract offsets due to orientation error
        output_array[1]=att.pitch_d-output_array[11] #Subtract offsets due to orientation error
        output_array[2]=att.yaw_d
        
        # If using LPF
        #output_array[3]=psi_dot_d
        #output_array[4]=phi_dot_d
        #output_array[5]=theta_dot_d
        # If not using LPF
        output_array[3]=att.psid_d
        output_array[4]=att.phid_d
        output_array[5]=att.thetad_d
        
        output_array[12]=ax
        output_array[13]=ay
        output_array[14]=az
        output_array[15]=gx
        output_array[16]=gy
        output_array[17]=gz
    
    print('AHRS process stopped.')

# Subprocess for pressure transducers
def ARSP_ALT_process(processEXIT,output_array):
    # output_array[0]=Velocity (ft/s)
    # output_array[1]=Altitude (ft)
    # output_array[2]=Total pressure offset (PSF)
    # output_array[3]=Static pressure offset (mbar) - Pressure altitude at initialization location
    # output_array[4]=Filtered velocity (ft/s)
    # output_array[5]=Filtered altitude (ft)
    # output_array[6]=Filtered velocity_dot (ft/s)
    # output_array[7]=Filtered altitude_dot [VSI] (ft)
    # output_array[8]=Static pressure transducer temperature (degF)

    print('Starting airspeed/altitude process.')

    # Create and initialize pressure transducers
    pt=SSC005D.HWSSC(0x28) #Pitot-static, differential
    ps=MS5805.MS5805(0x76) #Static, absolute
    ps.initialize()
    time.sleep(1)
    
    # Create tracking filter
    #ALTITUDE
    h_TF=Alpha_Beta_Filter.trackfilt(0.05,0.005)
    #AIRSPEED
    V_TF=Alpha_Beta_Filter.trackfilt(0.2,0.005)

    # Setup the GPIO and set GPIO 17 and 24 low. The NAVDAQ shields contains a 74HC4052DIP multiplexer and both pins low access the pitot-static tube pressure transducer
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(17, GPIO.OUT)
    GPIO.setup(24, GPIO.OUT)
    GPIO.output(17, GPIO.LOW)
    GPIO.output(24, GPIO.LOW)
    
    INIT_SAMP=10
    COUNT=0
    # Set inital pressures incase the first loop has the 'input/output error'
    pt_press=0
    ps_press=1013.25
    

    while processEXIT.value==0:
        t1=time.time()
        # Read from the sensors using a try/except loop since a random IOerror occasionally pops up and will prevent all subsequent reads 
        try:
            # Read total pressure for airspeed
            pt.readPressure_raw()
            pt_press=pt.convertPressure(1,5)*5.2023300231 #inH2O to PSF

            # Read static pressure for altitude
            ps.read_pressure_temperature()
            ps_temp=ps.getTemperature_degF()
            ps_press=ps.getPressure_mbar()
            flag=0
        # If an IOerror (or any other error) occurs, reset the I2C bus by calling the i2cdetect ... this appears to be workaround from many forums
        except:
            print('Error reading from I2C!')
            subprocess.call(['i2cdetect', '-y', '1'], stdout=open(os.devnull, "w"))
            flag=1
        
        # Airspeed Calculation
        # Indicated airspeed (rho=0.0023769 slug/ft^3)
        # Only calculate velocity if the total pressure is greater than the offset, ie. it's positve so the sqrt function doesn't freak out
        if pt_press>output_array[2]:
            output_array[0]=(math.sqrt((pt_press-output_array[2])*841.4321175)) #Indicated velocity in ft/s (841.43=2/0.0023769)
        else:
            output_array[0]=0

        # Altitude Calculation
        # Based on standard atmosphere equation
        output_array[1]=(1.4536645e5-38951.51955*(ps_press)**0.190284) #Pressure altitude in ft
        output_array[8]=ps_temp
        
        # Get loop time for filter
        t2=time.time()
        dt=t2-t1
        # Run altitude and airspeed through filter
        output_array[5],output_array[7]=h_TF.track(output_array[1],dt)
        output_array[4],output_array[6]=V_TF.track(output_array[0],dt)
    
        # Find the average total and static pressure during the first INIT_SAMP samples to use as the transducer offsets
        if (COUNT<(INIT_SAMP) and flag==0):
            output_array[2]=output_array[2]+pt_press/INIT_SAMP #Average total pressure offset
            output_array[3]=output_array[3]+ps_press/INIT_SAMP #Average static pressure offset
            COUNT=COUNT+1

    # Clean up the GPIO before ending the process
    GPIO.cleanup()
    print('Airspeed/altitude process stopped.')

def check_CLI_inputs():
    #Modes:
    # 1 = Pass through
    # 2 = Preprogrammed maneuver [open loop]
    # 3 = Preprogrammed maneuver [closed loop]
    
    if len(sys.argv)==1:
        mode=1
        print('No command line inputs found ... entering MODE 1')
    elif len(sys.argv)>1:
        if sys.argv[1]=='1':
            mode=1
            print('Entering MODE 1: pass through')
        
        elif sys.argv[1]=='2':
            if len(sys.argv)<3:
                sys.exit('Maneuver file not given! Enter maneuver file after mode value.')
            mode=2
            print('Entering MODE 2: preprogrammed open loop maneuver')
        
        elif sys.argv[1]=='3':
            if len(sys.argv)<3:
                sys.exit('Maneuver file not given! Enter maneuver file after mode value.')
            mode=3
            print('Entering MODE 3: preprogrammed closed loop maneuver')
        
        elif sys.argv[1]=='-1':
            if len(sys.argv)<5:
                print('Usage: sudo python3 AC_auto.py -1 pitch roll rudd dead_band')
                print('    pitch = abs(maximum pitch angle) [deg]')
                print('     roll = abs(maximum roll angle) [deg]')
                print('     rudd = abs(maximum rudder angle) [deg]')
                print('dead_band = window around neutral for zero command [u_sec]')
                print('All values should be entered as integers')
                sys.exit('** Not enough inputs! **')
            mode=-1
            print('Entering control mapping calibration mode.')
        
        else:
            exit_sequence(1)
            sys.exit('Unknown input argument!')
    return mode

def get_current_RCinputs():
    # Definition is based of Assan X8R6 receiver
    d_a_pwm=float(rcin.read(1)) #Aileron
    d_e_pwm=float(rcin.read(2)) #Elevator
    d_T_pwm=float(rcin.read(0)) #Throttle
    d_r_pwm=float(rcin.read(3)) #Rudder
    return d_a_pwm,d_e_pwm,d_T_pwm,d_r_pwm

def set_initial_cmds(PM,AHRS_data,d_T_cmd,ARSP_ALT_data):
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
        V_cmd=ARSP_ALT_data[0]
    
    return phi_cmd,theta_cmd,psi_cmd,d_T_cmd,V_cmd

## READ THE CONTROL MAPPING CALIBRATION FILE
def read_calibration_file():
    #Open the config file
    CM=[0]*9 #ControlMapping
    try:
        CMF = open('ControlMapping.txt', 'r')
        data_uf=CMF.read()
        data=[float(s) for s in data_uf.split()]
        CM[0]=data[16] #Rm
        CM[1]=data[17] #Rbl
        CM[2]=data[18] #Rbh
        CM[3]=data[19] #Pm
        CM[4]=data[20] #Pbl
        CM[5]=data[21] #Pbh
        CM[6]=data[22] #Ym
        CM[7]=data[23] #Ybl
        CM[8]=data[24] #Ybh
        CMF.close()
        return CM
    except:
        exit_sequence(1)
        sys.exit('Error reading control mapping file!')

def exit_sequence(flag):
    if flag==1:
        led.setColor('Red')
        set_ext_LED('Red')
    else:
        led.setColor('Black')
        set_ext_LED('Black')
    try:
        flt_log.close()
    except:
        pass      
    exit_flag=1
    process_EXIT.value=1
    AHRS_proc.join()
    ARSP_ALT_proc.join()

def set_ext_LED(color):
    # Set exterior LED to black
    rcou7.set_duty_cycle(0)
    rcou8.set_duty_cycle(0)
    rcou9.set_duty_cycle(0)
    
    if color=='Green':
        rcou8.set_duty_cycle(2)
    elif color=='Red':
        rcou7.set_duty_cycle(2)
    elif color=='Blue':
        rcou9.set_duty_cycle(2)
    elif color=='Cyan':
        rcou8.set_duty_cycle(2)
        rcou9.set_duty_cycle(2)
    elif color=='Yellow':
        rcou7.set_duty_cycle(10)
        rcou8.set_duty_cycle(2)
    elif color=='White':
        rcou7.set_duty_cycle(2)
        rcou8.set_duty_cycle(2)
        rcou9.set_duty_cycle(2)
    elif color=='Black':
        pass
    else:
        pass

##### MAIN PROGRAM #####
# Setup LED
led=leds.Led()

# Read command line inputs during program excecution and direct program accordingly
mode=check_CLI_inputs()

# Setup RCinput
rcin=rcinput.RCInput()

# NORMAL OPERATION
if (mode>0):
    # Setup PWM outputs for controls
    rcou1=pwm.PWM(0)
    rcou2=pwm.PWM(1)
    rcou3=pwm.PWM(2)
    rcou4=pwm.PWM(3)
 
    # Setup external status LED
    rcou7=pwm.PWM(6)
    rcou8=pwm.PWM(7)
    rcou9=pwm.PWM(8)

    # Setup PWM frequencies
    rcou1.set_period(50) #Hz
    rcou2.set_period(50)
    rcou3.set_period(50)
    rcou4.set_period(50)
    rcou7.set_period(50)
    rcou8.set_period(50)
    rcou9.set_period(50)

    # Set LED
    led.setColor('Yellow')
    set_ext_LED('Yellow')
    
    ## Subprocesses
    # Subprocess array for AHRS and pressure transducer data
    AHRS_data=Array('d', [0.0,0.0,0.0,0.0,0.0,0.0,hix,hiy,hiz,0.0,phi_offset,theta_offset,0.0,0.0,0.0,0.0,0.0,0.0,ORIENTATION])
    ARSP_ALT_data=Array('d',[0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0])
    
    # Subprocess value that sets exit flag
    process_EXIT=Value('i', 0)
    
    # Define the subprocesses
    AHRS_proc=Process(target=AHRS_process, args=(process_EXIT,AHRS_data))
    ARSP_ALT_proc=Process(target=ARSP_ALT_process, args=(process_EXIT,ARSP_ALT_data))
    
    # Start the subprocesses
    AHRS_proc.start()
    ARSP_ALT_proc.start()
    
    # Initialize controllers
    CsCal=ControlSurface_Calibration.CS_cal()
    LowLevel=Control_LowLevel.LL_controls()
    MidLevel=Control_MidLevel.ML_controls()
    
    # Wait for all subprocesses to startup and get through initialization
    time.sleep(3)
    
    # Check to make sure magnetometer is functioning correctly and not reporting all zeros
    if AHRS_data[9]==1.0:
        exit_sequence(1)
        sys.exit('Magnetometer reading zero!')

    # Setup up data log
    if FLTLOG==1:
        print('Setting up flight log.')
        try:
            log_timestr = time.strftime("%d%m%Y-%H%M")
            log_timestr2= time.asctime()
            flt_log=open('flight_log_'+log_timestr+'.txt', 'w')
            flt_log.write('# '+log_timestr2+' local time')
            flt_log.write('\n\n')
            flt_log.write('# Total and static pressure offsets\n')
            flt_log.write('# PSF mbar\n')
            flt_log.write('# %.3f %.2f\n' % (ARSP_ALT_data[2],ARSP_ALT_data[3]))
            flt_log.write('\n')
            flt_log.write('T DT PHI THETA PSI PHI_DOT THETA_DOT PSI_DOT P Q R AX AY AZ VIAS ALT VIAS_F ALT_F VACC_F VSI_F ELEV AIL THR RUDD ELEV_CMD AIL_CMD THR_CMD RUDD_CMD AV_BAY_TEMP\n')
            flt_log.write('# sec sec deg deg deg deg/s deg/s deg/s deg/s deg/s deg/s g g g ft/s ft ft/s ft ft/s^2 ft/s PWM PWM PWM PWM deg deg % deg degF\n')
        except:
            exit_sequence(1)
            sys.exit('Error creating log file!')
    else:
        print('** Flight log setup excepted **')
            
    if mode==2 or mode==3:
        print('Processing maneuver file.')
        try:
            PM=read_preprogrammed_maneuver.read_maneuver_file(sys.argv[2])
            PM.read_file()
            if PM.error_flag==1:
                exit_sequence(1)
                sys.exit('Format problem with maneuver file!')
        except:
            exit_sequence(1)
            sys.exit('Error processing maneuver file!')


    print('Initialization complete.')
    print('Starting autopilot loop...')
    t_start=time.time()
    
    count=0 # Used for reduced frame rate output to screen
    auto_at_auto_flag=1
    led.setColor('Cyan')
    set_ext_LED('Cyan')
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
            if auto_at_auto_flag==1:
                if mode==1:
                    # Nothing to do if in pass through mode
                    pass
                
                elif mode==2:
                    # Set flags for stepping through maneuver
                    man_flag=1
                    maneuver_count=0
                    t_man_start=time.time()
                    # Get current control surface commands which will be held constant until maneuver starts
                    d_a_pwm,d_e_pwm,d_T_pwm,d_r_pwm=get_current_RCinputs()
                    # Convert control surface commands to angles
                    d_a_cmd,d_e_cmd,d_r_cmd,d_T_cmd=CsCal.pwm_to_delta(d_a_pwm,d_e_pwm,d_r_pwm,d_T_pwm) #control surfaces
                    d_a_at_auto=d_a_cmd
                    d_e_at_auto=d_e_cmd
                    d_r_at_auto=d_r_cmd
                    
                elif mode==3:
                    #Initialize variables
                    count_at_steady_state_pitch=0
                    count_at_steady_state_roll=0
                    count_at_steady_state_vel=0
                    steady_state_condition_achieved_pitch=0
                    steady_state_condition_achieved_roll=0
                    steady_state_condition_achieved_vel=0
                    man_flag=0
                    maneuver_count=0
                    # Get current control surface commands
                    d_a_pwm,d_e_pwm,d_T_pwm,d_r_pwm=get_current_RCinputs()
                    # Convert control surface commands to angles
                    d_a_cmd,d_e_cmd,d_r_cmd,d_T_cmd=CsCal.pwm_to_delta(d_a_pwm,d_e_pwm,d_r_pwm,d_T_pwm) #control surfaces
                    # Set initial commands
                    phi_cmd,theta_cmd,psi_cmd,d_T_cmd,V_cmd=set_initial_cmds(PM,AHRS_data,d_T_cmd,ARSP_ALT_data[0])

                    #Seed controllers
                    LowLevel.ail_PID.integral_term=d_a_cmd
                    LowLevel.elev_PID.integral_term=d_e_cmd
                
                #Change flag value (only needs to be active at the moment controller is activated
                auto_at_auto_flag=0
            
            # PASS THROUGH MODE
            if (mode==1):
                d_a_pwm,d_e_pwm,d_T_pwm,d_r_pwm=get_current_RCinputs()
                d_e_cmd=0
                d_a_cmd=0
                d_T_cmd=0
                d_r_cmd=0
                phi_cmd=0
                theta_cmd=0
                psi_cmd=0
                V_cmd=0
            
            # PREPROGRAMMED MANEUVER MODE - OPEN LOOP
            elif mode==2:
                t_current=time.time()
                if (t_current-t_man_start)>=PM.man_time[maneuver_count] and man_flag<2:
                    # If the current time is equal to (or just barely greater than) the maneuver time, change the target values
                    if PM.def_type==1: #Absolute deflection
                        d_e_cmd=PM.man_elev[maneuver_count] #elevator deflection
                        d_a_cmd=PM.man_ail[maneuver_count] #aileron deflection
                        d_r_cmd=PM.man_rudd[maneuver_count] #rudder deflection
                        d_T_cmd=0 #throttle command set to current setting
                    if PM.def_type==2: #Relative deflection (maneuver+command at AP activation)
                        d_e_cmd=PM.man_elev[maneuver_count]+d_e_at_auto #elevator deflection
                        d_a_cmd=PM.man_ail[maneuver_count]+d_a_at_auto #aileron deflection
                        d_r_cmd=PM.man_rudd[maneuver_count]+d_r_at_auto #rudder deflection
                        d_T_cmd=0 #throttle command set to current setting
                    # Increment count to set next maneuver time
                    maneuver_count+=1
                    if maneuver_count>=len(PM.man_time):
                        man_flag=2
                        maneuver_count=0
                else:
                    # After maneuver is complete, just hold the last command for each control surface
                    pass
                
                # Convert the commanded control surface angles to PWM
                d_a_pwm,d_e_pwm,d_r_pwm,d_T_pwm=CsCal.delta_to_pwm(d_a_cmd,d_e_cmd,d_r_cmd,d_T_cmd)
                d_T_pwm=float(rcin.read(0)) ##Throttle hard coded to manual
            
            # PREPROGRAMMED MANEUVER MODE - CLOSED LOOP
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
                        if abs(ARSP_ALT_data[0]-V_cmd)<steady_state_vel_range and count_at_steady_state_vel<(steady_state_time/loop_dt):
                            count_at_steady_state_vel+=1
                        elif abs(ARSP_ALT_data[0]-V_cmd)>steady_state_vel_range:
                            count_at_steady_state_vel=0
                        else:
                            steady_state_condition_achieved_vel=1
                    #Auto set flag when throttle is set as the initial condition
                    elif PM.ic_type==4:
                        steady_state_condition_achieved_vel=1
        
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
        
        # Current time 
        t_2=time.time()
        #dt=t_2-t_1
        
        # Time since script was started
        t_elapsed=t_2-t_start
        
        if FLTLOG==1:
            if gear_switch>1500:       
                #              T    DT   PHI THETA PSI  PHID THtD PSID  P    Q    R   AX   AY   AZ   IAS  ALT  IASF ALTF VACF VSIF E  A  T  R  EC   AC   TC RC   DEGF
                flt_log.write('%.3f %.4f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.3f %.3f %.3f %.1f %.0f %.1f %.0f %.1f %.0f %d %d %d %d %.1f %.1f %d %.1f %.1f\n' % (t_elapsed,
                                                                                                         dt3,
                                                                                                         AHRS_data[0],
                                                                                                         AHRS_data[1],
                                                                                                         AHRS_data[2],
                                                                                                         AHRS_data[4],
                                                                                                         AHRS_data[5],
                                                                                                         AHRS_data[3],
                                                                                                         AHRS_data[15],
                                                                                                         AHRS_data[16],
                                                                                                         AHRS_data[17],
                                                                                                         AHRS_data[12],
                                                                                                         AHRS_data[13],
                                                                                                         AHRS_data[14],
                                                                                                         ARSP_ALT_data[0],
                                                                                                         ARSP_ALT_data[1],
                                                                                                         ARSP_ALT_data[4],
                                                                                                         ARSP_ALT_data[5],
                                                                                                         ARSP_ALT_data[6],
                                                                                                         ARSP_ALT_data[7],
                                                                                                         d_e_pwm,
                                                                                                         d_a_pwm,
                                                                                                         d_T_pwm,
                                                                                                         d_r_pwm,
                                                                                                         d_e_cmd,
                                                                                                         d_a_cmd,
                                                                                                         d_T_cmd,
                                                                                                         d_r_cmd,
                                                                                                         ARSP_ALT_data[8]))
        else:
            pass
        
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
        #print(ARSP_ALT_data[0])
        #print(ARSP_ALT_data[1])
        
# ---------- Calibration Modes ---------- #
# CONTROL CALIBRATION MODE
# in this mode the PWM to pitch/roll/yaw curves are determined based on the user input file
elif mode==-1:
    led.setColor('Black')
    #set_ext_LED('Black')

    # Intialize PWM array
    RCinput=[0]*4

    print('CONTROL CALIBRATION MODE')
    print(' ')
    print('Step 1: Determine center points.')
    print('        Do not move sticks for 5 seconds.')
    print(' ')
    print(' ** Set gear switch to AUTO mode before proceeding! **')
    print(' ')
    dummy=input('Press RETURN to start ')
    print(' ')
    print('Determining center points...')
    print(' ')
    CENTER_PWM=[0,0,0,0]
    t1=time.time()
    x=0.0
    i=1
    while x<5.0:
        # Get inputs from the receiver
        # Definition is based of Assan X8R6
        RCinput[0]=float(rcin.read(1))
        RCinput[1]=float(rcin.read(2))
        RCinput[2]=float(rcin.read(0))
        RCinput[3]=float(rcin.read(3))
        
        if i==1:
            c0=RCinput[0]
            c1=RCinput[1]
            c2=RCinput[2]
            c3=RCinput[3]
        else:
            c0=c0+(RCinput[0]-c0)/(i+1)
            c1=c1+(RCinput[1]-c1)/(i+1)
            c2=c2+(RCinput[2]-c2)/(i+1)
            c3=c3+(RCinput[3]-c3)/(i+1)

        i=i+1
        t2=time.time()
        x=t2-t1
            
    c0=int(c0)
    c1=int(c1)
    c2=int(c2)
    c3=int(c3)
    print('Average center points found to be (uS)')
    print('Ch0   Ch1   Ch2   Ch3')
    print('%d  %d  %d  %d' % (c0,c1,c2,c3))
    print(' ')
    print(' ')
    print('Step 2: Determine maximum and minimum PWM values from the RX.')
    print('        Move all sticks to their extremes within 10 seconds')
    print(' ')
    dummy=input('Press RETURN to start ')
    print(' ')
    print('Determining max and min PWM values...')
    print(' ')
    MAX_PWM=[1500,1500,1500,1500]
    MIN_PWM=[1500,1500,1500,1500]
    t1=time.time()
    x=0.0
    while x<10.0:
        # Get inputs from the receiver and save the max and min values
        # Definition is based of Assan X8R6
        RCinput[0]=float(rcin.read(1))
        RCinput[1]=float(rcin.read(2))
        RCinput[2]=float(rcin.read(0))
        RCinput[3]=float(rcin.read(3))

        if RCinput[0]>MAX_PWM[0]:
            MAX_PWM[0]=RCinput[0]
        if RCinput[0]<MIN_PWM[0]:
            MIN_PWM[0]=RCinput[0]
        
        if RCinput[1]>MAX_PWM[1]:
            MAX_PWM[1]=RCinput[1]
        if RCinput[1]<MIN_PWM[1]:
            MIN_PWM[1]=RCinput[1]
        
        if RCinput[2]>MAX_PWM[2]:
            MAX_PWM[2]=RCinput[2]
        if RCinput[2]<MIN_PWM[2]:
            MIN_PWM[2]=RCinput[2]
        
        if RCinput[3]>MAX_PWM[3]:
            MAX_PWM[3]=RCinput[3]
        if RCinput[3]<MIN_PWM[3]:
            MIN_PWM[3]=RCinput[3]

        t2=time.time()
        x=t2-t1

    max_p=int(sys.argv[2])
    max_r=int(sys.argv[3])
    max_rud=int(sys.argv[4])
    dead_band=int(sys.argv[5])

    print('Channel max and mins values collected (uS)')
    print('     Ch0   Ch1   Ch2   Ch3')
    print('MIN: %d  %d  %d  %d' % (MIN_PWM[0],MIN_PWM[1],MIN_PWM[2],MIN_PWM[3]))
    print('MAX: %d  %d  %d  %d' % (MAX_PWM[0],MAX_PWM[1],MAX_PWM[2],MAX_PWM[3]))
    print(' ')
    print('Determining slopes and intercepts based on:')
    print('    Pitch limit (deg):  %d' % max_p)
    print('    Roll limit (deg):   %d' % max_r)
    print('    Rudder limit (deg): %d' % max_rud)
    print('    Dead band (uS):     %d' % dead_band)
    print(' ')
                      
    # PITCH
    # Determine slopes and intercepts
    mp_H=-max_p/(-MAX_PWM[1]+c1+dead_band)
    bp_H=(c1+dead_band)*max_p/(-MAX_PWM[1]+c1+dead_band)
    mp_L=max_p/(-MIN_PWM[1]+c1-dead_band)
    bp_L=-(c1-dead_band)*max_p/(-MIN_PWM[1]+c1-dead_band)

    # Find out which one is smaller and then compile the appropriate slope and intercepts
    if mp_H<=mp_L:
        bp_L2=-mp_H*(c1-dead_band)
        PTA_P=[mp_H,bp_L2,bp_H]
    elif mp_H>mp_L:
        bp_H2=-mp_L*(c1+dead_band)
        PTA_P=[mp_L,bp_L,bp_H2]

    # ROLL
    # Determine slopes and intercepts
    mr_H=-max_r/(-MAX_PWM[0]+c0+dead_band)
    br_H=(c0+dead_band)*max_r/(-MAX_PWM[0]+c0+dead_band)
    mr_L=max_r/(-MIN_PWM[0]+c0-dead_band)
    br_L=-(c0-dead_band)*max_r/(-MIN_PWM[0]+c0-dead_band)

    # Find out which one is smaller and then compile the appropriate slope and intercepts
    if mr_H<=mr_L:
        br_L2=-mr_H*(c0-dead_band)
        PTA_R=[mr_H,br_L2,br_H]
    elif mr_H>mr_L:
        br_H2=-mr_L*(c0+dead_band)
        PTA_R=[mr_L,br_L,br_H2]

    # YAW
    # Determine slopes and intercepts
    my_H=-max_rud/(-MAX_PWM[3]+c3+dead_band)
    by_H=(c3+dead_band)*max_rud/(-MAX_PWM[3]+c3+dead_band)
    my_L=max_rud/(-MIN_PWM[3]+c3-dead_band)
    by_L=-(c3-dead_band)*max_rud/(-MIN_PWM[3]+c3-dead_band)

    # Find out which one is smaller and then compile the appropriate slope and intercepts
    if my_H<=my_L:
        by_L2=-my_H*(c3-dead_band)
        PTA_Y=[my_H,by_L2,by_H]
    elif my_H>my_L:
        by_H2=-my_L*(c3+dead_band)
        PTA_Y=[my_L,by_L,by_H2]

    print('Slopes and intercepts for roll, pitch, and rudder control stick  PWM to target angle')
    print('Roll   = %.3f %.3f %.3f' % (PTA_R[0],PTA_R[1],PTA_R[2]))
    print('Pitch  = %.3f %.3f %.3f' % (PTA_P[0],PTA_P[1],PTA_P[2]))
    print('Rudder = %.3f %.3f %.3f' % (PTA_Y[0],PTA_Y[1],PTA_Y[2]))
    print(' ')

    calib=open('ControlMapping.txt', 'w')
    calib.write('%d %d %d %d\n' % (max_r,max_p,max_rud,dead_band))
    calib.write('%d %d %d %d\n' % (c0,c1,c2,c3))
    calib.write('%d %d %d %d\n' % (MIN_PWM[0],MIN_PWM[1],MIN_PWM[2],MIN_PWM[3]))
    calib.write('%d %d %d %d\n' % (MAX_PWM[0],MAX_PWM[1],MAX_PWM[2],MAX_PWM[3]))
    calib.write('%.4f %.4f %.4f\n' % (PTA_R[0],PTA_R[1],PTA_R[2]))
    calib.write('%.4f %.4f %.4f\n' % (PTA_P[0],PTA_P[1],PTA_P[2]))
    calib.write('%.4f %.4f %.4f' % (PTA_Y[0],PTA_Y[1],PTA_Y[2]))
    calib.close()

    print('Control calibration complete and calibration file written.')
    print(' ')
    print('Done.')

# ---------- Exit Sequence ---------- #
if (mode>0):
    exit_sequence(0)

    led.setColor('Red')
    set_ext_LED('Red')
    time.sleep(0.1)
    led.setColor('Blue')
    set_ext_LED('Blue')
    time.sleep(0.1)
    led.setColor('Red')
    set_ext_LED('Red')
    time.sleep(0.1)
    led.setColor('Blue')
    set_ext_LED('Blue')
    time.sleep(0.1)
    led.setColor('Red')
    set_ext_LED('Red')
    time.sleep(0.1)
    led.setColor('Blue')
    set_ext_LED('Blue')
    time.sleep(0.1)
    led.setColor('Red')
    set_ext_LED('Red')
    time.sleep(0.1)
    led.setColor('Blue')
    set_ext_LED('Blue')
    time.sleep(0.1)
    led.setColor('Red')
    set_ext_LED('Red')
    time.sleep(0.1)
    led.setColor('Blue')
    set_ext_LED('Blue')
    time.sleep(0.1)
    led.setColor('Red')
    set_ext_LED('Red')
    time.sleep(0.1)
    led.setColor('Blue')
    set_ext_LED('Blue')
    time.sleep(0.1)
    led.setColor('Red')
    set_ext_LED('Red')
    time.sleep(0.1)
    led.setColor('Blue')
    set_ext_LED('Blue')
    time.sleep(0.1)
    led.setColor('Red')
    set_ext_LED('Red')
    time.sleep(0.1)
    led.setColor('Blue')
    set_ext_LED('Blue')
    time.sleep(0.1)
    led.setColor('Black')
    set_ext_LED('Black')
    print('Done.')
