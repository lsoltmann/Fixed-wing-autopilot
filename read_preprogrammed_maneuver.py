'''
    read_preprogrammed_maneuver.py
    
    Description: Class for reading preprogrammed maneuver file
    
    Revision History
    27 Mar 2017 - Created and debugged
    
    Author: Lars Soltmann
    
    References: None
    
    Calls: None
    
    Inputs: maneuver_CL.txt or maneuver_OL.txt
            
    Outputs:
    
    Notes:
    - Written for Python3
    - See maneuver file for file format:
    
'''

import sys

class read_maneuver_file:
    def __init__(self,maneuver_file):
        self.maneuver_file=maneuver_file
        self.flag=0
        self.error_flag=0
        self.length_check=[]

    def read_file(self):
        #Open the config file
        file1 = open(self.maneuver_file, 'r')
        self.flag=0
        
        for line in file1:
            #Skip lines that are blank or start with
            if (line[0]=='#' or line=='\n'):
                pass

            #### TIME (1)
            #If this phrase is found, set a flag so that the next time around the line is read into the appropriate variables
            #The units for time should be in seconds. TIME=0 when controller is activated
            elif line=='TIME\n' or self.flag==1:
                if self.flag==1:
                    try:
                        #Read all maneuver points and get the length
                        self.man_time=[float(s) for s in line.split()]
                        self.length_check.append(len(self.man_time))
                        self.flag=0
                    except:
                        print('Error reading maneuver time!')
                        self.error_flag=1
                else:
                    self.flag=1

            #### THETA (2) [deg] - CL
            elif line=='THETA\n' or self.flag==2:
                 if self.flag==2:
                     try:
                        self.man_theta=[float(s) for s in line.split()]
                        self.length_check.append(len(self.man_theta))
                        self.flag=0
                     except:
                        print('Error reading maneuver pitch angles!')
                        self.error_flag=1
                 else:
                     self.flag=2

            #### PHI (3) [deg] - CL
            elif line=='PHI\n' or self.flag==3:
                 if self.flag==3:
                     try:
                        self.man_phi=[float(s) for s in line.split()]
                        self.length_check.append(len(self.man_phi))
                        self.flag=0
                     except:
                        print('Error reading maneuver roll angles!')
                        self.error_flag=1
                 else:
                     self.flag=3

            #### VELOCITY (4) [ft/s] - CL
            elif line=='VEL\n' or self.flag==4:
                 if self.flag==4:
                     try:
                        self.man_vel=[float(s) for s in line.split()]
                        self.length_check.append(len(self.man_vel))
                        if (len(self.man_vel)==1 and self.man_vel[0]==-1):
                            del self.length_check[-1]
                        self.flag=0
                     except:
                        print('Error reading maneuver velocity!')
                        self.error_flag=1
                 else:
                     self.flag=4

            #### THROTTLE (5) [percentage] - CL
            elif line=='THR\n' or self.flag==5:
                 if self.flag==5:
                     try:
                        self.man_thr=[float(s) for s in line.split()]
                        self.length_check.append(len(self.man_thr))
                        if (len(self.man_thr)==1 and self.man_thr[0]==-1):
                            del self.length_check[-1]
                        self.flag=0
                     except:
                        print('Error reading maneuver throttle!')
                        self.error_flag=1
                 else:
                     self.flag=5

            #### IC_TYPE (6) *See maneuver file for description*  - CL
            elif line=='IC_TYPE\n' or self.flag==6:
                 if self.flag==6:
                     try:
                        self.ic_type=[float(s) for s in line.split()]
                        self.ic_type=self.ic_type[0]
                        if (self.ic_type!=1 and  self.ic_type!=2):
                            print('Invalid input for maneuver IC_TYPE!')
                            self.error_flag=1
                        self.flag=0
                     except:
                        print('Error reading maneuver throttle!')
                        self.error_flag=1
                 else:
                     self.flag=6
                        
            #### ELEVATOR DEFLECTION (7) [deg]  - OL
            elif line=='DE\n' or self.flag==7:
                 if self.flag==7:
                     try:
                        self.man_elev=[float(s) for s in line.split()]
                        self.length_check.append(len(self.man_elev))
                        self.flag=0
                     except:
                        print('Error reading maneuver elevator deflection!')
                        self.error_flag=1
                 else:
                     self.flag=7
  
            #### AILERON DEFLECTION (8) [deg]  - OL
            elif line=='DA\n' or self.flag==8:
                 if self.flag==8:
                     try:
                        self.man_ail=[float(s) for s in line.split()]
                        self.length_check.append(len(self.man_ail))
                        self.flag=0
                     except:
                        print('Error reading maneuver aileron deflection!')
                        self.error_flag=1
                 else:
                     self.flag=8
                        
            #### RUDDER DEFLECTION (9) [deg]  - OL
            elif line=='DR\n' or self.flag==9:
                 if self.flag==9:
                     try:
                        self.man_rudd=[float(s) for s in line.split()]
                        self.length_check.append(len(self.man_rudd))
                        self.flag=0
                     except:
                        print('Error reading maneuver rudder deflection!')
                        self.error_flag=1
                 else:
                     self.flag=9
                        
            #### DEF_TYPE (10) *See maneuver file for description*  - OL
            elif line=='DEF_TYPE\n' or self.flag==10:
                 if self.flag==10:
                     try:
                        self.def_type=[float(s) for s in line.split()]
                        self.def_type=self.def_type[0]
                        if self.def_type!=1 and self.def_type!=2:
                            print('Invalid input for maneuver DEF_TYPE!')
                            self.error_flag=1
                        self.flag=0
                     except:
                        print('Error reading maneuver throttle!')
                        self.error_flag=1
                 else:
                     self.flag=10

        ##Check to make sure the length of all maneuver points are equal
        if (all(x == self.length_check[0] for x in self.length_check)==True):
            pass
        else:
            print('Number of maneuver points not equal across states!')
            self.error_flag=1

        if (self.maneuver_file=='maneuver_CL.txt'):
            ##Make sure that first time entry is zero if IC_TYPE=2
            if (IC_TYPE==2 and self.man_time[0]!=0):
                print('First maneuver time entry needs to be zero if IC_TYPE=2!')
                self.error_flag=1

            ##Change the IC_TYPE to reflect whether velocity or throttle is to be used
            if self.man_thr[0]==-1:
                self.ic_type=3
            elif self.man_vel[0]==-1:
                self.ic_type=4
            else:
                print('Error occured while assigning IC_TYPE from velocity/throttle!')
                self.error_flag=1
