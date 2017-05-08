'''
    read_preprogrammed_maneuver.py
    
    Description: Class for reading preprogrammed maneuver file
    
    Revision History
    27 Mar 2017 - Created and debugged
    
    Author: Lars Soltmann
    
    References: None
    
    Calls: None
    
    Inputs: maneuver.txt
            
    Outputs:
    
    Notes:
    - Written for Python3
    - File format for configuration file:
    ______________________________
    #Configuration file
    #
    #General format for file is:
    #'Variable'
    #<values>
    #
    
    IC_TYPE <-- Either 1 or 2 [1=current conditions, 2=specified conditions]
    1
    
    TIME
    0 0.5 1 1.5 2 2.5

    THETA
    0 0 5 0 0 -5

    VEL
    60 60 60 70 70 70
    
    ... etc
    ______________________________
    
    *The number of columns for each variable must be equal across all variables
    
'''

import sys

class read_maneuver_file:
    def __init__(self,maneuver_file):
        self.maneuver_file=maneuver_file
        self.flag=0
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
                        sys.exit('Error reading maneuver time!')
                else:
                    self.flag=1
        
            #### THETA (2) [deg]
            elif line=='THETA\n' or self.flag==2:
                 if self.flag==2:
                     try:
                        self.man_theta=[float(s) for s in line.split()]
                        self.length_check.append(len(self.man_theta))
                        self.flag=0
                     except:
                        sys.exit('Error reading maneuver pitch angles!')
                 else:
                     self.flag=2
                         
            #### PHI (3) [deg]
            elif line=='PHI\n' or self.flag==3:
                 if self.flag==3:
                     try:
                        self.man_phi=[float(s) for s in line.split()]
                        self.length_check.append(len(self.man_phi))
                        self.flag=0
                     except:
                        sys.exit('Error reading maneuver roll angles!')
                 else:
                     self.flag=3

            #### VELOCITY (4) [ft/s]
            elif line=='VEL\n' or self.flag==4:
                 if self.flag==4:
                     try:
                        self.man_vel=[float(s) for s in line.split()]
                        self.length_check.append(len(self.man_vel))
                        if (len(self.man_vel)==1 and self.man_vel[0]==-1):
                            del self.length_check[-1]
                        self.flag=0
                     except:
                        sys.exit('Error reading maneuver velocity!')
                 else:
                     self.flag=4

            #### THROTTLE (5) [percentage]
            elif line=='THR\n' or self.flag==5:
                 if self.flag==5:
                     try:
                        self.man_thr=[float(s) for s in line.split()]
                        self.length_check.append(len(self.man_thr))
                        if (len(self.man_thr)==1 and self.man_thr[0]==-1):
                            del self.length_check[-1]
                        self.flag=0
                     except:
                        sys.exit('Error reading maneuver throttle!')
                 else:
                     self.flag=5
                         
            #### IC_TYPE (6) *See maneuver file for description*
            elif line=='IC_TYPE\n' or self.flag==6:
                 if self.flag==6:
                     try:
                        self.ic_type=[float(s) for s in line.split()]
                        self.ic_type=self.ic_type[0]
                        if self.ic_type!=1 or self.ic_type!=2:
                            sys.exit('Invalid input for maneuver IC_TYPE!')
                        self.flag=0
                     except:
                        sys.exit('Error reading maneuver throttle!')
                 else:
                     self.flag=6

        ##Check to make sure the length of all maneuver points are equal
        if (all(x == self.length_check[0] for x in self.length_check)==True):
            pass
        else:
            sys.exit('Number of maneuver points not equal across states!')

        ##Make sure that first time entry is zero if IC_TYPE=2
        if self.man_time[0]!=0:
            sys.exit('First maneuver time entry needs to be zero if IC_TYPE=2!')

        ##Change the IC_TYPE to reflect whether velocity or throttle is to be used
        if self.man_thr[0]==-1:
            self.ic_type=3
        elif self.man_vel[0]==-1:
            self.ic_type=4
        else:
            sys.exit('Error occured while assigning IC_TYPE from velocity/throttle!')
