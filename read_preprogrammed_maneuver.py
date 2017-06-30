'''
    read_preprogrammed_maneuver.py
    
    Description: Class for reading preprogrammed maneuver file
    
    Revision History
    30 Jun 2017 - Created and debugged
    
    Author: Lars Soltmann
    
    References: None
    
    Calls: None
    
    Inputs: maneuver_CL.txt or maneuver_OL.txt
            
    Outputs:
    
    Notes:
    - Written for Python3
    - See maneuver file for file format
    
'''

import sys
import numpy as np

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

            #### CONTROL_INPUTS (1) - CL or OL
            #Units are seconds and degrees. TIME=0 when controller is activated
            elif line=='CONTROL_INPUTS\n' or self.flag==1:
                # Once the tag has been found, start processing the data
                if self.flag==1:
                    #try:
                    # If the end of the maneuver is found, set the flag back to 0
                    if line=='END\n' or line=='END':
                        self.flag=0
                    # Read in the maneuver data
                    else:
                        cntrl_line=[float(s) for s in line.split()]
                        try:
                            self.MANEUVER_ARRAY=np.vstack((self.MANEUVER_ARRAY,cntrl_line))
                        except:
                            self.MANEUVER_ARRAY=np.array(cntrl_line)
                    #except:
                        #print('Error reading maneuver sequence!')
                        #self.error_flag=1
                # If the tag was found, set flag so that it will read the data in during the next round
                else:
                    self.flag=1

            #### IC_TYPE (6) *See maneuver file for description*  - CL
            elif line=='IC_TYPE\n' or self.flag==6:
                 if self.flag==6:
                     try:
                        self.IC_TYPE=[int(s) for s in line.split()]
                        self.IC_TYPE=self.IC_TYPE[0]
                        if (self.IC_TYPE!=1 and  self.IC_TYPE!=2):
                            print('Invalid input for maneuver IC_TYPE!')
                            self.error_flag=1
                        self.flag=0
                     except:
                        print('Error reading maneuver throttle!')
                        self.error_flag=1
                 else:
                     self.flag=6
                        
            #### DEF_TYPE (10) *See maneuver file for description*  - OL
            elif line=='DEF_TYPE\n' or self.flag==10:
                 if self.flag==10:
                     try:
                        self.DEF_TYPE=[int(s) for s in line.split()]
                        self.DEF_TYPE=self.DEF_TYPE[0]
                        if self.DEF_TYPE!=1 and self.DEF_TYPE!=2 and self.DEF_TYPE!=3:
                            print('Invalid input for maneuver DEF_TYPE!')
                            self.error_flag=1
                        self.flag=0
                     except:
                        print('Error reading maneuver throttle!')
                        self.error_flag=1
                 else:
                     self.flag=10
