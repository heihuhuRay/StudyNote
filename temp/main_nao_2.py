# -*- coding: utf-8 -*-
import naoqi
from naoqi import ALProxy
import math
import time
from naoqi import ALModule
import matplotlib.pyplot as plt
import numpy as np
import scipy.signal as signal
from set_timing import *
from ml_mp_cpg import *
from nao_motor import *
from random import randint
import lowpass_filter as lpf

#NAOIP = "nao.local"

NAOIP = 'nao.local'

PORT = 9559

number_cpg = 26

global All_Command
global All_Sensor

global LFsrFL,LFsrFR,LFsrBL,LFsrBR,RFsrFL,RFsrFR,RFsrBL,RFsrBR,LHandBackSensor,LHandLeftSensor,LHandRightSensor,RHandBackSensor,RHandLeftSensor,RHandRightSensor 

global myCont,CurPos,myT


All_Command = []
All_Sensor = []

#################################################################################################################################
####################################### My variables ############################################################################
#################################################################################################################################
left_foot_time_list = []
right_foot_time_list = []
left_knee_time_list = []
right_knee_time_list = []
left_foot_hit_timestamp_list = []

total_left_FSR_dir_list = ['Device/SubDeviceList/LFoot/FSR/TotalWeight/Sensor/Value']
total_right_FSR_dir_list = ['Device/SubDeviceList/RFoot/FSR/TotalWeight/Sensor/Value']

FSR_list_left_foot = []
FSR_list_right_foot = []
temp_FSR_time_list = []
#################################################################################################################################
####################################### My variables ############################################################################
#################################################################################################################################


######################################
# Connect to the module ALMotionProxy 
movObj = ALProxy("ALMotion",NAOIP,PORT)
movObj.setStiffnesses('Body',0.99)

# Connect to the module ALTextToSpeechProxy
TextObj = ALProxy("ALTextToSpeech",NAOIP,PORT)

# Connect to the module ALRobotPostureProxy
postObj = ALProxy("ALRobotPosture",NAOIP,PORT)

# Connect to the module ALVideoDeviceProxy
vidObj = ALProxy("ALVideoDevice",NAOIP,PORT)

# Connect to the module ALMemoryProxy
memProxy = ALProxy("ALMemory",NAOIP,PORT)

# Connect to the module DCM
dcm = ALProxy("DCM",NAOIP,PORT)

# Connect to the module ALLeds
ledObj = ALProxy("ALLeds", NAOIP, PORT)


######################################
# ALVideoDeviceProxy
subscriberID = 'matlab'
fps = 30
Resolution = 2
ColorSpace = 13
CameraIndex = 0
vidObj.subscribeCamera(subscriberID,CameraIndex ,Resolution ,ColorSpace, fps)
vidObj.setParam(18, 0)
vidObj.setFrameRate(subscriberID,10)

# Image container is an array as follow: (indexing is in cpp)
    #[0]: width.
    #[1]: height.
    #[2]: number of layers.
    #[3]: ColorSpace.
    #[4]: time stamp (seconds).
    #[5]: time stamp (micro-seconds).
    #[6]: array of size height * width * nblayers containing image data.
    #[7]: camera ID (kTop=0, kBottom=1).
    #[8]: left angle (radian).
    #[9]: topAngle (radian).
    #[10]: rightAngle (radian).
    #[11]: bottomAngle (radian).    
######################################
   
time.sleep(1)

# Control "RElbowYaw" joint 
fractionMaxSpeed = 1.0

# Disable Fall Manager 
TextObj.say('Attention') 
# panpan
movObj.setFallManagerEnabled(False) # True False
time.sleep(1)

# http://doc.aldebaran.com/2-1/family/robots/postures_robot.html#robot-postures
#TextObj.say('Initial posture: standing')
postObj.goToPosture("StandInit",0.8)
time.sleep(2)

######################################
# ALMemory 
ReadlistData = [
              ## Head Touch
              "Device/SubDeviceList/Head/Touch/Front/Sensor/Value",  # 0
              "Device/SubDeviceList/Head/Touch/Rear/Sensor/Value",   # 1
              "Device/SubDeviceList/Head/Touch/Middle/Sensor/Value", #2
              ## LeftHandTouch
              "Device/SubDeviceList/LHand/Touch/Back/Sensor/Value", #3 
              "Device/SubDeviceList/LHand/Touch/Left/Sensor/Value", #4 
              "Device/SubDeviceList/LHand/Touch/Right/Sensor/Value", #5
              ## RightHandTouch 
              "Device/SubDeviceList/RHand/Touch/Back/Sensor/Value", #6
              "Device/SubDeviceList/RHand/Touch/Left/Sensor/Value", #7 
              "Device/SubDeviceList/RHand/Touch/Right/Sensor/Value" #8 
              ## LeftFSR 
              "Device/SubDeviceList/LFoot/FSR/FrontLeft/Sensor/Value", #9
              "Device/SubDeviceList/LFoot/FSR/FrontRight/Sensor/Value", #10 
              "Device/SubDeviceList/LFoot/FSR/RearLeft/Sensor/Value",  #11 
              "Device/SubDeviceList/LFoot/FSR/RearRight/Sensor/Value",  #12
              ## RightFSR 
              "Device/SubDeviceList/RFoot/FSR/FrontLeft/Sensor/Value", #13 
              "Device/SubDeviceList/RFoot/FSR/FrontRight/Sensor/Value", #14
              "Device/SubDeviceList/RFoot/FSR/RearLeft/Sensor/Value",  #15
              "Device/SubDeviceList/RFoot/FSR/RearRight/Sensor/Value", #16 
              ## Body Angle
              "Device/SubDeviceList/InertialSensor/AngleX/Sensor/Value", #17 
              "Device/SubDeviceList/InertialSensor/AngleY/Sensor/Value"  #18 
           ]


listValRead = memProxy.getListData( ReadlistData )
print("listVal:" ,listValRead)
#print "listVal[0]:" ,listValRead[0] 
#print "listVal[1]:" ,listValRead[1] 

###################################### 
#start preparation
######################################
myT = fSetTiming()

myCont = fnewMLMPcpg(number_cpg)

myCont = fSetCPGNet(myCont,'MyNao.txt','MyNaoPsitiveAngle_E_or_F.txt')
#myCont2[25].RG.E.Es = 99

# set oscillator for RG_Pattern
PatternOsc0 = RG_Patterns(10,10,1,0.1)
PatternOsc1 = RG_Patterns(17.5,17.5,1,0.05)
PatternOsc2 = RG_Patterns(10,10,1,0.1) 
PatternOsc3 = RG_Patterns(2,10,1,0.1) # This is a smooth patern 
PatternOsc4 = RG_Patterns(1.5,10,1,0.1) 
PatternOsc_faster_walking = RG_Patterns(17.5, 23.5, 1, 0.05) #sigma_s range [13, 23]

PatternOsc = PatternOsc_faster_walking

# Several patterns in the following can be found in the reference paper
# PatternOsc1.fPrintPattern()
# Plateau
"""
  Plateau
   ^
   |     ------------
   |   /
   |  / 
   | /
   ------------------------> 
 """
PatternPL1 = RG_Patterns(5,0.1,1,0.1)
PatternPL2 = RG_Patterns(5,0.1,1,0.4)
#PatternPL1.fPrintPattern()

"""
  QUIESCENT
   ^
   |     
   |   /\
   |  /  \
   | /    \____________
   ------------------------> 
 """
PatternQU = RG_Patterns(0.5,0.5,5,0.1)
#PatternQU.fPrintPattern()

"""
  Almost-Osc
   ^
   |     
   |   /\
   |  /  \    /\  
   | /    \  /  \/\-----
   |/      \/
   ------------------------> 
 """
PatternAosc = RG_Patterns(0.9,5,1,0.1)
#PatternAosc.fPrint()

#LSP_PFPattern = PF_Patterns(0.1,0)
LSP_PFPattern = PF_Patterns(0.0,0)

LHR_PFPattern = PF_Patterns(0.022,0)
RHR_PFPattern = LHR_PFPattern

LAR_PFPattern = PF_Patterns(0.022,0)
RAR_PFPattern = LAR_PFPattern


LHP_PFPattern = PF_Patterns(0.02,0)
RHP_PFPattern = LHP_PFPattern

LKP_PFPattern = PF_Patterns(0.015,0) # or 0.02
RKP_PFPattern = LKP_PFPattern

LAP_PFPattern = PF_Patterns(0.00,0)
RAP_PFPattern = LAP_PFPattern


#LSP_PFPattern.fPrint()

myCont[L_SHOULDER_PITCH].fSetPatternPF(LSP_PFPattern)
myCont[L_SHOULDER_PITCH].fSetPatternRG(PatternOsc) # PatternPL1 PatternOsc3 PatternAosc PatternQU

myCont[L_SHOULDER_ROLL].fSetPatternPF(LSP_PFPattern)
myCont[L_SHOULDER_ROLL].fSetPatternRG(PatternOsc) # PatternPL1 PatternOsc3 PatternAosc PatternQU

myCont[R_SHOULDER_PITCH].fSetPatternPF(LSP_PFPattern)
myCont[R_SHOULDER_PITCH].fSetPatternRG(PatternOsc) # PatternPL1 PatternOsc3 PatternAosc PatternQU

myCont[R_SHOULDER_ROLL].fSetPatternPF(LSP_PFPattern)
myCont[R_SHOULDER_ROLL].fSetPatternRG(PatternOsc) # PatternPL1 PatternOsc3 PatternAosc PatternQU


# for roll legs motors 
# HIP
myCont[L_HIP_ROLL].fSetPatternPF(LHR_PFPattern)
myCont[L_HIP_ROLL].fSetPatternRG(PatternOsc) # PatternPL1 PatternOsc3 PatternAosc PatternQU

myCont[R_HIP_ROLL].fSetPatternPF(RHR_PFPattern)
myCont[R_HIP_ROLL].fSetPatternRG(PatternOsc) # PatternPL1 PatternOsc3 PatternAosc PatternQU

# ANKLE
myCont[L_ANKLE_ROLL].fSetPatternPF(LAR_PFPattern)
myCont[L_ANKLE_ROLL].fSetPatternRG(PatternOsc) # PatternPL1 PatternOsc3 PatternAosc PatternQU

myCont[R_ANKLE_ROLL].fSetPatternPF(RAR_PFPattern)
myCont[R_ANKLE_ROLL].fSetPatternRG(PatternOsc) # PatternPL1 PatternOsc3 PatternAosc PatternQU

# for pitch legs motors 
# Right
myCont[R_HIP_PITCH].fSetPatternPF(RHP_PFPattern)
myCont[R_HIP_PITCH].fSetPatternRG(PatternOsc) # PatternPL1 PatternOsc3 PatternAosc PatternQU

myCont[R_KNEE_PITCH].fSetPatternPF(RKP_PFPattern)
myCont[R_KNEE_PITCH].fSetPatternRG(PatternOsc) # PatternPL1 PatternOsc3 PatternAosc PatternQU

myCont[R_ANKLE_PITCH].fSetPatternPF(RAP_PFPattern)
myCont[R_ANKLE_PITCH].fSetPatternRG(PatternOsc) # PatternPL1 PatternOsc3 PatternAosc PatternQU

# Left
myCont[L_HIP_PITCH].fSetPatternPF(LHP_PFPattern)
myCont[L_HIP_PITCH].fSetPatternRG(PatternOsc) # PatternPL1 PatternOsc3 PatternAosc PatternQU

myCont[L_KNEE_PITCH].fSetPatternPF(LKP_PFPattern)
myCont[L_KNEE_PITCH].fSetPatternRG(PatternOsc) # PatternPL1 PatternOsc3 PatternAosc PatternQU

myCont[L_ANKLE_PITCH].fSetPatternPF(LAP_PFPattern)
myCont[L_ANKLE_PITCH].fSetPatternRG(PatternOsc) # PatternPL1 PatternOsc3 PatternAosc PatternQU

#myCont[L_SHOULDER_PITCH].fPrint()

"""
0_HEAD_YAW, 1_HEAD_PITCH, 2_L_SHOULDER_PITCH,L_SHOULDER_ROLL,L_ELBOW_YAW,L_ELBOW_ROLL,
L_WRIST_YAW,L_HAND,L_HIP_YAW_PITCH,L_HIP_ROLL,L_HIP_PITCH,11_L_KNEE_PITCH,L_ANKLE_PITCH,
L_ANKLE_ROLL,R_HIP_YAW_PITCH,R_HIP_ROLL,R_HIP_PITCH,17_R_KNEE_PITCH,R_ANKLE_PITCH,
R_ANKLE_ROLL,R_SHOULDER_PITCH,R_SHOULDER_ROLL,R_ELBOW_YAW,R_ELBOW_ROLL,
R_WRIST_YAW,25_R_HAND
"""

CurPos = movObj.getAngles('Body',True)  # ALMotionProxy::getAngles(const AL::ALValue& names, const bool& useSensors)
                                        #Parameters:	
                                        #   names – Names the joints, chains, “Body”, “JointActuators”, “Joints” or “Actuators”.
                                        #   useSensors – If true, sensor angles will be returned
                                        #Returns:	Joint angles in radians.

for i in range(0, len(myCont)):
    myCont[i].fUpdateInitPos(CurPos[i])
    # these init values are for the CPGs that are not used
# how does this init value come from?
# according to experience, it is the initial value
myCont[L_HIP_ROLL].fUpdateInitPos(2*math.pi/180.0)
myCont[R_HIP_ROLL].fUpdateInitPos(-2*math.pi/180.0)
myCont[L_ANKLE_ROLL].fUpdateInitPos(2*math.pi/180.0)
myCont[R_ANKLE_ROLL].fUpdateInitPos(-2*math.pi/180.0)

myCont[L_HIP_PITCH].fUpdateInitPos(-0.4) # -0.4
myCont[R_HIP_PITCH].fUpdateInitPos(myCont[L_HIP_PITCH].joint.init_motor_pos) 
#already updated in L294, here is just for understand that the init position just move its Left leg 

myCont[L_KNEE_PITCH].fUpdateInitPos(0.7) # 0.7
myCont[R_KNEE_PITCH].fUpdateInitPos(myCont[L_KNEE_PITCH].joint.init_motor_pos)

myCont[L_ANKLE_PITCH].fUpdateInitPos(-0.35) # -0.35
myCont[R_ANKLE_PITCH].fUpdateInitPos(myCont[L_ANKLE_PITCH].joint.init_motor_pos)
"""
 9. LHipYawPitch* 	Left hip joint twist (Y-Z 45deg)          -65.62 to 42.44 	-1.145303 to 0.740810
10. LHipRoll      Left hip joint right and left (X)           -21.74 to 45.29 	-0.379472 to 0.790477
11. LHipPitch 	Left hip joint front and back (Y)           -88.00 to 27.73 	-1.535889 to 0.484090
12. LKneePitch 	Left knee joint (Y)                         -5.29 to 121.04 	-0.092346 to 2.112528
13. LAnklePitch 	Left ankle joint front and back (Y)         -68.15 to 52.86 	-1.189516 to 0.922747
14. LAnkleRoll 	Left ankle joint right and left (X)         -22.79 to 44.06 	-0.397880 to 0.769001
"""
# the weight is 0 because we don't want to use the feedback from SN(sensor neuron)
myCont[L_SHOULDER_PITCH].W_E_SN2MN=-0.0
myCont[L_SHOULDER_PITCH].W_F_SN2MN=-0.0

myCont[L_SHOULDER_ROLL].W_E_SN2MN=-0.0
myCont[L_SHOULDER_ROLL].W_F_SN2MN=-0.0
    
#############################
for i in range(0, len(myCont)):
    myCont[i].fUpdateLocomotionNetwork(myT, CurPos[i])
#############################
time1 = time.time()

def filter_data(data_to_filter):
    b_f = lpf.Butter_Filter()
    data_after_filter = b_f.butter_lowpass_filter(data_to_filter)
    return data_after_filter
    
def update_FRS_time_list(FSR_value):
    # record the time_axis
    left_foot_time_list.append(time.time())
    # record the FSR_axis
    FSR_list_left_foot.append(FSR_value)

def calc_frequency(timestamp_list):
    if(len(timestamp_list) < 3):
        timestamp_list = [0,1]
    print('timestamp_list', timestamp_list)
    print('timestamp_list[-1]', timestamp_list[-1])
    print('timestamp_list[-2]', timestamp_list[-2])
    delta_t = timestamp_list[-1] - timestamp_list[-2]
    f = 1/delta_t
    return f
     
def mark_hit_timestamp(temp_list):
    cur_time = 0
    cur_sensor_val = 0
    pre_sensor_val = 0
    
    for i in range(1, len(temp_list) - 1):
        print('temp_list[i][0]', temp_list[i][0])
        pre_sensor_val = temp_list[i-1][0]
        cur_sensor_val = temp_list[i][0]
        next_sensor_val = temp_list[i+1][0]
        cur_time = temp_list[i][1]
        if(next_sensor_val < cur_sensor_val) and (cur_sensor_val > pre_sensor_val):
            print('current time',cur_time)
            return cur_time

def calc_mean_sensor_value(parameter_list, parameter_num):
    sum_sensor_value = 0
    for sensor_dir in parameter_list:
        sum_sensor_value = sum_sensor_value + memProxy.getData(sensor_dir)
    mean_value = sum_sensor_value/parameter_num
    return mean_value

while True:
    mean_value_left_foot  = calc_mean_sensor_value(total_left_FSR_dir_list, 1)
    
    # update the FSR&time list for left foot
    update_FRS_time_list(mean_value_left_foot)

    MiddleTactileON = memProxy.getData('Device/SubDeviceList/Head/Touch/Middle/Sensor/Value')
    if (MiddleTactileON):
        movObj.setAngles('LHand', 0, 0.5)
        time.sleep(1)
        break
############################################################################################################################
################################# main start from here #####################################################################
############################################################################################################################
MAT_Iinj = []
num_loop_times = 1
start_time = time.time()
for I in range(0, int(myT.N_Loop/25)):
    t = I*myT.T
    print("****** I = ", I)
    #?? as I understand the InjCurrent is just a constant factor which value is 0???
    # #################
    # ExtInjCurr
    # #################
    if t >= myT.T1 and t < myT.T2:
        ExtInjCurr = 1
    #elif t >= myT.T3 and t < myT.T4:
    #    ExtInjCurr = 4.5
    #elif t >= myT.T5 and t < myT.T6:    
    #    ExtInjCurr = -4.5
    else: 
        ExtInjCurr = 0
        
    if t >= myT.T3 and t < myT.T4:
        ExtInjCurr2 = 1
    #elif t >= myT.T3 and t < myT.T4:
    #    ExtInjCurr = 4.5
    #elif t >= myT.T5 and t < myT.T6:    
    #    ExtInjCurr = -4.5
    else: 
        ExtInjCurr2 = 0

    MAT_Iinj.append(ExtInjCurr) 

    #L_SHOULDER_PITCH , L_SHOULDER_ROLL,
    for ii in [ L_HIP_ROLL, L_ANKLE_ROLL]:
        myCont[ii].RG.F.InjCurrent_value = +1*ExtInjCurr* myCont[ii].RG.F.InjCurrent_MultiplicationFactor
        myCont[ii].RG.E.InjCurrent_value =  -1*ExtInjCurr* myCont[ii].RG.E.InjCurrent_MultiplicationFactor
    #R_SHOULDER_PITCH, R_SHOULDER_ROLL,
    for ii in [ R_HIP_ROLL, R_ANKLE_ROLL]:
        myCont[ii].RG.F.InjCurrent_value = -1*ExtInjCurr* myCont[ii].RG.F.InjCurrent_MultiplicationFactor
        myCont[ii].RG.E.InjCurrent_value = +1*ExtInjCurr* myCont[ii].RG.E.InjCurrent_MultiplicationFactor


    for ii in [R_HIP_PITCH, R_KNEE_PITCH, L_ANKLE_PITCH]:
        myCont[ii].RG.F.InjCurrent_value = +1*ExtInjCurr2* myCont[ii].RG.F.InjCurrent_MultiplicationFactor
        myCont[ii].RG.E.InjCurrent_value =  -1*ExtInjCurr2* myCont[ii].RG.E.InjCurrent_MultiplicationFactor

    for ii in [L_HIP_PITCH, L_KNEE_PITCH, R_ANKLE_PITCH]:
        myCont[ii].RG.F.InjCurrent_value = -1*ExtInjCurr2* myCont[ii].RG.F.InjCurrent_MultiplicationFactor
        myCont[ii].RG.E.InjCurrent_value = +1*ExtInjCurr2* myCont[ii].RG.E.InjCurrent_MultiplicationFactor
    
    for i in [L_HIP_ROLL, L_ANKLE_ROLL,R_HIP_ROLL, R_ANKLE_ROLL,R_HIP_PITCH, R_KNEE_PITCH, L_ANKLE_PITCH,L_HIP_PITCH, L_KNEE_PITCH, R_ANKLE_PITCH]:
        myCont[i].fUpdateLocomotionNetwork(myT,CurPos[i])

        
    for i in range(0, len(myCont)):
        MotorCommand[i] = myCont[i].joint.joint_motor_signal
    
    movObj.setAngles('Body', MotorCommand , fractionMaxSpeed)

    #movObj.angleInterpolation('Body',MotorCommand , 0.03, True)

############################################################################################################################
###################################### My contribution starts ##############################################################
############################################################################################################################
    print('Loop: ', num_loop_times)
    num_loop_times = num_loop_times + 1
    #get mean FSR value of left foot
    mean_value_left_foot  = calc_mean_sensor_value(total_left_FSR_dir_list, 1)
    mean_value_right_foot = calc_mean_sensor_value(total_right_FSR_dir_list, 1)
    
    # update the FSR&time list for left foot
    update_FRS_time_list(mean_value_left_foot)

    # filter it and store it in data_after_filt
    filter_data(FSR_list_left_foot)
   
    # combine 2 lists(left_foot_time_list, FSR_list_left_foot)
    # 因为是一一对应的关系 所以可以存到一个列表里temp_FSR_time_list
    for i in range(len(left_foot_time_list)):
        temp_FSR_time_list.append([data_after_filt[i], left_foot_time_list[i]])
    
    if(mark_hit_timestamp(temp_FSR_time_list) != None):
        left_foot_hit_timestamp_list.append(mark_hit_timestamp(temp_FSR_time_list))
    print('left_foot_hit_timestamp_list', left_foot_hit_timestamp_list)
    print(calc_frequency(left_foot_hit_timestamp_list))

    angle_left_knee_pitch = CurPos[11] # 11 means L_Knee_Pitch, refer to CurPos,
    angle_right_knee_pitch = CurPos[17] # Note! Starting from 0
    # Record delta_time of Hip joints
    
    if(angle_left_knee_pitch > 0.7):
        t = time.time()
        left_knee_time_list.append(t)
    if(angle_right_knee_pitch > 0.7):
        t = time.time()
        right_knee_time_list.append(t)

    #!! frequency of joints, f1
    #print('Right Knee Pitch Sensor position value:', memProxy.getData('Device/SubDeviceList/RKneePitch/Position/Sensor/Value'))
    #!! frequency of foot, f2
    #print('Right Foot left front Sensor value:', memProxy.getData('Device/SubDeviceList/RFoot/FSR/FrontLeft/Sensor/Value'))

############################################################################################################################
###################################### My contribution ends ################################################################
############################################################################################################################    
    
    # Read motor angel from robot joints 
    CurPos = movObj.getAngles('Body',True)
    
    All_Command.append(MotorCommand[:]) 
    All_Sensor.append(CurPos)
    listValRead = memProxy.getListData(ReadlistData)

    # "RearTactileON" stop 
    # 摸摸头
    if (listValRead[1]):
        print('Walking has been stoped by toutching the rear tactile head sensor.\n')
        break
    
    """    
    # "MiddleTactileON" CLOSE THE HAND TO PICK THE BALL 
    MiddleTactileON = 0 
    MiddleTactileON = memProxy.getData('Device/SubDeviceList/Head/Touch/Rear/Sensor/Value')
    if (MiddleTactileON):
        print 'Walking has been stoped by toutching the rear tactile head sensor.\n'
        break
    """
    ###################################### 
    # Msurement of hand toutch sensors (Tactile Hands)
    # http://doc.aldebaran.com/2-1/family/robots/contact-sensors_robot.html#robot-contact-hand 
    """
    LHandBackSensor = memProxy.getData('Device/SubDeviceList/LHand/Touch/Back/Sensor/Value')
    LHandLeftSensor = memProxy.getData('Device/SubDeviceList/LHand/Touch/Left/Sensor/Value')
    LHandRightSensor = memProxy.getData('Device/SubDeviceList/LHand/Touch/Right/Sensor/Value')

    RHandBackSensor = memProxy.getData('Device/SubDeviceList/RHand/Touch/Back/Sensor/Value')
    RHandLeftSensor = memProxy.getData('Device/SubDeviceList/RHand/Touch/Left/Sensor/Value')
    RHandRightSensor = memProxy.getData('Device/SubDeviceList/RHand/Touch/Right/Sensor/Value')
    """

    LHandBackSensor = listValRead[3]
    LHandLeftSensor = listValRead[4]
    LHandRightSensor = listValRead[5]

    RHandBackSensor = listValRead[6]
    RHandLeftSensor = listValRead[7]
    RHandRightSensor = listValRead[8]
    
    #print 'LHandBack:', LHandBackSensor,' LHandLeft:',LHandLeftSensor,' LHandRight:',LHandRightSensor,' RHandBack:',RHandBackSensor,' RHandLeft:',RHandLeftSensor,' RHandRight:',RHandRightSensor
    """ 
    if (LHandBackSensor==1 or LHandLeftSensor==1  or LHandRightSensor==1 ):
        movObj.setStiffnesses('LArm',0.0)
    else:
        movObj.setStiffnesses('LArm',0.1)
    """    
    movObj.setStiffnesses('LArm', 0.5 * int( not ((LHandBackSensor == 1) or (LHandLeftSensor == 1)  or (LHandRightSensor == 1))))
    
    """    
    if (RHandBackSensor==1  or RHandLeftSensor==1  or RHandRightSensor==1 ):
        movObj.setStiffnesses('RArm',0.0)
    else:
        movObj.setStiffnesses('RArm',0.1)
    """
    movObj.setStiffnesses('RArm', 0.5 * int( not ((RHandBackSensor == 1) or (RHandLeftSensor == 1)  or (RHandRightSensor == 1))))
    
    ######################################
    """
    if (LHandBackSensor): 
        ledObj.on("LeftEarsMidle") 
    else:
        ledObj.off("LeftEarsMidle")

    if (LHandLeftSensor): 
        ledObj.on("LeftEarsFront") 
    else:
        ledObj.off("LeftEarsFront")
    
    if (LHandRightSensor): 
        ledObj.on("LeftEarsBack") 
    else:
        ledObj.off("LeftEarsBack")


    if (RHandBackSensor): 
        ledObj.on("RightEarsMidle") 
    else:
        ledObj.off("RightEarsMidle")

    if (RHandLeftSensor): 
        ledObj.on("RightEarsBack") 
    else:
        ledObj.off("RightEarsBack")
    
    if (RHandRightSensor): 
        ledObj.on("RightEarsFront") 
    else:
        ledObj.off("RightEarsFront")
    """

    #ledObj.on("LeftEarsFront")LeftEarsMidle LeftEarsBack
    ######################################
    
    # Get The Left Foot Force Sensor Values
    """   
    LFsrFL = memProxy.getData("Device/SubDeviceList/LFoot/FSR/FrontLeft/Sensor/Value")
    LFsrFR = memProxy.getData("Device/SubDeviceList/LFoot/FSR/FrontRight/Sensor/Value")
    LFsrBL = memProxy.getData("Device/SubDeviceList/LFoot/FSR/RearLeft/Sensor/Value")
    LFsrBR = memProxy.getData("Device/SubDeviceList/LFoot/FSR/RearRight/Sensor/Value")
    """
    LFsrFL = listValRead[9]
    LFsrFR = listValRead[10]
    LFsrBL = listValRead[11]
    LFsrBR = listValRead[12]
    

    #print( "Left FSR [Kg] : %.2f %.2f %.2f %.2f" %  (LFsrFL, LFsrFR, LFsrBL, LFsrBR) )
    #print( "Left FSR [Kg] : %.2f " %  (LFsrFL+ LFsrFR+ LFsrBL+ LFsrBR) )
    
    # Get The Right Foot Force Sensor Values
    """
    RFsrFL = memProxy.getData("Device/SubDeviceList/RFoot/FSR/FrontLeft/Sensor/Value")
    RFsrFR = memProxy.getData("Device/SubDeviceList/RFoot/FSR/FrontRight/Sensor/Value")
    RFsrBL = memProxy.getData("Device/SubDeviceList/RFoot/FSR/RearLeft/Sensor/Value")
    RFsrBR = memProxy.getData("Device/SubDeviceList/RFoot/FSR/RearRight/Sensor/Value")
    """

    RFsrFL = listValRead[13]
    RFsrFR = listValRead[14]
    RFsrBL = listValRead[15]
    RFsrBR = listValRead[16]


    #print( "Right FSR [Kg] : %.2f %.2f %.2f %.2f" %  (RFsrFL, RFsrFR, RFsrBL, RFsrBR) )
    #print( "Right FSR [Kg] : %.2f " %  (RFsrFL+ RFsrFR+ RFsrBL+ RFsrBR) )
    
    #print( "Left FSR [Kg] : %.2f " %  (LFsrFL+ LFsrFR+ LFsrBL+ LFsrBR),  "Right FSR [Kg] : %.2f " %  (RFsrFL+ RFsrFR+ RFsrBL+ RFsrBR) )
    
    
    
    time2 = time.time()
    while (time2 - time1)<0.04 :
        time2 = time.time()
    #print('time diff: ', time2 - time1)
    #time_diff  = time2 - time1
    time1 = time2 

#print 'time diff: ', time_diff
##############################

postObj.goToPosture("Crouch",0.8)
time.sleep(2)
movObj.setStiffnesses('Body',0.0)
############################################################################################################################
###################################### My 2nd contribution starts ##########################################################
############################################################################################################################
#!! problems here!
# !! first plot the image of (FSR, time)

def draw_plot(x_axis_list, y_axis_list):
    if(len(x_axis_list) != len(y_axis_list)):
        print('Error: x_axis_len and y_axis_len does not match')
    x = x_axis_list
    y = y_axis_list
    plt.plot(x, y, 'o') # just draw the dots
    plt.plot(x, y)
    plt.show()
def calc_delta_t(parameter_list):
    delta_t_list = []
    for i in range(len(parameter_list) - 1):
        interval_t = parameter_list[i+1] - parameter_list[i]
        delta_t_list.append(interval_t)
    #print(delta_t_list)
    delta_t = np.mean(delta_t_list)
    return delta_t

delta_t_left_foot = calc_delta_t(left_foot_time_list)
delta_t_right_foot = calc_delta_t(right_foot_time_list)
delta_t_left_knee_pitch = calc_delta_t(left_knee_time_list)
delta_t_right_knee_pitch = calc_delta_t(right_knee_time_list)


draw_plot(left_foot_time_list, FSR_list_left_foot)
# draw the 2nd figure which is the filtered figure
b_f = lpf.Butter_Filter()
b_f._data_ = FSR_list_left_foot
data_after_filt = b_f.butter_lowpass_filter()
draw_plot(left_foot_time_list, data_after_filt)
runtime_duration = time.time()- start_time
print('walking time: ', runtime_duration)
print('left_foot_frequency: ', calc_freuency_foot(data_after_filt)/runtime_duration)
'''
# the calculate the mean time and mean frequency
print('left_foot: ', len(delta_t_left_foot))
print('right_foot: ', len(delta_t_right_foot))
print('left_knee', len(delta_t_left_knee_pitch))
print('right_knee', len(delta_t_right_knee_pitch))
'''
#f_left_foot = 1/delta_t_left_foot
############################################################################################################################
###################################### My 2nd contribution ends ############################################################
############################################################################################################################    



# Save motion data into files
##############################
f = open('AllCommands', 'w')
s = str(All_Command)
f.write(s)
f.close()

f = open('AllAngels', 'w')
s = str(All_Sensor)
f.write(s)
f.close()
# store the data from the left foot FSR
f = open('RareData', 'w')
s = str(FSR_list_left_foot)
f.write(s)
f.close()
