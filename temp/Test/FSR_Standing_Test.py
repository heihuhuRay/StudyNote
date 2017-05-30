# -*- encoding: UTF-8 -*- 

'''Walk: Small example to make Nao walk'''
import sys
import motion
import time
from naoqi import ALProxy
from naoqi import ALModule


def StiffnessOn(proxy):
    # We use the "Body" name to signify the collection of all joints
    pNames = "Body"
    pStiffnessLists = 1.0
    pTimeLists = 1.0
    proxy.stiffnessInterpolation(pNames, pStiffnessLists, pTimeLists)


def main(robotIP):
    # Init proxies.
    # Connect to the module ALMemoryProxy
    memProxy = ALProxy("ALMemory", robotIP, 9559)
    try:
        motionProxy = ALProxy("ALMotion", robotIP, 9559)
    except Exception, e:
        print "Could not create proxy to ALMotion"
        print "Error was: ", e

    try:
        postureProxy = ALProxy("ALRobotPosture", robotIP, 9559)
    except Exception, e:
        print "Could not create proxy to ALRobotPosture"
        print "Error was: ", e

    # Set NAO in Stiffness On
    StiffnessOn(motionProxy)

    # Send NAO to Pose Init
    postureProxy.goToPosture("StandInit", 0.5)

    motionProxy.setStiffnesses('Body',0.0)
    while True:
    	#!! frequency of joints, f1
	    #print('Right Knee Pitch Sensor position value:', memProxy.getData('Device/SubDeviceList/RKneePitch/Position/Sensor/Value'))
	    #!! frequency of foot, f2
	    #print('Right Foot left front Sensor value:', memProxy.getData('Device/SubDeviceList/RFoot/FSR/FrontLeft/Sensor/Value'))
	    print('L_Hip_Pitch: ', motionProxy.getAngles('Body',True)[11])
	    time.sleep(0.2)
	    # "RearTactileON" stop 
	    # 摸摸头
	    is_head_touched = memProxy.getData("Device/SubDeviceList/Head/Touch/Rear/Sensor/Value")
	    if (is_head_touched):
	        print('Walking has been stoped by toutching the rear tactile head sensor.\n')
	        break
	
    postureProxy.goToPosture("Crouch",0.8)
    motionProxy.setStiffnesses('Body',0.0)

    




if __name__ == "__main__":
    robotIp = "169.254.28.144"

    if len(sys.argv) <= 1:
        print "Usage python motion_walk.py robotIP (optional default: 127.0.0.1)"
    else:
        robotIp = sys.argv[1]

    main(robotIp)