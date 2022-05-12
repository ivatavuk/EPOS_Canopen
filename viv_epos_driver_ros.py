#!/usr/bin/python3
import sys
import os 

from epos import Epos
import logging
import canopen

import rospy
from std_msgs.msg import Float64MultiArray, Int16MultiArray, Header
from sensor_msgs.msg import JointState

velocity_commands_ = [0.0, 0.0, 0.0, 0.0]
reduction_ratio = 81.0
eds_path = os.path.dirname(__file__)

def checkObjectDictionary(epos_motor, object_dictionary):
    # test record a single record
    error_log = epos_motor.node.sdo['Error History']
    # Iterate over arrays or record
    for error in error_log.values():
        print("Error 0x%X was found in the log" % error.raw)

    print('----------------------------------------------------------', flush=True)

def setupLogging():
    # set up logging to file - see previous section for more details
    logging.basicConfig(level=logging.INFO,
                        format='[%(asctime)s.%(msecs)03d] [%(name)-20s]: %(levelname)-8s %(message)s',
                        datefmt='%d-%m-%Y %H:%M:%S',
                        filename='epos.log',
                        filemode='w')
    # define a Handler which writes INFO messages or higher
    console = logging.StreamHandler()
    console.setLevel(logging.INFO)
    # set a format which is simpler for console use
    formatter = logging.Formatter('%(name)-20s: %(levelname)-8s %(message)s')
    # tell the handler to use this format
    console.setFormatter(formatter)
    # add the handler to the root logger
    logging.getLogger('').addHandler(console)
    return logging

def initializeEpos( epos_motor, logging, nodeID, rate = 125000,  object_dictionary = eds_path + '/viv_maxon-70_10.eds', channel = 'can0', bus = 'socketcan'):
    if not ( epos_motor.begin(nodeID, object_dictionary=object_dictionary) ):
        logging.info('Failed to begin connection with EPOS device')
        logging.info('Exiting now')
        return

    checkObjectDictionary(epos_motor, object_dictionary)

    # use simple hex values
    # try to read status word
    statusword = epos_motor.read_object(0x6041, 0)
    if not statusword:
        print("[EPOS] Error trying to read EPOS statusword\n")
        return
    else:
        print('----------------------------------------------------------', flush=True)
        print("The statusword is \n Hex={0:#06X} Bin={0:#018b}".format(
            int.from_bytes(statusword, 'little')))

    # test print_statusword and state
    print('----------------------------------------------------------', flush=True)
    print('Testing print of StatusWord and State and ControlWord')
    print('----------------------------------------------------------', flush=True)
    epos_motor.print_state()
    print('----------------------------------------------------------', flush=True)
    epos_motor.print_statusword()
    print('----------------------------------------------------------', flush=True)
    # try to read controlword using hex codes
    controlword = epos_motor.read_object(0x6040, 0)
    if not controlword:
        logging.info("[EPOS] Error trying to read EPOS controlword\n")
    else:
        print("The controlword is \n Hex={0:#06X} Bin={0:#018b}".format(
            int.from_bytes(controlword, 'little')))
        print('----------------------------------------------------------', flush=True)
        # perform a reset, by using controlword
        controlword = int.from_bytes(controlword, 'little')
        controlword = (controlword | (1 << 7))
        print('----------------------------------------------------------', flush=True)
        print("The new controlword is \n Hex={0:#06X} Bin={0:#018b}".format(
            controlword))
        print('----------------------------------------------------------', flush=True)
        # sending new controlword
        controlword = controlword.to_bytes(2, 'little')
        epos_motor.write_object(0x6040, 0, controlword)
        # check led status to see if it is green and blinking
    #epos_motor.print_position_control_parameters()
    #epos_motor.print_motor_config()
    #epos_motor.print_sensor_config()
    return epos_motor

def motor_velocities_callback(msg):
    global velocity_commands_
    motor_vel_list = list(msg.data)
    if (not len(motor_vel_list) == 4):
        print ("Motor velocity list must be 4 dimensional!!")
    else:
        velocity_commands_ = motor_vel_list.copy()

def set_velocity_mode(epos_motor):
    if not epos_motor.change_state("shutdown"): print ("shutdown failed")
    if not epos_motor.change_state("switch on"): print ("switch on failed")
    if not epos_motor.change_state('enable operation'): print ("enable operation failed")
    if not epos_motor.set_op_mode(-2): print ("set operation mode vel failed")

def set_velocity_modes(epos_motors):
    for epos_motor in epos_motors:
        set_velocity_mode(epos_motor)

def initialize_epos_motors(epos_motors):
    logging = setupLogging()
    nodeID_counter = 1
    for epos_motor in epos_motors:
        epos_motor = initializeEpos(epos_motor, logging, nodeID=nodeID_counter)
        nodeID_counter = nodeID_counter + 1
        if epos_motor == None: 
            print("Epos initialization failed")
            return False # return error
    return True #return success

def disconnect_epos_motors(epos_motors):
    for epos_motor in epos_motors:
        epos_motor.disconnect()

def shutdown_epos_motors(epos_motors):
    for epos_motor in epos_motors:
        epos_motor.change_state("shutdown")

def set_motor_velocity_commands(epos_motors):
    global velocity_commands_
    if not len(epos_motors) == len(velocity_commands_):
        return
    for index in range(len(epos_motors)):
        max_int_vel_command = 10000
        int_vel_command = round(velocity_commands_[index] * reduction_ratio) 

        if int_vel_command > max_int_vel_command:
            int_vel_command = max_int_vel_command
        if int_vel_command < -max_int_vel_command:
            int_vel_command = -max_int_vel_command

        epos_motors[index].set_velocity_mode_setting(int_vel_command)

def get_joint_states(epos_motors):
    return_msg = JointState()
    return_msg.header = Header()
    return_msg.header.stamp = rospy.Time.now()
    return_msg.name = ["epos_mot_1", "epos_mot_2", "epos_mot_3", "epos_mot_4"]
    return_msg.position = [] 
    return_msg.velocity = [] 
    return_msg.effort = [] 
    for epos_motor in epos_motors:
        pos, _ = epos_motor.read_position_value()
        vel, _ = epos_motor.read_velocity_value_averaged()
        return_msg.position.append(pos)
        return_msg.velocity.append( int.from_bytes(vel, 'little', signed=True)/ reduction_ratio)  
        return_msg.effort.append(0.0)

    return return_msg

def get_current(epos_motors):
    return_msg = Int16MultiArray()
    return_msg.data = []
    #pom_index = 1
    for epos_motor in epos_motors:
        curr, _ = epos_motor.read_current_value() #raw
        #print("Motor ", pom_index , ": ", curr)
        if isinstance(curr, int):
            return_msg.data.append(curr) #zasad se publisha bas int koji dobijemo s cana, u mA
        else:
            return_msg.data.append(0)
        #pom_index = pom_index + 1
    
    return return_msg

def get_current_averaged(epos_motors):
    return_msg = Int16MultiArray()
    return_msg.data = []
    #pom_index = 1
    for epos_motor in epos_motors:
        curr, _ = epos_motor.read_current_value_averaged() #filtrirano s 50Hz
        #print("Motor ", pom_index , ": ", curr)
        if isinstance(curr, int):
            return_msg.data.append(curr) #zasad se publisha bas int koji dobijemo s cana, u mA
        else:
            return_msg.data.append(0)
        #pom_index = pom_index + 1
    
    return return_msg
    

def main():
    import argparse
    
    rospy.init_node("viv_epos_driver")
    node_rate = rospy.Rate(30)
    motor_velocities_sub = rospy.Subscriber('viv_epos_driver/motor_velocities', Float64MultiArray, motor_velocities_callback)
    joint_state_pub = rospy.Publisher('viv_epos_driver/joint_state', JointState, queue_size=1)
    current_averaged_pub = rospy.Publisher('viv_epos_driver/motor_current_averaged', Int16MultiArray, queue_size=1)
    current_pub = rospy.Publisher('viv_epos_driver/motor_current', Int16MultiArray, queue_size=1)

    if sys.version_info < (3, 0):
        print("Please use python version 3")
        return

    # instantiate object
    epos_motor_1 = Epos()
    epos_motor_2 = Epos()
    epos_motor_3 = Epos()
    epos_motor_4 = Epos()

    logging = setupLogging()
    #print("Pocinjemo")
    epos_motors = [epos_motor_1, epos_motor_2, epos_motor_3, epos_motor_4]
    #epos_motors = [epos_motor_2]
    if not initialize_epos_motors(epos_motors):
        return

    set_velocity_modes(epos_motors)

    print("----------------------------Listening to velocity commands----------------------------")
    
    while not rospy.is_shutdown():
        set_motor_velocity_commands(epos_motors)
        joint_state_pub.publish(get_joint_states(epos_motors))
        current_averaged_pub.publish(get_current_averaged(epos_motors))
        current_pub.publish(get_current(epos_motors))
        node_rate.sleep()

    print("----------------------------Shutting down----------------------------")
    
    shutdown_epos_motors(epos_motors)
    return

if __name__ == '__main__':
    main()
