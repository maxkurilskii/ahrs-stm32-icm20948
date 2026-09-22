#!/usr/bin/env python3

import sys
import argparse
import rospy
import time
import serial
from pathlib import Path
import struct 
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Vector3

# insert project path in SYS.PATH
sys.path.insert(0, str(Path(__file__).resolve().parents[1])) 
from utils import UartCommunicator, MessageCode, UartReadTimeout 


def convert_to_imu_msg(q: Quaternion, timestamp_us: int) -> Imu:
    imu_msg = Imu()
    imu_msg.header.frame_id = "base_link"
    secs = timestamp_us // 10**6
    nanosecs = timestamp_us % 10**6 * 1000
    imu_msg.header.stamp = rospy.Time(secs, nanosecs)

    imu_msg.orientation = q
    imu_msg.orientation_covariance = [0, 0, 0, 0, 0, 0, 0, 0, 0]

    imu_msg.linear_acceleration = Vector3(0, 0, 0)
    imu_msg.linear_acceleration_covariance = [0.000003, 0, 0, 
                                   0, 0.000002, 0,
                                   0, 0, 0.000004]
    imu_msg.angular_velocity = Vector3(0, 0, 0)
    imu_msg.angular_velocity_covariance = [0.007881, 0, 0,
                                           0, 0.007751, 0,
                                           0, 0, 0.006328]

    return imu_msg


def main(args):
    try:
        imu_publisher = rospy.Publisher("/IMU_ICM20948", Imu, queue_size=1)
        host_com = UartCommunicator(port = '/dev/ttyACM0', timeout = 0.5)
        if args.get('first_msg', 0) == True:
            while not rospy.is_shutdown():
                reference_time_us = time.time_ns() // 10**3 # int object in the end
                pkg = reference_time_us.to_bytes(8, byteorder='little') #cast to uint64 in little-endian
                host_com.uart_write_package(msg_code = MessageCode.START_CMD.value, data = pkg) # 23 10 08 60 da 50 2f 62 5b 06 00 29 84
                try:
                    rx_msg = host_com.uart_read_package()
                    # Expected ACK: # [start_byte + ERROR_STATUS + data_len + error_code + CRC]
                    rospy.loginfo(f"Received_msg (hex bytes): {rx_msg.hex(' ')}" ) 
                    if (len(rx_msg) != 6 
                        or rx_msg[0] != UartCommunicator.START_BYTE 
                        or rx_msg[1] != MessageCode.ERROR_STATUS.value 
                        or rx_msg[2] != 1 ):
                            rospy.logwarn("Invalid ACK message, retrying START_CMD")
                            rospy.sleep(1)
                            continue
                    err_code = rx_msg[3]
                    if err_code == 0:  
                        rospy.loginfo("START_CMD acknowledged")
                        break
                    else:
                        rospy.logerr(f"STM32 returned error code: 0x{err_code:02X}, sending again ...")
                except UartReadTimeout:
                    raise
                
        while not rospy.is_shutdown():
            try:
                msg = host_com.uart_read_package() #blocking !!!
                w, x, y, z, timestamp, _ = struct.unpack_from('<4f1Q1H', msg, offset = 3)
                q = Quaternion(x, y, z, w)
                ros_imu_msg = convert_to_imu_msg(q, timestamp_us=timestamp)
                imu_publisher.publish(ros_imu_msg)

            except UartReadTimeout as e:
                raise
                
    except UartReadTimeout as e:
        rospy.logwarn(f'Uart protocol: {e}')
    except serial.SerialException as e:
        rospy.logwarn(f"{e}.")
    except KeyboardInterrupt:
        rospy.logwarn('End event was raised')
    except struct.error as e:
        rospy.logerr(f'Struct error: {e}')
    

if __name__ == "__main__":
    parser = argparse.ArgumentParser("Communciation options parser")
    parser.add_argument('-fm', '--first-msg', action='store_true', default=False,
                        help="Whether to send start command message to activate meas stream on STM32")
    args = parser.parse_args(rospy.myargv()[1:])

    rospy.init_node("STM32_communication_node")
    arg_dir = vars(args)
    rospy.loginfo(f"Launch args: {arg_dir}")
    rospy.sleep(3.0)
    main(arg_dir)
    # rospy.spin()

 