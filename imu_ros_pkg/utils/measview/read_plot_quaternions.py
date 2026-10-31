import argparse
import serial
import time
import struct
import pandas as pd
from pathlib import Path
import matplotlib.pyplot as plt
from typing import Optional

# from stm32_uart_comm import UartCom
from utils.uartcom.uart_worker import UartCommunicator, UartReadTimeout
from utils.uartcom.protocol_codes import PROTOTOL_START_BYTE, MessageCode
from utils.measlog.imu_csv_logger import ImuLogger
from utils.uartcom.base_dataclasses import QuaternionMsgResponse


plt.rcParams["font.size"] = 12
plt.rcParams["axes.grid"] = True
# plt.rcParams["figure.autolayout"] = True


def plot_quaternion_meas(file):
    
    df = pd.read_csv(file,
                    header=0,
                    names=["Time_us", "W", "X", "Y", "Z"])  
    df['Time_s'] = df['Time_us'] / 10**6
    fig, axes = plt.subplots(4,1, sharex=True, figsize=(10,6))
    tight_par = False
    # W
    df.plot(ax=axes[0], x='Time_s', y='W', color='blue', linewidth=2, legend=False)
    axes[0].set_ylabel('W')
    axes[0].grid(True, alpha=0.3)
    axes[0].autoscale(enable=None, axis="x", tight=tight_par)
    # X
    df.plot(ax=axes[1], x='Time_s', y='X', color='green', linewidth=2, legend=False)
    axes[1].set_ylabel('X')
    axes[1].grid(True, alpha=0.3)
    axes[1].autoscale(enable=None, axis="x", tight=tight_par)
    # Y
    df.plot(ax=axes[2], x='Time_s', y='Y', color='red', linewidth=2, legend=False)
    axes[2].set_ylabel('Y')
    axes[2].grid(True, alpha=0.3)
    axes[2].autoscale(enable=None, axis="x", tight=tight_par)
    # Z
    df.plot(ax=axes[3], x='Time_s', y='Z', color='red', linewidth=2, legend=False)
    axes[3].set_ylabel('Z')
    axes[3].set_xlabel('Time (s)')
    axes[3].grid(True, alpha=0.3)
    axes[3].autoscale(enable=None, axis="x", tight=tight_par)
    fig.suptitle('Orientation in quaternions after IMU meas fusion', fontsize=14, fontweight='bold')
    fig.align_ylabels()
    plt.tight_layout(pad=0.4, w_pad=0.5, h_pad=1.0)
    plt.show()
    
    
def main(args):
    if args.get('cmd', 0) == 'meas':
        print('Getting new measurements, ignoring specified file')
        print('To stop measuremnts collection press Ctrl + C')
        time_str = 'Time_us'
        header = [f"{time_str:>22}"] + [f"{data:>9}" for data in ["W", "X", "Y", "Z"]]
        logger = ImuLogger(header = header)
        is_received_any_msg = False
        try:
            host_com = UartCommunicator(timeout = 0.5)
            if args.get('first_msg', 0) == True:
                while(True):
                    reference_time_us = time.time_ns() // 10**3 # int object in the end
                    pkg = reference_time_us.to_bytes(8, byteorder='little') #cast to uint64 in little-endian
                    # 23 10 08 60 da 50 2f 62 5b 06 00 29 84 (test start msg), 23 11 08 60 da 50 2f 62 5b 06 00 46 c1 (test stop msg)
                    host_com.uart_write_package(msg_code = MessageCode.START_CMD.value, data = pkg) 
                    try:
                        rx_msg = host_com.uart_read_package()
                        # Expected ACK: # [start_byte + ERROR_STATUS + data_len + error_code + CRC]
                        print("Received_msg (integer): ", list(rx_msg)) 
                        if (len(rx_msg) != 6 
                            or rx_msg[0] != PROTOTOL_START_BYTE 
                            or rx_msg[1] != MessageCode.ERROR_STATUS.value 
                            or rx_msg[2] != 1 ):
                                print("Invalid ACK message, retrying START_CMD")
                                continue
                        err_code = rx_msg[3]
                        if err_code == 0:  
                            print("START_CMD acknowledged")
                            break
                        else:
                            print(f"STM32 returned error code: 0x{err_code:02X}, sending again ...")
                    except UartReadTimeout:
                        raise
                    
            while(True):
                try:
                    msg = host_com.uart_read_package() #blocking !!!
                    w, x, y, z, timestamp, _ = struct.unpack_from('<4f1Q1H', msg, offset = 3)
                    logger.save_quaternions(QuaternionMsgResponse(w, x, y, z, timestamp))
                    is_received_any_msg = True
                except UartReadTimeout as e:
                    raise
                    
        except UartReadTimeout as e:
            print(f'Uart protocol: {e}')
        except serial.SerialException as e:
            print(f"{e}.")
        except KeyboardInterrupt:
            print('End event was raised')
        except struct.error as e:
            print(f'Struct error: {e}')
        finally:  
            logger.flush_buffer()     
            print('Data saved')       
            if is_received_any_msg:  plot_quaternion_meas(logger.file_name)
    else:
        file_name = args.get('filename', 0)
        if not file_name: raise ValueError('No file specified')
        log_dir = Path("log_data")
        log_dir.mkdir(exist_ok=True)
        file_name = Path(log_dir)/file_name
        if not Path(file_name).exists():
            raise FileNotFoundError(f'No file exists: {file_name} (use "meas" arg to create new)')
        
        print("Processing file: ", file_name)
        plot_quaternion_meas(file_name)
    

if __name__ == "__main__":
    # Choose file 
    parser = argparse.ArgumentParser(prog = 'Build plots of quaternion measurements')

    parser.add_argument('-f','--filename', 
                        default='imu_log_2026-09-09_18-10-56.csv', 
                        help='Choose file to be processed in log_data folder!')
    
    subparser = parser.add_subparsers(dest='cmd', 
                                    title='subcommand',
                                    help='Collect new measurements from serial (ignore measurements from chosen file)')
    
    make_meas = subparser.add_parser(name='meas')
    
    make_meas.add_argument('-fm', '--first-msg', action='store_true', default=False,
                        help='Argument to initiate handshake (start cmd + timestamp send to STM32)')
    
    # make_meas.add_argument('-ns', '--no-save', dest='save', action='store_false', default=True, 
    #                     help='Whether to save received measurements (default: True)')
    
    args = vars(parser.parse_args())
    print(args)    
    
    main(args)
    