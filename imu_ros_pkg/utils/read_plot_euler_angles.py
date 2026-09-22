import argparse
import serial
import time
import struct
import pandas as pd
from pathlib import Path
import matplotlib.pyplot as plt
from typing import Optional

from . uart_com import UartCommunicator, MessageCode, UartReadTimeout
from . imu_csv_logger import ImuLogger
from . base_dataclasses import ReadImuEulerResponce


plt.rcParams["font.size"] = 12
plt.rcParams["axes.grid"] = True
# plt.rcParams["figure.autolayout"] = True


def plot_euler_meas(file):
    
    df = pd.read_csv(file,
                    header=0,
                    names=['Time_s', "Roll", "Pitch", "Yaw"])  
    df['Time_s'] = df['Time_s'] / 10**6
    fig, axes = plt.subplots(3,1, sharex=True, figsize=(10,6))
    tight_par = False

    # Roll
    df.plot(ax=axes[0], x='Time_s', y='Roll', color='blue', linewidth=2, legend=True)
    axes[0].set_ylabel('Roll (deg)')
    axes[0].grid(True, alpha=0.3)
    axes[0].autoscale(enable=None, axis="x", tight=tight_par)

    # Pitch
    df.plot(ax=axes[1], x='Time_s', y='Pitch', color='green', linewidth=2, legend=True)
    axes[1].set_ylabel('Pitch (deg)')
    axes[1].grid(True, alpha=0.3)
    axes[1].autoscale(enable=None, axis="x", tight=tight_par)

    # Yaw
    df.plot(ax=axes[2], x='Time_s', y='Yaw', color='red', linewidth=2, legend=True)
    axes[2].set_ylabel('Yaw (deg)')
    axes[2].set_xlabel('Time (s)')
    axes[2].grid(True, alpha=0.3)
    axes[2].legend(loc='lower left')
    axes[2].autoscale(enable=None, axis="x", tight=tight_par)
    fig.suptitle('Euler Angles After Filtration', fontsize=14, fontweight='bold')
    fig.align_ylabels()
    plt.tight_layout(pad=0.4, w_pad=0.5, h_pad=1.0)
    plt.show()
    
   
def main(args):
    if args.get('cmd', 0) == 'meas':
        print('Getting new measurements, ignoring specified file')
        print('To stop measurements collection press Ctrl + C')
        time_str = 'Time_us'
        header = [f"{time_str:>22}"] + [f"{data:>9}" for data in ['Roll', 'Pitch', 'Yaw']]
        logger = ImuLogger(header = header)
        is_received_any_msg = False
        try:
            host_com = UartCommunicator(timeout = 0.1)
            if args.get('first_msg', 0) == True:
                while(True):
                    try:
                        reference_time_us = time.time_ns() // 10**3 # int object in the end
                        tx_pkg = reference_time_us.to_bytes(8, byteorder='little') #cast to uint64 in little-endian
                        host_com.uart_write_package(msg_code = MessageCode.START_CMD.value, data = tx_pkg) # 23 10 08 60 da 50 2f 62 5b 06 00 29 84
                        rx_msg = host_com.uart_read_package()
                        # Expected ACK: # [start_byte + ERROR_STATUS + data_len + error_code + CRC]
                        print("Received_msg (integer): ", list(rx_msg)) 
                        if (len(rx_msg) != 6 
                            or rx_msg[0] != UartCommunicator.START_BYTE 
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
                    roll, pitch, yaw, timestamp, _ = struct.unpack_from('<3f1Q1H', msg, offset = 3)
                    logger.save_angle_data(ReadImuEulerResponce(roll, pitch, yaw, timestamp))
                    is_received_any_msg = True
                except UartReadTimeout:
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
            if is_received_any_msg: plot_euler_meas(logger.file_name)
    else:
        file_name = args.get('filename', 0)
        if not file_name: raise ValueError('No file specified')
        log_dir = Path("log_data")
        log_dir.mkdir(exist_ok=True)
        file_name = Path(log_dir)/file_name
        if not Path(file_name).exists():
            raise FileNotFoundError(f'No file exists: {file_name} (use "meas" arg to create new)')
        
        print("Processing file: ", file_name)
        plot_euler_meas(file_name)
    

if __name__ == "__main__":
    # Choose file 
    parser = argparse.ArgumentParser(prog = 'Build plots of euler angle measurements')
    parser.add_argument('-f','--filename', default='imu_log_2026-09-09_18-10-56.csv', 
                        help='Choose file to be processed in log_data folder!')
    subparser = parser.add_subparsers(dest='cmd', title='subcommand', help='Collect new measurements from serial (ignore measurements from chosen file)')
    make_meas = subparser.add_parser(name='meas')
    make_meas.add_argument('-fm', '--first-msg', action='store_true', default=False,
                        help='Argument to initiate handshake (start cmd + timestamp are sent)')
    # make_meas.add_argument('-ns', '--no-save', dest='save', action='store_false', default=True, 
    #                     help='Whether to save received measurements (default: True)')
    
    args = vars(parser.parse_args())
    print(args)    
    
    main(args)
    
    

# def build_plot(file, a):
    
#     file_name = args.get('filename', 0)
#     if not file_name: raise ValueError('No file specified')
#     file = Path("log_data")/file_name
#     print("Processing file: ", file)
#     df = pd.read_csv(file,
#             header=0,
#             names=['Time_s', "Roll", "Pitch", "Yaw"])  