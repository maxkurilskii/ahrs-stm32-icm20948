from pynput.keyboard import Key, KeyCode, Listener
from enum import Enum
from queue import Queue, Empty
import time
from threading import Event


class ControlCommmands(Enum):
    START_QUAT_MEAS     = 0
    STOP_QUAT_MEAS      = 1
    # SET_REF_TIME      = 2 #
    START_CALIBRATE     = 3
    STOP_CALIBRATE      = 4
    SEND_CALIB_DATA     = 5

class ShutdownException(Exception):
    pass

class KeyboardListener:

    # Команды - Кнопки
    KEY_CMD_MAP = {
        'Q': ControlCommmands.START_QUAT_MEAS,
        'S': ControlCommmands.STOP_QUAT_MEAS,
        # 'T': ControlCommmands.SET_REF_TIME,
        'C': ControlCommmands.START_CALIBRATE,
        'E': ControlCommmands.STOP_CALIBRATE,
        'U': ControlCommmands.SEND_CALIB_DATA
    }

    def __init__(self, queue):
        self.listener = None
        self.queue = queue
        self.pressed_keys = set()
        self._press_cntr = 0

    def _on_press(self, key):
            # receive key and interpret (may be use prioritires for some cmds later)

            if key is None:
                print("\nNo such command!")
                return

            if key == Key.esc:
                raise ShutdownException("ESC pressed!")
            
            if isinstance(key, KeyCode):
                print("\nНажатие обычной клавиши")
                if key.char is None:
                    return
                key_n = key.char.upper()

                if key_n in self.pressed_keys:
                    return
                self.pressed_keys.add(key_n)

                # print(f"Сама клавиша: {key}, char: {key.char}, fchar: {key.from_char('A')}, is_dead: {key.is_dead}")
                command = self.KEY_CMD_MAP.get(key_n)
                if command is not None:
                    self.queue.put(command)


    def _on_release(self, key):
        if key is not None and isinstance(key, KeyCode) and key.char is not None:
            self.pressed_keys.discard(key.char.upper())
            
    def start(self):
        if self.listener is None:
            # start listener thread
            self.listener = Listener(on_press=self._on_press, 
                                     on_release=self._on_release)
            self.listener.start()
            self.listener.wait()
        else:
            RuntimeError("Listener is already running!")
            

    def stop(self):
        if self.listener is not None:
            self.listener.stop() # stop listener thread
            self.listener.join() # wait for main thread to stop listener thread
            self.listener = None

    def join(self):
        if self.listener is not None:
            self.listener.join()
        else:
            raise RuntimeError("Поток Listener не запущен!")

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, exc_type, exc, tb):
        self.stop()
                

        
    

def main():
    TxQueue = Queue()
    with KeyboardListener(TxQueue) as key_cmd_src:
        try:
            key_cmd_src.join()
    
        except ShutdownException as e:
            print("App exitting ...")

        except Exception as e:
            print(f"Unexpected error: {e}")

if __name__ == "__main__":
    main()