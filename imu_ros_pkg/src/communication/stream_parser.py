"""
UART binary protocol stream parser.

This module implements an incremental parser for processing incoming
UART data streams according to the project-specific binary protocol.

Protocol frame format:
    [START][MSG_CODE][PAYLOAD_LEN][PAYLOAD][CRC_H][CRC_L]

The parser uses buffered stream processing: incoming data chunks are
appended to an internal buffer, where candidate frames are detected
and validated. Automatic resynchronization is performed by scanning
for subsequent frame start markers whenever an invalid frame is
encountered.

The implementation includes:
    - Finite-state machine for tracking the parsing process.
    - Protocol message-code and expected payload-length validation.
    - CRC16 validation.
    - Automatic stream resynchronization.
    - Optimized buffer management using a parsing marker index.
    - memoryview-based access to candidate frames without copying
      data during CRC validation.
    - Bounded message queue with drop-oldest overflow policy.

Main components:
    StreamParser: Incremental UART stream parser.
    ParserQueueEmptyError: Exception raised when retrieving a frame
        from an empty message queue.
"""

from collections import deque
from enum import Enum

from src.communication.crc16 import calculate_crc16
from src.communication.com_protocol_codes import PROTOTOL_START_BYTE, MessageCode, MsgCodePayloadDict


class ParserQueueEmptyError(IndexError):
    """" Raised on accessing message from empty queue (dequeue) """ 
    pass
   

ParseStates = Enum(
    value="ParseStates",
    names=("WAIT_START", "WAIT_HEADER", "WAIT_BODY")
)

class StreamParser:
    """
    Incremental parser for the binary UART protocol.

    Incoming data chunks are appended to an internal bytearray buffer.
    The parser scans the accumulated data for frame start byte (marker)
    and processes candidate frames using a finite-state machine.

    Automatically attempts to resynchronize after invalid message codes,
    payload lengths or CRC checks by searching for the next start marker.
    Processing continues until no further start marker is found or an
    incomplete frame requires additional input data.

    Buffer management is optimized using a marker index to avoid
    unnecessary data deletion and memoryview to access candidate frames
    without copying their contents during CRC validation.

    Successfully validated frames are copied into a bounded deque.
    When the queue is full, the oldest frame is automatically discarded
    to make room for the newest one.

    Attributes:
        _msg: Internal buffer containing incoming and unprocessed data.
        _msg_queue: Bounded queue holding successfully parsed frames.
        _mark_index: Current position in the buffer from which parsing
            continues.
        _queue_overflow_count: Number of frames discarded due to queue
            overflow.
        _crc16_error_cntr: Number of frames rejected due to CRC errors.
    """
    def __init__(self, max_frame_size = 260, min_frame_size = 6, max_queue_size = 3) -> None:
        if not all(type(i) is int for i in [max_frame_size, min_frame_size, max_queue_size]):
            raise TypeError("Expected integer parameters")
            
        if not (6 <= max_frame_size <= 260 and 6 <= min_frame_size <= 260 and max_frame_size >= min_frame_size):
            raise ValueError("Incorrect frame size: max min should be within 6..260, max >= min")        
        
        if not (0 < max_queue_size <= 100):
            raise ValueError("Incorrect queue size: should be >0 and <=100")

        self._max_frame_size = max_frame_size
        self._min_frame_size = min_frame_size
        self._max_queue_size = max_queue_size
        
        self._msg = bytearray()
        # очередь для собранных пакетов типа bytearray
        self._msg_queue = deque(maxlen = max_queue_size) 

        self._cur_parse_state = ParseStates.WAIT_START
        self._mark_index = 0
        self._payload_len = 0
        
        # stats 
        self._queue_overflow_count = 0        
        self._crc16_error_cntr = 0

    def _reset_parser(self):
        # очищение буфера, сброс состояния парсера и инкремента данных
        self._msg.clear() # or = bytearray() -  what is faster?
        self._cur_parse_state = ParseStates.WAIT_START
        self._mark_index = 0
        self._payload_len = 0

    def is_msg_available(self):
        """
        Check whether at least one completed frame is available in the queue.

        Returns:
            True if the message queue is not empty, otherwise False.
        """
        return len(self._msg_queue) > 0

    def get_message(self):
        """
        Retrieve the oldest available frame from the message queue.

        The retrieved frame is removed from the queue.

        Returns:
            The next complete frame as a bytearray.

        Raises:
            ParserQueueEmptyError: If the message queue is empty.
        """
        if not self.is_msg_available():
            raise  ParserQueueEmptyError("Message queue is still empty")
        return self._msg_queue.popleft()

    def _is_valid_msg_code(self, raw_byte) -> bool:
        try:
            MessageCode(raw_byte)
        except ValueError:
            # print("Incorrect MSG CODE")
            return False
        else:
            return True


    def process_bytes(self, stream: bytes):
        """
        Append incoming data to the internal buffer and process available frames.

        Incoming bytes are accumulated in the internal bytearray instead of
        being processed individually. The parser continuously scans the buffer
        for frame start markers and validates candidate frames.

        Processing steps:
            1. Search for the next frame start marker.
            2. Validate the message code and expected payload length.
            3. Wait for the complete frame if not enough data is available.
            4. Validate the frame using CRC16.
            5. Enqueue successfully validated frames.

        If a candidate frame is invalid, the parser advances the marker index
        and continues searching for the next possible frame. This provides
        automatic resynchronization without a separate recovery routine.

        Buffer management:
            - _mark_index tracks the current parsing position and prevents
            unnecessary deletion of already processed data.
            - Consumed buffer contents are removed only when compaction
            becomes necessary.
            - memoryview provides zero-copy access to candidate frames
            during CRC validation. A separate copy is created only when
            a successfully validated frame is added to the message queue.

        Processing stops when no further start marker is found or when
        additional input is required to complete a candidate frame.

        Args:
            stream: Incoming byte chunk received from the UART interface.

        Returns:
            None.
        """
        if not stream:
            return

        self._msg.extend(stream) 
        # print(f"New bytes received, msg len: {len(self._msg)}")
        if self._mark_index >= self._max_frame_size:
            # print("Buffer is overloaded, clearing unness bytes ...")
            del self._msg[:self._mark_index]
            self._mark_index = 0

        while(True):
            if self._cur_parse_state == ParseStates.WAIT_START:
                if (m := self._msg.find(PROTOTOL_START_BYTE, self._mark_index)) == -1:
                    self._msg.clear()
                    self._mark_index = 0
                    return
                self._mark_index = m
                # self._msg = self._msg[m:]
                self._cur_parse_state = ParseStates.WAIT_HEADER

            if self._cur_parse_state == ParseStates.WAIT_HEADER:
                if  (left_len := (len(self._msg) - self._mark_index)) < 3:
                    return # выходим из цикла если не накопили информационные байты
                # print(f"LEN MSG: {left_len}, mark index: {self._mark_index}")
                msg_code_value: int = self._msg[self._mark_index + 1]
                if not self._is_valid_msg_code(msg_code_value):
                    # print(f"MSG CODE Error! {msg_code_value}")
                    self._mark_index += 1
                    self._cur_parse_state = ParseStates.WAIT_START
                    continue # снова начинаем парсинг со следущего байта

                if self._msg[self._mark_index + 2] == MsgCodePayloadDict[msg_code_value]:
                    self._cur_parse_state = ParseStates.WAIT_BODY
                    self._payload_len = MsgCodePayloadDict[msg_code_value]
                else:
                    # print("DATA LEN Error!")
                    self._mark_index += 1
                    self._cur_parse_state = ParseStates.WAIT_START
                    continue # снова начинаем парсинг со следущего байта

            if self._cur_parse_state == ParseStates.WAIT_BODY:
                # print(f"Frame size: {self._payload_len + 5}")
                if self._payload_len + 5 > len(self._msg) - self._mark_index:
                    # ждем все байты данных + crc 
                    return 

            # Считаем crc и делаем проверку
            # print(f"Buffer len: {self._msg[self._mark_index:]}")
            with memoryview(self._msg) as view:
                with view[self._mark_index:self._mark_index + self._payload_len + 5] as possible_msg:
                    if calculate_crc16(possible_msg, len(possible_msg)) != 0x00:
                        # print("CRC error", crc)
                        self._crc16_error_cntr += 1
                        self._mark_index += 1 # снова начинаем парсинг со следущего байта  
                    else:
                        if len(self._msg_queue) == self._msg_queue.maxlen:  # Drop Oldest
                            self._queue_overflow_count += 1
                        self._msg_queue.append(bytearray(possible_msg))
                        # print(f"MSG enqueued, queue len: {len(self._msg_queue)}")
                        # manual reset 
                        # del self._msg[:self._mark_index]
                        self._mark_index += self._payload_len + 5 #  начинаем парсинг c начала следущего пакета! 
                        print(f"Last msg cleared from buffer, current len: {len(self._msg) - self._mark_index}") # 0 !
                    self._cur_parse_state = ParseStates.WAIT_START

