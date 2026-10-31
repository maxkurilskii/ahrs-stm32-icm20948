from utils.uartcom.protocol_codes import PROTOTOL_START_BYTE
from utils.uartcom.crc16 import calculate_crc16


class ProtocolEncoder:
    """Builds binary UART frames according to the STM32 protocol."""
         
    @staticmethod
    def build_message(msg_code: int, data: bytes) -> bytes:
        """
        Build a UART frame containing the message code and payload.

        The resulting frame has the following format:
            [START][MSG_CODE][DATA_LEN][DATA][CRC_H][CRC_L]

        Args:
            msg_code: Protocol message identifier.
            data: Message payload.

        Returns:
            Encoded UART frame as bytes.
        """
        tx_frame = bytearray()
        tx_frame.extend([PROTOTOL_START_BYTE, 
                        msg_code, 
                        len(data),
                        *data])
        crc16_val = calculate_crc16(tx_frame)
        crc16_h = (crc16_val >> 8) & 0xFF
        crc16_l = crc16_val & 0xFF
        tx_frame.extend([crc16_h, crc16_l])
        return bytes(tx_frame)
        