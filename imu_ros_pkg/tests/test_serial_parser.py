import pytest
import random
from utils.uart_com import ProtocolSerialParser, ParseStates 
from utils.uart_com import MessageCode, ParserQueueEmptyError, ParserCrc16Error 
from utils.crc16 import calculate_crc16

@pytest.fixture
def serial_parser():
    return ProtocolSerialParser(max_frame_size = 260, min_frame_size = 6, msg_queue_size = 5)

def random_bytes(size):
    return bytes([random.randint(0, 255) for _ in range(size)])

def random_bytes_except_marker(size):
    return bytes([random.randint(0, 34) for _ in range(size)] + [random.randint(36, 255) for _ in range(size)])


# Тест начального состояния парсера после создания
def test_initialization(serial_parser):
    assert serial_parser._cur_parse_state == ParseStates.WAIT_START
    assert serial_parser._data_len_cntr == 0
    assert serial_parser._msg == b''
    assert serial_parser.is_msg_available() == False


# Тест правильности расчета CRC16 (см. CRC-16/IBM-3740 или CRC-16/CCITT-FALSE)
# https://crccalc.com/?crc=123456789&method=CRC-16%2FIBM-3740&datatype=ascii&outtype=hex
@pytest.mark.parametrize("test_input, expected_crc",
        [("123456789".encode('ascii'), 0x29b1),
        (bytes.fromhex("23 10 08 60 da 50 2f 62 5b 06 00"), 0x2984),
        (bytes.fromhex("23 11 08 60 da 50 2f 62 5b 06 00"), 0x46c1)])
def test_crc16_calculation(test_input, expected_crc):
    test_bytearray = bytearray(test_input)
    crc = calculate_crc16(test_bytearray, len(test_bytearray))
    print(test_input.hex(' '), 'длина: ', len(test_bytearray))
    assert (crc & 0xFFFF) ==  expected_crc
    crc_h, crc_l = (crc >> 8) & 0xFF,  crc & 0xFF 
    full_msg = test_bytearray +  bytearray([crc_h, crc_l])
    assert calculate_crc16(full_msg, len(test_input) + 2) == 0x00



def create_correct_msg(msg_code, payload_size):
    part_one = bytes([0x23, msg_code, payload_size]) + random_bytes(payload_size)
    crc = calculate_crc16(bytearray(part_one), 3 + payload_size)
    return bytearray(part_one) + bytearray([(crc >> 8) & 0xFF, crc & 0xFF]) # crc_h, crc_l

# @pytest.mark.parametrize("input_bytes, expected_pkg", 
                        # один полный привильный пакет, совпадающий с размером буфера
                        # команда: 0x10, payload len = 2
                        #  [(bytes([0x23, 0x10]) + random_bytes(2), )])
# ,  
                        # один полный + часть второго
                        #   (1, 1)]) 

# Тест обработки одного корректного пакета (MSG code, Data len и CRC16 верные), 
# совпадающего с максимльным размером буфера msg (max_frame_size)
def test_one_correct_max_size_msg(serial_parser):
    input_stream: bytes = bytes(create_correct_msg(MessageCode.STOP_CMD.value, 
                                                   serial_parser._max_frame_size - 5))
    serial_parser.process_bytes(input_stream)
    assert serial_parser.is_msg_available() == True
    assert serial_parser.get_message() == input_stream
    assert serial_parser._cur_parse_state == ParseStates.WAIT_START

# Тест обработки одного корректного пакета (MSG code, Data len и CRC16 верные), 
# совпадающего с мнимальным размером буфера msg (max_frame_size)
def test_one_correct_min_size_msg(serial_parser):
    input_stream:bytes = bytes(create_correct_msg(MessageCode.STOP_CMD.value, 
                                                   serial_parser._min_frame_size - 5))
    serial_parser.process_bytes(input_stream)
    assert serial_parser.is_msg_available() == True
    assert serial_parser.get_message() == input_stream
    assert serial_parser._cur_parse_state == ParseStates.WAIT_START


# Тест на сброса буфера при получении неправильной длины данных (меньше минимальной длины кадра) 
def test_buffer_underflow(serial_parser):
    wrong_payload_size = 0 # по протколу меньше единицы не может быть
    input_stream = bytes([0x23, MessageCode.START_CMD.value, wrong_payload_size]) 
    serial_parser.process_bytes(input_stream)
    assert serial_parser._cur_parse_state == ParseStates.WAIT_START
    assert serial_parser._data_len_cntr == 0
    assert serial_parser._msg == b''
    assert serial_parser.is_msg_available() == False



# Положительный тест на опредление кода команд (msg code) 
@pytest.mark.parametrize("msg_code_value", [v.value for v in MessageCode])
def test_buffer_valid_msg_code(serial_parser, msg_code_value: int):
    input_stream = bytes(create_correct_msg(msg_code_value, random.randint(7, 255)))
    serial_parser.process_bytes(input_stream)
    assert serial_parser._cur_parse_state == ParseStates.WAIT_START
    assert serial_parser.is_msg_available() == True
    assert serial_parser.get_message() == input_stream
    assert serial_parser.is_msg_available() == False


# Негативный тест на опредление кода команд (msg code) - берем несущестующие команды
# уже существующие: 66, 16, 17, 18, 20, 21 
@pytest.mark.parametrize("msg_code_value", [random.randint(67, 255) for _ in range(10)])
def test_buffer_invalid_msg_code(serial_parser, msg_code_value: int):
    input_stream = bytes([0x23, msg_code_value])
    serial_parser.process_bytes(input_stream)
    assert serial_parser._cur_parse_state == ParseStates.WAIT_START
    assert serial_parser.is_msg_available() == False


# Тест обработки одного полного пакета + часть второго
@pytest.mark.parametrize("input_2",
        # тлько первые 3 инфомрационных байта
        [bytes([0x23, MessageCode.STOP_CMD.value, 0x08]),  
        # 3 информационных + часть payload
        create_correct_msg(MessageCode.STOP_CMD.value, 8)[:6],  
        # все кроме crc16
        bytes([0x23, MessageCode.STOP_CMD.value, 0x08])][:11]) # все кроме crc16


def test_partial_second(serial_parser, input_2):
    input_1 = bytes(create_correct_msg(MessageCode.START_CMD.value, random.randint(7, 255)))
    input_2 = bytes([0x23, MessageCode.STOP_CMD.value, 0x08])  
    serial_parser.process_bytes(input_1 + input_2)
    assert serial_parser._cur_parse_state != ParseStates.WAIT_START
    assert serial_parser.is_msg_available() == True
    assert serial_parser.get_message() == input_1
    assert serial_parser.is_msg_available() == False
    assert serial_parser._cur_parse_state == ParseStates.WAIT_DATA
    assert bytes(serial_parser._msg)[:len(input_2)] == input_2


# Негативный тест CRC16 (должно возникнуть несовпадение CRC16 при поврежденных данных) 
# пакет был корректен, но появились ошибки в данных при передаче 
def test_crc16_negative(serial_parser):
    # поменяем несоклько байт в данных 
    spoiled_stream = create_correct_msg(MessageCode.START_CMD.value, random.randint(7, 10))
    initial_crc16 = spoiled_stream[-2:]
    spoiled_stream[3:6] = b'\x00\x01\x02'
    print(spoiled_stream)
    assert calculate_crc16(spoiled_stream, len(spoiled_stream)) != initial_crc16

# Тест обработки части 1ого пакета -> оставшаяся часть 1ого -> весь второй
def test_partial_first(serial_parser):
    input_1 = bytes(create_correct_msg(MessageCode.START_CMD.value, random.randint(7, 255)))
    input_2 = bytes(create_correct_msg(MessageCode.STOP_CMD.value, random.randint(7, 255)))
    delim_ind = random.randint(1, len(input_1) - 2)
    first_part, second_part  = input_1[:delim_ind], input_1[delim_ind:]
    serial_parser.process_bytes(first_part) 
    assert serial_parser.is_msg_available() == False
    assert serial_parser._cur_parse_state != ParseStates.WAIT_START
    
    # при доступе к очереди должно явно взываться исключение
    with pytest.raises(ParserQueueEmptyError) as e:
        serial_parser.get_message()

    serial_parser.process_bytes(second_part + input_2)
    assert serial_parser.is_msg_available() == True
    assert serial_parser.get_message() == input_1
    assert serial_parser.is_msg_available() == True
    assert serial_parser.get_message() == input_2
    assert serial_parser._cur_parse_state == ParseStates.WAIT_START



# Тест обработки одного пакета с изначальным мусорными байтами (НЕ содержащие СТАРТОВЫЙ байт) 
def test_garbage_first_nomarker(serial_parser):
    input_stream = random_bytes_except_marker(100)
    serial_parser.process_bytes(input_stream) 
    assert serial_parser._cur_parse_state == ParseStates.WAIT_START
    assert serial_parser.is_msg_available() == False

    input_stream = bytes(create_correct_msg(MessageCode.START_CMD.value, random.randint(7, 255)))
    serial_parser.process_bytes(input_stream)  
    assert serial_parser.is_msg_available() == True
    assert serial_parser.get_message() == input_stream
    assert serial_parser.is_msg_available() == False
    assert serial_parser._cur_parse_state == ParseStates.WAIT_START




# ---------------------------------------------------------------------------------------
# Тест обработки одного пакета с изначальным мусорными байтами, содержащие СТАРТОВЫЙ байт 
# Возможны несколько ситуаций: 
# 1) За ложным стартовым следует некорректная команда -> уже проверяли - буфер сбросится
# 2) За ложным стартовым следует некорректная длина данных (команда случайно совпала с имеющимися) -> уже проверяли - буфер сбросится
# 3) За ложным стартовым байтом последовал некорректный CRC16 (причины): 
#     команда и длина данных были корректны (случайно) + мусор 
#     команда и длина данных были корректны (случайно) + начало следующей корректной посылки (то есть должен быть старт байт)
# ---------------------------------------------------------------------------------------
# Тест обработки пакета с корректными пакетом, но поврежденным CRC16.
# ---------------------------------------------------------------------------------------
# Глобально: пасер должен рессинхронизировться.
# ---------------------------------------------------------------------------------------


def test_garbage_first_with_marker_one(serial_parser):
    # мусор содержащий СТАРТОВЫЙ БАЙТ + данные : 
    garbage_stream = random_bytes_except_marker(1) + b'\x23' + bytes([MessageCode.UPD_TIME_CMD.value, 0x08]) + random_bytes_except_marker(8)
    # специально посчитаем crc для такой последовательности и изменим значение, 
    # чтобы случайно не совпал crc
    crc: int = calculate_crc16(bytearray(garbage_stream), len(garbage_stream))
    garbage_stream = garbage_stream + crc.to_bytes(2, 'little') # поменяем местами crc_h, crc_l в ложной посылке
    with pytest.raises(ParserCrc16Error) as e:
        serial_parser.process_bytes(garbage_stream) 
    assert serial_parser.is_msg_available() == False
    assert serial_parser._cur_parse_state == ParseStates.WAIT_START


@pytest.mark.xfail(reason = "Стрим парсер не ресинхронизируется при возникновении ошибки CRC16")
def test_garbage_first_with_marker_two(serial_parser):
    # изначальный мусор содержащий СТАРТОВЫЙ БАЙТ: 
    semi_garbage_stream = random_bytes_except_marker(1) + b'\x23' + \
        bytes([MessageCode.UPD_TIME_CMD.value, 0x08]) + \
        bytes(create_correct_msg(MessageCode.START_CMD.value, 5))
    
    with pytest.raises(ParserCrc16Error) as e:
        serial_parser.process_bytes(semi_garbage_stream) 

    assert serial_parser.is_msg_available() == True
    assert serial_parser._cur_parse_state == ParseStates.WAIT_START