import pytest
import random
from utils.uartcom.protocol_codes import PROTOTOL_START_BYTE, MessageCode, MsgCodePayloadDict
# from utils.uartcom.msg_parser import StreamParser, ParseStates 
from utils.uartcom.stream_parser import StreamParser, ParseStates, ParserQueueEmptyError
from utils.uartcom.crc16 import calculate_crc16

@pytest.fixture
def stream_parser():
    return StreamParser(max_frame_size = 260, min_frame_size = 6, max_queue_size = 5)

def random_bytes(size):
    return bytes([random.randint(0, 255) for _ in range(size)])

def random_bytes_except(size, ex_byte):
    return bytes(filter(lambda x: x != ex_byte, [random.randint(0, 255) for _ in range(size)]))


# Тест инициализациии парсера  (положительный)
def test_initialization(stream_parser):
    assert stream_parser._cur_parse_state == ParseStates.WAIT_START
    assert stream_parser._msg == b''
    assert stream_parser.is_msg_available() == False


# Отрицательный тест инициализациии парсера (ввод вещественных чисел)
@pytest.mark.parametrize("param_name", [
    "max_frame_size",
    "min_frame_size",
    "max_queue_size"
])
@pytest.mark.parametrize("invalid_value", [
    10.5,
    "100",
    None,
    True
])
def test_incorrect_init_type_values(param_name, invalid_value):
    with pytest.raises(TypeError, match="Expected integer parameters"):
        StreamParser(**{param_name: invalid_value})


# Негативный тест инициализациии парсера (ввод отрциательных чисел)
@pytest.mark.parametrize("param_name", [
    "max_frame_size",
    "min_frame_size"
])
@pytest.mark.parametrize("invalid_value", [-1, -10, -260])
def test_incorrect_init_negative_values(param_name, invalid_value):
    with pytest.raises(ValueError, match="Incorrect frame size"):
        StreamParser(**{param_name: invalid_value})

# Негативный тест инициализациии парсера (неверная длина кадра)
@pytest.mark.parametrize("param_name", [
    "max_frame_size",
    "min_frame_size"
])
@pytest.mark.parametrize("invalid_value", [0, 1, 5, 261, 500])
def test_incorrect_init_frame_values(param_name, invalid_value):
    with pytest.raises(ValueError, match="Incorrect frame size"):
        StreamParser(**{param_name: invalid_value})


# Негативный тест инициализациии парсера (макс размер кадра меньше минимального)
@pytest.mark.parametrize("max_size, min_size", [
    (6, 7),
    (10, 20),
    (100, 200),
    (259, 260)
])
def test_incorrect_init_frame_order(max_size, min_size):
    with pytest.raises(ValueError, match="Incorrect frame size"):
        StreamParser(
            max_frame_size=max_size,
            min_frame_size=min_size
        )

# Негативный тест инициализациии парсера (недопустимый размер очереди)
@pytest.mark.parametrize("invalid_value", [
    -100, -1, 0, 101, 200, 1000
])
def test_incorrect_init_queue_size(invalid_value):
    with pytest.raises(ValueError, match="Incorrect queue size"):
        StreamParser(max_queue_size=invalid_value)

# Положительный тест инициализациии парсера (граничные значения)
@pytest.mark.parametrize("max_size, min_size, queue_size", [
    (6, 6, 1),
    (260, 6, 100),
    (100, 100, 50),
    (260, 260, 1)
])
def test_correct_init_boundary_values(max_size, min_size, queue_size):
    parser = StreamParser(
        max_frame_size=max_size,
        min_frame_size=min_size,
        max_queue_size=queue_size
    )

    assert parser._max_frame_size == max_size
    assert parser._min_frame_size == min_size
    assert parser._max_queue_size == queue_size



# ----------------------------------------------------------------------------------------

# Тест правильности расчета CRC16 (см. CRC-16/IBM-3740 или CRC-16/CCITT-FALSE)
# https://crccalc.com/?crc=123456789&method=CRC-16%2FIBM-3740&datatype=ascii&outtype=hex
@pytest.mark.parametrize("test_input, expected_crc",
        [("123456789".encode('ascii'), 0x29b1),
        (bytes.fromhex("23 10 08 60 da 50 2f 62 5b 06 00"), 0x2984),
        (bytes.fromhex("23 11 08 60 da 50 2f 62 5b 06 00"), 0x46c1)])
def test_crc16_calculation_basic(test_input, expected_crc):
    test_bytearray = bytearray(test_input)
    crc = calculate_crc16(test_bytearray, len(test_bytearray))
    print(test_input.hex(' '), 'длина: ', len(test_bytearray))
    assert (crc & 0xFFFF) ==  expected_crc
    crc_h, crc_l = (crc >> 8) & 0xFF,  crc & 0xFF 
    full_msg = test_bytearray +  bytearray([crc_h, crc_l])
    assert calculate_crc16(full_msg, len(test_input) + 2) == 0x00

    


def create_correct_msg(msg_code: int, payload_size: int):
    part_one = bytes([PROTOTOL_START_BYTE, msg_code, payload_size]) + random_bytes(payload_size)
    crc = calculate_crc16(bytearray(part_one), payload_size + 3)
    return bytearray(part_one) + bytearray([(crc >> 8) & 0xFF, crc & 0xFF]) # crc_h, crc_l



# Негативный тест CRC16 (должно возникнуть несовпадение CRC16 при поврежденных данных) 
# пакет был корректен, но появились ошибки в данных при передаче 
def test_crc16_negative():
    for _ in range(100):
        # поменяем несоклько байт в данных 
        msg_code_val = random.choice([v.value for v in MessageCode])
        msg = create_correct_msg(msg_code_val, MsgCodePayloadDict[msg_code_val])
                                         
        l_bound = random.randint(3, 254)
        r_bound = random.randint(l_bound, 255)
        msg[l_bound : r_bound] = random_bytes(r_bound - l_bound  + 1)
        assert calculate_crc16(msg, len(msg)) != 0x00


# ---------------------------------------------------------------------------------------
# Тест обработки одного пакета с поврежденным CRC16.
# ---------------------------------------------------------------------------------------
def test_crc16_corruption(stream_parser):
    for _ in range(100):
        msg_code_val = random.choice([v.value for v in MessageCode])
        corrupted_msg = create_correct_msg(msg_code_val, 
                                            MsgCodePayloadDict[msg_code_val])
        # выберем один из байтов crc16
        t = random.randint(1,2)
        # заменим его на любой кроме него самого (это так же исключает сценарии когда 2 байта crc одинаковые)
        corrupted_msg[-t] = int.from_bytes(random_bytes_except(1, corrupted_msg[-t]), 'little')
        # assert calculate_crc16(corrupted_msg, len(corrupted_msg)) != 0x00
        stream_parser.process_bytes(corrupted_msg)
        assert not stream_parser.is_msg_available()
        # после crc16 парсер в ресинзронизации может сразу перейти на состояние WAIT HEADER
        # assert stream_parser._cur_parse_state == ParseStates.WAIT_START 
        # поэтмоу для этого теста надо сбрасывать парсер полностью
        stream_parser._msg = bytearray()
        stream_parser._mark_index = 0
        stream_parser._cur_parse_state = ParseStates.WAIT_START

    assert stream_parser._crc16_error_cntr == 100



# Положительный тест на определение кода команд (msg code) 
@pytest.mark.parametrize("msg_code_value", [v.value for v in MessageCode])
def test_buffer_valid_msg_code(stream_parser, msg_code_value: int):
    input_stream = bytes(create_correct_msg(msg_code_value, MsgCodePayloadDict[msg_code_value]))
    stream_parser.process_bytes(input_stream)
    assert stream_parser._cur_parse_state == ParseStates.WAIT_START
    assert stream_parser.is_msg_available()
    assert stream_parser.get_message() == input_stream
    assert not stream_parser.is_msg_available()



# Негативный тест на определение кода команд (msg code) - берем несуществующие команды
# уже существующие: 66, 16, 17, 18, 20, 21 
@pytest.mark.parametrize("inv_msg_code_val", [random.randint(67, 255) for _ in range(3)])
def test_buffer_invalid_msg_code(stream_parser, inv_msg_code_val: int):
    input_stream = bytes([PROTOTOL_START_BYTE, inv_msg_code_val]) + random_bytes_except(10, PROTOTOL_START_BYTE)
    stream_parser.process_bytes(input_stream)
    assert stream_parser._cur_parse_state == ParseStates.WAIT_START
    assert stream_parser.is_msg_available() == False




# Тест обработки части 1ого пакета + оставшаяся часть 1ого + весь второй
def test_partial_first(stream_parser):
    for _ in range(len(MessageCode)):
        msg_code_value = random.choice([v.value for v in MessageCode])
        input_1 = bytes(create_correct_msg(msg_code_value, 
                                        MsgCodePayloadDict[msg_code_value]))
       
        delim_ind = random.randint(1, len(input_1) - 2)
        first_part, second_part  = input_1[:delim_ind], input_1[delim_ind:]
        stream_parser.process_bytes(first_part) 
        assert stream_parser.is_msg_available() == False
        assert stream_parser._cur_parse_state != ParseStates.WAIT_START
        
        # при доступе к очереди должно явно вызываться исключение
        with pytest.raises(ParserQueueEmptyError) as e:
            stream_parser.get_message()

        msg_code_value = random.choice([v.value for v in MessageCode])
        input_2 = bytes(create_correct_msg(msg_code_value, 
                                        MsgCodePayloadDict[msg_code_value]))
        
        stream_parser.process_bytes(second_part + input_2)
        assert stream_parser.is_msg_available()

        res = bytearray()
        while(stream_parser.is_msg_available()):
            res += stream_parser.get_message()

        assert res == input_1 + input_2

        assert not stream_parser.is_msg_available()
        assert stream_parser._cur_parse_state == ParseStates.WAIT_START


@pytest.mark.parametrize("frames_number", range(6)) # диапазон от 0..5 , так как
# фикстура stream_parser по умолчанию имеет макс размер очереди равный 5
def test_multliple_correct_frames_without_queue_ovf(stream_parser, frames_number):
    for _ in range(len(MessageCode)):
        input_stream = b''
        for _ in range(frames_number):
            msg_code_val = random.choice([v.value for v in MessageCode])
            input_stream += bytes(create_correct_msg(msg_code_val, MsgCodePayloadDict[msg_code_val]))

        if frames_number == 0:
            print(input_stream)
        stream_parser.process_bytes(input_stream)
        assert stream_parser._cur_parse_state == ParseStates.WAIT_START
        assert stream_parser.is_msg_available() == (True if frames_number else False)
        assert stream_parser._queue_overflow_count == 0 # при размере очереди = 5 !

        res = bytearray()
        while(stream_parser.is_msg_available()):
            res += stream_parser.get_message()

        assert not stream_parser.is_msg_available()
        assert res == bytearray(input_stream)[-len(res):]


@pytest.mark.parametrize("frames_number", range(6, 11)) # диапазон для переполнения 
# при условии, что фикстура stream_parser имеет макс размер очереди равный 5
def test_multliple_correct_frames_queue_ovf(stream_parser, frames_number):
    for _ in range(len(MessageCode)):
        input_stream = b''
        for _ in range(frames_number):
            msg_code_val = random.choice([v.value for v in MessageCode])
            input_stream += bytes(create_correct_msg(msg_code_val, MsgCodePayloadDict[msg_code_val]))
        
        stream_parser.process_bytes(input_stream)
        assert stream_parser._cur_parse_state == ParseStates.WAIT_START
        assert stream_parser.is_msg_available()

        assert len(stream_parser._msg_queue) == stream_parser._max_queue_size

        res = bytearray()
        while(stream_parser.is_msg_available()):
            res += stream_parser.get_message()

        assert not stream_parser.is_msg_available()
        assert res == bytearray(input_stream)[-len(res):]

        assert stream_parser._queue_overflow_count == frames_number - stream_parser._max_queue_size

        # сбрасываем вручную флаг, чтобы предыдщущие первышеия не влияли на следующей итерации
        stream_parser._queue_overflow_count = 0


# ------------------------------------------------------------------------------------------
# Тест обработки одного пакета с частью предыдущего пакета в начале (сценарий 1)
# часть предыдщего пакета НЕ содержащит СТАРТОВЫЙ байт + один целый пакет 
# ------------------------------------------------------------------------------------------

def test_garbage_first_nomarker(stream_parser):
    for _ in range(len(MessageCode)):
        input_stream = random_bytes_except(5, PROTOTOL_START_BYTE)
        assert input_stream.count(PROTOTOL_START_BYTE) == 0

        stream_parser.process_bytes(input_stream) 
        assert stream_parser._cur_parse_state == ParseStates.WAIT_START
        assert stream_parser.is_msg_available() == False

        msg_code_val = random.choice([v.value for v in MessageCode])
        input_stream = bytes(create_correct_msg(msg_code_val, MsgCodePayloadDict[msg_code_val]))
               
        stream_parser.process_bytes(input_stream)  
        assert stream_parser.is_msg_available()
        assert stream_parser.get_message() == input_stream
        assert not stream_parser.is_msg_available()
        assert stream_parser._cur_parse_state == ParseStates.WAIT_START




# ------------------------------------------------------------------------------------------
# Тест обработки одного пакета с частью предыдущего пакета в начале (сценарий 2):
# часть данных предыдщего пакета (в данных случайно оказался маркер байт) + один целый пакет
# ------------------------------------------------------------------------------------------

def test_garbage_first_with_marker(stream_parser):
    for _ in range(100):
        # данные предыдего пакета, включающие СТАРТОВЫЙ БАЙТ: 
        partial_previous = random_bytes_except(5, PROTOTOL_START_BYTE) + \
            PROTOTOL_START_BYTE.to_bytes(1,'little') + \
            random_bytes_except(30, PROTOTOL_START_BYTE)

        assert partial_previous.count(PROTOTOL_START_BYTE) == 1

        msg_code_val = random.choice([v.value for v in MessageCode])
        msg = bytes(create_correct_msg(msg_code_val, MsgCodePayloadDict[msg_code_val]))
        
        full_stream = partial_previous + msg
        print(f"Full: {full_stream}")
        stream_parser.process_bytes(full_stream)
        print(f"Msg buf: {stream_parser._msg}")
        assert stream_parser.is_msg_available()
        assert len(stream_parser._msg_queue) == 1

        assert stream_parser.get_message() == msg
        assert not stream_parser.is_msg_available()
        assert stream_parser._cur_parse_state == ParseStates.WAIT_START




# ---------------------------------------------------------------------------------------
# Тест обработки одного пакета с частью предыдущего пакета в начале (сценарий 3):
# часть данных предыдщего пакета + один целый пакет
# (в данных случайно оказался маркер байт и совпали msg code и data len, то, есть header)
# ---------------------------------------------------------------------------------------
def test_garbage_first_with_marker_and_header(stream_parser):
    # данные предыдущего пакета, включающие СТАРТОВЫЙ БАЙТ и валидный MSG CODE и валидный Payload length: 
    for _ in range(100):
        partial_previous = \
            PROTOTOL_START_BYTE.to_bytes(1,'little') + \
            bytes([MessageCode.START_CMD.value, MsgCodePayloadDict[MessageCode.START_CMD.value]]) + \
            random_bytes_except(30, PROTOTOL_START_BYTE)

        assert partial_previous.count(PROTOTOL_START_BYTE) == 1

        # специально посчитаем crc для такой случайной последовательности (как для пакета STOP)
        # и изменим его значение, чтобы случайно не совпал
        partial_previous = bytearray(partial_previous)
        crc: int = calculate_crc16(partial_previous, 11)
        # поменяем местами crc_h, crc_l в ложной посылке
        partial_previous[11] = int.from_bytes(random_bytes_except(1, partial_previous[11]), 'little')
        msg = bytes(create_correct_msg(MessageCode.START_CMD.value, 
                                        MsgCodePayloadDict[MessageCode.START_CMD.value]))
        full_stream = partial_previous + msg
        stream_parser.process_bytes(full_stream)
        assert stream_parser._crc16_error_cntr != 0
        assert stream_parser.is_msg_available()
        assert len(stream_parser._msg_queue) == 1

        assert stream_parser.get_message() == msg
        assert not stream_parser.is_msg_available()
        assert stream_parser._cur_parse_state == ParseStates.WAIT_START
    