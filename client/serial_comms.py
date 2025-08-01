# sudo adduser $USER dialout

import enum
import queue
import threading
import time
import struct
import logging

import serial

BAUD_RATE = 115200


HEADER_SIZE = 3
N_STATE = 3
N_MEASUREMENT = 3
START_BYTE = 255

class MsgType(enum.Enum):
    ACCEL_GYRO = 0
    STATE_EST_COV = 1
    DEBUG_STR = 2
    END_MSG = 3

ENDIANNESS = "little"
ENDIAN_CHAR = "<" if ENDIANNESS == "little" else ">"

MAX_Q_SIZE = 5

def parse_accel_gyro_msg(payload):

    accel_start = 0
    accel_end = accel_start + 4 * N_MEASUREMENT
    num_accel_bytes = accel_end - accel_start

    gyro_start = accel_end
    gyro_end = 4 * N_MEASUREMENT + gyro_start
    num_gyro_bytes = gyro_end - gyro_start

    accel_data = struct.unpack(f"{ENDIAN_CHAR}{num_accel_bytes // 4}f", payload[accel_start:accel_end])
    gyro_data = struct.unpack(f"{ENDIAN_CHAR}{num_gyro_bytes // 4}f", payload[gyro_start:gyro_end])

    return accel_data, gyro_data

def parse_state_est_cov(payload):
    state_start = 0
    state_end = N_STATE * 4
    num_state_bytes = state_end - state_start

    cov_start = state_end
    cov_end = cov_start + N_STATE * N_STATE * 4
    num_cov_bytes = cov_end - cov_start

    state_data = struct.unpack(f"{ENDIAN_CHAR}{num_state_bytes//4}f", payload[state_start:state_end])
    cov_data = struct.unpack(f"{ENDIAN_CHAR}{num_cov_bytes//4}f", payload[cov_start:cov_end])

    return state_data, cov_data

def parse_str(payload):
    return payload.decode("ascii")

def check_and_add_to_q(val, q):
    if q.full():
        q.get()
    q.put(val)


class SerialCommunicator:
    def __init__(self, port: str = "/dev/ttyACM0", timeout: float = 1, check_for_ready: bool = False):
        self.port = port
        self.timeout = timeout
        self.check_for_ready = check_for_ready
        self.__serial_comm = serial.Serial(timeout=0.5)
        self.__configure_serial_port()
        self.__exit_flag = False
        self.__command_q = queue.Queue()
        self.__serial_thread = threading.Thread(target=self.__run_serial_communicator)

        self.measurement_q = queue.Queue(maxsize=MAX_Q_SIZE)
        self.state_cov_q = queue.Queue(maxsize=MAX_Q_SIZE)
        self.debug_msg_q = queue.Queue(maxsize=MAX_Q_SIZE)

    def _send_cmd(self, cmd: str):
        self.__command_q.put(cmd.encode())

    def get_sensor_data(self):
        return self.measurement_q.get()
    def get_state_cov_data(self):
        return self.state_cov_q.get()

    def __run_serial_communicator(self):


        while not self.__exit_flag:
            if self.__command_q.qsize() > 0:
                cmd = self.__command_q.get(block=False, timeout=SERIAL_CHECK_FOR_COMMANDS_DELAY / 2)
                self.__serial_comm.write(cmd)

            if self.__serial_comm.in_waiting > 0:
                header_bytes = self.__serial_comm.read(1)

                possible_header_decoded = struct.unpack(f"<B", header_bytes)[0]
                if possible_header_decoded != START_BYTE:
                    continue
                
                header_bytes = self.__serial_comm.read(HEADER_SIZE-1)
                msg_type, msg_len = struct.unpack(f"<{HEADER_SIZE-1}B", header_bytes)
                try:
                    msg_type_class = MsgType(msg_type)
                except ValueError:
                    logging.warning("bad msg type received, skipping...")
                    continue
                data_bytes = self.__serial_comm.read(msg_len)
                if msg_type_class == MsgType.ACCEL_GYRO:
                    accel, gyro = parse_accel_gyro_msg(data_bytes)
                    check_and_add_to_q((accel, gyro), self.measurement_q)
                elif msg_type_class == MsgType.STATE_EST_COV:
                    state, cov = parse_state_est_cov(data_bytes)
                    check_and_add_to_q((state, cov), self.state_cov_q)
                elif msg_type_class == MsgType.DEBUG_STR:
                    msg = parse_str(data_bytes)
                    check_and_add_to_q((msg,), self.debug_msg_q)
                    print(msg)

    def __configure_serial_port(self):
        self.__serial_comm.baudrate = BAUD_RATE
        self.__serial_comm.port = self.port
        self.__serial_comm.timeout = self.timeout

    def __start_communication(self):
        self.__serial_comm.open()
        self.__serial_thread.start()

    def start(self):
        self.__start_communication()

    def stop(self):
        self.__exit_flag = True
        self.__serial_thread.join()
        self.__serial_comm.close()




if __name__ == "__main__":
    comms = SerialCommunicator()

    comms.start()

    start_time = time.time()

    print("starting")

    while time.time() - start_time < 20:

        time.sleep(0.5)
    comms.stop()