import logging
import socket
import threading
import time

import msgpack

class RobotMonitorSocket(threading.Thread):
    def __init__(self, host_address: str, port: int):
        super().__init__()
        self.name = "RobotMontiorSocket"
        self.daemon = True
        self.logger = logging.getLogger(self.__class__.__name__)
        self.__host_address = host_address
        self.__port = port
        self._stop_event = True
        self._data_event = threading.Condition()
        self._data_access = threading.Lock()
        self._monitor_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._monitor_sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self._monitor_sock.connect((self.__host_address, self.__port))
        self._timestamp = None
        
        # Data Variables
        self._ctrl_time_stamp = None
        self._q_actual = None
        self._tcp_actual = None
        # self._io_status = None
        self._program_status = None
        self._program_state_id = None
        self._robot_mode = None
        self._robot_code = None
        self._last_ctrl_time_stamp = 0
        
        self.__recv_time = 0
        
    def __recv_bytes(self, nBytes):
        recv_time = 0
        data = b''
        while len(data) < nBytes:
            data += self._monitor_sock.recv(nBytes - len(data))
            if recv_time == 0:
                recv_time = time.time()
        self.__recv_time = recv_time
        return data
    
    def wait(self):
        with self._data_event:
            self._data_event.wait()
            
    def q_actual(self , wait=False):
        if wait:
            self.wait()
        with self._data_access:
            return self._q_actual
    getQActual = q_actual
        
    def tcp_actual(self, wait=False):
        if wait:
            self.wait()
        with self._data_access:
            return self._tcp_actual
    getTCPActual = tcp_actual
        
    # def io_status(self, wait=False):
    #     if wait:
    #         self.wait()
    #     with self._data_access:
    #         return self._io_status
    # getIOStatus = io_status
        
    def program_status(self,wait=False):
        if wait:
            self.wait()
        with self._data_access:
            return self._program_status
    getProgramStatus = program_status
        
    def program_state_id(self , wait=False):
        if wait:
            self.wait()
        with self._data_access:
            return self._program_state_id
    getProgramStateID = program_state_id
        
    def robot_mode(self , wait=False):
        if wait:
            self.wait()
        with self._data_access:
            return self._robot_mode
    getRobotMode = robot_mode
        
    def robot_code(self , wait=False):
        if wait:
            self.wait()
        with self._data_access:
            return self._robot_code
    getRobotCode = robot_code
        
    def __recv_monitor_data(self):
        header_bytes = self.__recv_bytes(4)
        timestamp = self.__recv_time
        packet_size = int.from_bytes(header_bytes, byteorder="big")
        self.logger.info('Received header telling that package is %s bytes long',packet_size)
        packet_payload = msgpack.unpackb(self.__recv_bytes(packet_size))
        with self._data_access:
            self._timestamp = timestamp
            self._ctrl_time_stamp = packet_payload[0]
            if self._last_ctrl_time_stamp != 0 and (
                    self._ctrl_time_stamp -
                    self._last_ctrl_time_stamp) > 0.1:
                self.logger.warn("Error the controller failed to send us a packet: time since last packet %s s ",
                    self._ctrl_time_stamp - self._last_ctrl_time_stamp)
            self._last_ctrl_time_stamp = self._ctrl_time_stamp
            self._q_actual = packet_payload[1]
            self._tcp_actual = packet_payload[2]
            # self._io_status = packet_payload[3] #Not Available
            self._program_status = packet_payload[4]
            self._program_state_id = packet_payload[5]
            # 
            self._robot_mode = packet_payload[7]
            self._robot_code = packet_payload[8]
        with self._data_event:
            self._data_event.notifyAll()
            
    def get_all_data(self, wait=True):
        if wait:
            self.wait()
        with self._data_access:
            return dict(
                timestamp=self._timestamp,
                ctrltimestamp=self._ctrl_time_stamp,
                q_actual = self._q_actual,
                tcp_actual =self._tcp_actual,
                # io_status = self._io_status,
                program_status = self._program_status,
                program_state_id = self._program_state_id,
                robot_mode = self._robot_mode,
                robot_code  = self._robot_code)
    getALLData = get_all_data

    def stop(self):
        self._stop_event = True

    def close(self):
        self.stop()
        self.join()

    def run(self):
        self._stop_event = False
        while not self._stop_event:
            self.__recv_monitor_data()
        self._monitor_sock.close()