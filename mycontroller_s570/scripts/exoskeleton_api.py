import socket
import threading
import serial

lock = threading.Lock()


class Exoskeleton:
    def __init__(self, port):
        self.ser = serial.Serial(port=port, baudrate=1000000)

    def _parse_data(self, data):
        parsed_data = []
        for i in range(7):
            data_h = data[0 + i * 4: 2 + i * 4]
            data_l = data[2 + i * 4: 4 + i * 4]
            encode = int(data_h + data_l, 16)
            angle = 0 if encode == 2048 else (180 * (encode - 2048) / 2048 if encode > 2048 else -180 * (2048 - encode) / 2048)
            parsed_data.append(round(angle, 2))

        button = bin(int(data[28: 30], 16))[2:].rjust(4, "0")
        parsed_data.extend([int(button[-4]), int(button[-1]), int(button[-3]), int(button[-2]), int(data[30: 32], 16), int(data[32: 34], 16)])
        return parsed_data

    def _commmon(self, command_array):
        with lock:
            commmon_id = command_array[3]
            self.ser.write(command_array)
            start1 = self.ser.read().hex()
            if start1 != "fe" or self.ser.read().hex() != "fe":
                return None
            data_len = int(self.ser.read().hex(), 16)
            count = self.ser.in_waiting
            if data_len == count:
                data = self.ser.read(count).hex()
                if data[-2:] == "fa" and int(data[0: 2], 6) == commmon_id:
                    return data[2: -2]
        return None

    def get_all_data(self):
        get_all_array = [0xFE, 0xFE, 0x02, 0x01, 0xFA]
        data = self._commmon(get_all_array)
        if data is None:
            return None
        left_data = self._parse_data(data)
        right_data = self._parse_data(data[34:])
        return [left_data, right_data]

    def get_arm_data(self, arm):
        if arm not in [1, 2]:
            raise ValueError("error arm")

        send_array = [0xFE, 0xFE, 0x03, 0x02, arm, 0xFA]
        data = self._commmon(send_array)
        if data is None:
            return None
        return self._parse_data(data)

    def get_joint_data(self, arm, id):
        if arm not in [1, 2] or id < 1 or id > 7:
            raise ValueError("error arm or id")

        send_array = [0xFE, 0xFE, 0x04, 0x03, arm, id, 0xFA]
        data = self._commmon(send_array)
        if data is None:
            return None
        encode = int(data[0: 2] + data[2: 4], 16)
        angle = 0 if encode == 2048 else (180 * (encode - 2048) / 2048 if encode > 2048 else -180 * (2048 - encode) / 2048)
        return round(angle, 2)

    def set_zero(self, arm, id):
        if arm not in [1, 2] or id < 1 or id > 7:
            raise ValueError("error arm or id")

        send_array = [0xFE, 0xFE, 0x04, 0x04, arm, id, 0xFA]
        with lock:
            self.ser.write(bytearray(send_array))

    def set_color(self, arm, red, green, blue):
        if arm not in [1, 2]:
            raise ValueError("error arm")
        send_array = [0xFE, 0xFE, 0x06, 0x05, arm, red, green, blue, 0xFA]
        with lock:
            self.ser.write(bytearray(send_array))


class ExoskeletonSocket:
    def __init__(self, ip='192.168.4.1', port=80):
        self.client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.client.connect((ip, port))

    def _commmon(self, command_array):
        with lock:
            commmon_id = command_array[3]
            self.client.sendall(bytearray(command_array))
            if self.client.recv(1).hex() != "fe" or self.client.recv(1).hex() != "fe":
                return None
            data_len = int(self.client.recv(1).hex(), 16) * 2
            data = self.client.recv(1024).hex()
            if len(data) == data_len and data[-2:] == "fa" and int(data[0: 2], 6) == commmon_id:
                return data[2: -2]
        return None

    def _parse_data(self, data):
        parsed_data = []
        for i in range(7):
            data_h = data[0 + i * 4: 2 + i * 4]
            data_l = data[2 + i * 4: 4 + i * 4]
            encode = int(data_h + data_l, 16)
            angle = 0 if encode == 2048 else (180 * (encode - 2048) / 2048 if encode > 2048 else -180 * (2048 - encode) / 2048)
            parsed_data.append(round(angle, 2))

        button = bin(int(data[28: 30], 16))[2:].rjust(4, "0")
        parsed_data.extend([int(button[-4]), int(button[-1]), int(button[-3]), int(button[-2]), int(data[30: 32], 16), int(data[32: 34], 16)])
        return parsed_data

    def get_all_data(self):
        get_all_array = [0xFE, 0xFE, 0x02, 0x01, 0xFA]
        data = self._commmon(get_all_array)
        if data is None:
            return None
        left_data = self._parse_data(data)
        right_data = self._parse_data(data[34:])
        return [left_data, right_data]

    def get_arm_data(self, arm):
        if arm not in [1, 2]:
            raise ValueError("error arm")

        send_array = [0xFE, 0xFE, 0x03, 0x02, arm, 0xFA]
        data = self._commmon(send_array)
        if data is None:
            return None
        return self._parse_data(data)

    def get_joint_data(self, arm, id):
        if arm not in [1, 2] or id < 1 or id > 7:
            raise ValueError("error arm or id")

        send_array = [0xFE, 0xFE, 0x04, 0x03, arm, id, 0xFA]
        data = self._commmon(send_array)
        if data is None:
            return None
        encode = int(data[0: 2] + data[2: 4], 16)
        angle = 0 if encode == 2048 else (180 * (encode - 2048) / 2048 if encode > 2048 else -180 * (2048 - encode) / 2048)
        return round(angle, 2)

    def set_zero(self, arm, id):
        if arm not in [1, 2] or id < 1 or id > 7:
            raise ValueError("error arm or id")

        send_array = [0xFE, 0xFE, 0x04, 0x04, arm, id, 0xFA]
        with lock:
            self.client.sendall(bytearray(send_array))

    def set_color(self, arm, red, green, blue):
        if arm not in [1, 2]:
            raise ValueError("error arm")
        send_array = [0xFE, 0xFE, 0x06, 0x05, arm, red, green, blue, 0xFA]
        with lock:
            self.client.sendall(bytearray(send_array))
