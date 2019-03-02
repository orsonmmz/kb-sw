import serial
import struct
import time
import binascii     # TODO remove
import commands_def as cmd_defs

class KiconBadge:
    def __init__(self, port):
        self._serial = None
        self._serial = serial.Serial(port, 115200, timeout=0.1)   # TODO baud?

    def __del__(self):
        if self._serial:
            self._serial.close()

    def crc(self, data):
        crc = 0

        try:
            for c in data:
                crc = crc ^ c
        except TypeError:
            crc = data  # handle int

        return crc

    def make_resp(self, resp):
        return struct.pack('>BBB', 1, resp, self.crc(resp))

    def make_cmd(self, cmd, data=None):
        if data:
            data_bytes = bytes('%c%s' % (cmd, data), 'ascii')
            data_len = len(data_bytes)
            return struct.pack('>B%dsB' % data_len,    # struct format
                    data_len, data_bytes, self.crc(data_bytes))
        else:
            return struct.pack('>BBB', 1, cmd, self.crc(cmd))

    def check_resp(self):
        expected_resp = self.make_resp(cmd_defs.CMD_RESP_OK)
        actual_resp = self._serial.read(len(expected_resp))
        if expected_resp != actual_resp:
            raise Exception('Error! Response: %s' % str(binascii.hexlify(actual_resp), 'ascii'))

    def reset(self):
        # special command type, cannot be created with make_cmd
        reset_cmd = struct.pack('>B', cmd_defs.CMD_TYPE_RESET)
        reset_resp = self.make_resp(cmd_defs.CMD_RESP_RESET)

        self._serial.write(reset_cmd)
        time.sleep(0.1)
        self._serial.read(len(reset_resp))

        trials = 0
        while self._serial.read(len(reset_resp)) != reset_resp:
            self._serial.write(reset_cmd)
            trials = trials + 1
            time.sleep(0.1)

            if trials == 256:
                raise Exception('No response after reset')

    def uart_send(self, data):
        uart_cmd = self.make_cmd(cmd_defs.CMD_TYPE_UART, data)
        self._serial.write(uart_cmd)
        self.check_resp()
