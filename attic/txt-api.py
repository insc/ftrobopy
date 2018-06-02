from __future__ import print_function
import threading
import struct
import time
import binascii
from math import sqrt, log


def tracing(F):
    def wrapper(*args):
        print("\nCall method '{0}'".format(F.__name__))
        F(*args)
        print("End '{0}'\n".format(F.__name__))
    return wrapper

class ftTXT(object):

    C_VOLTAGE = 0
    C_SWITCH = 1
    C_RESISTOR = 1
    C_ULTRASONIC = 3
    C_ANALOG = 0
    C_DIGITAL = 1
    C_OUTPUT = 0
    C_MOTOR = 1

    # command codes for TXT motor shield
    C_MOT_CMD_CONFIG_IO = 0x51
    C_MOT_CMD_EXCHANGE_DATA = 0x54

    # input configuration codes for TXT motor shield
    C_MOT_INPUT_DIGITAL_VOLTAGE = 0
    C_MOT_INPUT_DIGITAL_5K = 1
    C_MOT_INPUT_ANALOG_VOLTAGE = 2
    C_MOT_INPUT_ANALOG_5K = 3
    C_MOT_INPUT_ULTRASONIC = 4

    def __init__(self):
        print('Init ftTXT' )
        self._ser_port = '/dev/ttyO2'
        import serial
        self._ser_ms = serial.Serial(self._ser_port, 230000, timeout=1)

        self._txt_stop_event = threading.Event()
        self._txt_stop_event.set()
        self._exchange_data_lock = threading.RLock()
        self._txt_thread = None
        self._update_status = 0
        self._cycle_count = 0
        self._config_id = 0
        self._config_id_old = 0
        self._m_extension_id = 0
        self._ftX1_pgm_state_req = 0
        self._ftX1_old_FtTransfer = 0
        self._ftX1_dummy = b'\x00\x00'
        self._ftX1_motor = [1, 1, 1, 1]
        self._ftX1_uni = [1, 1, b'\x00\x00',
                          1, 1, b'\x00\x00',
                          1, 1, b'\x00\x00',
                          1, 1, b'\x00\x00',
                          1, 1, b'\x00\x00',
                          1, 1, b'\x00\x00',
                          1, 1, b'\x00\x00',
                          1, 1, b'\x00\x00']
        self._ftX1_cnt = [1, b'\x00\x00\x00',
                          1, b'\x00\x00\x00',
                          1, b'\x00\x00\x00',
                          1, b'\x00\x00\x00']
        self._ftX1_motor_config = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        self._exchange_data_lock.acquire()
        self._pwm = [0, 0, 0, 0, 0, 0, 0, 0]
        self._motor_sync = [0, 0, 0, 0]
        self._motor_dist = [0, 0, 0, 0]
        self._motor_cmd_id = [0, 0, 0, 0]
        self._counter = [0, 0, 0, 0]
        self._current_input = [0, 0, 0, 0, 0, 0, 0, 0]
        self._current_counter = [0, 0, 0, 0]
        self._current_counter_value = [0, 0, 0, 0]
        self._current_counter_cmd_id = [0, 0, 0, 0]
        self._current_motor_cmd_id = [0, 0, 0, 0]
        self._current_ir = range(26)
        self._ir_current_ljoy_left_right = [0, 0, 0, 0, 0]  # -15 ... 15
        self._ir_current_ljoy_up_down = [0, 0, 0, 0, 0]  # -15 ... 15
        self._ir_current_rjoy_left_right = [0, 0, 0, 0, 0]  # -15 ... 15
        self._ir_current_rjoy_up_down = [0, 0, 0, 0, 0]  # -15 ... 15
        self._ir_current_buttons = [0, 0, 0, 0, 0]  # 0:OFF 1:ON
        # 0:all 1:0-0 2:1-0 3:0-1 4:1-1
        self._ir_current_dip_switch = [0, 0, 0, 0, 0]
        self._current_power = 0  # voltage of battery or power supply
        self._current_temperature = 0  # temperature of ARM CPU
        self._current_reference_power = 0
        self._current_extension_power = 0
        self._debug = []
        self._exchange_data_lock.release()


    def startOnline(self):
        print('Start online')
        if self._txt_stop_event.isSet():
            self._txt_stop_event.clear()
        if self._txt_thread is None:
            self._txt_thread = ftTXTexchange(
                txt=self, sleep_between_updates=0.02, stop_event=self._txt_stop_event)
            self._txt_thread.setDaemon(True)
            self._txt_thread.start()
        return None


    def stopOnline(self):
        print('Stop online')
        self._txt_stop_event.set()
        self._txt_thread = None
        return None


    def setConfig(self, M, I):
        print('setConfig')
        self._config_id += 1
        # Configuration of motors
        # 0=single output O1/O2
        # 1=motor output M1
        # self.ftX1_motor          = [M[0],M[1],M[2],M[3]]  # BOOL8[4]
        self._ftX1_motor = M
        # Universal input mode, see enum InputMode:
        # MODE_U=0
        # MODE_R=1
        # MODE_R2=2
        # MODE_ULTRASONIC=3
        # MODE_INVALID=4
        # print("setConfig I=", I)
        self._ftX1_uni = [I[0][0], I[0][1], b'\x00\x00',
                          I[1][0], I[1][1], b'\x00\x00',
                          I[2][0], I[2][1], b'\x00\x00',
                          I[3][0], I[3][1], b'\x00\x00',
                          I[4][0], I[4][1], b'\x00\x00',
                          I[5][0], I[5][1], b'\x00\x00',
                          I[6][0], I[6][1], b'\x00\x00',
                          I[7][0], I[7][1], b'\x00\x00']
        return None

    def getConfig(self):
        m = self._ftX1_motor
        i = self._ftX1_uni
        ii = [(i[0], i[1]), (i[3], i[4]), (i[6], i[7]), (i[9], i[10]),
              (i[12], i[13]), (i[15], i[16]), (i[18], i[19]), (i[21], i[22])]
        return m, ii

    def incrMotorCmdId(self, idx):
        self._exchange_data_lock.acquire()
        self._motor_cmd_id[idx] += 1
        self._motor_cmd_id[idx] &= 0x07
        self._exchange_data_lock.release()
        return None

    def getMotorCmdId(self, idx=None):
        if idx != None:
            ret = self._motor_cmd_id[idx]
        else:
            ret = self._motor_cmd_id
        return ret

    def incrCounterCmdId(self, idx):
        self._exchange_data_lock.acquire()
        self._counter[idx] += 1
        self._counter[idx] &= 0x07
        self._exchange_data_lock.release()
        return None

    def getCounterCmdId(self, idx=None):
        if idx != None:
            ret = self._counter[idx]
        else:
            ret = self._counter
        return ret

    def setPwm(self, idx, value):
        self._exchange_data_lock.acquire()
        print('set pwm ', idx, " : ", value)
        self._pwm[idx] = value
        self._exchange_data_lock.release()
        return None

    def stopAll(self):
        for i in range(8):
            self.setPwm(i, 0)
        return

    def getPwm(self, idx=None):
        if idx != None:
            ret = self._pwm[idx]
        else:
            ret = self._pwm
        return ret

    def setMotorSyncMaster(self, idx, value):
        self._exchange_data_lock.acquire()
        self._motor_sync[idx] = value
        self._exchange_data_lock.release()
        return None

    def getMotorSyncMaster(self, idx=None):
        if idx != None:
            ret = self._motor_sync[idx]
        else:
            ret = self._motor_sync
        return ret

    def setMotorDistance(self, idx, value):
        self._exchange_data_lock.acquire()
        self._motor_dist[idx] = value
        self._exchange_data_lock.release()
        return None

    def getMotorDistance(self, idx=None):
        if idx != None:
            ret = self._motor_dist[idx]
        else:
            ret = self._motor_dist
        return ret

    def getCurrentInput(self, idx=None):
        if idx != None:
            ret = self._current_input[idx]
        else:
            ret = self._current_input
        return ret

    def getCurrentCounterInput(self, idx=None):
        if idx != None:
            ret = self._current_counter[idx]
        else:
            ret = self._current_counter
        return ret

    def getCurrentCounterValue(self, idx=None):
        if idx != None:
            ret = self._current_counter_value[idx]
        else:
            ret = self._current_counter_value
        return ret

    def getCurrentCounterCmdId(self, idx=None):
        if idx != None:
            ret = self._current_counter_cmd_id[idx]
        else:
            ret = self._current_counter_cmd_id
        return ret

    def getCurrentMotorCmdId(self, idx=None):
        if idx != None:
            ret = self._current_motor_cmd_id[idx]
        else:
            ret = self._current_motor_cmd_id
        return ret


    def getPower(self):
        return self._current_power


    def getTemperature(self):
        return self._current_temperature


    def getReferencePower(self):
        return self._current_reference_power


    def getExtensionPower(self):
        return self._current_extension_power


    def SyncDataBegin(self):
        self._exchange_data_lock.acquire()

    def SyncDataEnd(self):
        self._exchange_data_lock.release()

    def updateWait(self, minimum_time=0.001):
        self._exchange_data_lock.acquire()
        self._update_status = 0
        self._exchange_data_lock.release()
        while self._update_status == 0:
            time.sleep(minimum_time)


class ftTXTexchange(threading.Thread):

    def __init__(self, txt, sleep_between_updates, stop_event):
        threading.Thread.__init__(self)
        self._txt = txt
        self._txt_sleep_between_updates = sleep_between_updates
        self._txt_stop_event = stop_event
        self._txt_interval_timer = time.time()
        return

    def run(self):
        while not self._txt_stop_event.is_set():
            if (self._txt_sleep_between_updates > 0):
                time.sleep(self._txt_sleep_between_updates)
                print('ftTXTexchange.run sleep')

            self._txt._cycle_count += 1
            if self._txt._cycle_count > 15:
                self._txt._cycle_count = 0

            self._txt._exchange_data_lock.acquire()

            if self._txt._config_id != self._txt._config_id_old:
                self._txt._config_id_old = self._txt._config_id
                #
                # at first, transfer i/o config data from TXT to motor shield
                # (this is only necessary, if config data has been changed, e.g. the config_id number has been increased)
                #
                fields = []
                fmtstr = '<BBB BBBB H BBBBBB'
                # fmtstr = '<' # little endian
                fields.append(ftTXT.C_MOT_CMD_CONFIG_IO)
                # cycle counter of transmitted and received data have to match (not yet checked here yet !)
                fields.append(self._txt._cycle_count)
                fields.append(0)  # only master
                # fmtstr += 'BBB'
                inp = [0, 0, 0, 0]
                for k in range(8):
                    mode = self._txt._ftX1_uni[k * 3]
                    digital = self._txt._ftX1_uni[k * 3 + 1]
                    if (mode, digital) == (ftTXT.C_SWITCH, ftTXT.C_DIGITAL):  # ftrobopy.input
                        # digital switch with 5k pull up
                        direct_mode = ftTXT.C_MOT_INPUT_DIGITAL_5K
                        # is 0 if voltage over pull up is < 1600 mV (switch closed) else 1 (switch open)
                    elif (mode, digital) == (ftTXT.C_VOLTAGE, ftTXT.C_DIGITAL):  # currently not used in ftrobopy
                        # digital voltage is 1 if Input > 600 mV else 0
                        direct_mode = ftTXT.C_MOT_INPUT_DIGITAL_VOLTAGE
                    elif (mode, digital) == (ftTXT.C_RESISTOR, ftTXT.C_ANALOG):  # ftrobopy.resistor
                        # analog resistor with 5k pull up [0 - 15K Ohm]
                        direct_mode = ftTXT.C_MOT_INPUT_ANALOG_5K
                        # unit of return value is [Ohm]
                    elif (mode, digital) == (ftTXT.C_VOLTAGE, ftTXT.C_ANALOG):  # ftrobopy.voltage
                        # analog voltage [5 mV - 10V]
                        direct_mode = ftTXT.C_MOT_INPUT_ANALOG_VOLTAGE
                        # bit in response[4] for digital input is also set to 1 if value > 600 mV else 0
                    elif mode == ftTXT.C_ULTRASONIC:  # ftrobopy.ultrasonic
                        # ultrasonic for both C_ANALOG and C_DIGITAL
                        direct_mode = ftTXT.C_MOT_INPUT_ULTRASONIC
                    else:
                        # fall back to default case
                        direct_mode = ftTXT.C_MOT_INPUT_ANALOG_VOLTAGE

                    inp[int(k / 2)] |= (direct_mode & 0x0F) << (4 * (k % 2))
                fields.append(inp[0])
                fields.append(inp[1])
                fields.append(inp[2])
                fields.append(inp[3])
                # fmtstr += 'BBBB'
                fields.append(0)  # CRC (not used ?)
                # fmtstr += 'H'
                fields.append(0)
                fields.append(0)
                fields.append(0)
                fields.append(0)
                fields.append(0)
                fields.append(0)
                # fmtstr += 'BBBBBB' # dummy bytes to fill up structure to 15 bytes in total
                buflen = struct.calcsize(fmtstr)
                buf = struct.pack(fmtstr, *fields)
                print('IO buf w>', binascii.hexlify(buf))
                self._txt._ser_ms.write(buf)
                data = self._txt._ser_ms.read(len(buf))
                print('IO data r>', binascii.hexlify(data))

            #
            # transfer parameter data from TXT to motor shield
            #
            fields = []
            fmtstr = '<BBBB BBBBBBBB BB BBBB HHHH BBBB BBBBBBBBBBBB H'
            # fmtstr = '<' # little endian
            fields.append(ftTXT.C_MOT_CMD_EXCHANGE_DATA)
            fields.append(0)  # number of bytes to transfer will be set below
            fields.append(self._txt._cycle_count)
            # bit pattern of connected txt extension modules, 0 = only master
            fields.append(0)
            # fmtstr += 'BBBB'

            # pwm data
            #
            for k in range(8):
                if self._txt._pwm[k] == 512:
                    pwm = 255
                else:
                    pwm = int(self._txt._pwm[k] / 2)
                fields.append(pwm)
                # fmtstr += 'B'

            # synchronization data (for encoder motors)
            #
            # low byte: M1:0000 M2:0000, high byte: M3:0000 M4:0000
            # Mx = 0000      : no synchronization
            # Mx = 1 - 4     : synchronize to motor n
            # Mx = 5 - 8     : "error injection" into synchronization to allow for closed loops (together with distance values)
            S = self._txt.getMotorSyncMaster()
            sync_low = (S[0] & 0x0F) | ((S[1] & 0x0F) << 4)
            sync_high = (S[2] & 0x0F) | ((S[3] & 0x0F) << 4)
            fields.append(sync_low)
            fields.append(sync_high)
            # fmtstr += 'BB'

            # cmd id data
            #
            # "counter reset cmd id" (bits 0-2) of 4 counters and "motor cmd id" (bits 0-2) of 4 motors
            # are packed into 3 bytes + 1 reserve byte = 1 32bit unsigned integer
            # lowest byte  : c3 c3 c2 c2 c2 c1 c1 c1 (bit7 .. bit0)
            # next byte    : m2 m1 m1 m1 c4 c4 c4 c3 (bit7 .. bit0)
            # next byte    : m4 m4 m4 m3 m3 m3 m2 m2 (bit7 .. bit 0)
            # highest byte : 00 00 00 00 00 00 00 00 (reserved byte)
            M = self._txt.getMotorCmdId()
            C = self._txt.getCounterCmdId()
            b0 = C[0] & 0x07
            b0 |= (C[1] & 0x07) << 3
            b0 |= (C[2] & 0x03) << 6
            b1 = (C[2] & 0x04) >> 2
            b1 |= (C[3] & 0x07) << 1
            b1 |= (M[0] & 0x07) << 4
            b1 |= (M[1] & 0x01) << 7
            b2 = (M[1] & 0x06) >> 1
            b2 |= (M[2] & 0x07) << 2
            b2 |= (M[3] & 0x07) << 5
            fields.append(b0)
            fields.append(b1)
            fields.append(b2)
            fields.append(0)
            # fmtstr += 'BBBB'

            # distance counters
            #
            D = self._txt.getMotorDistance()
            fields.append(D[0])  # distance counter 1
            fields.append(D[1])  # distance counter 2
            fields.append(D[2])  # distance counter 3
            fields.append(D[3])  # distance counter 4
            # fmtstr += 'HHHH'

            # reserve bytes
            #
            fields.append(0)
            fields.append(0)
            fields.append(0)
            fields.append(0)
            # fmtstr += 'BBBB'

            # more filler bytes
            #
            # the length of the transmitted data block (from the txt to the motor shield)
            # has to be at least as large as the length of the expected data block
            # (the answer of the motor shield will never be longer than the initial send)
            for k in range(12):
                fields.append(0)
                # fmtstr += 'B'

            # crc
            #
            # it seems that the crc is not used on the motor shield
            fields.append(0)
            # fmtstr += 'H'

            buflen = struct.calcsize(fmtstr)
            fields[1] = buflen
            buf = struct.pack(fmtstr, *fields)
            print('DATA buf len', buflen, len(buf))
            print('DATA buf w>', binascii.hexlify(buf))
            self._txt._ser_ms.write(buf)
            data = self._txt._ser_ms.read(len(buf))
            print('DATA data len', len(data))
            print('DATA data r>', binascii.hexlify(data))
            for ind in range(24):
                print('_ftX1_uni', ind, self._txt._ftX1_uni[ind])
            # the answer of the motor shield has the following format
            #
            # fmtstr  = '<'
            # fmtstr += 'B'    # [0]     command code
            # fmtstr += 'B'    # [1]     length of data block
            # fmtstr += 'B'    # [2]     cycle counter
            # fmtstr += 'B'    # [3]     bit pattern of connected txt extension modules, 0 = only master
            # fmtstr += 'B'    # [4]     digital input bits
            # fmtstr += 'BBBB' # [5:9]   analog inputs I1-I4 bits 0-7
            # fmtstr += 'BBB'  # [9:12]  analog inputs I1-I4 bits 8-13 : 22111111 33332222 44444433  |  44444433 33332222 22111111
            # fmtstr += 'BBBB' # [12:16] analog inputs I5-I8 bits 0-7
            # fmtstr += 'BBB'  # [16:19] analog inputs I5-I8 bits 8-13 : 66555555 77776666 88888877  |  88888877 77776666 66555555
            # fmtstr += 'B'    # [19]    voltage power supply analog bits 0-7
            # fmtstr += 'B'    # [20]    temperature analog bits 0-7
            # fmtstr += 'B'    # [21]    pwr and temp bits 8-12: ttpp pppp
            # fmtstr += 'B'    # [22]    reference voltage analog bits 0-7
            # fmtstr += 'B'    # [23]    extension voltage VBUS analog bits 0-7
            # fmtstr += 'B'    # [24]    ref and ext analog bits 8-12 : eeee rrrr
            # fmtstr += 'B'    # [25]    bit pattern of fast counters (bit0=C1 .. bit3=C2, bit4-7 not used)
            #         specifies, if fast counter value changed since last data exchange
            # fmtstr += 'H'    # [26]    counter 1 value
            # fmtstr += 'H'    # [27]    counter 2 value
            # fmtstr += 'H'    # [28]    counter 3 value
            # fmtstr += 'H'    # [29]    counter 4 value
            # fmtstr += 'B'    # [30]    ir byte 0
            # fmtstr += 'B'    # [31]    ir byte 1
            # fmtstr += 'B'    # [32]    ir byte 2
            # fmtstr += 'B'    # [33]    (?)
            # fmtstr += 'B'    # [34]    motor cmd id
            # fmtstr += 'B'    # [35]    motor cmd id and counter reset cmd id
            # fmtstr += 'B'    # [36]    counter reset cmd id
            # fmtstr += 'B'    # [37]    reserve byte 1
            # fmtstr += 'BB'   # [38:39] 2 byte crc (not used)

            fmtstr = '<BBBBB BBBB BBB BBBB BBB BBBBBBB HHHH BBBBBBBB BB'

            if len(data) == struct.calcsize(fmtstr):
                response = struct.unpack(fmtstr, data)
            else:
                response = ['i', [0] * len(data)]

            #
            # convert received data and write to ftrobopy data structures
            #

            # inputs
            #
            print("ftTXTexchange.run -> getConfig")
            m, i = self._txt.getConfig()
            for k in range(8):
                if i[k][1] == ftTXT.C_DIGITAL:
                    if response[4] & (1 << k):
                        self._txt._current_input[k] = 1
                    else:
                        self._txt._current_input[k] = 0
                else:
                    if k == 0:
                        self._txt._current_input[k] = response[5] + \
                                                      256 * (response[9] & 0x3F)
                    elif k == 1:
                        self._txt._current_input[k] = response[6] + 256 * \
                                                      (((response[9] >> 6) & 0x03) + ((response[10] << 2) & 0x3C))
                    elif k == 2:
                        self._txt._current_input[k] = response[7] + 256 * \
                                                      (((response[10] >> 4) & 0x0F) + ((response[11] << 4) & 0x30))
                    elif k == 3:
                        self._txt._current_input[k] = response[8] + \
                                                      256 * ((response[11] >> 2) & 0x3F)
                    elif k == 4:
                        self._txt._current_input[k] = response[12] + \
                                                      256 * (response[16] & 0x3F)
                    elif k == 5:
                        self._txt._current_input[k] = response[13] + 256 * \
                                                      (((response[16] >> 6) & 0x03) + ((response[17] << 2) & 0x3C))
                    elif k == 6:
                        self._txt._current_input[k] = response[14] + 256 * \
                                                      (((response[17] >> 4) & 0x0F) + ((response[18] << 4) & 0x30))
                    elif k == 7:
                        self._txt._current_input[k] = response[15] + \
                                                      256 * ((response[18] >> 2) & 0x3F)

            # power (of battery and/or main power supply) in volt and internal TXT temperature
            #
            self._txt._current_power = response[19] + 256 * (response[21] & 0x3F)
            self._txt._current_temperature = response[20] + \
                                             256 * ((response[21] >> 6) & 0x03)

            # reference voltage and extension voltage
            #
            self._txt._current_reference_power = response[22] + 256 * (
                    response[24] & 0x0F)
            self._txt._current_extension_power = response[23] + 256 * (
                    (response[24] >> 4) & 0x0F)

            # signals which fast counters did change since last data exchange
            #
            for k in range(4):
                if response[25] & (1 << k):
                    self._txt._current_counter[k] = 1
                else:
                    self._txt._current_counter[k] = 0
            self._txt.debug = response[25]

            # current values of fast counters
            #
            self._txt._current_counter_value = response[26:30]

            # - ir data: response[30:33]
            #
            # ir remote 0 (any)

            self._txt._ir_current_buttons[0] = (response[30] >> 4) & 0x03
            self._txt._ir_current_dip_switch[0] = (response[30] >> 6) & 0x03
            if response[30] & 0x01:
                self._txt._ir_current_rjoy_left_right[0] = response[31] & 0x0F
            else:
                self._txt._ir_current_rjoy_left_right[0] = -(response[31] & 0x0F)
            if response[30] & 0x02:
                self._txt._ir_current_rjoy_up_down[0] = (response[31] >> 4) & 0x0F
            else:
                self._txt._ir_current_rjoy_up_down[0] = -((response[31] >> 4) & 0x0F)
            if response[30] & 0x04:
                self._txt._ir_current_ljoy_left_right[0] = response[32] & 0x0F
            else:
                self._txt._ir_current_ljoy_left_right[0] = -(response[32] & 0x0F)
            if response[30] & 0x08:
                self._txt._ir_current_ljoy_up_down[0] = (response[32] >> 4) & 0x0F
            else:
                self._txt._ir_current_ljoy_up_down[0] = -((response[32] >> 4) & 0x0F)
            # ir remote 1-4 ( = copy of ir remote 0)
            irNr = ((response[30] >> 6) & 0x03) + 1
            self._txt._ir_current_buttons[irNr] = self._txt._ir_current_buttons[0]
            self._txt._ir_current_dip_switch[irNr] = self._txt._ir_current_dip_switch[0]
            self._txt._ir_current_rjoy_left_right[irNr] = self._txt._ir_current_rjoy_left_right[0]
            self._txt._ir_current_rjoy_up_down[irNr] = self._txt._ir_current_rjoy_up_down[0]
            self._txt._ir_current_ljoy_left_right[irNr] = self._txt._ir_current_ljoy_left_right[0]
            self._txt._ir_current_ljoy_up_down[irNr] = self._txt._ir_current_ljoy_up_down[0]

            # current values of motor cmd id and counter reset id
            #
            # packed into 3 bytes
            # lowest byte  : c3 c3 c2 c2 c2 c1 c1 c1 (bit7 .. bit0)
            # next byte    : m2 m1 m1 m1 c4 c4 c4 c3 (bit7 .. bit0)
            # next byte    : m4 m4 m4 m3 m3 m3 m2 m2 (bit7 .. bit 0)

            b0 = response[34]
            b1 = response[35]
            b2 = response[36]
            self._txt._debug = [b0, b1, b2]
            # get pointers to current counter and motor cmd id data structures
            cC = self._txt.getCurrentCounterCmdId()
            cM = self._txt.getCurrentMotorCmdId()
            cC[0] = b0 & 0x07
            cC[1] = (b0 >> 3) & 0x07
            cC[2] = (b0 >> 6) & 0x03 | (b1 << 2) & 0x04
            cC[3] = (b1 >> 1) & 0x07
            cM[0] = (b1 >> 4) & 0x07
            cM[1] = (b1 >> 7) & 0x01 | (b2 << 1) & 0x06
            cM[2] = (b2 >> 2) & 0x07
            cM[3] = (b2 >> 5) & 0x07

            self._txt._update_status = 1
            self._txt._exchange_data_lock.release()



class ftrobopy(ftTXT):

    def __init__(self):
        ftTXT.__init__(self)
        self._txt_is_initialzed = True
        print('Connected to txt.')
        for i in range(8):
            self.setPwm(i, 0)
        self.startOnline()

    def __del__(self):
        if self._txt_is_initialized:
            self.stopOnline()
            if self._ser_ms:
                self._ser_ms.close()

    def motor(self, output, wait=True):

        class mot(object):
            def __init__(self, outer, output):
                self._outer = outer
                self._output = output
                self._speed = 0
                self._distance = 0
                self._outer._exchange_data_lock.acquire()
                self.setSpeed(0)
                self.setDistance(0)
                self._outer._exchange_data_lock.release()

            def setSpeed(self, speed):
                self._outer._exchange_data_lock.acquire()
                self._speed = speed
                if speed > 0:
                    self._outer.setPwm((self._output - 1) * 2, self._speed)
                    self._outer.setPwm((self._output - 1) * 2 + 1, 0)
                else:
                    self._outer.setPwm((self._output - 1) * 2, 0)
                    self._outer.setPwm((self._output - 1) * 2 + 1, -self._speed)
                self._outer._exchange_data_lock.release()

            def setDistance(self, distance, syncto=None):
                self._outer._exchange_data_lock.acquire()
                if syncto:
                    self._distance = distance
                    syncto._distance = distance
                    self._command_id = self._outer.getCurrentMotorCmdId(self._output - 1)
                    syncto._command_id = syncto._outer.getCurrentMotorCmdId(
                        self._output - 1)
                    self._outer.setMotorDistance(self._output - 1, distance)
                    self._outer.setMotorDistance(syncto._output - 1, distance)
                    self._outer.setMotorSyncMaster(self._output - 1, syncto._output)
                    self._outer.setMotorSyncMaster(syncto._output - 1, self._output)
                    self._outer.incrMotorCmdId(self._output - 1)
                    self._outer.incrMotorCmdId(syncto._output - 1)
                else:
                    self._distance = distance
                    self._command_id = self._outer.getCurrentMotorCmdId(self._output - 1)
                    self._outer.setMotorDistance(self._output - 1, distance)
                    self._outer.setMotorSyncMaster(self._output - 1, 0)
                    self._outer.incrMotorCmdId(self._output - 1)
                self._outer._exchange_data_lock.release()

            def finished(self):
                if self._outer.getMotorCmdId(self._output - 1) == self._outer.getCurrentMotorCmdId(self._output - 1):
                    return True
                else:
                    return False

            def getCurrentDistance(self):
                return self._outer.getCurrentCounterValue(idx=self._output - 1)

            def stop(self):
                self._outer._exchange_data_lock.acquire()
                self.setSpeed(0)
                self.setDistance(0)
                self._outer._exchange_data_lock.release()

        print('getConfig-> motor ', output)
        M, I = self.getConfig()
        M[output - 1] = ftTXT.C_MOTOR
        self.setConfig(M, I)
        if wait:
            self.updateWait()
        return mot(self, output)

    def output(self, num, level=0, wait=True):

        class out(object):
            def __init__(self, outer, num, level):
                self._outer = outer
                self._num = num
                self._level = level

            def setLevel(self, level):
                self._level = level
                self._outer._exchange_data_lock.acquire()
                self._outer.setPwm(num - 1, self._level)
                self._outer._exchange_data_lock.release()

        print('getConfig-> output ', num, ' level ', level)
        M, I = self.getConfig()
        M[int((num - 1) / 2)] = ftTXT.C_OUTPUT
        self.setConfig(M, I)
        if  wait:
            self.updateWait()
        return out(self, num, level)

    def input(self, num, wait=True):

        class inp(object):
            def __init__(self, outer, num):
                self._outer = outer
                self._num = num

            def state(self):
                return self._outer.getCurrentInput(num - 1)

        print('getConfig-> input ', num)
        M, I = self.getConfig()
        I[num - 1] = (ftTXT.C_SWITCH, ftTXT.C_DIGITAL)
        self.setConfig(M, I)
        if  wait:
            self.updateWait()
        return inp(self, num)

    def resistor(self, num, wait=True):

        class inp(object):
            def __init__(self, outer, num):
                self._outer = outer
                self._num = num

            def value(self):
                return self._outer.getCurrentInput(num - 1)

            def ntcTemperature(self):
                r = self.value()
                if r != 0:
                    x = log(self.value())
                    y = x * x * 1.39323522
                    z = x * -43.9417405
                    T = y + z + 271.870481
                else:
                    T = 10000
                return T

        print('getConfig-> resistor ', num)
        M, I = self.getConfig()
        I[num - 1] = (ftTXT.C_RESISTOR, ftTXT.C_ANALOG)
        self.setConfig(M, I)
        if wait:
            self.updateWait()
        return inp(self, num)

    def ultrasonic(self, num, wait=True):

        class inp(object):
            def __init__(self, outer, num):
                self._outer = outer
                self._num = num

            def distance(self):
                return self._outer.getCurrentInput(num - 1)

        print('getConfig-> ultrasonic ', num)
        M, I = self.getConfig()
        I[num - 1] = (ftTXT.C_ULTRASONIC, ftTXT.C_ANALOG)
        self.setConfig(M, I)
        if  wait:
            self.updateWait()
        return inp(self, num)

    def voltage(self, num, wait=True):
        class inp(object):
            def __init__(self, outer, num):
                self._outer = outer
                self._num = num

            def voltage(self):
                return self._outer.getCurrentInput(num - 1)

        print('getConfig-> voltage ', num)
        M, I = self.getConfig()
        I[num - 1] = (ftTXT.C_VOLTAGE, ftTXT.C_ANALOG)
        self.setConfig(M, I)
        if  wait:
            self.updateWait()
        return inp(self, num)

    def colorsensor(self, num, wait=True):
        class inp(object):
            def __init__(self, outer, num):
                self._outer = outer
                self._num = num

            def value(self):
                return self._outer.getCurrentInput(num - 1)

            def color(self):
                c = self._outer.getCurrentInput(num - 1)
                if c < 200:
                    return 'weiss'
                elif c < 1000:
                    return 'rot'
                else:
                    return 'blau'

        print('getConfig-> colorsensor ', num)
        M, I = self.getConfig()
        I[num - 1] = (ftTXT.C_VOLTAGE, ftTXT.C_ANALOG)
        self.setConfig(M, I)
        if  wait:
            self.updateWait()
        return inp(self, num)

    def trailfollower(self, num, wait=True):
        class inp(object):
            def __init__(self, outer, num):
                self._outer = outer
                self._num = num

            def state(self):
                # in direct-mode digital 1 is set by motor-shield if voltage is > 600mV
                if self._outer.getCurrentInput(num - 1) == 1:
                    return 1
                else:
                    # threshold in mV between digital 0 and 1. Use voltage()-Function instead, if analog value of trailfollower is needed.
                    if self._outer.getCurrentInput(num - 1) > 600:
                        return 1
                    else:
                        return 0

        print('getConfig-> trailfollower ', num)
        M, I = self.getConfig()
        I[num - 1] = (ftTXT.C_VOLTAGE, ftTXT.C_DIGITAL)
        self.setConfig(M, I)
        if  wait:
            self.updateWait()
        return inp(self, num)

TXT = ftrobopy()

print("Start ...")

Motor_rechts = TXT.motor(1)
Motor_links = TXT.motor(2)
Ultraschall = TXT.ultrasonic(8)
print('Sleep start');
time.sleep(2)  # wait until initilized
print('Sleep end');
Motor_rechts.setSpeed(-512)
Motor_links.setSpeed(-512)
Motor_rechts.setDistance(1000, syncto=Motor_links)
Motor_links.setDistance(1000, syncto=Motor_rechts)

while not Motor_rechts.finished():
    d = Ultraschall.distance()
    print(d)
    if d < 10:
        Motor_rechts.stop()
        Motor_links.stop()
