import serial

class qcb_debug:

    def __init__(self, serial_port='/dev/serial/by-id/usb-STMicroelectronics_STM32_STLink_0670FF515754888367133041-if02'):

        self.ser = serial.Serial(serial_port, 115200, timeout=0.4)

    def _send_command(self, command):
        string = command + '\0' * (25 - len(command))
        self.ser.write(string.encode('ascii'))

    def mbl_pwr(self, state):
        if state == 'on':
            self._send_command('mblon')
        else:
            self._send_command('mbloff')

    def mbl_set_lvl(self, num):
        if num < 4 and num > 1:
            self._send_command('mblsal {}'.format(num))

    def mbl_set_temp(self, temp):
        self._send_command('mblsdt {:.3f}'.format(temp))

    def mbl_setup(self):
        self._send_command('5v_on')
        self.mbl_pwr('on')
        self.mbl_set_lvl(3)

    def mbl_set_DAC(self, value):
        self._send_command("mblsdv {}".format(value))

