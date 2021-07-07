"""
Driver for APS100 Magnet Power Supply.
This power supplier applies voltage to the coil of superconducting wires to induce current flow through the coil. The current through the coil generates magnetic field in the centre. 
The conversion between the current and magnetic field is set as default_scale below (do not change it). When a constant magnetic field is to be 
used, the current through the magnet is to be constant. Becasue the current flows through the superconductor, we do not need to apply power to keep 
it. The magnet power supply can place the magnet into so called "persistent mode": it does not apply voltage to the magnet abd the current does not change.
Sometimes, the supeconducting wires stop being superconducting and a lot of heat is generated. This is not safe for the magnet and if it happens, the current must be decreased to zero. 
ANS100 detects this problem, called "a quench" and ramps the current to zero. As a user you need to then restart the magent.
 Please refer to Magnet Power Supply APS100 manual for further details and functionality.


All the default parameters should never be changed. The only variables are the magnetic field and the sweep rate.

How / when to detect quench?

"""

import visa
import logging
import time
from qcodes.utils.validators import Numbers, Enum
from qcodes.instrument.visa import VisaInstrument, DelayedKeyboardInterrupt
import pyvisa.constants as vi_const
import re


log = logging.getLogger(__name__)


class APS100(VisaInstrument):

    """
    The following hard-coded, default values for the magnet are safety limits
    and should not be modified.
    - these values should be set using the corresponding arguments when the class is called.
    """
    default_B_limit = 9.00  # [T]
    default_max_current_ramp_limit = 5 #[A/s]
    defalut_range_limit=[40,45,46,85,95]   #[A]
    default_rate_limit=[0.0356, 0.0178, 0.001, 0.0001, 0.0001]#[A/s]
    default_scale=0.1996301 #[T/A] 1996.301[G/A]

    """
    Driver for the APS100 magnet power supply.
    This class controls a single magnet PSU.
    Magnet and magnet PSU limits : max B=9T, I=???A, V=???V

    Args:
        name (str): a name for the instrument
        address (str): (serial to USB) COM number of the power supply
        coil_constant (float): coil constant in Tesla per ampere, fixed at 0.113375T/A  ???
        current_rating (float): maximum current rating in ampere, fixed at 105.84A ???
        current_ramp_limit (float): current ramp limit in ampere per second,     ???
            for 4K operation 0.12A/s (0.013605 T/s, 0.8163 T/min) - not recommended ???

    """

    # Reg. exp. to match a float or exponent in a string
    _re_float_exp = r'[-+]?(\d+(\.\d*)?|\.\d+)([eE][-+]?\d+)?'

    # Reg. exp. to match float + unit in a string
    _re_float_unit_exp = r'^({})\s?([a-zA-Z]*)$'.format(_re_float_exp)

    def __init__(self, name: str, address: str, **kwargs):

        log.debug('Initializing instrument')

        super().__init__(name, address, **kwargs)

        self.visa_handle.baud_rate = 19200
        self.visa_handle.parity = visa.constants.Parity.none
        self.visa_handle.stop_bits = visa.constants.StopBits.one
        self.visa_handle.data_bits = 8
        self.visa_handle.flow_control = 0
        self.visa_handle.write_termination = '\r'
        self.visa_handle.read_termination = '\r\n'
        self.visa_handle.flush(vi_const.VI_READ_BUF_DISCARD |
                               vi_const.VI_WRITE_BUF_DISCARD)  # keep for debugging

        # The following command already returns error because of the terminator but the above termination
        # is the only one that does not crush the communication. The solution is to requrest reply twice but I am not sure
        # if "get" command consists of both "write " and  "read"
        idn = self.IDN.get()
        print(idn)

        self.write('REMOTE')

        self.add_parameter('persistent_mode',
                           get_cmd='PSHTR?',
                           set_cmd=False,
                           val_mapping={'OFF': 0, 'ON': 1})

        self.add_parameter(name='B_field_now',
                           get_cmd='IMAG?',
                           get_parser=self._value_parser,
                           set_cmd='IMAG {}',
                           unit='A'
                           )

        self.add_parameter(name='units',
                           get_cmd='UNITS?',
                           set_cmd=self._set_units,
                           vals=Enum('A', 'kG')
                           )

        self.add_parameter('sweep',
                           get_cmd='SWEEP?',
                           set_cmd=self._set_sweep,
                           )

        self.add_parameter(name='sweep_rate',
                           get_cmd='RATE? 0',
                           set_cmd=False,
                           )

        self.add_parameter('pause_sweep',
                           set_cmd='SWEEP PAUSE',
                           )

        self.add_parameter('set_B',
                           set_cmd=self._set_B,
                           )

        self.add_parameter(name='reset_quench',
                           set_cmd='QRESET',)

        self.units('A')

    def ask_raw(self, cmd: str) -> str:
        """Override ask_raw to perform ``read`` and ``write`` separately instead of ``query``"""
        with DelayedKeyboardInterrupt():
            # Flush buffer, to remove old responses from device
            self.visa_handle.flush(vi_const.VI_READ_BUF_DISCARD)
            self.visa_log.debug(f"Querying: {cmd}")
            # Write command to instrument, like in ``self.write_raw``
            nr_bytes_written, ret_code = self.visa_handle.write(cmd)
            # Check return code
            self.check_error(ret_code)
            # Read first response (= previously written command)
            self.visa_handle.read()
            # Read actual response
            response = self.visa_handle.read()
            self.visa_log.debug(f"Response: {response}")
        return response

    def _value_parser(self, msg):
        match = re.match(self._re_float_unit_exp, msg)
        if not match:
            raise ValueError("Couldn't extract float from '{}'".format(msg))
        # If you need the unit, it's stored in match[5]
        return float(match[1])

    def _set_units(self, unit: str):
        # Sets the magnetic field unit (A or kG) for both the hardware and the B_field_now parameter
        self.write("UNITS {}".format(unit))
        self.B_field_now.unit = unit

    # I would like this function to set the magnetic field and sweep the field up (down) to that value.
    # We would like to read the values of magnetic field during the sweep so we could measure some sample parameters as a function of magnetic field.
    # I am not sure if reading the field is possible while the magnet power supply is changing the current that generates the field.
    # Below I assumed that this is the case.
    def _set_sweep(self,B_value:float):
        if B_value > default_B_limit:
            B_value=default_B_limit
        if B<-1*default_B_limit:
            B_value=-1*default_B_limit
            
        I_value = B_value/default_scale
        
        sweep_rate = self.ask('RATE?')
        ratge_value = sweep_rate.split(',')[0]
        #Making sure that the desired ramping rate is not faster than the value set in the magnet
        for i in range(0,4):
            if I_value <= default_range_limit[i]:
                if rate_value >default_rate_limit[i]:
                    rate_value =default_rate_limit[i] 
                self.write('RATE {} {}'.format(i,rate_value))
                break
        
        msg = self.ask('IMAG?')
        msg_1 = self.ask('IOUT?')
        if self.ask('PSHTR?')=='OFF':
            if (msg_1[0] < msg[0]): 
                self.write('SWEEP UP FAST')
                while msg_1[0] < msg[0]:
                    time.sleep(1)       
                    msg_1 = self.ask('IOUT?')
                    continue
            if (msg_1[0] >  msg[0]):    
                self.write('SWEEP DOWN FAST')
                while msg_1[0] > msg[0]:
                    time.sleep(1)       
                    msg_1 = self.ask('IOUT?')
                    continue
            msg_1 = self.ask('IOUT?')
            if msg_1[0]==msg[0]:
                self.write('PSHTR ON')
                time.sleep(20)       
        
        if I_value < msg:
            self.write('LLIM {}'.format(I_value))
            self.write('SWEEP DOWN SLOW')
        if I_value > msg:
            self.write('ULIM {}'.format(I_value))
            self.write('SWEEP UP SLOW')
        if I_value == msg:
            self.write('SWEEP ZERO SLOW')

    # This function is intended to set magnetic field (B_value) and leave it at the final value for the measurement. There is no need to read it for 
    # the experiment. I think I made the function too complex because I am checking there (all the time) if B_value is reached. Considering that reading from the magnet 
    # is strange it may be better to calculate the time it will take to reach the B_value and only then start checking. User does not need to know the intermediate values.
    # One more element of this function is putting the magnet into a persistent mode. Because the magnet is superconducting, once the magnetic field is reached and it is stable 
    # the magnet power supply (which we are programming) can stop applying current to the magnet. If we want to keep the magnetic field for many minutes or longer, it is best 
    # to do it by setting the persistent_mode on. If after setting this mode, we want to change the field, the magnet power supply needs to be taken from the persistent mode. 
    # I need to check how to do it safely.
    def _set_B(self,B_value:float):
        
        if B_value > default_B_limit:
            B_value=default_B_limit
        if B<-1*default_B_limit:
            B_value=-1*default_B_limit
            
        I_value = B_value/self._scale

        msg = self.ask('IMAG?')
        msg_1 = self.ask('IOUT ?')
        if self.ask('PSHTR ?')=='OFF':
            if (msg_1[0] <msg[0]): 
                self.write('SWEEP UP FAST')
                while msg_1[0] < msg[0]:
                    time.sleep(1)       
                    msg_1 = self.ask('IOUT ?')
                    continue
            if (msg_1[0] >  msg[0]):    
                self.write('SWEEP DOWN FAST')
                while msg_1[0] > msg[0]:
                    time.sleep(1)       
                    msg_1 = self.ask('IOUT ?')
                    continue
            msg_1 = self.ask('IOUT ?')
            if msg_1[0]==msg[0]:
                self.write('PSHTR ON')
                time.sleep(20)       
        
        if I_value < msg:
            self.write('LLIM {}'.format(I_value))
            self.write('SWEEP DOWN FAST') 
            while (msg[0] > I_value):    
                time.sleep(1)       
                msg = self.ask('IMAG ?')
                continue
        if I_value > msg:
            self.write('ULIM {}'.format(I_value))
            self.write('SWEEP UP FAST')
            while (msg[0] < I_value):    
                time.sleep(1)       
                msg = self.ask('IMAG ?')
                continue
        if I_value == 0.0:
            self.write('SWEEP ZERO FAST')
            while (msg[0] != 0.00):    
                time.sleep(1)       
                msg = self.ask('IMAG ?')
                continue
            
        self.write('PSHTR OFF')
        msg=self.ask('VMAG ?')
        if msg[0]==0.0: 
            self.write('SWEEP ZERO')
    
    def _close(self):
       #for i in range(0,4):
       #    self.write('RATE {} {}'.format(i,default_rate_limit[i]))
       self.write('LOCAL')
       super().close()
