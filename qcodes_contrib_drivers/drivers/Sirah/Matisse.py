import abc
import re
from typing import Optional, Union

import qcodes.utils.validators as vals
from qcodes.instrument.channel import InstrumentChannel, ChannelList
from qcodes.instrument.visa import VisaInstrument


# class SirahMatisseError(Exception):
#     def __init__(self, message: str, error_code: Optional[int]):
#         super().__init__(message, error_code)
#         self._message = message
#         self._error_code = error_code
#
#     def __str__(self):
#         return "{} ({})".format(self._message, self._error_code)


class SirahMatisseMotor(InstrumentChannel, abc.ABC):
    def __init__(self, parent: "SirahMatisse", name: str, cmd_prefix: str, **kwargs):
        super().__init__(parent, name, **kwargs)

        self._cmd_prefix = cmd_prefix.strip() + ":"

        self.add_parameter("position",
                           label="Current position of motor",
                           get_cmd=self._cmd_prefix + "POS?",
                           set_cmd=self._cmd_prefix + "POS {}",
                           get_parser=self._int_parser,
                           vals=vals.Ints(),
                           # TODO unit="???", # steps?
                           docstring="""
                           Gets the current position of stepper motor position or moves it to an
                           absolute position. When setting, the command does not wait for completion
                           of the motor movement.
                           """)

        self.add_parameter("status",
                           label="Status of motor controller",
                           get_cmd=self._cmd_prefix + "STA?",
                           get_parser=self._status_parser,
                           # TODO unit="???",
                           docstring="""
                           Retrieve the status and setting of motor controller. The status is binary
                           coded into a single 16-bit integer value. When getting this parameter,
                           the integer-bits are automatically converted into a dictionary with the
                           following keys:
                             - raw: Same as argument status
                             - status: Current status of the controller (bits 0 to 6)
                             - is_error: Set in case of an error status of the controller (bit 7)
                             - motor_running: Indicates that the motor is running (bit 8)
                             - motor_current_off: Indicates the motor current is switched off (bit
                                                  9)
                             - invalid_motor_pos: Indicates an invalid motor position after hold was
                                                  switched off (bit 10)
                             - limit_switch_1: Status of limit switch 1 (bit 11)
                             - limit_switch_2: Status of limit switch 2 (bit 12)
                             - home_switch: Status of home switch (bit 13)
                             - manual_control: Manual control enable/disable (bit 14)
                           """)

        self.add_parameter("maximum",
                           label="Maximum position of stepper "
                                 "motor.",
                           get_cmd=self._cmd_prefix + "MAX?",
                           get_parser=self._int_parser,
                           vals=vals.Ints(),
                           # TODO unit="???",
                           docstring="""Gets the maximum position of the stepper motor.""")

        self.add_function("move_relative",
                          call_cmd=self._cmd_prefix + "REL {}",
                          args=(vals.Ints(),),
                          docstring="""
                          Move the stepper motor the given number of steps relative to it's current
                          position. The command does not wait for completion of the motor movement.
                          """)

        self.add_function("move_home",
                          call_cmd=self._cmd_prefix + "HOME",
                          docstring="""
                          Move the stepper motor to its home position. The home position is defined
                          by the home switch. The controller positions the stepper motor at the
                          point where the home switch is actuated and resets the motor position to
                          zero (0). The command does not wait for completion of the motor movement.
                          """)

        self.add_function("halt",
                          call_cmd=self._cmd_prefix + "HALT",
                          docstring="""
                          Stop a motion of the birefringent filter's stepper motor. The command will
                          use a smooth deceleration to maintain accurate step tracking.
                          """)

        self.add_function("clear",
                          call_cmd=self._cmd_prefix + "CL",
                          docstring="""
                          Clear pending errors at the birefringent filter motor controller.
                          """)

        self.add_parameter("increment",
                           label="Number of steps made by birefringent filter motor",
                           get_cmd=self._cmd_prefix + "INC?",
                           set_cmd=self._cmd_prefix + "INC {}",
                           get_parser=self._int_parser,
                           vals=vals.Ints(),
                           # TODO unit="???", # steps?
                           docstring="""
                           Gets/sets the number of motor steps made by the Birefringent Filter when
                           the manual control button is pressed for a short time.
                           """)

    @staticmethod
    def _int_parser(value: str) -> int:
        return int(value.split()[-1])

    @staticmethod
    def _float_parser(value: str) -> float:
        return float(value.split()[-1])

    @staticmethod
    def _status_parser(status: int) -> dict:
        """Converts an integer with separate status-bits into a dictionary of ints and bools.

        Args:
            status: Raw status actually returned by parameter `status`.

        Returns:
            Dictionary with following keys:
                - raw: Same as argument status
                - status: Current status of the controller, negative means error (bits 0 to 7)
                - motor_running: Indicates that the motor is running (bit 8)
                - motor_current_off: Indicates the motor current is switched off (bit 9)
                - invalid_motor_pos: Indicates an invalid motor position after hold was switched off
                                     (bit 10)
                - limit_switch_1: Status of limit switch 1 (bit 11)
                - limit_switch_2: Status of limit switch 2 (bit 12)
                - home_switch: Status of home switch (bit 13)
                - manual_control: Manual control enable/disable (bit 14)
        """
        return dict(raw=status,
                    status=status & 0xff,  # bits 0 to 7
                    motor_running=bool((status >> 8) & 1),  # bit 8
                    motor_current_off=bool((status >> 9) & 1),  # bit 9
                    invalid_motor_pos=bool((status >> 10) & 1),  # bit 10
                    limit_switch_1=bool((status >> 11) & 1),  # bit 11
                    limit_switch_2=bool((status >> 12) & 1),  # bit 12
                    home_switch=bool((status >> 13) & 1),  # bit 13
                    manual_control=bool((status >> 14) & 1))  # bit 14


class SirahMatisseBiFiMotor(SirahMatisseMotor):
    def __init__(self, parent: "SirahMatisse", name: str, **kwargs):
        super().__init__(parent, name, "MOTBI", **kwargs)

        # Additional parameters for BiFi-motor
        self.add_function("move_constant_absolute",
                          call_cmd=self._cmd_prefix + "CABS {}",
                          args=(vals.Ints(),),
                          # TODO unit="???",
                          docstring="""
                          Move the stepper motor of the birefringent filter to an absolute position
                          using the constant speed defined by `bifi_frequency` (MOTBI:FREQ). The
                          command does not wait for completion of the motor movement. This commands
                          requires stepper motor driver firmware version R25, or higher.
                          """)

        self.add_function("move_constant_relative",
                          call_cmd=self._cmd_prefix + "CREL {}",
                          args=(vals.Ints(),),
                          # TODO unit="???",
                          docstring="""
                          Move the stepper motor of the birefringent filter relative to its current
                          position using the constant speed defined by `bifi_frequency`
                          (MOTBI:FREQ). The command does not wait for completion of the motor
                          movement. This commands requires stepper motor driver firmware version
                          R25, or higher
                          """)

        self.add_parameter("frequency",
                           label="Step frequency of birefringent filter motor",
                           get_cmd=self._cmd_prefix + "FREQ?",
                           set_cmd=self._cmd_prefix + "FREQ {}",
                           get_parser=self._int_parser,
                           vals=vals.Ints(),
                           # TODO unit="???", # Hz?
                           docstring="""
                           Gets/sets the step frequency used for the Birefringent Filter motor when
                           using constant speed scan commands (MOTBI:CABS, MOTBI:CREL).
                           """)

        self.add_parameter("wavelength",
                           label="BiFi-position in terms of a wavelength",
                           get_cmd=self._cmd_prefix + "WL?",
                           set_cmd=self._cmd_prefix + "WL {}",
                           get_parser=self._float_parser,
                           vals=vals.Numbers(),
                           unit="nm",
                           docstring="""
                           Gets/moves the current position of the birefringent filter in terms of a
                           wavelength. The position is given as nanometers. The resulting motor
                           position needs to be in between 0 and the maximum motor position, as
                           given by the `bifi_maximum` parameter (MOTBI:MAX).
                           """)

        self.add_parameter("cavity_scan",
                           label="Factor controlling how a cavity-scan influences birefringent "
                                 "filter motor position",
                           get_cmd=self._cmd_prefix + "CAVSCN?",
                           set_cmd=self._cmd_prefix + "CAVSCN {}",
                           get_parser=self._float_parser,
                           vals=vals.Numbers(),
                           # TODO unit="???",
                           docstring="""
                           Gets/sets the proportional factor that controls how a scan of the slow
                           cavity piezo influences the position of the birefringent filter motor.
                           """)

        self.add_parameter("reference_scan",
                           label="Factor controlling how a reference-scan influences birefringent "
                                 "filter position",
                           get_cmd=self._cmd_prefix + "REFSCN?",
                           set_cmd=self._cmd_prefix + "REFSCN {}",
                           get_parser=self._float_parser,
                           vals=vals.Numbers(),
                           # TODO unit="???",
                           docstring="""
                           Gets/sets the proportional factor that controls how a scan of the
                           reference cell piezo influences the position of the birefringent filter
                           motor.
                           """)

        self.add_parameter("motor_offset",
                           label="Calibration parameter for wavelength offset",
                           get_cmd=self._cmd_prefix + "MOTOFF?",
                           set_cmd=self._cmd_prefix + "MOTOFF {}",
                           vals=vals.Numbers(),
                           get_parser=self._float_parser,
                           # TODO unit="???",
                           docstring="""
                           Gets/sets the calibration parameter *WavelengthOffset* for the step motor
                           position to wavelength conversion.
                           """)

        self.add_parameter("motor_factor",
                           label="Calibration parameter for wavelength factor",
                           get_cmd=self._cmd_prefix + "MOTFAC?",
                           set_cmd=self._cmd_prefix + "MOTFAC {}",
                           get_parser=self._float_parser,
                           vals=vals.Numbers(),
                           # TODO unit="???",
                           docstring="""
                           Gets/sets the calibration parameter *WavelengthFactor* for the step motor
                           position to wavelength conversion.
                           """)

        self.add_parameter("thickness",
                           label="Calibration parameter for lever length",
                           get_cmd=self._cmd_prefix + "MOTTHNS?",
                           set_cmd=self._cmd_prefix + "MOTTHNS {}",
                           get_parser=self._float_parser,
                           vals=vals.Numbers(),
                           # TODO unit="???",
                           docstring="""
                           Gets/sets the calibration parameter *Leverlength* for the step motor
                           position to wavelength conversion.
                           """)

        self.add_parameter("order",
                           label="Calibration parameter for linear offset",
                           get_cmd=self._cmd_prefix + "ORDER?",
                           set_cmd=self._cmd_prefix + "ORDER {}",
                           get_parser=self._float_parser,
                           vals=vals.Numbers(),  # or maybe integer (manual is not clear on that)
                           # TODO unit="???",
                           docstring="""
                           Gets/sets the calibration parameter *LinearOffset* for the step motor
                           position to wavelength conversion.
                           """)


class SirahMatisseThEtMotor(SirahMatisseMotor):
    def __init__(self, parent: "SirahMatisse", name: str, **kwargs):
        super().__init__(parent, name, "MOTTE", **kwargs)


class SirahMatisse(VisaInstrument):
    def __init__(self, name: str, address: str):
        super().__init__(name, address)

        motor_channels = ChannelList(self, "motor_channels", SirahMatisseMotor, snapshotable=False)

        channel_classes = {"bifi": SirahMatisseBiFiMotor,
                           "thet": SirahMatisseThEtMotor}

        for name, cls in channel_classes.items():
            motor_chan = cls(self, name)
            motor_channels.append(motor_chan)
            self.add_submodule(motor_chan.short_name, motor_chan)

        motor_channels.lock()
        self.add_submodule("motor_channels", motor_channels)

        self.add_function("clear_errors",
                          call_cmd="ERR:CL",
                          docstring="")  # TODO

        self.add_parameter("power_diode_dc",
                           get_cmd="DPOW:DC?",
                           get_parser=self._float_parser,
                           vals=vals.Numbers(),
                           # TODO unit="???",
                           docstring="")  # TODO

        self.add_parameter("te_reflex",
                           get_cmd="TE:DC?",
                           get_parser=self._float_parser,
                           vals=vals.Numbers(),
                           # TODO unit="???",
                           docstring="")  # TODO

        self.connect_message()

    def write(self, *args, **kwargs):
        self.ask(*args, *kwargs)

    @staticmethod
    def _float_parser(value: str) -> float:
        return float(value.split()[-1])
