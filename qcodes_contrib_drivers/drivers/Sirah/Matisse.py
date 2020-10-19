import abc
import re
from typing import Dict, List, Optional, Union
import visa
import warnings

import qcodes.utils.validators as vals
from qcodes.instrument.channel import InstrumentChannel, ChannelList
from qcodes.instrument.visa import VisaInstrument
from qcodes.utils.delaykeyboardinterrupt import DelayedKeyboardInterrupt


class SirahMatisseError(Exception):
    """Exception class representing errors related to the Sirah Matisse driver

    Args:
        message: Error message
        error_code: Return/error code
    """

    def __init__(self, message: str, error_code: Optional[int] = None):
        super().__init__(message, error_code)
        self._message = message
        self._error_code = error_code

    def __str__(self):
        return "{} ({})".format(self._message, self._error_code)


class SirahMatissePowerDiode(InstrumentChannel):
    """Instrument channel for power diode of Sirah Matisse

    Args:
        parent: Parent SirahMatisse instrument object
        name: Name of this motor-channel
    """

    def __init__(self, parent: "SirahMatisse", name: str, **kwargs):
        super().__init__(parent, name, **kwargs)

        self._cmd_prefix = "DPOW:"

        self.add_parameter("dc_value",
                           label="DC-part",
                           get_cmd=self._cmd_prefix + "DC?",
                           get_parser=float,
                           unit="V",
                           docstring="""
                           Gets the DC-part of the integral laser output. The value is given in
                           volts at the controller input. This is a read-only value.
                           """)

        self.add_parameter("wave_table",
                           label="Current waveform of the AC-part",
                           get_cmd=self._cmd_prefix + "WAVTAB?",
                           get_parser=self._wave_table_parser,
                           unit="V",
                           docstring="""
                           Gets the current waveform of the AC-part of the integral laser output.
                           The values are normalized to be in the range [-1,1]. The number of values
                           is determined by the setting of `piezo_etalon.oversampling`.
                           """)

        self.add_parameter("low",
                           label="Current value of the low power level",
                           get_cmd=self._cmd_prefix + "LOW?",
                           set_cmd=self._cmd_prefix + "LOW ",
                           get_parser=float,
                           vals=vals.Numbers(),
                           unit="V",
                           docstring="""
                           Gets/sets the current value of the low power level. When the signal at
                           the integral power diode drops below this level all control loops are
                           deactivated. Setting the level to 0 (zero) de-activates this function.
                           """)

    @staticmethod
    def _wave_table_parser(raw_wave_table: str) -> List[float]:
        return [float(raw_value) for raw_value in raw_wave_table.split()]


class SirahMatisseMotor(InstrumentChannel, abc.ABC):
    """Base class for motor-submodules of Sirah Matisse (e.g. birefrigent filter motor, thin etalon
    motor)

    Args:
        parent: Parent SirahMatisse instrument object
        name: Name of this motor-channel
        cmd_prefix: Prefix for VISA-commands (e.g. MOTBI, MOTTE)
    """

    def __init__(self, parent: "SirahMatisse", name: str, cmd_prefix: str, **kwargs):
        super().__init__(parent, name, **kwargs)

        self._cmd_prefix = cmd_prefix.strip() + ":"

        self.add_parameter("position",
                           label="Current position of motor",
                           get_cmd=self._cmd_prefix + "POS?",
                           set_cmd=self._set_position,
                           get_parser=int,
                           vals=vals.Ints(),
                           # TODO unit="???", # steps?
                           docstring="""
                           Gets the current position of stepper motor position or moves it to an
                           absolute position. When setting, the command does not wait for completion
                           of the motor movement.
                           """)

        self.add_parameter("target_position",
                           label="Target position of motor",
                           get_cmd=None,
                           set_cmd=self._cmd_prefix + "POS {}",
                           vals=vals.Ints(),
                           docstring="""
                           Sets the target position of stepper motor position and moves to the
                           desired position asynchronously.
                           """)

        self.add_parameter("raw_status",
                           label="Status of motor controller",
                           get_cmd=self._cmd_prefix + "STA?",
                           get_parser=int,
                           vals=vals.Ints(),
                           docstring="""
                           Retrieve the status and setting of motor controller. The status is binary
                           coded into a single 16-bit integer value. The bits have the following
                           meanings:
                             - bits 0 to 7: Current status of the controller, negative means error
                             - bit 8: Indicates that the motor is running
                             - bit 9: Indicates the motor current is switched off
                             - bit 10: Indicates an invalid motor position after hold was switched
                                       off
                             - bit 11: Status of limit switch 1
                             - bit 12: Status of limit switch 2
                             - bit 13: Status of home switch
                             - bit 14: Manual control enable/disable
                           """)

        self.add_parameter("status",
                           label="Status of motor controller",
                           get_cmd=self._cmd_prefix + "STA?",
                           get_parser=self._status_parser,
                           docstring="""
                           Retrieve the status and setting of motor controller. The status is binary
                           coded into a single 16-bit integer value. When getting this parameter,
                           the integer-bits are automatically converted into a dictionary with the
                           following keys:
                             - raw: Same as argument status
                             - status: Current status of the controller, negative means error (bits
                                       0 to 7)
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
                           get_parser=int,
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
                           get_parser=int,
                           vals=vals.Ints(),
                           # TODO unit="???", # steps?
                           docstring="""
                           Gets/sets the number of motor steps made by the Birefringent Filter when
                           the manual control button is pressed for a short time.
                           """)

    def _set_position(self, target_position: int) -> None:
        """Sets the motor's target position and waits until it is reached.

        Args:
            target_position: Target position to move to
        """
        # Wait for completion of previous task
        self._wait_for_idle()
        # Set desired target-position
        self.target_position(target_position)
        # Wait for completion of movement to target-position
        self._wait_for_idle()

    def _wait_for_idle(self):
        """Waits until parameter `status` is 2 (= idle)"""
        while True:
            status = self.status()["status"]
            if status == 0x02:  # 0x02 := idle
                break
            if status < 0:  # < 0 := error (bit 7 is set)
                raise visa.VisaIOError(status)

        # TODO: Alternatively, try this. Check what which one better...
        # while self.position() != target_position:
        #     pass

    @staticmethod
    def _status_parser(raw_status: str) -> dict:
        """Converts an integer with separate status-bits into a dictionary of ints and bools.

        Args:
            raw_status: Raw status, returned by command :<MOTOR>:STATUS?.

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
        status = int(raw_status)
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
    """Birefringent filter motor of Sirah Matisse

    Args:
        parent: Parent SirahMatisse-instrument object
        name: Name of this motor-channel
    """

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
                           get_parser=int,
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
                           get_parser=float,
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
                           get_parser=float,
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
                           get_parser=float,
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
                           get_parser=float,
                           # TODO unit="???",
                           docstring="""
                           Gets/sets the calibration parameter *WavelengthOffset* for the step motor
                           position to wavelength conversion.
                           """)

        self.add_parameter("motor_factor",
                           label="Calibration parameter for wavelength factor",
                           get_cmd=self._cmd_prefix + "MOTFAC?",
                           set_cmd=self._cmd_prefix + "MOTFAC {}",
                           get_parser=float,
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
                           get_parser=float,
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
                           get_parser=float,
                           vals=vals.Numbers(),  # or maybe integer (manual is not clear on that)
                           # TODO unit="???",
                           docstring="""
                           Gets/sets the calibration parameter *LinearOffset* for the step motor
                           position to wavelength conversion.
                           """)


class SirahMatisseThinEtalonMotor(SirahMatisseMotor):
    """Thin etalon motor of Sirah Matisse

    Args:
        parent: Parent SirahMatisse-instrument object
        name: Name of this motor-channel
    """

    def __init__(self, parent: "SirahMatisse", name: str, **kwargs):
        super().__init__(parent, name, "MOTTE", **kwargs)


class SirahMatisseEtalonControl(InstrumentChannel, abc.ABC):
    """Base class for etalon-control-submodules of Sirah Matisse (e.g. piezo etalon control, thin
    etalon control)

    Args:
        parent: Parent SirahMatisse instrument object
        name: Name of this motor-channel
        cmd_prefix: Prefix for VISA-commands (e.g. PZETL, TE)
    """

    def __init__(self, parent: "SirahMatisse", name: str, cmd_prefix: str, **kwargs):
        super().__init__(parent, name, **kwargs)

        self._cmd_prefix = cmd_prefix.strip() + ":"

        self.add_parameter("control_status",
                           label="Status",
                           get_cmd=self._cmd_prefix + "CNTRSTA?",
                           set_cmd=self._cmd_prefix + "CNTRSTA {}",
                           val_mapping={"run": "RUN", "stop": "STOP"},
                           docstring="""
                           Starts/stops or gets the status of a control loop
                           """)

        self.add_parameter("control_proportional",
                           label="Proportional gain",
                           get_cmd=self._cmd_prefix + "CNTRPROP?",
                           set_cmd=self._cmd_prefix + "CNTRPROP {}",
                           get_parser=float,
                           vals=vals.Numbers(),
                           # TODO unit="???",
                           docstring="""
                           Gets/sets the proportional gain of a control loop
                           """)

        self.add_parameter("control_average",
                           label="Number of measurements averaged",
                           get_cmd=self._cmd_prefix + "CNTRAVG?",
                           set_cmd=self._cmd_prefix + "CNTRAVG {}",
                           get_parser=int,
                           vals=vals.Ints(),
                           # TODO unit="???",
                           docstring="""
                           Gets/sets the number of measurements averaged of a control loop
                           """)

        self.add_parameter("cavity_scan",
                           label="Proportional factor that controls how a scan of the slow cavity "
                                 "influences the position of the thick piezo etalon",
                           get_cmd=self._cmd_prefix + "CAVSCN?",
                           set_cmd=self._cmd_prefix + "CAVSCN {}",
                           get_parser=float,
                           vals=vals.Numbers(),
                           # TODO unit="???",
                           docstring="""
                           Gets/sets the proportional factor that controls how a scan of the slow
                           cavity piezo influences the position of the thick piezo etalon. The
                           factor results in an immediate piezo movement, even without the P (see
                           PID Loops) control loop enabled.
                           """)

        self.add_parameter("reference_scan",
                           label="Proportional factor that controls how a scan of the reference "
                                 "cell piezo influences the position of the thick piezo etalon",
                           get_cmd=self._cmd_prefix + "REFSCN?",
                           set_cmd=self._cmd_prefix + "REFSCN {}",
                           get_parser=float,
                           vals=vals.Numbers(),
                           # TODO unit="???",
                           docstring="""
                           Gets/sets the proportional factor that controls how a scan of the
                           reference cell piezo influences the position of the thick piezo etalon.
                           The factor results in an immediate piezo movement, even without the P
                           (see PID Loops) control loop enabled.
                           """)


class SirahMatissePiezoEtalonControl(SirahMatisseEtalonControl):
    def __init__(self, parent: "SirahMatisse", name: str, **kwargs):
        super().__init__(parent, name, "PZETL", **kwargs)

        self.add_parameter("oversampling",
                           label="Number of sample points",
                           get_cmd=self._cmd_prefix + "OVER?",
                           set_cmd=self._cmd_prefix + "OVER {}",
                           get_parser=int,
                           vals=vals.Ints(8, 64),
                           # TODO unit="???",
                           docstring="""
                           Gets/sets the number of sample points used for sine interpolation. The
                           minimum value is 8, the maximum value is 64 samples per period.
                           """)

        self.add_parameter("baseline",
                           label="Baseline of the modulation waveform",
                           get_cmd=self._cmd_prefix + "BASE?",
                           set_cmd=self._cmd_prefix + "BASE {}",
                           get_parser=float,
                           vals=vals.Numbers(-1, 1),
                           # TODO unit="???",
                           docstring="""
                           Gets/sets the baseline of the modulation waveform to a new value. The
                           value needs to be within the interval [-1,1].
                           """)

        self.add_parameter("amplitude",
                           label="Amplitude of the modulation",
                           get_cmd=self._cmd_prefix + "AMP?",
                           set_cmd=self._cmd_prefix + "AMP {}",
                           get_parser=float,
                           vals=vals.Numbers(),
                           # TODO unit="???",
                           docstring="""
                           Gets/sets the amplitude of the modulation of the thick etalon.
                           """)

        self.add_parameter("control_phaseshift",
                           label="Phaseshift used for PLL calculation",
                           get_cmd=self._cmd_prefix + "CNTRPHSF?",
                           set_cmd=self._cmd_prefix + "CNTRPHSF {}",
                           get_parser=int,
                           vals=vals.Ints(),
                           # TODO unit="???",
                           docstring="""
                           Gets/sets the phaseshift value used for the PLL calculation.
                           """)

        self.add_parameter("sample_rate",
                           label="Sample rate",
                           get_cmd=self._cmd_prefix + "SRATE?",
                           set_cmd=self._cmd_prefix + "SRATE {}",
                           get_parser=int,
                           val_mapping={8: 0, 32: 1, 48: 2, 96: 3},
                           unit="kHz",
                           docstring="""
                           Gets/sets the sample rate for the piezo etalon control loop. The product
                           of samplerate and oversampling determines the modulation frequency of the
                           etalon. Allowed values are 8, 32, 48 and 96 kHz.
                           """)


class SirahMatisseThinEtalonControl(SirahMatisseEtalonControl):
    def __init__(self, parent: "SirahMatisse", name: str, **kwargs):
        super().__init__(parent, name, "TE", **kwargs)

        self.add_parameter("dc_value",
                           label="DC-part of the thin etalon's reflex",
                           get_cmd=self._cmd_prefix + "DC?",
                           get_parser=float,
                           vals=vals.Numbers(),
                           unit="V",
                           docstring="""
                           Gets the DC-part of the reflex of the thin etalon. The value is given in
                           volts at the controller input.
                           """)

        self.add_parameter("control_error",
                           label="Current error value",
                           get_cmd=self._cmd_prefix + "CNTRERR?",
                           get_parser=float,
                           vals=vals.Numbers(),
                           # TODO unit="???",
                           docstring="""
                           Gets the current error value of a control loop
                           """)

        self.add_parameter("control_setpoint",
                           label="Control goal",
                           get_cmd=self._cmd_prefix + "CNTRSP?",
                           set_cmd=self._cmd_prefix + "CNTRSP {}",
                           get_parser=float,
                           vals=vals.Numbers(),
                           unit="V",  # TODO: unit V correct?
                           docstring="""
                           Gets/sets the control goal of the thin etalon control loop. The error
                           signal of the PI control loop is calculated according to:
                           Error = TE:CNTRSP - (TE:DCVALUE)/(DPOW:DCVALUE)
                           """)

        self.add_parameter("control_integral",
                           label="Integral gain",
                           get_cmd=self._cmd_prefix + "CNTRINT?",
                           set_cmd=self._cmd_prefix + "CNTRINT {}",
                           get_parser=float,
                           vals=vals.Numbers(),
                           # TODO unit="???",
                           docstring="""
                           Gets/sets the integral gain of the thin Etalon PI control loop.
                           """)


class SirahMatisse(VisaInstrument):
    """Sirah Matisse laser instrument driver

    Args:
        name: Name of the instrument
        address: VISA resource name (used to establish a connection to the instrument)
    """
    def __init__(self, name: str, address: str):
        super().__init__(name, address)

        self.add_function("error_codes",
                          call_cmd="ERR:CODE?",
                          docstring="""
                          Get all error codes raised since last `error_clear` command (or system
                          startup).
                          """)

        self.add_function("error_clear",
                          call_cmd="ERR:CL",
                          docstring="""
                          Clears error conditions and information. This command does not effect
                          error conditions from the motor controller.
                          """)

        channel_classes = {
            "power_diode": SirahMatissePowerDiode,             # power diode
            "motor_bifi": SirahMatisseBiFiMotor,               # birefringent filter motor
            "motor_thin_etalon": SirahMatisseThinEtalonMotor,  # thin etalon motor
            "piezo_etalon": SirahMatissePiezoEtalonControl,    # piezo/thick etalon control
            "thin_etalon": SirahMatisseThinEtalonControl       # thin etalon control
        }

        for name, cls in channel_classes.items():
            channel = cls(self, name)
            self.add_submodule(channel.short_name, channel)

        self.error_clear()
        self.motor_bifi.clear()
        self.motor_thin_etalon.clear()

        self.connect_message()

    def get_idn(self) -> Dict[str, Optional[str]]:
        """Queries and parses the identification string of the device.

        The identification string consists of the following components: model name, serial number,
        board version, firmware version, and version date. It typically looks like this:

            :IDN: "Matisse TS, S/N:05-25-20, DSP Rev. 01.00, Firmware: 1.6, Date: Apr 23 2007"

        Returns:
            A dictionary containing the parsed data from the identification string
        """
        idn = {"vendor": "Sirah",
               "model": self.name,
               "serial": None,
               "firmware": None}
        raw_idn = ""

        try:
            raw_idn = self.ask("*IDN?")
            parts = [part.strip() for part in raw_idn.split(",")]
            key_mapping = {"s/n": "serial",
                           "dsp rev.": "board"}

            idn["model"] = parts[0]

            for part in parts[1:]:
                key, value = part.split(":", maxsplit=1)
                key = key.lower()
                if key in key_mapping:
                    key = key_mapping[key]
                idn[key] = value
        except Exception:
            self.visa_log.debug("DEBUG: Error getting or interpreting *IDN?: " + repr(raw_idn))

        return idn

    def ask_raw(self, cmd: str) -> str:
        """
        Low-level interface to ``visa_handle.ask``.

        Args:
            cmd: The command to send to the instrument.

        Returns:
            str: The instrument's response.
        """
        with DelayedKeyboardInterrupt():
            self.visa_log.debug(f"Querying: {cmd}")
            response = self.visa_handle.query(cmd)
            self.visa_log.debug(f"Response: {response}")

            # Handle error response
            if response.startswith('!ERROR'):
                try:
                    # Extract error code from response
                    err_code = int(response.split(maxsplit=1)[1])
                    raise SirahMatisseError("Error querying \"{}\"".format(cmd), err_code)
                except SirahMatisseError:
                    raise
                except Exception as exc:
                    raise SirahMatisseError("Unknown error querying \"{}\"".format(cmd)) from exc
                finally:
                    # Try to clear error buffer
                    try:
                        self.clear_errors()
                    except Exception as exc:
                        warnings.warn("Couldn't clear error buffer after receiving an error "
                                      "response.\n -> {}: {}".format(type(exc).__name__, exc))
            elif response == "OK":
                return ""
            else:
                # Extract result from response-string
                response_split = response.split(maxsplit=1)
                if len(response_split) > 1:
                    result = response_split[1]

                    # If response is surrounded by quotes, remove them
                    if result[0] == result[-1] == "\"":
                        return result[1:-1]
                    else:
                        return result
                else:
                    return ""

    def write(self, *args, **kwargs):
        # Do same as ask, but don't return
        self.ask(*args, **kwargs)
