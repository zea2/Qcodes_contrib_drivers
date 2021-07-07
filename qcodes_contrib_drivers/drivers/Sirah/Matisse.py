"""Sirah Matisse laser instrument driver

The driver `SirahMatisse` controls the laser instrument Sirah Matisse. The hardware communication
works with VISA and is handled by `VisaInstrument`. There are separate instrument channels for the
different components/parts of the instrument.

Author:
    Lukas Lankes, Forschugnszentrum Jülich GmbH - ZEA-2, l.lankes@fz-juelich.de
"""

import abc
import time
from typing import Dict, List, Optional
import visa
import warnings

import qcodes.utils.validators as vals
from qcodes.instrument.channel import InstrumentChannel
from qcodes.instrument.visa import VisaInstrument
from qcodes.utils.delaykeyboardinterrupt import DelayedKeyboardInterrupt


class SirahMatisseError(Exception):
    """Exception class representing errors related to the Sirah Matisse driver

    Args:
        message: Error message
        error_code: Return/error code
    """

    def __init__(self, message: str, error_codes: Optional[List[int]] = None):
        super().__init__(message, error_codes)
        self._message = message
        self._error_codes = error_codes

    def __str__(self):
        if self._error_codes is None:
            return self._message
        else:
            err_codes = ",".join(str(c) for c in self._error_codes)
            return f"{self._message} ({err_codes})"


class SirahMatisseChannel(InstrumentChannel, abc.ABC):
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
        self._common_parameters = {}

        # Pre-defined parameters

        self._define_common_param("position",
                                  label="Current position",
                                  get_cmd=self._cmd_prefix + "POS?",
                                  set_cmd=self._set_position,
                                  get_parser=int,
                                  vals=vals.Ints(),
                                  docstring="""
                                  Gets the current position of stepper motor position or moves
                                  it to an absolute position.
                                  """)

        self._define_common_param("target_position",
                                  label="Target position of motor",
                                  get_cmd=None,
                                  set_cmd=self._cmd_prefix + "POS {}",
                                  vals=vals.Ints(),
                                  docstring="""
                                  Sets the target position of stepper motor position and moves
                                  to the desired position asynchronously.
                                  """)

        self._define_common_param("raw_status",
                                  label="Status of motor controller",
                                  get_cmd=self._cmd_prefix + "STA?",
                                  get_parser=int,
                                  vals=vals.Ints(),
                                  docstring="""
                                  Retrieve the status and setting of motor controller. The raw
                                  status is binary coded into a single 16-bit integer value. The
                                  bits have the following meanings:
                                    - bits 0 to 7: Current status of the controller, negative
                                                   means error
                                    - bit 8: Indicates that the motor is running
                                    - bit 9: Indicates the motor current is switched off
                                    - bit 10: Indicates an invalid motor position after hold was
                                              switched off
                                    - bit 11: Status of limit switch 1
                                    - bit 12: Status of limit switch 2
                                    - bit 13: Status of home switch
                                    - bit 14: Manual control enable/disable
                                  """)

        self._define_common_param("status",
                                  label="Status of motor controller",
                                  get_cmd=self._cmd_prefix + "STA?",
                                  get_parser=self._status_parser,
                                  docstring="""
                                  Retrieve the status and setting of motor controller. The raw
                                  status is binary coded into a single 16-bit integer value.
                                  When getting this parameter, the integer-bits are
                                  automatically converted into a dictionary with the following
                                  keys:
                                    - raw: Same as argument status
                                    - status: Current status of the controller, negative means
                                              error (bits 0 to 7)
                                    - motor_running: Indicates that the motor is running (bit 8)
                                    - motor_current_off: Indicates the motor current is switched
                                                         off (bit 9)
                                    - invalid_motor_pos: Indicates an invalid motor position
                                                         after hold was switched off (bit 10)
                                    - limit_switch_1: Status of limit switch 1 (bit 11)
                                    - limit_switch_2: Status of limit switch 2 (bit 12)
                                    - home_switch: Status of home switch (bit 13)
                                    - manual_control: Manual control enable/disable (bit 14)
                                  """)

        self._define_common_param("maximum",
                                  label="Maximum position of stepper motor.",
                                  get_cmd=self._cmd_prefix + "MAX?",
                                  get_parser=int,
                                  vals=vals.Ints(),
                                  docstring="""
                                  Gets the maximum position of the stepper motor.
                                  """)

        self._define_common_param("increment",
                                  label="Number of steps made by birefringent filter motor",
                                  get_cmd=self._cmd_prefix + "INC?",
                                  set_cmd=self._cmd_prefix + "INC {}",
                                  get_parser=int,
                                  vals=vals.Ints(),
                                  docstring="""
                                  Gets/sets the number of motor steps made by the Birefringent
                                  Filter when the manual control button is pressed for a short
                                  time.
                                  """)

        self._define_common_param("cavity_scan",
                                  label="Factor controlling how a cavity-scan influences birefringent "
                                        "filter motor position",
                                  get_cmd=self._cmd_prefix + "CAVSCN?",
                                  set_cmd=self._cmd_prefix + "CAVSCN {}",
                                  get_parser=float,
                                  vals=vals.Numbers(),
                                  docstring="""
                                  Gets/sets the proportional factor that controls how a scan of the
                                  slow cavity piezo influences the position of the birefringent
                                  filter motor.
                                  """)

        self._define_common_param("reference_scan",
                                  label="Factor controlling how a reference-scan influences"
                                        "birefringent filter position",
                                  get_cmd=self._cmd_prefix + "REFSCN?",
                                  set_cmd=self._cmd_prefix + "REFSCN {}",
                                  get_parser=float,
                                  vals=vals.Numbers(),
                                  docstring="""
                                  Gets/sets the proportional factor that controls how a scan of the
                                  reference cell piezo influences the position of the birefringent
                                  filter motor.
                                  """)

        self._define_common_param("now",
                                  label="Current scan position",
                                  get_cmd=self._cmd_prefix + "NOW?",
                                  set_cmd=self._cmd_prefix + "NOW {}",
                                  get_parser=float,
                                  vals=vals.Numbers(),
                                  docstring="""
                                  Gets/sets the current scan position.
                                  """)

        self._define_common_param("control_status",
                                  label="Control status",
                                  get_cmd=self._cmd_prefix + "CNTRSTA?",
                                  set_cmd=self._cmd_prefix + "CNTRSTA {}",
                                  val_mapping={"run": "RUN", "stop": "STOP"},
                                  docstring="""
                                  Starts/stops or gets the status of a control loop
                                  """)

        self._define_common_param("control_proportional",
                                  label="Proportional gain",
                                  get_cmd=self._cmd_prefix + "CNTRPROP?",
                                  set_cmd=self._cmd_prefix + "CNTRPROP {}",
                                  get_parser=float,
                                  vals=vals.Numbers(),
                                  docstring="""
                                  Gets/sets the proportional gain of a control loop
                                  """)

        self._define_common_param("control_average",
                                  label="Number of measurements averaged",
                                  get_cmd=self._cmd_prefix + "CNTRAVG?",
                                  set_cmd=self._cmd_prefix + "CNTRAVG {}",
                                  get_parser=int,
                                  vals=vals.Ints(),
                                  docstring="""
                                  Gets/sets the number of measurements averaged of a control loop
                                  """)

        self._define_common_param("control_setpoint",
                                  label="Control goal",
                                  get_cmd=self._cmd_prefix + "CNTRSP?",
                                  set_cmd=self._cmd_prefix + "CNTRSP {}",
                                  get_parser=float,
                                  vals=vals.Numbers(),
                                  unit="V",
                                  docstring="""
                                  Gets/sets the control goal of the thin etalon control loop. The
                                  error signal of the PI control loop is calculated according to:
                                  Error = TE:CNTRSP - (TE:DCVALUE)/(DPOW:DCVALUE)
                                  """)

        self._define_common_param("control_integral",
                                  label="Integral gain",
                                  get_cmd=self._cmd_prefix + "CNTRINT?",
                                  set_cmd=self._cmd_prefix + "CNTRINT {}",
                                  get_parser=float,
                                  vals=vals.Numbers(),
                                  docstring="""
                                  Gets/sets the integral gain of the thin Etalon PI control loop.
                                  """)

        self._define_common_param("dc_value",
                                  label="DC-part",
                                  get_cmd=self._cmd_prefix + "DC?",
                                  get_parser=float,
                                  unit="V",
                                  docstring="""
                                  Gets the DC-part of the integral laser output. The value is given
                                  in volts at the controller input. This is a read-only value.
                                  """)

        # Pre-defined functions

        self._define_common_param("move_relative",
                                  call_cmd=self._cmd_prefix + "REL {}",
                                  args=(vals.Ints(),),
                                  docstring="""
                                  Move the stepper motor the given number of steps relative to
                                  it's current position. The command does not wait for
                                  completion of the motor movement.
                                  """)

        self._define_common_param("move_home",
                                  call_cmd=self._cmd_prefix + "HOME",
                                  docstring="""
                                  Move the stepper motor to its home position. The home position
                                  is defined by the home switch. The controller positions the
                                  stepper motor at the point where the home switch is actuated
                                  and resets the motor position to zero (0). The command does
                                  not wait for completion of the motor movement.
                                  """)

        self._define_common_param("halt",
                                  call_cmd=self._cmd_prefix + "HALT",
                                  docstring="""
                                  Stop a motion of the birefringent filter's stepper motor. The
                                  command will use a smooth deceleration to maintain accurate
                                  step tracking.
                                  """)

        self._define_common_param("clear",
                                  call_cmd=self._cmd_prefix + "CL",
                                  docstring="""
                                  Clear pending errors at the birefringent filter motor
                                  controller.
                                  """)

    def add_common_parameter(self, name: str, **kwargs) -> None:
        """Adds a previously defined parameter to this InstrumentChannel. That makes it easier to
        define multiple channels with same or similar parameters."""
        # Override defaults from _common_parameters with arguments from this function call
        kwargs = {**self._common_parameters[name], **kwargs}

        self.add_parameter(name, **kwargs)

    def add_common_function(self, name: str, **kwargs) -> None:
        """Adds a previously defined function to this InstrumentChannel. That makes it easier to
        define multiple channels with same or similar functions."""
        # Override defaults from _common_parameters with arguments from this function call
        kwargs = {**self._common_parameters[name], **kwargs}

        self.add_function(name, **kwargs)

    def _define_common_param(self, name: str, **kwargs):
        """Pre-define a parameter or function to make it easier for subclasses to add these
        parameters and functions."""
        self._common_parameters[name] = kwargs

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
            time.sleep(0.05)  # TODO Just a test
            status = self.status()["status"]
            if status == 0x02:  # 0x02 := idle
                break
            if status < 0:  # < 0 := error (bit 7 is set)
                raise visa.VisaIOError(status)

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


class SirahMatissePidLoops(SirahMatisseChannel):
    """Instrument channel for PID loops of Sirah Matisse

    Args:
        parent: Parent SirahMatisse instrument object
        name: Name of this channel
    """

    def __init__(self, parent: "SirahMatisse", name: str, **kwargs):
        super().__init__(parent, name, "PID", **kwargs)

        self.add_parameter("protocol",
                           label="PID Protocol",
                           get_cmd=self._cmd_prefix + "PROT?",
                           set_cmd=self._cmd_prefix + "PROT {}",
                           get_parser=int,
                           val_mapping={
                               None: 0,
                               "": 0,
                               "none": 0,
                               "thin": 1,  # Thin Etalon
                               "thick": 2,  # Thick Etalon
                               "slow": 3,  # Slow Piezo
                               "fast": 4},  # Fast Piezo
                           docstring="""
                           Gets/sets the identifier number of the pid-loop to protocol.
                           """)

        self.add_parameter("process_statistics",
                           label="Process statistics",
                           get_cmd=self._cmd_prefix + "PSTAT?",
                           get_parser=self._parse_process_statistics,
                           docstring="""
                           Evaluate some statistics for the process values stored in the PID
                           protocol array. The values are calculated using the current contents of
                           the 256 entry ring buffer. During the evaluation the recording of new PID
                           protocol data is disabled.
                           """)

        self.add_parameter("ordinal",
                           label="Ordinal number of the protocol entries",
                           get_cmd=self._cmd_prefix + "ORD?",
                           get_parser=int,
                           docstring="""
                           Get the current value of the counter for the ordinal number of the
                           protocol entries.
                           """)

    @staticmethod
    def _parse_process_statistics(stat: str) -> Dict[str, float]:
        """Parses a process statistics response into a dictionary with float values.

        Args:
            stat: A string, consisting of four space-separated floats

        Returns:
            A dictionary with following keys: "minimum", "maximum", "average" and "deviation"
        """
        values = [float(v) for v in stat.split()]
        if len(values) != 4:
            raise ValueError("stat")

        return dict(zip(("minimum", "maximum", "average", "deviation"), values))


class SirahMatissePowerDiode(SirahMatisseChannel):
    """Instrument channel for power diode of Sirah Matisse

    Args:
        parent: Parent SirahMatisse instrument object
        name: Name of this channel
    """

    def __init__(self, parent: "SirahMatisse", name: str, **kwargs):
        super().__init__(parent, name, "DPOW", **kwargs)

        # Pre-defined parameters/functions
        self.add_common_parameter("dc_value")

        # Additional parameters for power diode
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


class SirahMatisseBiFiMotor(SirahMatisseChannel):
    """Birefringent filter motor of Sirah Matisse

    Args:
        parent: Parent SirahMatisse-instrument object
        name: Name of this motor-channel
    """

    def __init__(self, parent: "SirahMatisse", name: str, **kwargs):
        super().__init__(parent, name, "MOTBI", **kwargs)

        # Pre-defined parameters/functions
        self.add_common_parameter("position")
        self.add_common_parameter("target_position")
        self.add_common_parameter("raw_status")
        self.add_common_parameter("status")
        self.add_common_parameter("maximum")
        self.add_common_parameter("increment")
        self.add_common_parameter("cavity_scan")
        self.add_common_parameter("reference_scan")

        self.add_common_function("move_relative")
        self.add_common_function("move_home")
        self.add_common_function("halt")
        self.add_common_function("clear")

        # Additional parameters for BiFi-motor
        self.add_function("move_constant_absolute",
                          call_cmd=self._cmd_prefix + "CABS {}",
                          args=(vals.Ints(),),
                          docstring="""
                          Move the stepper motor of the birefringent filter to an absolute position
                          using the constant speed defined by `bifi_frequency` (MOTBI:FREQ). The
                          command does not wait for completion of the motor movement. This commands
                          requires stepper motor driver firmware version R25, or higher.
                          """)

        self.add_function("move_constant_relative",
                          call_cmd=self._cmd_prefix + "CREL {}",
                          args=(vals.Ints(),),
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


class SirahMatisseThinEtalonMotor(SirahMatisseChannel):
    """Thin etalon motor of Sirah Matisse

    Args:
        parent: Parent SirahMatisse-instrument object
        name: Name of this motor-channel
    """

    def __init__(self, parent: "SirahMatisse", name: str, **kwargs):
        super().__init__(parent, name, "MOTTE", **kwargs)

        # Add pre-defined parameters/functions
        self.add_common_parameter("position")
        self.add_common_parameter("target_position")
        self.add_common_parameter("raw_status")
        self.add_common_parameter("status")
        self.add_common_parameter("maximum")
        self.add_common_parameter("increment")

        self.add_common_function("move_relative")
        self.add_common_function("move_home")
        self.add_common_function("halt")
        self.add_common_function("clear")


class SirahMatisseScanControl(SirahMatisseChannel):
    """Instrument channel for the scanning mirror of Sirah Matisse

    Args:
        parent: Parent SirahMatisse instrument object
        name: Name of this channel
    """

    def __init__(self, parent: "SirahMatisse", name: str, **kwargs):
        super().__init__(parent, name, "SCAN", **kwargs)

        # Pre-defined parameters/functions
        self.add_common_parameter("now", vals=vals.Numbers(0, 0.7))

        # Specific parameters/functions
        self.add_parameter("status",
                           label="Status",
                           get_cmd=self._cmd_prefix + "STA?",
                           set_cmd=self._cmd_prefix + "STA {}",
                           val_mapping={"run": "RUN", "stop": "STOP"},
                           docstring="""Starts/stops a scan""")

        self.add_parameter("lower_limit",
                           label="Lower limit of scan pattern",
                           get_cmd=self._cmd_prefix + "LLM?",
                           set_cmd=self._cmd_prefix + "LLM {}",
                           get_parser=float,
                           vals=vals.Numbers(0, 0.7),
                           docstring="""
                           Gets/sets the lower limit of the scan pattern. Scan positions are within
                           the interval [0,0.7].
                           """)

        self.add_parameter("upper_limit",
                           label="Upper limit of scan pattern",
                           get_cmd=self._cmd_prefix + "ULM?",
                           set_cmd=self._cmd_prefix + "ULM {}",
                           get_parser=float,
                           vals=vals.Numbers(0, 0.7),
                           docstring="""
                           Gets/sets the upper limit of the scan pattern. Scan positions are within
                           the interval [0,0.7].
                           """)

        self.add_parameter("mode",
                           label="Scan mode",
                           get_cmd=self._cmd_prefix + "MODE?",
                           set_cmd=self._cmd_prefix + "MODE {}",
                           get_parser=int,
                           vals=vals.Ints(),
                           docstring="""
                           Gets/sets the current scan mode. The scan mode determines the direction
                           of the scan and whether it stops at one of the limits. The behaviour is
                           coded into the bits of this variable. When the scan device reaches one of
                           the limit values, the direction is inverted. As a next step the scan is
                           stopped at the limit, if the appropriate control bit is set.
                           
                           The bits have the following meaning:
                             - bit 0: scan is decreasing the scan voltage
                             - bit 1: scan stops at the lower limit
                             - bit 2: scan stops at the upper limit
                           """)

        self.add_parameter("device",
                           label="Device that controls the scan",
                           get_cmd=self._cmd_prefix + "DEV?",
                           set_cmd=self._cmd_prefix + "DEV {}",
                           get_parser=int,
                           val_mapping={
                               None: 0,
                               "": 0,
                               "none": 0,
                               "cav": 1,  # slow cavity piezo
                               "cavity": 1,  # slow cavity piezo
                               "ref": 2,  # reference cell piezo
                               "reference": 2  # reference cell piezo
                           },
                           docstring="""
                           Gets/sets the device that controls the scan of the Matisse laser. This
                           device is the master that controls the tuning of the system, all other
                           devices follow the master device either by open-loop (e. g. birefringent
                           filter) or closed-loop control (e. g. thick etalon). If the specified
                           device is already used by another command e.g. the SLOWPIEZO control
                           loop, an error message will be returned.
                           """)

        self.add_parameter("rising_speed",
                           label="Speed of voltage ramp-up",
                           get_cmd=self._cmd_prefix + "RSPD?",
                           set_cmd=self._cmd_prefix + "RSPD {}",
                           get_parser=float,
                           vals=vals.Numbers(),
                           unit="V/s",  # TODO unit V/s correct?
                           docstring="""
                           Gets/sets the speed of the voltage ramp-up of the scan mirror.
                           """)

        self.add_parameter("falling_speed",
                           label="Speed of voltage ramp-down",
                           get_cmd=self._cmd_prefix + "FSPD?",
                           set_cmd=self._cmd_prefix + "FSPD {}",
                           get_parser=float,
                           vals=vals.Numbers(),
                           unit="V/s",  # TODO unit V/s correct?
                           docstring="""
                           Gets/sets the speed of the voltage ramp-down of the scan mirror.
                           """)

        self.add_parameter("reference_calibration",
                           label="Scan device calibration factor for reference cell controlled scans",
                           get_cmd=self._cmd_prefix + "REFCAL?",
                           set_cmd=self._cmd_prefix + "REFCAL {}",
                           get_parser=float,
                           vals=vals.Numbers(),
                           docstring="""
                           Gets/sets the scan device calibration factor for reference cell
                           controlled scans. The value is stored into the laser's flash memory but
                           has no further influence on the operation.
                           """)

        self.add_parameter("cavity_calibration",
                           label="Scan device calibration factor for cavity scans",
                           get_cmd=self._cmd_prefix + "CAVCAL?",
                           set_cmd=self._cmd_prefix + "CAVCAL {}",
                           get_parser=float,
                           vals=vals.Numbers(),
                           docstring="""
                           Gets/sets the scan device calibration factor for cavity scans. The value
                           is stored into the laser's flash memory but has no further influence on
                           the operation.
                           """)


class SirahMatissePiezoEtalonControl(SirahMatisseChannel):
    def __init__(self, parent: "SirahMatisse", name: str, **kwargs):
        super().__init__(parent, name, "PZETL", **kwargs)

        # Pre-defined parameters/functions
        self.add_common_parameter("control_status")
        self.add_common_parameter("control_proportional")
        self.add_common_parameter("control_average")
        self.add_common_parameter("cavity_scan")
        self.add_common_parameter("reference_scan")

        # Additional parameters for piezo etalon
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


class SirahMatisseThinEtalonControl(SirahMatisseChannel):
    def __init__(self, parent: "SirahMatisse", name: str, **kwargs):
        super().__init__(parent, name, "TE", **kwargs)

        # Pre-defined parameters/functions
        self.add_common_parameter("control_status")
        self.add_common_parameter("control_proportional")
        self.add_common_parameter("control_average")
        self.add_common_parameter("cavity_scan")
        self.add_common_parameter("reference_scan")
        self.add_common_parameter("control_setpoint")
        self.add_common_parameter("control_integral")
        self.add_common_parameter("dc_value")

        # Additional parameters for thin etalon control
        self.add_parameter("control_error",
                           label="Current error value",
                           get_cmd=self._cmd_prefix + "CNTRERR?",
                           get_parser=float,
                           vals=vals.Numbers(),
                           # TODO unit="???",
                           docstring="""
                           Gets the current error value of a control loop
                           """)


class SirahMatisseFastPiezoControl(SirahMatisseChannel):
    """Instrument channel for fast piezo control of Sirah Matisse

    Args:
        parent: Parent SirahMatisse instrument object
        name: Name of this channel
    """

    def __init__(self, parent: "SirahMatisse", name: str, **kwargs):
        super().__init__(parent, name, "FPZT", **kwargs)

        # Pre-defined parameters/functions
        self.add_common_parameter("control_setpoint")
        self.add_common_parameter("control_status")
        self.add_common_parameter("control_integral")
        self.add_common_parameter("now", vals=vals.Numbers(0, 1))

        # Additional parameters for fast piezo control
        self.add_parameter("lock",
                           label="Was laser locked",
                           get_cmd=self._cmd_prefix + "LOCK?",
                           val_mapping={True: "TRUE", False: "FALSE"},
                           docstring="""
                           This parameter tracks whether the laser was locked. The laser is
                           considered to be locked whenever the tweeter is within 5%...95% of its
                           tuning range.
                           
                           The concept behind that criteria is the following: if the laser is not
                           locked, it will not react to the tweeter. So any error will be integrated
                           until the tweeter is on the lower or upper end of its tuning range.
                           Retrieving this value automatically reset the variable value to TRUE.
                           Whenever the tweeter is within the last 5% of it's tuning range the
                           variable is set FALSE.
                           """)

        self.add_parameter("input",
                           label="Current voltage at reference cell or external input",
                           get_cmd=self._cmd_prefix + "INP?",
                           vals=vals.Numbers(-1, 1),
                           docstring="""
                           Gets the current value of the diode at the reference cell (or the current
                           voltage at the external input of the DSP board). The value is normalized
                           to be in the range -1..1. This is a read only value.
                           """)

        self.add_parameter("lock_point",
                           label="Initial control goal value",
                           get_cmd=self._cmd_prefix + "LKP?",
                           set_cmd=self._cmd_prefix + "LKP {}",
                           get_parser=float,
                           vals=vals.Numbers(),
                           # TODO unit="???",
                           docstring="""
                           Gets/sets the value for the initial control goal value. When the laser
                           needs to perform an initial lock or a relocking, the control loop will
                           lock the laser to the value given by FASTPIEZO:LOCKPOINT. After the laser
                           is stabilized to the FASTPIEZO:LOCKPOINT value, the control loop will
                           change its control goal to FASTPIEZO:CONTROLSETPOINT after a while. The
                           change will be smooth.
                           """)


class SirahMatisseSlowPiezoControl(SirahMatisseChannel):
    """Instrument channel for slow piezo control of Sirah Matisse

    Args:
        parent: Parent SirahMatisse instrument object
        name: Name of this channel
    """

    def __init__(self, parent: "SirahMatisse", name: str, **kwargs):
        super().__init__(parent, name, "SPZT", **kwargs)

        # Pre-defined parameters/functions
        self.add_common_parameter("reference_scan")
        self.add_common_parameter("control_setpoint")
        self.add_common_parameter("control_status")
        self.add_common_parameter("now")

        # Additional parameters for slow piezo control
        self.add_parameter("lock_proportional",
                           label="Proportional gain",
                           get_cmd=self._cmd_prefix + "LPROP?",
                           set_cmd=self._cmd_prefix + "LPROP {}",
                           get_parser=float,
                           vals=vals.Numbers(),
                           docstring="""
                           Gets/sets the proportional gain of the slow piezo (cavity scan piezo)
                           control loop. This value is used when the control loop detects that the
                           laser is locked to the reference cell.
                           """)

        self.add_parameter("lock_integral",
                           label="Integral gain",
                           get_cmd=self._cmd_prefix + "LINT?",
                           set_cmd=self._cmd_prefix + "LINT {}",
                           get_parser=float,
                           vals=vals.Numbers(),
                           docstring="""
                           Gets/sets the integral gain of the slow piezo (cavity scan piezo) control
                           loop. This value is used when the control loop detects that the laser is
                           locked to the reference cell.
                           """)

        self.add_parameter("free_speed",
                           label="Speed",
                           get_cmd=self._cmd_prefix + "FRSP?",
                           set_cmd=self._cmd_prefix + "FRSP {}",
                           get_parser=float,
                           vals=vals.Numbers(),
                           docstring="""
                           Gets/sets the speed of the slow piezo (cavity scan piezo) control loop.
                           This value is used when the control loop detects that the laser is not
                           locked to the reference cell.
                           """)


class SirahMatisse(VisaInstrument):
    """Sirah Matisse laser instrument driver

    Args:
        name: Name of the instrument
        address: VISA resource name (used to establish a connection to the instrument)
    """

    def __init__(self, name: str, address: str):
        super().__init__(name, address)

        self.add_parameter("error_codes",
                           label="Last error codes",
                           get_cmd="ERR:CODE?",
                           get_parser=self._parse_error_codes,
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
            "pid_loops": SirahMatissePidLoops,  # PID loops
            "power_diode": SirahMatissePowerDiode,  # power diode
            "motor_bifi": SirahMatisseBiFiMotor,  # birefringent filter motor
            "motor_thin_etalon": SirahMatisseThinEtalonMotor,  # thin etalon motor
            "piezo_etalon": SirahMatissePiezoEtalonControl,  # piezo etalon control
            "thin_etalon": SirahMatisseThinEtalonControl,  # thin etalon control
            "scan": SirahMatisseScanControl,  # scan control
            "fast_piezo": SirahMatisseFastPiezoControl,  # fast piezo control
            "slow_piezo": SirahMatisseSlowPiezoControl  # slow piezo control
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
                    exc = None
                except Exception as exc:
                    err_code = None

                try:
                    err_codes_list = self.error_codes()
                except Exception:
                    err_codes_list = [err_code] if err_code is not None else None

                # Try to clear error buffer
                try:
                    self.clear_errors()
                except Exception as exc:
                    warnings.warn(f"Couldn't clear error buffer after receiving an error "
                                  f"response.\n -> {type(exc).__name__}: {exc}")

                if exc is None:
                    raise SirahMatisseError(f"Error querying \"{cmd}\": {err_code}",
                                            err_codes_list)
                else:
                    raise SirahMatisseError(f"Unknown error querying \"{cmd}\"",
                                            err_codes_list) from exc
            elif response == "OK":
                return ""
            else:
                try:
                    # Extract result from response-string
                    result = response.split(maxsplit=1)[1]

                    # If response is surrounded by quotes, remove them
                    if result[0] == result[-1] == "\"":
                        return result[1:-1]
                    else:
                        return result
                except Exception:
                    return ""

    def write(self, *args, **kwargs):
        """This instrument always gives an answer, even when writing. Therefore the write-function
        is redirected to the ask-function, so that the answer is popped from the answer queue.
        """
        self.ask(*args, **kwargs)

    @staticmethod
    def _parse_error_codes(error_codes: str) -> List[int]:
        """Convert string with space-separated numbers into list of integers

        Args:
            error_codes: Error-codes string which needs to be parsed. This is the response string
                         when requesting `ERR:CODES?`

        Returns:
            List of integer error-codes
        """
        return [int(code) for code in error_codes.split()]
