"""
Hardware Abstraction Layer for Motor Drivers, Quadrature Encoders, Relay, and E-Stop.
Supports both Physical Raspberry Pi 4B hardware and Mock simulation mode.
"""

import abc
import time
import math
from typing import Tuple


class BaseHardwareInterface(abc.ABC):
    """
    Abstract hardware interface for mobile robot base actuation and feedback.
    """

    @abc.abstractmethod
    def set_motor_speeds(self, left_pwm: float, right_pwm: float, left_dir: bool, right_dir: bool) -> None:
        """
        Apply PWM and direction signals to motor drivers.
        :param left_pwm: 0.0 to 100.0 duty cycle
        :param right_pwm: 0.0 to 100.0 duty cycle
        :param left_dir: True = Forward, False = Reverse
        :param right_dir: True = Forward, False = Reverse
        """
        pass

    @abc.abstractmethod
    def read_encoder_ticks(self) -> Tuple[int, int]:
        """
        Return accumulated (left_ticks, right_ticks).
        """
        pass

    @abc.abstractmethod
    def read_estop(self) -> bool:
        """
        Return True if Emergency Stop button is pressed/triggered.
        """
        pass

    @abc.abstractmethod
    def set_relay(self, active: bool) -> None:
        """
        Enable (True) or cut (False) motor power relay.
        """
        pass

    @abc.abstractmethod
    def stop_all(self) -> None:
        """
        Immediately command zero power to motors.
        """
        pass

    @abc.abstractmethod
    def cleanup(self) -> None:
        """
        Safely de-energize pins and release hardware resources.
        """
        pass


class MockHardwareDriver(BaseHardwareInterface):
    """
    Simulated Hardware Interface for testing on development workstations
    without physical motor drivers or Raspberry Pi GPIO.
    """

    def __init__(self, ticks_per_rev: int = 1920, wheel_radius: float = 0.075,
                 wheel_separation: float = 0.40):
        self.ticks_per_rev = ticks_per_rev
        self.wheel_radius = wheel_radius
        self.wheel_separation = wheel_separation

        self.left_ticks = 0
        self.right_ticks = 0

        self.left_target_speed_mps = 0.0
        self.right_target_speed_mps = 0.0
        self.estop_state = False
        self.relay_state = True
        self.last_update_time = time.time()

    def set_motor_speeds(self, left_pwm: float, right_pwm: float, left_dir: bool, right_dir: bool) -> None:
        self._update_simulation()
        # Max speed 0.6 m/s corresponds to 100% PWM
        max_speed = 0.60
        left_sign = 1.0 if left_dir else -1.0
        right_sign = 1.0 if right_dir else -1.0

        self.left_target_speed_mps = left_sign * (min(100.0, max(0.0, left_pwm)) / 100.0) * max_speed
        self.right_target_speed_mps = right_sign * (min(100.0, max(0.0, right_pwm)) / 100.0) * max_speed

    def _update_simulation(self) -> None:
        now = time.time()
        dt = now - self.last_update_time
        self.last_update_time = now

        if not self.relay_state or self.estop_state:
            return

        # Distance moved in meters
        d_left = self.left_target_speed_mps * dt
        d_right = self.right_target_speed_mps * dt

        # Convert to encoder ticks
        ticks_per_meter = self.ticks_per_rev / (2.0 * math.pi * self.wheel_radius)
        self.left_ticks += int(round(d_left * ticks_per_meter))
        self.right_ticks += int(round(d_right * ticks_per_meter))

    def read_encoder_ticks(self) -> Tuple[int, int]:
        self._update_simulation()
        return self.left_ticks, self.right_ticks

    def read_estop(self) -> bool:
        return self.estop_state

    def set_estop_simulated(self, active: bool) -> None:
        self.estop_state = active
        if active:
            self.stop_all()

    def set_relay(self, active: bool) -> None:
        self.relay_state = active
        if not active:
            self.stop_all()

    def stop_all(self) -> None:
        self.left_target_speed_mps = 0.0
        self.right_target_speed_mps = 0.0

    def cleanup(self) -> None:
        self.stop_all()


class RealHardwareDriver(BaseHardwareInterface):
    """
    Physical Raspberry Pi 4B Hardware Driver for Cytron / BTS7960 Motor Driver
    and Hall Effect Quadrature Encoders.
    """

    def __init__(self,
                 left_pwm_pin: int = 12, left_dir_pin: int = 23,
                 right_pwm_pin: int = 13, right_dir_pin: int = 24,
                 left_enc_a: int = 17, left_enc_b: int = 27,
                 right_enc_a: int = 22, right_enc_b: int = 10,
                 estop_pin: int = 25, relay_pin: int = 18,
                 pwm_freq: int = 20000):

        self.left_pwm_pin = left_pwm_pin
        self.left_dir_pin = left_dir_pin
        self.right_pwm_pin = right_pwm_pin
        self.right_dir_pin = right_dir_pin
        self.left_enc_a = left_enc_a
        self.left_enc_b = left_enc_b
        self.right_enc_a = right_enc_a
        self.right_enc_b = right_enc_b
        self.estop_pin = estop_pin
        self.relay_pin = relay_pin
        self.pwm_freq = pwm_freq

        self.left_ticks = 0
        self.right_ticks = 0
        self.is_initialized = False

        self._init_gpio()

    def _init_gpio(self) -> None:
        try:
            import RPi.GPIO as GPIO  # type: ignore
            self.GPIO = GPIO
            self.GPIO.setmode(self.GPIO.BCM)
            self.GPIO.setwarnings(False)

            # Motors
            self.GPIO.setup(self.left_dir_pin, self.GPIO.OUT, initial=self.GPIO.LOW)
            self.GPIO.setup(self.right_dir_pin, self.GPIO.OUT, initial=self.GPIO.LOW)
            self.GPIO.setup(self.left_pwm_pin, self.GPIO.OUT)
            self.GPIO.setup(self.right_pwm_pin, self.GPIO.OUT)

            self.left_pwm = self.GPIO.PWM(self.left_pwm_pin, self.pwm_freq)
            self.right_pwm = self.GPIO.PWM(self.right_pwm_pin, self.pwm_freq)
            self.left_pwm.start(0.0)
            self.right_pwm.start(0.0)

            # Encoders
            self.GPIO.setup(self.left_enc_a, self.GPIO.IN, pull_up_down=self.GPIO.PUD_UP)
            self.GPIO.setup(self.left_enc_b, self.GPIO.IN, pull_up_down=self.GPIO.PUD_UP)
            self.GPIO.setup(self.right_enc_a, self.GPIO.IN, pull_up_down=self.GPIO.PUD_UP)
            self.GPIO.setup(self.right_enc_b, self.GPIO.IN, pull_up_down=self.GPIO.PUD_UP)

            # Attach interrupts
            self.GPIO.add_event_detect(self.left_enc_a, self.GPIO.RISING, callback=self._left_enc_cb)
            self.GPIO.add_event_detect(self.right_enc_a, self.GPIO.RISING, callback=self._right_enc_cb)

            # E-Stop and Relay
            self.GPIO.setup(self.estop_pin, self.GPIO.IN, pull_up_down=self.GPIO.PUD_UP)
            self.GPIO.setup(self.relay_pin, self.GPIO.OUT, initial=self.GPIO.HIGH)

            self.is_initialized = True
        except ImportError:
            raise RuntimeError(
                "RPi.GPIO is not available on this system. Set 'use_mock_hardware: true' "
                "or install RPi.GPIO on Raspberry Pi."
            )

    def _left_enc_cb(self, channel: int) -> None:
        state_b = self.GPIO.input(self.left_enc_b)
        if state_b:
            self.left_ticks += 1
        else:
            self.left_ticks -= 1

    def _right_enc_cb(self, channel: int) -> None:
        state_b = self.GPIO.input(self.right_enc_b)
        if state_b:
            self.right_ticks -= 1
        else:
            self.right_ticks += 1

    def set_motor_speeds(self, left_pwm: float, right_pwm: float, left_dir: bool, right_dir: bool) -> None:
        if not self.is_initialized:
            return
        self.GPIO.output(self.left_dir_pin, self.GPIO.HIGH if left_dir else self.GPIO.LOW)
        self.GPIO.output(self.right_dir_pin, self.GPIO.HIGH if right_dir else self.GPIO.LOW)
        self.left_pwm.ChangeDutyCycle(min(100.0, max(0.0, left_pwm)))
        self.right_pwm.ChangeDutyCycle(min(100.0, max(0.0, right_pwm)))

    def read_encoder_ticks(self) -> Tuple[int, int]:
        return self.left_ticks, self.right_ticks

    def read_estop(self) -> bool:
        if not self.is_initialized:
            return False
        # Normally-Closed (NC) E-stop: LOW means switch opened / button pushed
        return self.GPIO.input(self.estop_pin) == self.GPIO.LOW

    def set_relay(self, active: bool) -> None:
        if self.is_initialized:
            self.GPIO.output(self.relay_pin, self.GPIO.HIGH if active else self.GPIO.LOW)

    def stop_all(self) -> None:
        if self.is_initialized:
            self.left_pwm.ChangeDutyCycle(0.0)
            self.right_pwm.ChangeDutyCycle(0.0)

    def cleanup(self) -> None:
        if self.is_initialized:
            self.stop_all()
            self.set_relay(False)
            self.left_pwm.stop()
            self.right_pwm.stop()
            self.GPIO.cleanup()
            self.is_initialized = False
