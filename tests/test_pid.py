"""Testes determinísticos, sem CoppeliaSim, NumPy ou Raspberry Pi."""
import importlib
import math
import pathlib
import subprocess
import sys
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[1]


class PIDContract:
    def make_pid(self, *gains, **kwargs):
        pid = self.module.PID(*(gains or (2, 1, 0.5)), **kwargs)
        pid.set_mode(self.module.AUTOMATIC)
        return pid

    def test_reference_sequence(self):
        # Resultados de PID_v1: Ts=.1, kp=2, ki=.1, kd=5.
        pid = self.make_pid(input_value=1, output=.25,
                            output_limits=(-100, 100))
        self.assertTrue(pid.compute(1, 2, now=0))
        self.assertAlmostEqual(pid.output, 2.35)
        self.assertFalse(pid.compute(9, 2, now=.05))
        self.assertAlmostEqual(pid.output, 2.35)
        self.assertTrue(pid.compute(1.2, 2, now=.1))
        self.assertAlmostEqual(pid.output, 1.03)
        self.assertTrue(pid.compute(1.3, 3, now=.3))
        self.assertAlmostEqual(pid.output, 3.5)

    def test_reference_filter(self):
        pid = self.make_pid(1, 0, 0, sample_time=.1,
                            reference_filter_tau=.2,
                            output_limits=(-100, 100))
        self.assertAlmostEqual(pid.update(0, 0, now=0), 0)
        self.assertAlmostEqual(pid.update(0, 1, now=.1),
                               1 - math.exp(-.5))
        self.assertAlmostEqual(pid.update(0, 1, now=.2),
                               1 - math.exp(-1.0))

    def test_reference_filter_validation(self):
        with self.assertRaises(ValueError):
            self.module.PID(1, 0, 0, reference_filter_tau=-1)

    def test_defaults_and_manual(self):
        pid = self.module.PID(1, 2, 3, output=7)
        self.assertEqual(pid.mode, self.module.MANUAL)
        self.assertEqual(pid.sample_time, .1)
        self.assertEqual(pid.output_limits, (0, 255))
        self.assertFalse(pid.compute(10, 20, now=0))
        self.assertEqual(pid.output, 7)

    def test_update_and_injected_clock(self):
        ticks = iter([0, .05, .1])
        pid = self.make_pid(0, 1, 0, clock=lambda: next(ticks))
        self.assertAlmostEqual(pid.update(0, 1), .1)
        self.assertAlmostEqual(pid.update(0, 1), .1)
        self.assertAlmostEqual(pid.update(0, 1), .2)

    def test_long_delay_is_one_nominal_step(self):
        pid = self.make_pid(0, 1, 0)
        self.assertAlmostEqual(pid.update(0, 1, now=0), .1)
        self.assertAlmostEqual(pid.update(0, 1, now=10), .2)

    def test_floating_point_sample_boundaries(self):
        pid = self.make_pid(0, 1, 0)
        for tick in range(101):
            self.assertTrue(pid.compute(0, 1, now=tick * .1))
        self.assertAlmostEqual(pid.output, 10.1)
        self.assertFalse(pid.compute(0, 1, now=10.099))

    def test_integrator_clamping_and_recovery(self):
        pid = self.make_pid(0, 10, 0, output_limits=(-1, 1))
        for tick in range(20):
            self.assertEqual(pid.update(0, 10, now=tick), 1)
        self.assertEqual(pid.update(0, -.5, now=20), .5)
        for tick in range(21, 40):
            self.assertEqual(pid.update(0, -10, now=tick), -1)
        self.assertEqual(pid.update(0, .5, now=40), -.5)

    def test_no_derivative_kick_on_setpoint(self):
        pid = self.make_pid(0, 0, 1, input_value=10,
                            output_limits=(-100, 100))
        self.assertEqual(pid.update(10, 10, now=0), 0)
        self.assertEqual(pid.update(10, 20, now=.1), 0)
        self.assertAlmostEqual(pid.update(11, 20, now=.2), -10)

    def test_proportional_on_measurement(self):
        pid = self.make_pid(input_value=2, output=5,
                            proportional_on=self.module.P_ON_M)
        self.assertAlmostEqual(pid.update(2, 3, now=0), 5.1)
        self.assertAlmostEqual(pid.update(2.2, 3, now=.1), 3.78)

    def test_reverse_action(self):
        direct = self.make_pid(output_limits=(-100, 100))
        reverse = self.make_pid(direction=self.module.REVERSE,
                                output_limits=(-100, 100))
        for t, measurement in [(0, 0), (.1, .2), (.2, .4)]:
            self.assertAlmostEqual(direct.update(measurement, 1, now=t),
                                   -reverse.update(measurement, 1, now=t))

    def test_direction_changes_in_manual_and_auto(self):
        pid = self.make_pid(1, 0, 0, output_limits=(-10, 10))
        self.assertEqual(pid.update(0, 1, now=0), 1)
        pid.set_controller_direction(self.module.REVERSE)
        self.assertEqual(pid.update(0, 1, now=.1), -1)
        pid.set_mode(self.module.MANUAL)
        pid.set_controller_direction(self.module.DIRECT)
        pid.output = 0
        pid.set_mode(self.module.AUTOMATIC)
        self.assertEqual(pid.update(0, 1, now=.2), 1)

    def test_manual_transfer_uses_current_input_and_output(self):
        pid = self.make_pid(0, 0, 2, output_limits=(-100, 100))
        pid.update(0, 0, now=0)
        pid.set_mode(self.module.MANUAL)
        pid.output = 4
        self.assertFalse(pid.compute(8, 8, now=1))
        pid.set_mode(self.module.AUTOMATIC)
        self.assertEqual(pid.update(8, 8, now=1), 4)
        pid.set_mode(self.module.AUTOMATIC)  # não reinicializa enquanto ativo
        self.assertAlmostEqual(pid.update(8.1, 8, now=1.1), 2)

    def test_sample_time_rescales_integral_and_derivative(self):
        pid = self.make_pid(0, 2, 1, output_limits=(-100, 100))
        self.assertAlmostEqual(pid.update(0, 1, now=0), .2)
        pid.set_sample_time(.2)
        self.assertFalse(pid.compute(1, 1, now=.1))
        self.assertAlmostEqual(pid.update(1, 1, now=.2), -4.8)
        self.assertEqual(pid.tunings, (0, 2, 1))

    def test_new_limits_clamp_output_and_accumulator(self):
        pid = self.make_pid(0, 10, 0, output_limits=(-10, 10))
        self.assertEqual(pid.update(0, 10, now=0), 10)
        pid.set_output_limits(-1, 1)
        self.assertEqual(pid.output, 1)
        self.assertEqual(pid.update(0, 0, now=.1), 1)

    def test_reset_after_simulation_restart(self):
        pid = self.make_pid(0, 0, 1)
        pid.update(0, 0, now=20)
        with self.assertRaises(ValueError):
            pid.update(0, 0, now=0)
        pid.reset(input_value=4, output=5)
        self.assertEqual(pid.update(4, 0, now=0), 5)
        self.assertEqual(pid.mode, self.module.AUTOMATIC)

    def test_arduino_aliases_and_runtime_tuning(self):
        pid = self.module.PID(1, 2, 3)
        pid.SetSampleTime(200)
        pid.SetTunings(2, 3, 4, self.module.P_ON_M)
        pid.SetTunings(3, 4, 5)  # preserva P_ON_M
        pid.SetOutputLimits(-10, 10)
        pid.SetMode(self.module.AUTOMATIC)
        pid.input, pid.setpoint = 0, 1
        self.assertTrue(pid.Compute(now=0))
        self.assertAlmostEqual(pid.output, .8)
        self.assertEqual((pid.GetKp(), pid.GetKi(), pid.GetKd()), (3, 4, 5))
        self.assertEqual(pid.sample_time, .2)
        self.assertEqual(pid.GetMode(), self.module.AUTOMATIC)
        self.assertEqual(pid.GetDirection(), self.module.DIRECT)

    def test_invalid_configuration(self):
        for options in ({"sample_time": 0}, {"sample_time": -1},
                        {"sample_time": float("nan")},
                        {"output_limits": (1, 1)},
                        {"output_limits": (0, float("inf"))},
                        {"direction": 7}, {"proportional_on": 7}):
            with self.subTest(options=options), self.assertRaises(ValueError):
                self.module.PID(1, 1, 1, **options)
        for gains in [(-1, 0, 0), (0, float("nan"), 0), (0, 0, float("inf"))]:
            with self.assertRaises(ValueError):
                self.module.PID(*gains)

    def test_bad_samples_do_not_corrupt_history(self):
        pid = self.make_pid(0, 1, 0)
        pid.update(0, 1, now=0)
        for sample, reference, now in [(float("nan"), 1, .1),
                                       (0, float("inf"), .1),
                                       (0, 1, float("nan"))]:
            with self.assertRaises(ValueError):
                pid.compute(sample, reference, now=now)
        self.assertAlmostEqual(pid.update(0, 1, now=.1), .2)


class SimulatorPID(PIDContract, unittest.TestCase):
    module = importlib.import_module("simulador.fva_car.pid")


class RealPID(PIDContract, unittest.TestCase):
    module = importlib.import_module("veiculo_real.fva_car.pid")


class Distribution(unittest.TestCase):
    def test_copies_are_identical(self):
        self.assertEqual((ROOT / "simulador/fva_car/pid.py").read_bytes(),
                         (ROOT / "veiculo_real/fva_car/pid.py").read_bytes())

    def test_standalone_import_without_hardware_dependencies(self):
        for environment in ("simulador", "veiculo_real"):
            result = subprocess.run(
                [sys.executable, "-S", "-B", "-c",
                 "from fva_car import PID, AUTOMATIC; "
                 "p = PID(1, 0, 0); p.set_mode(AUTOMATIC); "
                 "assert p.update(0, 1, now=0) == 1"],
                cwd=ROOT / environment, capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stderr)


if __name__ == "__main__":
    unittest.main()
