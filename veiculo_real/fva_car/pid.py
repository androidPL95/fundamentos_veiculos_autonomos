"""PID digital baseado em PID_v1 1.2.1, de Brett Beauregard (licença MIT).

API Python em segundos; SetSampleTime em milissegundos, como no Arduino.
Distribuído identicamente nos dois ambientes para permitir sua cópia isolada.
"""

import math
import time

MANUAL, AUTOMATIC = 0, 1
DIRECT, REVERSE = 0, 1
P_ON_M, P_ON_E = 0, 1
__all__ = ["PID", "MANUAL", "AUTOMATIC", "DIRECT", "REVERSE", "P_ON_M", "P_ON_E"]


def _finite(value, name):
    value = float(value)
    if not math.isfinite(value):
        raise ValueError(f"{name} deve ser finito")
    return value


class PID:
    """PID posicional com derivada na medição e saturação do acumulador.

    Ki é multiplicado por Ts; Kd é dividido por Ts. Uma chamada atrasada
    executa apenas uma atualização com Ts nominal, como PID_v1.
    O modo inicial é MANUAL. Defina input/output antes de ativar AUTOMATIC
    para inicializar o histórico com a medição e o comando atuais.
    reference_filter_tau define, em segundos, o filtro de primeira ordem
    aplicado ao setpoint; zero desativa o filtro.
    Use sempre a mesma base de tempo: now=car.t no simulador ou clock
    (time.monotonic por padrão) no ambiente real. Use reset ao reiniciar
    o tempo. Uma instância pertence a uma única malha/thread de controle.
    """

    MANUAL, AUTOMATIC = MANUAL, AUTOMATIC
    DIRECT, REVERSE = DIRECT, REVERSE
    P_ON_M, P_ON_E = P_ON_M, P_ON_E

    def __init__(self, kp, ki, kd, direction=DIRECT, proportional_on=P_ON_E,
                 *, sample_time=0.1, output_limits=(0.0, 255.0),
                 setpoint=0.0, input_value=0.0, output=0.0, clock=None,
                 reference_filter_tau=0.0):
        self.input = _finite(input_value, "input_value")
        self.setpoint = _finite(setpoint, "setpoint")
        self.output = _finite(output, "output")
        self._clock = time.monotonic if clock is None else clock
        self._auto = False
        self._last_time = None
        self._last_input = self.input
        self._output_sum = 0.0
        self._sample_time = _finite(sample_time, "sample_time")
        if self._sample_time <= 0:
            raise ValueError("sample_time deve ser > 0")
        self._reference_filter_tau = _finite(
            reference_filter_tau, "reference_filter_tau")
        if self._reference_filter_tau < 0:
            raise ValueError("reference_filter_tau deve ser >= 0")
        self._filtered_setpoint = self.setpoint
        if direction not in (DIRECT, REVERSE):
            raise ValueError("direction deve ser DIRECT ou REVERSE")
        self._direction = direction
        self._proportional_on = P_ON_E
        self.set_output_limits(*output_limits)
        self.set_tunings(kp, ki, kd, proportional_on)

    @property
    def tunings(self):
        return self._tunings

    @property
    def sample_time(self):
        return self._sample_time

    @property
    def output_limits(self):
        return self._out_min, self._out_max

    @property
    def reference_filter_tau(self):
        return self._reference_filter_tau

    @property
    def mode(self):
        return AUTOMATIC if self._auto else MANUAL

    @property
    def direction(self):
        return self._direction

    def _clamp(self, value):
        return min(self._out_max, max(self._out_min, value))

    def compute(self, input_value=None, setpoint=None, *, now=None):
        """Atualiza output se automático e Ts venceu; retorna se calculou.

        A primeira chamada automática calcula imediatamente. Antes de Ts,
        mantém saída e histórico, mesmo recebendo novas medições.
        input/setpoint também podem ser atribuídos diretamente, como as
        variáveis ligadas por ponteiros na biblioteca original.
        """
        measurement = _finite(self.input if input_value is None else input_value,
                              "input_value")
        reference = _finite(self.setpoint if setpoint is None else setpoint,
                            "setpoint")
        timestamp = _finite(self._clock() if now is None else now, "now")
        if self._last_time is not None and timestamp < self._last_time:
            raise ValueError("O tempo retrocedeu; chame reset() antes de reiniciar")
        self.input, self.setpoint = measurement, reference
        if not self._auto:
            return False
        if self._last_time is not None:
            elapsed = timestamp - self._last_time
            # Tolera apenas arredondamento nas fronteiras de amostragem.
            if elapsed < self._sample_time and not math.isclose(
                    elapsed, self._sample_time, rel_tol=1e-9, abs_tol=0.0):
                return False
        if self._reference_filter_tau > 0.0:
            alpha = math.exp(-self._sample_time / self._reference_filter_tau)
            self._filtered_setpoint = (
                alpha * self._filtered_setpoint
                + (1.0 - alpha) * reference
            )
        else:
            self._filtered_setpoint = reference
        error = self._filtered_setpoint - measurement
        delta_input = measurement - self._last_input
        output_sum = self._output_sum + self._ki * error
        if self._proportional_on == P_ON_M:
            output_sum -= self._kp * delta_input
        output_sum = self._clamp(output_sum)
        proportional = self._kp * error if self._proportional_on == P_ON_E else 0.0
        output = proportional + output_sum - self._kd * delta_input
        self.output = self._clamp(output)
        self._output_sum = output_sum
        self._last_input = measurement
        self._last_time = timestamp
        return True

    def update(self, input_value, setpoint=None, *, now=None):
        """Retorna a saída calculada ou mantida entre amostras."""
        self.compute(input_value, setpoint, now=now)
        return self.output

    def set_tunings(self, kp, ki, kd, proportional_on=None):
        gains = tuple(_finite(v, name) for v, name in
                      zip((kp, ki, kd), ("kp", "ki", "kd")))
        if any(v < 0 for v in gains):
            raise ValueError("Os ganhos devem ser >= 0; use REVERSE para inverter")
        if proportional_on is None:
            proportional_on = self._proportional_on
        if proportional_on not in (P_ON_E, P_ON_M):
            raise ValueError("proportional_on deve ser P_ON_E ou P_ON_M")
        self._tunings = gains
        self._proportional_on = proportional_on
        sign = -1.0 if self._direction == REVERSE else 1.0
        self._kp = sign * gains[0]
        self._ki = sign * gains[1] * self._sample_time
        self._kd = sign * gains[2] / self._sample_time

    def set_sample_time(self, sample_time):
        """Altera Ts em segundos, reescalando os ganhos discretos."""
        sample_time = _finite(sample_time, "sample_time")
        if sample_time <= 0:
            raise ValueError("sample_time deve ser > 0")
        self._sample_time = sample_time
        self.set_tunings(*self._tunings)

    def set_output_limits(self, minimum, maximum):
        minimum = _finite(minimum, "minimum")
        maximum = _finite(maximum, "maximum")
        if minimum >= maximum:
            raise ValueError("minimum deve ser menor que maximum")
        self._out_min, self._out_max = minimum, maximum
        if self._auto:
            self.output = self._clamp(self.output)
            self._output_sum = self._clamp(self._output_sum)

    def initialize(self):
        """Sincroniza acumulador/medição na transição manual → automático.

        Como PID_v1, inicializa o acumulador com o comando atual; P_ON_E
        ainda pode produzir um salto proporcional se houver erro.
        """
        output = _finite(self.output, "output")
        measurement = _finite(self.input, "input")
        self._output_sum = self._clamp(output)
        self._last_input = measurement

    def set_mode(self, mode):
        if mode not in (MANUAL, AUTOMATIC):
            raise ValueError("mode deve ser MANUAL ou AUTOMATIC")
        if mode == AUTOMATIC and not self._auto:
            self.initialize()
        self._auto = mode == AUTOMATIC

    def set_controller_direction(self, direction):
        if direction not in (DIRECT, REVERSE):
            raise ValueError("direction deve ser DIRECT ou REVERSE")
        self._direction = direction
        # Recalcula também em MANUAL para não reativar com sinais antigos.
        self.set_tunings(*self._tunings)

    def reset(self, input_value=0.0, output=0.0):
        """Reinicia histórico/relógio, preservando modo, ganhos e referência."""
        measurement = _finite(input_value, "input_value")
        output = _finite(output, "output")
        self.input = measurement
        self.output = self._clamp(output)
        self._filtered_setpoint = self.setpoint
        self.initialize()
        self._last_time = None

    Compute = compute
    SetMode = set_mode
    SetOutputLimits = set_output_limits
    SetTunings = set_tunings
    SetControllerDirection = set_controller_direction
    Initialize = initialize

    def SetSampleTime(self, milliseconds):
        self.set_sample_time(_finite(milliseconds, "milliseconds") / 1000.0)

    def GetKp(self):
        return self.tunings[0]

    def GetKi(self):
        return self.tunings[1]

    def GetKd(self):
        return self.tunings[2]

    def GetMode(self):
        return self.mode

    def GetDirection(self):
        return self.direction
