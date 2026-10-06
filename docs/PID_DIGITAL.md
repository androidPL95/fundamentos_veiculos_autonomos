# PID digital em Python

Os módulos `simulador/fva_car/pid.py` e `veiculo_real/fva_car/pid.py` implementam
as equações de `Arduino-PID-Library-master/PID_v1.cpp` e `PID_v1.h`, versão
1.2.1 de Brett Beauregard, sob licença MIT. Usam apenas a biblioteca padrão.
As duas cópias são idênticas e verificadas por teste, pois o Docker monta apenas
`simulador/` e o veículo pode receber apenas `veiculo_real/`.

Em qualquer uma dessas pastas:

```python
from fva_car import PID, AUTOMATIC

pid = PID(0.4, 0.1, 0.02, sample_time=0.1, output_limits=(-1.0, 1.0))
pid.set_mode(AUTOMATIC)
u = pid.update(input_value=0.2, setpoint=0.7, now=0.0)
```

Os ganhos do exemplo são ilustrativos, não uma sintonia validada para os carros.
O módulo está disponível para novas malhas; os programas `main.py` e os
controladores atuais de `Car.set_vel()` mantêm seu comportamento.

## Equações e unidades

Para referência filtrada `r_f`, medição y e período Ts, os ganhos internos são
`kp = Kp`, `ki = Ki * Ts`, `kd = Kd / Ts`. Em cada atualização:

1. `erro = r_f - y` e `delta_y = y - y_anterior`.
2. `acumulador += ki * erro`.
3. Com `P_ON_M`, também `acumulador -= kp * delta_y`.
4. O acumulador é limitado entre os limites de saída (anti-windup por saturação).
5. `u = acumulador - kd * delta_y`, somando `kp * erro` com `P_ON_E`.
6. A saída também é limitada.

`REVERSE` inverte o sinal dos três ganhos internos. Os ganhos fornecidos
continuam não negativos. A derivada atua na medição, evitando impulso derivativo
quando apenas a referência muda. O limite do acumulador não compensa saturações
adicionais externas ao PID.

`sample_time` e `now` são em **segundos**. A primeira chamada automática calcula
imediatamente; chamadas antes de Ts mantêm a saída. Um atraso executa uma única
atualização com Ts nominal, sem recuperar amostras perdidas. Portanto, a malha
precisa chamar o PID com frequência suficiente. Não use o `dt` como `now`:
`now` é o tempo acumulado. Ao reiniciar uma missão/relógio, chame `reset()`.

### Filtro interno da referência

O construtor aceita `reference_filter_tau`, em segundos, para suavizar o
`setpoint` dentro do próprio PID. Quando `reference_filter_tau > 0`, em cada
atualização aceita pelo período de amostragem, o controlador calcula:

```text
alpha = exp(-sample_time / reference_filter_tau)
r_f = alpha * r_f_anterior + (1 - alpha) * r
```

O filtro começa com `r_f = setpoint` e `reference_filter_tau=0.0` (padrão)
desativa a filtragem, fazendo `r_f = r`. Chamadas ignoradas por ocorrerem antes
de `sample_time` não avançam o filtro. `reset()` reinicia `r_f` para o
`setpoint` atual. O filtro reduz mudanças bruscas de referência, mas acrescenta
atraso à malha; considere esse atraso ao ajustar os ganhos.

Exemplo, com constante de tempo de 0,5 s:

```python
pid = PID(
  0.4, 0.1, 0.02,
  sample_time=0.1,
  reference_filter_tau=0.5,
  output_limits=(-1.0, 1.0),
)
```

## Aplicação no simulador

Crie uma instância depois de `car.start_mission()` e reutilize-a no laço.
Este exemplo controla velocidade para frente, usando o tempo do CoppeliaSim:

```python
from fva_car import PID, AUTOMATIC
from fva_car.car import CAR

car.set_forward()
pid = PID(
    0.4, 0.1, 0.02,  # ajustar os ganhos para o modelo
    sample_time=0.1,
    output_limits=(-CAR['ACCELMAX'], CAR['ACCELMAX']),
    input_value=abs(car.v), output=car.u,
)
pid.set_mode(AUTOMATIC)

# Dentro do laço principal, depois de car.step():
car.vref = 0.7  # m/s; referência para frente e para o log
car.set_u(pid.update(abs(car.v), car.vref, now=car.t))
```

A saída já é o comando de aceleração aceito por `set_u`: não some novamente a
`car.u` nem multiplique por `car.dt`. Não chame `car.set_vel()` no mesmo ciclo,
pois esse método executa outro controlador. Para controlar ré, a seleção da
marcha deve continuar sob responsabilidade do carro; reinicialize o PID com a
medição e o comando atuais após a troca.

## Aplicação no veículo real

Use a mesma inicialização e os limites de `CAR['ACCELMAX']`. `sample_time=0.02`
corresponde à taxa nominal de sensores de 50 Hz deste projeto; os ganhos devem
ser ajustados considerando também seus filtros. O PID usa `time.monotonic()`
quando `now` é omitido, ou aceita `now=car.t` em todas as chamadas.

Depois de `car.step()`, um exemplo para frente, com o ultrassom do programa:

```python
dist, valid = car.get_distance()
if car.emergencia or not valid or dist < 0.20:
    car.vref = 0.0
    car.set_u(-CAR['ACCELMAX'])
    pid.reset(input_value=abs(car.v), output=car.u)
else:
    car.vref = 0.7
    car.set_u(pid.update(abs(car.v), car.vref))
```

Mantenha `car.close()` no `finally`, como nos programas atuais. Esses trechos
usam referência direta; não aplicam o filtro de referência de `Car.set_vel()`.

## Configuração e compatibilidade

- `compute(input_value, setpoint, now=...)` retorna `True` quando calcula;
  leia o comando em `pid.output`. `update(...)` retorna diretamente o comando.
- `set_mode(MANUAL/AUTOMATIC)` alterna o modo. No manual, atribua `pid.output`
  ao comando aplicado e atualize `pid.input` com a medição antes de reativar.
  A transição inicializa o acumulador com a saída atual e a derivada com a
  medição atual, como no Arduino. Um erro não nulo ainda pode produzir salto P.
- `set_tunings(kp, ki, kd, proportional_on=P_ON_M)` altera os ganhos e o modo
  proporcional. Omitir `proportional_on` preserva o modo atual.
- `set_sample_time(segundos)` reescala Ki e Kd discretos.
- `set_output_limits(minimo, maximo)` limita saída e acumulador quando ativo.
- `reference_filter_tau` configura a constante de tempo do filtro interno do
  `setpoint`.
- `set_controller_direction(DIRECT/REVERSE)` altera a ação, inclusive no manual.
- `reset(input_value=0.0, output=0.0)` reinicia histórico e relógio, preservando
  referência, modo e configuração.

Também há `Compute`, `SetMode`, `SetTunings`, `SetOutputLimits`,
`SetControllerDirection`, `Initialize`, `GetKp`, `GetKi`, `GetKd`, `GetMode` e
`GetDirection`. **`SetSampleTime(ms)` recebe milissegundos**, como no Arduino.
O construtor Python recebe ganhos primeiro, sem ponteiros: `input`, `output`
e `setpoint` são atributos. O padrão é MANUAL, Ts=0.1 s, limites [0, 255],
DIRECT e P_ON_E; configure os limites para as unidades do seu atuador.

Diferenças deliberadas da referência C++: parâmetros inválidos e valores não
finitos geram `ValueError`; mudança de direção em MANUAL também recalcula os
sinais internos; o relógio não simula o overflow de `millis()`. O PID não é
seguro para acesso concorrente: mantenha cada instância na thread de controle.

## Verificação

Na raiz do repositório:

```bash
python3 -B -m unittest discover -s tests -v
```

Os testes cobrem sequência numérica de referência, amostragem, saturação e
recuperação integral, derivada, modos, direção, ajuste de ganhos/período,
reinício do relógio e importação independente dos ambientes. Não exigem
hardware nem conexão com o simulador; não substituem a sintonia em cada planta.
