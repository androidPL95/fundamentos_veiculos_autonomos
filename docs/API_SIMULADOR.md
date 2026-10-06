# API do carrinho - simulacao

Este documento descreve os recursos expostos por `simulador/fva_car`. A API
publica e exportada por `simulador/fva_car/__init__.py` e pode ser usada com:

```python
from fva_car import Car
```

## Configuracao

`Car(parameters)` inicia uma conexao com o CoppeliaSim, localiza o modelo
`/Car` e cria uma pasta de log com data e hora. O dicionario deve conter:

| Chave | Uso |
| --- | --- |
| `logfile` | Diretorio pai dos logs; obrigatorio para criar a pasta do experimento. |
| `beep` | Habilita (`True`) ou desabilita (`False`) os avisos sonoros no terminal. |
| `ts` | Usado pelos exemplos de `main.py`; nao e consumido diretamente pelo construtor. |
| `save` | Usado pelo programa principal para decidir se chama `save()`. |

Parametros fisicos disponiveis em `car.CAR`:

| Nome | Valor | Unidade |
| --- | ---: | --- |
| `VELMAX` | 1.5 | m/s |
| `ACCELMAX` | 1.0 | m/s2 |
| `STEERMAX` | 20 graus | rad |
| `MASS` | 6.3 | kg |
| `L` | 0.302 | m |
| `RW` | 0.08 | m |
| `MI` | 0.05 | - |

## `Car`

### Experimentos

O loop comum de `simulador/main.py` chama uma funcao com a assinatura
`experiment_func(car)` a cada passo. Para trocar o experimento, importe a
funcao desejada e altere somente a atribuicao de `experiment_func`.

O modulo `simulador/experiments.py` contem `control_experiment`, que preserva o
comportamento original, e `coastdown_experiment`. O coastdown usa os parametros
opcionais `coastdown_target_speed` e `coastdown_acceleration`, acelera ate a
velocidade alvo e entao chama `car.set_u(0.0)` em todos os passos seguintes.

Os dados continuam sendo registrados por `Car.step()` e salvos por `Car.save()`.

### Ciclo de vida e simulacao

- `Car(parameters)`: cria o objeto e conecta ao CoppeliaSim.
- `start_mission()`: habilita o modo stepping, inicia a simulacao, zera os
  comandos e inicializa tempo, estados e log.
- `step() -> bool`: avanca exatamente um passo do simulador, atualiza estados,
  calcula `dt` e registra uma amostra. Retorna `False` apos `KeyboardInterrupt`.
- `stop_mission()`: aplica desaceleracao ativa, centraliza o volante, espera o
  carro quase parar e interrompe a simulacao.
- `close()`: encerra a missao e garante que a simulacao seja parada. Deve ser
  chamado em um bloco `finally`.

### Estados e sensores

- `get_states() -> (p, v, a, th, w, t)`: atualiza e retorna posicao `p` (array
  `[x, y]` em m), velocidade longitudinal `v` (m/s), aceleracao `a` (m/s2),
  yaw `th` (rad), velocidade angular `w` (rad/s) e tempo de missao `t` (s).
- `get_time() -> float`: tempo de simulacao do CoppeliaSim.
- `get_pos() -> ndarray`: posicao do objeto no plano.
- `get_yaw() -> float`: yaw normalizado no intervalo `[0, 2*pi]`.
- `quaternion_to_yaw(q) -> float`: converte quaternion `[qx, qy, qz, qw]` em yaw.
- `get_vel() -> (v, w)`: le a velocidade do objeto, aplica os filtros de
  velocidade e retorna as componentes longitudinal e angular.
- `get_accel() -> float`: estima `a = delta_v / dt` e aplica filtro.
- `get_image(gray=False) -> ndarray`: retorna a imagem da camera; com `gray=True`
  retorna uma imagem em tons de cinza.
- `get_distance(max_dist=4.0) -> (distance, valid)`: retorna distancia em m e
  validade. Quando nao ha obstaculo, retorna `max_dist`; no simulador a medida
  e sempre marcada como valida.

Os estados tambem ficam disponiveis nos atributos `p`, `v`, `a`, `th`, `w`,
`t`, `dt`, `vref`, `u`, `st` e `gear`.

### Atuacao

- `set_u(u)`: define o comando longitudinal limitado a `[-ACCELMAX, ACCELMAX]`.
  O valor e interpretado como aceleracao de controle equivalente, convertido
  em forca e torque e aplicado aos dois motores.
- `set_vel(vref)`: controlador PD de velocidade. Limita a referencia a
  `[-VELMAX, VELMAX]` e troca a marcha quando necessario.
- `set_steer(st)`: define o esterçamento em radianos, limitado a `STEERMAX`, e
  converte o comando para os dois angulos de Ackermann.
- `set_forward()`: seleciona marcha a frente; se necessario, freia antes de
  trocar.
- `set_reverse()`: seleciona marcha a re; se necessario, freia antes de trocar.
- `beep(durations=0.1, silence=0.1)`: emite um ou mais avisos sonoros no
  terminal, desde que `parameters['beep']` seja verdadeiro.

Para um coastdown, mantenha `st=0`, acelere usando `set_u()` ou `set_vel()` e,
no instante de soltura, use `set_u(0.0)` sem chamar `stop_mission()` ou
`set_vel(0.0)`. No modelo atual, `set_u(0)` remove a forca de controle, mas a
forca de atrito longitudinal continua sendo aplicada.

### Registro

- `save_traj()`: adiciona uma amostra ao atributo `traj`.
- `save()`: grava `traj` em `<logfile>/car.csv` com as colunas `t,x,y,v,a,vref,th,w,u`.

`step()` chama `save_traj()` automaticamente. Portanto, depois da missao,
basta chamar `save()` uma vez.

## `Navigation`

Importacao: `from fva_car.navigation import Navigation`.

`Navigation(car)` usa os estados do objeto `Car` e oferece:

- `distance_to_waypoint(waypoint) -> float`: distancia euclidiana em m.
- `heading_to_waypoint(waypoint) -> float`: direcao desejada em radianos.
- `heading_error(waypoint) -> float`: erro angular embrulhado em `[-pi, pi]`.
- `steer_to_waypoint2(waypoint, Kp=0.2) -> float`: controle proporcional direto.
- `steer_to_waypoint(waypoint, Kp=0.2) -> float`: controle proporcional com
  limitacao da variacao de estercamento por passo.
- `speed_to_waypoint(waypoint, Kv=0.1) -> float`: define referencia
  `max(Kv * distancia, MIN_SPEED)`.
- `waypoint_reached(waypoint, radius=2.0) -> bool`.
- `go_to_waypoint(waypoint, radius=2.0, Kv=0.2, Kp=1.0) -> bool`: executa
  controle de direcao e velocidade e informa se chegou.

## Filtros

`fva_car.filter` fornece `BaseFilter`, `MovingAverage`, `AlphaFilter`,
`MedianFilter`, `Kalman1D` e `make_filter(name, **kwargs)`. Todos possuem
`filter(sample) -> float`, `reset(...)` e a propriedade `value`.

- `MovingAverage(n=4, initial=0.0)` usa janela fixa.
- `AlphaFilter(alpha=0.5, initial=0.0)` usa filtro exponencial.
- `MedianFilter(n=5, initial=0.0)` usa mediana da janela.
- `Kalman1D(process_variance=1e-5, measurement_variance=1e-2,
  initial=0.0, initial_error_variance=1.0)` implementa Kalman escalar.

## Exemplo minimo

```python
from fva_car import Car

parameters = {"logfile": "logs/", "beep": False}
car = Car(parameters)
try:
    car.start_mission()
    while car.t < 20.0:
        car.step()
        car.set_steer(0.0)
        if car.t < 5.0:
            car.set_u(0.8)
        else:
            car.set_u(0.0)  # soltura para coastdown
    car.save()
finally:
    car.close()
```

O CoppeliaSim deve estar aberto com uma cena que contenha `/Car`, seus joints,
`Vision_sensor` e `/Car/ultra_front`, alem do cliente Remote API usado pelo
projeto.