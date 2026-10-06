# API do carrinho - veiculo real

Este documento descreve as APIs de `veiculo_real/fva_car`. O ponto de entrada
usual e:

```python
from fva_car import Car
```

O hardware esperado é uma Raspberry Pi com servos, encoder em Arduino, IMU,
ultrassom e, opcionalmente, camera USB e GPS de celular Android via ADB.

## Configuracao de `Car`

`Car(parameters)` inicializa os sensores e atuadores. Chaves usadas:

| Chave | Tipo | Uso |
| --- | --- | --- |
| `ts` | float | Duracao usada pelos exemplos de `main.py`. |
| `save` | bool | Cria log e habilita o uso de `save()`. |
| `logfile` | str | Diretorio pai dos logs. |
| `camera` | bool | Inicializa `Camera` quando verdadeiro. |
| `ultrasonic_steering` | bool | Faz camera/ultrassom acompanhar o estercamento. |
| `us_buzzer` | bool | Emite alerta quando o ultrassom esta invalido ou proximo. |
| `initial_position` | `[x, y, theta]` | Origem em m e yaw inicial em rad. |

Parametros fisicos em `car.CAR`: `VELMAX=1.5 m/s`, `ACCELMAX=1.0 m/s2`,
`STEERMAX=20 graus`, `MASS=5.16 kg`, `L=0.36 m`, `RW=0.08 m`, `MI=0.04` e
`PERIOD=50 ms`.

## `Car`

### Ciclo de vida

- `Car(parameters)`: identifica o carrinho pela interface de rede e inicializa
  servos, encoder, IMU, ultrassom, buzzer e recursos opcionais.
- `start_mission()`: desativa a emergencia, define origem GPS quando disponivel,
  inicializa tempo/estados, zera comandos e emite tres bipes.
- `step() -> bool`: espera o periodo de amostragem, atualiza sensores, estima
  estados e salva uma amostra. Retorna `False` se interrompido por teclado.
- `stop_mission()`: aciona a emergencia, solicita desaceleracao maxima, centraliza
  o estercamento e aguarda por no maximo aproximadamente 3 s.
- `close()`: para o carrinho e fecha buzzer, encoder, servos, ultrassom, IMU,
  camera e GPS, quando presentes. Deve ser chamado em `finally`.

### Estados e sensores

- `get_states() -> (p, v, a, th, w, t)`: retorna posicao `[x,y]` em m,
  velocidade em m/s, aceleracao em m/s2, yaw em rad, velocidade angular em
  rad/s e tempo relativo em s.
- `get_time() -> float`: tempo monotonico do sistema.
- `get_pos() -> ndarray`: integra o modelo cinematico e funde GPS novo com ganho
  `K=0.1`, quando o GPS esta disponivel.
- `get_yaw() -> float`: integra `w` e funde bussola com ganho `K=0.05` quando
  o GPS esta disponivel; retorna yaw em `[0, 2*pi)`.
- `get_vel() -> (v, w)`: le o encoder, filtra `v`, calcula `w_model` por
  `v/L*tan(st)`, le `w_imu` da IMU e funde ambos com `K=0.8`.
- `get_accel() -> float`: funde aceleracao do encoder com eixo X da IMU usando
  `K=0.2` e filtra o resultado.
- `get_image(gray=False) -> ndarray`: delega a captura para a camera. Requer
  `camera=True`.
- `get_distance(max_dist=4.0, d_min=0.3) -> (distance, valid)`: le o ultrassom,
  limita a distancia maxima e pode acionar o buzzer de proximidade.
- `get_car_color() -> str | None`: identifica `verde`, `vermelho` ou `roxo` pelo
  MAC configurado em `MACS_CARS`.

Os estados ficam tambem nos atributos `p`, `v`, `a`, `th`, `w`, `t`, `dt`,
`vref`, `u`, `st`, `gear`, `a_model`, `a_x`, `w_model`, `w_imu`, `yaw_mag` e
`p_gps`.

### Atuacao

- `set_u(u)`: limita o comando a `[-ACCELMAX, ACCELMAX]`, converte para o
  pseudo-torque do servo e chama `Servos.set_torque()`.
- `set_vel(vref)`: controlador PD de velocidade; limita referencia a
  `[-VELMAX, VELMAX]` e troca a marcha quando necessario.
- `set_steer(st)`: limita o estercamento a `STEERMAX` e envia o comando aos servos.
- `set_forward()` / `set_reverse()`: trocam a marcha. A troca bloqueia a
  atuacao por aproximadamente `GEAR_SHIFTING_TIME=5 s` e espera o carrinho parar.

### Registro

- `save_traj()`: adiciona uma amostra ao atributo `traj`.
- `save()`: grava `<logfile>/car.csv` com `t,x,y,v,a,vref,th,w,u,a_model,a_x,
  w_model,w_imu,yaw_mag`.

`step()` chama `save_traj()` automaticamente. O arquivo so e criado por
`save()`, normalmente depois do ensaio.

## Coastdown

Use uma pista segura, plana e longa, mantenha o estercamento em zero e colete pelo
menos `t`, `v`, `a`, `u` e `vref`:

```python
from fva_car import Car

parameters = {
    "ts": 30.0,
    "save": True,
    "logfile": "logs/",
    "camera": False,
    "ultrasonic_steering": False,
    "us_buzzer": False,
    "initial_position": [0.0, 0.0, 0.0],
}

car = Car(parameters)
try:
    car.start_mission()
    while car.t < parameters["ts"]:
        car.step()
        car.set_steer(0.0)
        if car.t < 5.0:
            car.set_vel(0.8)
        else:
            car.set_u(0.0)  # ver a observacao abaixo
    car.save()
finally:
    car.close()
```

### Limitacao importante do comando longitudinal

No hardware atual, `set_u(0)` chama `Servos.set_torque(0)`, que define a taxa
de variacao do PWM como zero. O PWM ja aplicado pode permanecer no valor atual;
isso nao garante que o ESC volte ao neutro e, portanto, nao garante coastdown
livre. `set_vel(0)` tambem e inadequado para medir a inercia, pois fecha o
controlador de velocidade. O ensaio fisico deve ser liberado somente depois de
definir uma estrategia explicita de retorno do throttle ao neutro, verificando
que ela nao introduza frenagem ativa. Essa e uma limitacao da implementacao
atual que deve ser resolvida antes de usar os dados para identificar o modelo.

## Modulos de hardware

### `encoder.Encoder`

- `Encoder()`: encontra automaticamente um Arduino Nano nas portas seriais.
- `get_vel() -> (vel, valid)`: retorna velocidade linear em m/s e validade;
  invalida apos mais de `0.30 s` sem medida.
- `find_arduino() -> str | None`: retorna o device serial encontrado.
- `close()`: fecha a porta serial.

O encoder usa `115200 baud`, reducao `7.80` e raio efetivo `0.08 m`.

### `imu.IMU`

- `IMU(address=0x68, bus_id=4, samples=100, sample_delay=0.01,
  do_wake=True, auto_calibrate=True, mag_cal_file=None)`.
- `get_accel_g() -> (ax, ay, az)`: aceleracao calibrada em g.
- `get_accel() -> (ax, ay, az)`: aceleracao calibrada em m/s2.
- `get_gyro() -> (gx, gy, gz)`: velocidade angular calibrada em graus/s.
- `get_mag() -> (mx, my, mz) | None`: campo magnetico em uT.
- `get_euler(degrees=True) -> (roll, pitch, yaw)`: angulos; yaw depende do
  magnetometro e pode ser `None`.
- `calibrate_mag(samples=600, delay=0.02, filename=None) -> dict`: calibra por
  min/max, opcionalmente salvando JSON.
- `load_mag_cal(filename)`: carrega uma calibracao JSON.
- `recalibrate(samples=None, sample_delay=None)`: recalibra acelerometro e gyro;
  o carrinho deve estar parado.
- `close()`: fecha o barramento I2C.

### `ultrasonic.Ultrasonic`

- `Ultrasonic(trigger_pin=None, echo_pin=None, min_range=0.0, max_range=4.0)`.
- `get_distance() -> (distance, valid)`: retorna a ultima medida em m e sua
  validade; a leitura ocorre em thread a aproximadamente 10 Hz.
- `get_measure() -> float | None`: dispara uma medida imediata.
- `cleanup()`: libera GPIO.
- `close()`: para a thread e libera GPIO.

### `servos.Servos`

- `Servos(steering=0.0, throttle=0.0, velmax=1.5, dt=0.03, ultrasonic=True)`.
- `set_torque(T)`: define pseudo-torque, que altera gradualmente o PWM.
- `set_steer(st)`: limita e filtra estercamento em radianos.
- `set_forward()` / `set_reverse()`: troca marcha com parada e tempo de seguranca.
- `get_gear() -> Gear`: retorna `Gear.FORWARD` ou `Gear.REVERSE`.
- `set_trim(steer=0.0, throttle=0.0, pan=0.0)`: ajusta trims em radianos.
- `close()`: para a thread, centraliza direcao e coloca ESC em neutro.

### `camera.Camera`

Disponivel quando `camera=True`:

- `Camera(cam_index=None, resolution=(640, 480), fps=30)`.
- `find_camera() -> int | None`, `get_resolution() -> (width, height)` e
  `get_fps() -> float`.
- `get_image(gray=False) -> ndarray`: captura frame BGR ou tons de cinza.
- `detect_placa(img) -> ndarray`: executa YOLO e devolve imagem anotada.
- `detect_aruco(img, aruco_id=23) -> (img, center)`: devolve imagem anotada e
  centro `(x, y)` do marcador solicitado, ou `None`.
- `show(img, fps=None) -> bool`: exibe frame; retorna `False` quando `q` e
  pressionado.
- `close()`: libera camera e janelas OpenCV.

### `gps.GPS`

- `GPS()`: cria o cliente Android/ADB.
- `is_available() -> bool`: verifica se existe celular autorizado.
- `get_position() -> dict | None`: retorna `lat`, `lon`, `accuracy` e `speed`.
- `set_origin() -> bool`: define a origem local na posicao atual.
- `get_xy(position=None) -> (x, y) | None`: converte latitude/longitude para m
  usando aproximacao local ENU.
- `get_accuracy() -> float | None` e `close()`.

### `buzzer.Buzzer`

- `beep(durations=0.1, silence=0.1) -> bool`: executa um padrao assincrono;
  retorna `False` se ja estiver tocando ou fechado.
- `victory_tune()`: executa o padrao final.
- `cleanup()` e `close()`: desligam o buzzer e liberam GPIO.

## Navegacao e filtros

`fva_car.navigation.Navigation` possui a mesma API do simulador:
`distance_to_waypoint`, `heading_to_waypoint`, `heading_error`,
`steer_to_waypoint2`, `steer_to_waypoint`, `speed_to_waypoint`,
`waypoint_reached` e `go_to_waypoint`.

`fva_car.filter` possui `BaseFilter`, `MovingAverage`, `AlphaFilter`,
`MedianFilter`, `Kalman1D` e `make_filter`, todos com a mesma API descrita no
documento da simulacao.

Antes de um ensaio real, confirme o funcionamento do encoder, a calibracao da
IMU, a validade do ultrassom, o retorno do throttle ao neutro e o acionamento da
emergencia. Nunca use `close()` como mecanismo normal de soltura: ele executa
`stop_mission()` antes de liberar os perifericos.