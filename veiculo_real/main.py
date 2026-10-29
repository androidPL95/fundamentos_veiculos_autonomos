# -*- coding: utf-8 -*-
########################################
# Disciplina: Topicos em Engenharia de Controle e Automacao IV (ENG075): 
# Fundamentos de Veiculos Autonomos - 2026/2
# Professores: Armando Alves Neto e Leonardo A. Mozelli
# Cursos: Engenharia de Controle e Automacao
# DELT - Escola de Engenharia
# Universidade Federal de Minas Gerais
########################################
# -*- coding: utf-8 -*-
from collections import deque

from fva_car import Car
import numpy as np
import matplotlib.pyplot as plt
import threading
import time
import cv2
# from fva_car.ADAS import ADAS

MAIN_VEL = 0.7

# ad = ADAS()
SET_VEL = 1.0
historico_d = deque(maxlen=5)
AEB_LATCHED = False

USAR_VISAO_NO_LATERAL = True

SET_VEL = 0.5
SET_Y = -9.0
SET_THETA = np.pi

# Ganhos da referência manual
K_STANLEY = 1.5
K_HEADING = 1.0

# Ganhos da visão
K_LATERAL_VISAO = 0.1
K_HEADING_VISAO = 0.1

SINAL_OFFSET_VISAO = -1.0
SINAL_HEADING_VISAO = 1.0

MAX_STEER = np.deg2rad(5.0)
MAX_STEER_RATE = np.deg2rad(25.0)

AMORTECIMENTO_THETA = 0.15
DISTANCIA_ENTRE_EIXOS = 0.302

#Rx = -0.0931
    
########################################
# thread de ADAS
########################################

def adas(car, setpoint):
    global AEB_LATCHED
    v = car.get_vel()[0]
    d, valid = car.get_distance()
    print(f"Valid: {valid}")
    
    
    historico_d.append(d)
    d = sum(historico_d) / len(historico_d)
    print(f"distancia media {d: .2f}")
    
    # if not valid:
    #     return setpoint
    
    if AEB_LATCHED and d <= 3.8:
        print("latched")
        return 0.0
    
    if d > 3.8:
        AEB_LATCHED = False
        print("liberado")
        return setpoint
    
    d_stop = (v**2) * 3.0
    d_aeb = d_stop + 0.5
    d_fcw = d_aeb + 1.0
    
    print(f"distancia de frenagem {d_stop: .2f}")
    if d <= d_aeb:
        print("Frenagem autônoma de emergência ativada, risco de colisão")
        AEB_LATCHED = True
        return 0.0
    elif d <= d_fcw and d > d_aeb:
        return setpoint
        # print("Alerta de colisão, reduza a velocidade")
        # if AEB_LATCHED:
        #     return 0.0
        # else:
        #     return setpoint
        
    else:
        return setpoint
    
########################################
# thread de controle de velocidade
########################################
def control_func(car, ref_filtrada, theta_filtrado, tau, erro_visao, theta_visao):
    
    setpoint = adas(car, SET_VEL)
    # setpoint = ad.ACC(car, 3.0, 0.5)
    
    # setpoint = SET_VEL
    car.vref = setpoint
    
    # print(f"Setpoint: {setpoint:.2f} m/s")

    ref_filtrada = ref_filtrada + (car.dt / tau) * (setpoint - ref_filtrada)

    
    v, w = car.get_vel()
    erro = ref_filtrada - v
    # erro = setpoint - v
    print(f"Velocidade {v:.2f} m/s")
    print(f"Distancia: {car.get_distance()[0]}")
    
    kp = 0.5
    kd = 0.25
    
    a = car.get_accel()
    
    d_term = kd*a
    tot = (kp*erro) - d_term
    # tot = (kp*erro)
    tot = np.clip(tot, -1.0, 1.0)
    
    
    
    if setpoint == 0.0 and v <= 0.05:
        car.set_u(-1.0)
        # print(f"Controlador desligado")s
    else:
        car.set_u(tot)
        
    print(F"setpoint {setpoint}")
    # print(f"Car.set_u {tot}")

    # Primeiro segundo sem comando lateral
    if car.t < 1.0:
        theta_filtrado = float(car.th)
        delta_lat = 0.0
    else:
        if USAR_VISAO_NO_LATERAL:
            if erro_visao is None or theta_visao is None:
                delta_alvo = float(car.st) # Mantem o ultimo esterçamento se perder a linha
            else:
                erro_lateral = SINAL_OFFSET_VISAO * erro_visao
                erro_heading = SINAL_HEADING_VISAO * theta_visao
                
                v_lateral = max(float(car.v), 0.1)
                
                termo_heading = K_HEADING_VISAO * erro_heading
                termo_lateral = np.arctan2(K_LATERAL_VISAO * erro_lateral, v_lateral)
                
                delta_alvo = termo_heading + termo_lateral
        else:
            diferenca = np.arctan2(np.sin(car.th - theta_filtrado), np.cos(car.th - theta_filtrado)) # Referência manual, filtrando a orientação


            theta_filtrado += AMORTECIMENTO_THETA * diferenca

            erro_heading = np.arctan2(np.sin(theta_filtrado - SET_THETA), np.cos(theta_filtrado - SET_THETA))

            # Aproximação do eixo dianteiro
            y_dianteiro = (float(car.p[1]) + DISTANCIA_ENTRE_EIXOS * np.sin(theta_filtrado - np.pi))

            erro_lateral = y_dianteiro - SET_Y
            v_lateral = max(float(car.v), 0.1)

            termo_heading = K_HEADING * erro_heading
            termo_lateral = np.arctan2(K_STANLEY * erro_lateral, v_lateral)

            delta_alvo = termo_heading + termo_lateral
            
        delta_alvo = float(np.clip(delta_alvo, -MAX_STEER, MAX_STEER))
        
        passo_max = MAX_STEER_RATE * car.dt
        
        delta_lat = float(np.clip(delta_alvo, car.st - passo_max, car.st + passo_max))
        
    # O comando longitudinal ja foi enviado acima (incluindo a frenagem).
    car.set_steer(delta_lat)
    
    return ref_filtrada, theta_filtrado


########################################
# thread de visao
########################################
def vision_func(car):
    image = car.get_image(gray=False)

    if image is None:
        return None, None, None

    vis_img = image.copy()
    h, w, _ = vis_img.shape
    centro_x = w / 2.0

    hsv = cv2.cvtColor(vis_img, cv2.COLOR_RGB2HSV)

    amarelo_baixo = np.array([15, 80, 80])
    amarelo_alto = np.array([40, 255, 255])
    mask = cv2.inRange(hsv, amarelo_baixo, amarelo_alto)

    y_perto = int(h * 0.85)
    y_longe = int(h * 0.60)

    cols_perto = np.where(mask[y_perto, :] > 0)[0]
    cols_longe = np.where(mask[y_longe, :] > 0)[0]

    cv2.line(
        vis_img,
        (int(centro_x), int(h * 0.5)),
        (int(centro_x), h - 1),
        (255, 255, 255),
        1,
    )

    if len(cols_perto) == 0 or len(cols_longe) == 0:
        return vis_img, None, None

    x_perto = float(np.mean(cols_perto))
    x_longe = float(np.mean(cols_longe))

    # Offset normalizado pela metade da largura da imagem.
    erro_lat_norm = (centro_x - x_perto) / (w / 2.0)

    # Inclinação da linha na imagem, sujeita a perspectiva
    dx = x_longe - x_perto
    dy = y_perto - y_longe
    theta_e = float(np.arctan2(-dx, dy))

    cv2.circle(
        vis_img,
        (int(x_perto), y_perto),
        5,
        (255, 0, 0),
        -1,
    )

    cv2.circle(
        vis_img,
        (int(x_longe), y_longe),
        5,
        (0, 255, 0),
        -1,
    )

    cv2.line(
        vis_img,
        (int(x_perto), y_perto),
        (int(x_longe), y_longe),
        (0, 255, 255),
        2,
    )

    return vis_img, erro_lat_norm, theta_e
    
########################################
# Thread de visao: publica o resultado mais recente
########################################

def vision_loop(car, vision_data, vision_lock, stop_event):
    try:
        while not stop_event.is_set():
            # O processamento demorado acontece FORA do lock.
            frame, erro_visao, theta_visao = vision_func(car)

            # Publica os tres valores juntos: pertencem a mesma imagem.
            with vision_lock:
                vision_data["frame"] = frame
                vision_data["erro_visao"] = erro_visao
                vision_data["theta_visao"] = theta_visao

            # Pequena pausa; tambem permite encerrar a thread prontamente.
            stop_event.wait(0.01)
    finally:
        # Se a thread terminar por uma excecao, encerra tambem o main.
        stop_event.set()


########################################
# Main: thread principal executa sensores, controle e plots
########################################
if __name__ == "__main__":
    parameters = {
        'ts': 20.0,
        'save': True,
        'logfile': 'logs/',
        'camera': True,  # Necessaria para o controle por visao.
        'us_buzzer': True,
        'initial_position': [0, 0, np.deg2rad(0)],
    }

    car = Car(parameters)

    # Ambas as threads recebem os MESMOS objetos.
    vision_data = {
        "frame": None,
        "erro_visao": None,
        "theta_visao": None,
    }
    vision_lock = threading.Lock()
    stop_event = threading.Event()
    thread_vision = None

    try:
        car.start_mission()

        if parameters['camera']:
            thread_vision = threading.Thread(
                target=vision_loop,
                args=(car, vision_data, vision_lock, stop_event),
            )
            thread_vision.start()
            plt.ion()
            plt.figure(1)

        t_plot = time.monotonic()
        ref_filtrada = 0.0
        theta_filtrado = float(car.th)
        tau = 0.45

        while car.t < parameters['ts'] and not stop_event.is_set():
            if not car.step():
                break

            # Copia rapidamente os ultimos dados publicados pela visao.
            with vision_lock:
                erro_visao = vision_data["erro_visao"]
                theta_visao = vision_data["theta_visao"]
                frame = vision_data["frame"]

            # O controle acontece FORA do lock.
            # Os estados dos filtros retornam para a proxima iteracao.
            ref_filtrada, theta_filtrado = control_func(
                car, ref_filtrada, theta_filtrado, tau,
                erro_visao, theta_visao,
            )

            print(
                f"DATA,"
                f"{car.t:.2f},"
                f"{car.p[0]:.2f},"
                f"{car.p[1]:.2f},"
                f"{car.v:.2f},"
                f"{car.vref:.2f},"
                f"{car.a:.2f},"
                f"{car.u:.2f},"
                f"{car.w:.2f},"
                f"{car.th:.2f}",
                flush=True,
            )

            # Matplotlib permanece na thread principal.
            if time.monotonic() - t_plot >= 1.0:
                if parameters['camera'] and frame is not None:
                    plt.cla()
                    plt.imshow(frame)
                    plt.pause(0.001)
                t_plot = time.monotonic()

    finally:
        stop_event.set()
        if thread_vision is not None:
            # Aguarda a visao terminar antes de fechar a camera/conexao.
            thread_vision.join()
        try:
            if parameters['save']:
                car.save()
        finally:
            car.close()

    print('Terminou...')
