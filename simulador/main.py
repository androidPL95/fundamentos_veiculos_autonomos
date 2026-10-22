# -*- coding: utf-8 -*-
# Disciplina: Tópicos em Engenharia de Controle e Automação IV (ENG075)
# Fundamentos de Veículos Autônomos - 2026/2
# Professores: Armando Alves Neto e Leonardo A. Mozelli
# Alunos: Davi Zumpano, Henrique Carneiro, Marcos Vinicius, Pedro Lucas e Vinicius Azi
# DELT - Escola de Engenharia
# Universidade Federal de Minas Gerais
########################################################################################

import cv2
from fva_car import Car
import numpy as np
import matplotlib
from collections import deque

matplotlib.use("QtAgg")
import matplotlib.pyplot as plt

plt.rcParams["figure.figsize"] = (7, 9)


# Parâmetros

parameters = {
    "ts": 15.0,
    "save": True,
    "logfile": "logs/",
    "beep": True,
}

# Chavear a fonte de set-point (visão computacional ou referência manual)
USAR_VISAO_NO_LATERAL = False

SET_VEL = 1.0           # Referência de velocidade longitudinal 
SET_Y = -9.0            # Ajuste manual de referência simples de eixo pra seguir
SET_THETA = np.pi       # No car.py, andar reto para +x corresponde a theta = pi

K_STANLEY = 0.6                     # Ganho do erro lateral
K_HEADING = 0.7                   # Ganho de orientação
K_DAMPING_W = 0.10                  # Amortecimento usando a velocidade angular filtrada
V_SOFT = 0.5                        # Constante de suavização de baixa velocidade
MAX_STEER = np.deg2rad(20.0)        # Limite físico do carro
MAX_STEER_RATE = np.deg2rad(25.0)   # Taxa máxima de giro do volante (igual ao navigation.py)
AMORTECIMENTO_THETA = 0.15                     # Filtro passa-baixas para limpar a oscilação do car.th
theta_filtrado = np.pi                 # Estado inicial do filtro de yaw

# Variáveis globais aux
historico_d = deque(maxlen=5)
AEB_LATCHED = False





# ============================================================
# ADAS longitudinal: FCW + AEB
# ============================================================

def adas(car, setpoint):
    global AEB_LATCHED
    v = car.get_vel()[0]
    d = car.get_distance()[0]
    
    if d is None or v is None:
        return 0.0

    historico_d.append(float(d))
    d_media = sum(historico_d) / len(historico_d)
    
    if AEB_LATCHED and d_media <= 3.8:
        return 0.0
    
    if d_media > 3.8:
        AEB_LATCHED = False
        return setpoint
    
    d_stop = (v**2) * 3.0
    d_aeb = d_stop + 0.5
    d_fcw = d_aeb + 1.0
    
    if d_media <= d_aeb:
        print("AEB Ativado! Risco de colisão.")
        AEB_LATCHED = True
        return 0.0
    elif d_media <= d_fcw:
        return setpoint
    else:
        return setpoint





# ============================================================
# Visão computacional clássica pra detecção de linha amarela
# ============================================================

def vision_func(car):
    image = car.get_image(gray=False)
    if image is None:
        return None, 0.0, 0.0

    vis_img = image.copy()
    h, w, _ = vis_img.shape
    centro_x = w / 2.0

    # Converte de RGB para HSV para isolar a cor amarela
    hsv = cv2.cvtColor(vis_img, cv2.COLOR_RGB2HSV)
    amarelo_baixo = np.array([15, 80, 80])   # Saturação >= 80 
    amarelo_alto  = np.array([40, 255, 255])
    mask = cv2.inRange(hsv, amarelo_baixo, amarelo_alto)

    # Analisa duas alturas na metade inferior da imagem (ROI)
    y_perto = int(h * 0.85)  # Ponto próximo para o erro lateral (e)
    y_longe = int(h * 0.60)  # Ponto à frente para a inclinação da reta (theta_e)

    cols_perto = np.where(mask[y_perto, :] > 0)[0]
    cols_longe = np.where(mask[y_longe, :] > 0)[0]

    erro_lat_norm = 0.0
    theta_e = 0.0

    if len(cols_perto) > 0:
        x_perto = np.mean(cols_perto)
        # Se a faixa está à esquerda do centro (x_perto < centro_x), erro > 0 (virar à esquerda)
        erro_lat_norm = (centro_x - x_perto) / (w / 2.0)
        cv2.circle(vis_img, (int(x_perto), y_perto), 5, (255, 0, 0), -1)

        if len(cols_longe) > 0:
            x_longe = np.mean(cols_longe)
            dx = x_longe - x_perto
            dy = y_perto - y_longe
            theta_e = np.arctan2(-dx, dy)
            cv2.circle(vis_img, (int(x_longe), y_longe), 5, (0, 255, 0), -1)
            cv2.line(vis_img, (int(x_perto), y_perto), (int(x_longe), y_longe), (0, 255, 255), 2)

    # Linha central de referência do robô
    cv2.line(vis_img, (int(centro_x), int(h * 0.5)), (int(centro_x), h), (255, 255, 255), 1)

    return vis_img, erro_lat_norm, theta_e





# ============================================================
# Controlador Stanley
# ============================================================

def stanley_control(car, erro_visao, theta_visao):
    global theta_filtrado

    # Ignora os primeiros 0.15s de ruído da simulação
    if car.t < 0.15:
        theta_filtrado = float(car.th)
        return 0.0

    v = max(0.0, float(car.v))

    # chaveamento entre usar visão ou nao, 'e' e 'theta_e' são os erros de angulação e offset do carro pra ref
    if USAR_VISAO_NO_LATERAL:
        e = erro_visao
        theta_e = theta_visao
    else:
        # Filtra o ângulo theta pra eliminar oscilação do simulador
        theta_atual = float(car.th)
        
        # Se o angulo tiver descontinuidade (359° pra 1° por exemplo), ele entende apenas a variação real (2°)
        delta_theta = np.arctan2(np.sin(theta_atual - theta_filtrado), np.cos(theta_atual - theta_filtrado))
        
        theta_filtrado = theta_filtrado + AMORTECIMENTO_THETA * delta_theta

        # Posição lateral projetada no eixo dianteiro (L = 0.302m do car.py)
        L = 0.302
        theta_real_movimento = theta_filtrado - np.pi
        y_dianteiro = float(car.p[1]) + L * np.sin(theta_real_movimento) # Onde tá o eixo dianteiro do carro

        # Se y_dianteiro > SET_Y (carro à esquerda), e > 0 -> esterça para a direita (st > 0)
        e = y_dianteiro - SET_Y

        # Se theta_filtrado > SET_THETA (apontando p/ esquerda), theta_e > 0 -> esterça p/ direita (st > 0)
        theta_e = np.arctan2(np.sin(theta_filtrado - SET_THETA), np.cos(theta_filtrado - SET_THETA))

    # Lei de Controle de Stanley e amortecimento de velocidade angular
    termo_alinhamento = K_HEADING * theta_e
    termo_crosstrack  = np.arctan2(K_STANLEY * e, v + V_SOFT)
    termo_amortecimento = K_DAMPING_W * float(car.w) # termo a mais pra freiar/controlar ainda mais o esterçamento, a partir da velocidade angular do carro 

    # soma dos termos
    delta_alvo = termo_alinhamento + termo_crosstrack + termo_amortecimento

    # Limite na taxa de variação do volante pra evitar variações bruscas no esterçamento
    if car.dt > 0.0:
        d_delta_max = MAX_STEER_RATE * car.dt
        d_delta = np.clip(delta_alvo - car.st, -d_delta_max, d_delta_max)
        delta_cmd = car.st + d_delta
    else:
        delta_cmd = delta_alvo

    # nao deixa passar dos limites de esterçamento
    return float(np.clip(delta_cmd, -MAX_STEER, MAX_STEER))





# ============================================================
# Controle longitudinal + lateral
# ============================================================

def control_func(car, integral, ref_filtrada, tau, erro_visao, theta_visao):
    # Controle longitudinal (ADAS + PD com filtro de referência)
    setpoint = adas(car, SET_VEL)
    ref_filtrada = ref_filtrada + (car.dt / tau) * (setpoint - ref_filtrada)

    v, _ = car.get_vel()
    erro_v = ref_filtrada - v
    
    kp = 10.0
    kd = 0.8
    a = car.get_accel()
    
    u_long = float(np.clip((kp * erro_v) + (kd * a), -1.0, 1.0))

    # Controle lateral (Stanley)
    delta_lat = stanley_control(car, erro_visao, theta_visao)

    # Aplicação dos comandos
    car.set_u(u_long)
    car.set_steer(delta_lat)

    return integral, (kp * erro_v), (kd * a), u_long, delta_lat, ref_filtrada





# ============================================================
# Main
# ============================================================

if __name__ == "__main__":
    plt.figure(1)
    plt.ion()
    
    car = Car(parameters)
    
    try:
        car.start_mission()

        integral = 0.0
        ref_filtrada = 0.0
        tau = 0.45

        historico_delta = []
        historico_t = []

        while car.t <= parameters['ts']:
            # 1. Atualiza passo e sensores do simulador
            car.step()
            
            # 2. Processa Visão Computacional
            image, erro_visao, theta_visao = vision_func(car)
            
            # 3. Executa Controle Longitudinal e Lateral
            integral, prop, d_term, u_long, delta_lat, ref_filtrada = control_func(
                car, integral, ref_filtrada, tau, erro_visao, theta_visao
            )
            
            historico_t.append(car.t)
            historico_delta.append(np.rad2deg(delta_lat))
            
            ########################################
            # Plota gráficos para diagnóstico rápido
            plt.subplot(311)
            plt.cla()
            if image is not None:
                plt.gca().imshow(image)
            plt.title(
                f"t={car.t:.2f}s | y={car.p[1]:.2f}m | th={np.rad2deg(car.th):.1f}° | delta={np.rad2deg(delta_lat):.1f}°"
            )
            
            plt.subplot(312)
            plt.cla()
            t_list = [traj['t'] for traj in car.traj]
            y_list = [traj['y'] for traj in car.traj]
            plt.plot(t_list, y_list, label='y atual [m]', color='tab:orange')
            if not USAR_VISAO_NO_LATERAL:
                plt.axhline(SET_Y, color="green", linestyle="--", label=f'Ref Y ({SET_Y}m)')
            plt.ylabel('Posição Y [m]')
            plt.legend(loc='upper right')
            plt.grid(True)

            plt.subplot(313)
            plt.cla()
            plt.plot(historico_t, historico_delta, color='purple', label='Steer delta (°)')
            plt.axhline(0.0, color="black", linestyle="--")
            plt.ylim(-25, 25)
            plt.ylabel('Direção [graus]')
            plt.xlabel('t [s]')
            plt.legend(loc='upper right')
            plt.grid(True)
            
            plt.tight_layout()
            plt.show()
            plt.pause(0.01)

        if parameters['save']:
            car.save()
    
    except KeyboardInterrupt:
        print("\nMissão interrompida pelo usuário.")
        
    finally:
        car.close()