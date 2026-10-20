
# -*- coding: utf-8 -*-
# Disciplina: Tópicos em Engenharia de Controle e Automação IV (ENG075)
# Fundamentos de Veículos Autônomos - 2026/2
# Professores: Armando Alves Neto e Leonardo A. Mozelli
# DELT - Escola de Engenharia
# Universidade Federal de Minas Gerais
########################################

from cv2 import integral

from fva_car import Car
import numpy as np
import matplotlib
from collections import deque

matplotlib.use("QtAgg")

import matplotlib.pyplot as plt

plt.rcParams["figure.figsize"] = (6, 8)


# ============================================================
# PARÂMETROS GERAIS
# ============================================================

parameters = {
    "ts": 15.0,
    "save": True,
    "logfile": "logs/",
    "beep": True,
}

SET_VEL = 1.0
historico_d = deque(maxlen=5)
AEB_LATCHED = False

# ============================================================
# ADAS: FCW + AEB
# O ADAS DEFINE SOMENTE O SETPOINT DE VELOCIDADE.
# Ele NÃO chama car.set_u().
# ============================================================

def adas_fcw_aeb(car, estado_adas):
    """
    Retorna:
        setpoint: referência de velocidade definida pelo ADAS
        estado: estado atual do ADAS

    Estados:
        CRUISE   - operação normal
        FCW      - alerta de colisão
        AEB      - frenagem automática solicitada pelo setpoint zero
        LIBERADO - obstáculo removido e veículo parado
    """

    # Parâmetros de segurança
    A_CD = 0.104
    ATRASO = 0.10
    MARGEM = 0.20
    MARGEM_FCW = 1.0

    D_LIVRE = 3.9
    V_PARADO = 0.03
    N_LEITURAS = 20

    # Leitura dos sensores
    d, _ = car.get_distance()
    v, _ = car.get_vel()

    # --------------------------------------------------------
    # AEB TRAVADO
    # Só libera quando o obstáculo for removido E o carro parar
    # --------------------------------------------------------

    if estado_adas["aeb_latched"]:

        if d is not None and float(d) >= D_LIVRE:
            estado_adas["clear_count"] += 1
        else:
            estado_adas["clear_count"] = 0

        obstaculo_removido = (
            estado_adas["clear_count"] >= N_LEITURAS
        )

        velocidade_zero = (
            v is not None
            and abs(float(v)) <= V_PARADO
        )

        if obstaculo_removido and velocidade_zero:
            estado_adas["aeb_latched"] = False
            estado_adas["clear_count"] = 0

            return SET_VEL, "LIBERADO"

        # Mantém o setpoint em zero enquanto o AEB estiver travado
        return 0.0, "AEB"

    # --------------------------------------------------------
    # PROTEÇÃO CONTRA LEITURAS INVÁLIDAS
    # --------------------------------------------------------

    if d is None or v is None:
        estado_adas["aeb_latched"] = True
        estado_adas["clear_count"] = 0

        return 0.0, "AEB"

    d = float(d)
    v = max(0.0, float(v))

    # --------------------------------------------------------
    # CÁLCULO DAS DISTÂNCIAS DE SEGURANÇA
    # --------------------------------------------------------

    d_stop = (v ** 2) / (2.0 * A_CD)

    d_aeb = (
        d_stop
        + v * ATRASO
        + MARGEM
    )

    d_fcw = d_aeb + MARGEM_FCW

    # --------------------------------------------------------
    # AEB: SETPOINT ZERO
    # --------------------------------------------------------

    if d <= d_aeb:

        estado_adas["aeb_latched"] = True
        estado_adas["clear_count"] = 0

        return 0.0, "AEB"

    # --------------------------------------------------------
    # FCW: ALERTA, SEM ALTERAR O SETPOINT
    # --------------------------------------------------------

    elif d <= d_fcw:

        return SET_VEL, "FCW"

    # --------------------------------------------------------
    # CONDUÇÃO NORMAL
    # --------------------------------------------------------

    return SET_VEL, "CRUISE"
    
    


def adas(car, setpoint):
    global AEB_LATCHED
    v = car.get_vel()[0]
    d = car.get_distance()[0]
    
    
    historico_d.append(d)
    d = sum(historico_d) / len(historico_d)
    print(f"distancia media {d: .2f}")
    
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
    
    


# ============================================================
# CONTROLADOR PI DE VELOCIDADE
# O PI É O ÚNICO RESPONSÁVEL POR APLICAR u AO VEÍCULO.
# ============================================================

def control_func_pi(car, integral, ref_filtrada, tau):
    # setpoint = adas(car, SET_VEL)
    setpoint = SET_VEL
    ref_filtrada = ref_filtrada + (car.dt / tau) * (setpoint - ref_filtrada)

	
    v, w = car.get_vel()
    erro = ref_filtrada - v
    integral = integral + erro*car.dt
    prop = 10.0 * erro
    i = 5.0 * integral
    tot = prop + i
    # Controlador PI: o total antes da saturacao mostra as duas contribuicoes
    car.set_u(tot)

    return integral, prop, i, tot, ref_filtrada

def control_func(car, integral, ref_filtrada, tau):
    
    setpoint = adas(car, SET_VEL)
    # setpoint = SET_VEL
    
    print(f"Setpoint: {setpoint:.2f} m/s")

    ref_filtrada = ref_filtrada + (car.dt / tau) * (setpoint - ref_filtrada)

	
    v, w = car.get_vel()
    erro = ref_filtrada - v
    # erro = setpoint - v
    
    kp = 10
    kd = 0.8
    
    a = car.get_accel()
    
    tot = (kp*erro) + kd*a
    # tot = (kp*erro)
    tot = np.clip(tot, -1.0, 1.0)
    
    car.set_u(tot)

    return integral, (kp*erro), (kd*a), tot, ref_filtrada

# ============================================================
# CONTROLADOR ON/OFF ALTERNATIVO
# Não utilizado no controle principal.
# ============================================================

def control_OnOff(car, vel):

    v, w = car.get_vel()

    if v < vel:
        car.set_u(0.5)
    else:
        car.set_u(0.0)


# ============================================================
# THREAD DE VISÃO
# ============================================================

def vision_func(car):

    image = car.get_image(gray=False)

    dist, _ = car.get_distance()

    return image


# ============================================================
# PROGRAMA PRINCIPAL
# ============================================================

if __name__ == "__main__":
	
	plt.figure(1)
	plt.ion()
	
	# cria comunicação com o carrinho
	car = Car(parameters)
	
	try:
		# começa a simulação
		car.start_mission()

		integral = 0
		ref_filtrada = 0.0
		tau = 0.45

		control_params = []

		# main loop
		while car.t <= parameters['ts']:
			
			# lê senores
			car.step()
						
			# funcao de controle
			integral, prop, i, tot, ref_filtrada = control_func(car, integral, ref_filtrada, tau)
			# data = {"t": car.t, "p": prop, "i": i, "tot": tot}
			# control_params.append(data)
			
			# funcao de visao
			image = vision_func(car)
			
			########################################
			# plota	
			plt.subplot(311)
			plt.cla()
			plt.gca().imshow(image, cmap='gray')
			plt.title(f't = {car.t:.2f}, v = {car.get_vel()[0]:.2f}, u = {car.u:.2f}')
			
			plt.subplot(312)
			plt.cla()
			t = [traj['t'] for traj in car.traj]
			v = [traj['v'] for traj in car.traj]
			plt.plot(t,v)
			plt.axhline(1.0, color="green", linestyle="--")
			plt.ylabel('v[m/s]')
			plt.xlabel('t[s]')

			plt.subplot(313)
			plt.cla()
			# t_control = [data["t"] for data in control_params]
			# p = [data["p"] for data in control_params]
			# i = [data["i"] for data in control_params]
			# tot = [data["tot"] for data in control_params]
			# plt.plot(t_control, p, label='P', color='red')
			# plt.plot(t_control, i, label='I')
			# plt.plot(t_control, tot, label='PI')
			plt.axhline(0.0, color="green", linestyle="--")
			plt.ylabel('u')
			plt.xlabel('t[s]')
			plt.legend()
			
			plt.tight_layout()
			
			plt.show()
			plt.pause(0.01)

		# salva
		if parameters['save']:
			car.save()
	
	except KeyboardInterrupt:
		print("\nMissao interrompida pelo usuario.")
		
	finally:
		car.close()
