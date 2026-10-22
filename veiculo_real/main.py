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

MAIN_VEL = 0.7

SET_VEL = 1.0
historico_d = deque(maxlen=5)
AEB_LATCHED = False

#Rx = -0.0931
	
########################################
# thread de ADAS
########################################

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
    
########################################
# thread de controle de velocidade
########################################
def control_func(car, integral, ref_filtrada, tau):
    
    # setpoint = adas(car, SET_VEL)
    setpoint = SET_VEL
    
    # print(f"Setpoint: {setpoint:.2f} m/s")

    ref_filtrada = ref_filtrada + (car.dt / tau) * (setpoint - ref_filtrada)

	
    v, w = car.get_vel()
    erro = ref_filtrada - v
    # erro = setpoint - v
    print(f"Velocidade {v:.2f} m/s")
    
    kp = 0.42
    kd = 0.25
    
    a = car.get_accel()
    
    tot = (kp*erro) - kd*a
    # tot = (kp*erro)
    tot = np.clip(tot, -1.0, 1.0)
    
    car.set_u(tot)

    return integral, (kp*erro), (kd*a), tot, ref_filtrada


########################################
# thread de visao
########################################
def vision_func(car, vision_data, stop_event):

	W, H = car.cam.get_resolution()

	while not stop_event.is_set():

		# pega imagem
		frame = car.get_image(gray=True)

		# detecta aruco
		frame, point = car.cam.detect_aruco(
			frame,
			aruco_id=23
		)

		# disponibiliza imagem para o main
		vision_data["frame"] = frame

		if point is None:
			continue

		# esterçamento aponta para o aruco
		cx = point[0] - W/2

		vision_data["refste"] = -np.deg2rad(20.0*cx/(W/2))
		
########################################
# main
########################################
if __name__ == "__main__":

	parameters = {	
				'ts'					: 20.0, 	# tempo da execucao
				'save'					: True,		# salva dados da trajetoria
				'logfile'				: 'logs/',	# log file
				'camera'				: False,	# habilitar camera e thread de visao
				'us_buzzer'				: True,		# aviso sonoro para objetos proximos
				'initial_position'		: [0, 0, np.deg2rad(0)]	# (x, y, theta) configuracao inicial
			}

	car = Car(parameters)
	
	vision_data = {"refste": 0.0, "frame": None}

	stop_event = threading.Event()
	thread_vision = None

	try:
		car.start_mission()

		# inicia visao somente se solicitada
		if parameters['camera']:
			thread_vision = threading.Thread(
												target=vision_func,
												args=(car, vision_data, stop_event),
												daemon=True
											)
			thread_vision.start()

		if parameters['camera']:
			plt.ion()
			plt.figure(1)

		t_plot = time.monotonic()

		integral = 0
		ref_filtrada = 0.0
		tau = 0.45

		# controle fica na thread principal
		while car.t < parameters['ts']:

			# atualiza sensores
			if not car.step():
				break

			# # direcao
			# car.set_steer(vision_data["refste"])

			# # ultrassom
			# dist, valid = car.get_distance()

			# if (not valid) or (dist < 0.20):
			# 	print(f"Colisao: distance {dist:.2f} [m]")
			# 	car.set_vel(0.0)
			# else:
			# 	car.set_vel(MAIN_VEL)

			car.set_steer(0.0)

			integral, prop, i, tot, ref_filtrada = control_func(car, integral, ref_filtrada, tau)

			# telemetria para plots remotos
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
				flush=True
			)

			# atualiza grafico aproximadamente 1 Hz
			if time.monotonic() - t_plot >= 1.0:

				if parameters['camera']:
					frame = vision_data["frame"]

					if frame is not None:
						plt.cla()
						plt.imshow(frame, cmap='gray')
						plt.pause(0.001)

				t_plot = time.monotonic()

		# salva dados
		if parameters['save']:
			car.save()

	finally:
		# termina a thread de visao
		stop_event.set()

		if thread_vision is not None:
			thread_vision.join(timeout=1.0)

		car.close()

	print('Terminou...')
