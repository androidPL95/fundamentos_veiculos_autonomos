# -*- coding: utf-8 -*-
# Disciplina: Tópicos em Engenharia de Controle e Automação IV (ENG075): 
# Fundamentos de Veículos Autônomos - 2026/1
# Professores: Armando Alves Neto e Leonardo A. Mozelli
# Cursos: Engenharia de Controle e Automação
# DELT – Escola de Engenharia
# Universidade Federal de Minas Gerais
########################################
from fva_car import Car
import numpy as np
import os
import matplotlib
matplotlib.use("QtAgg")
import matplotlib.pyplot as plt
plt.rcParams['figure.figsize'] = (6,8)

# Globais
parameters = {	
				'ts'		: 15.0, 			# tempo da simulacao
				'save'		: True,
				'logfile'	: 'logs/',
				'beep'		: True,
			}

SET_VEL = 1.0

#Rx = -0.0931
	
########################################
# thread de controle de velocidade
########################################

def control_func(car, integral, ref_filtrada, tau, estado_adas):

    ref_filtrada += (car.dt / tau) * (SET_VEL - ref_filtrada)

    v, w = car.get_vel()
    erro = ref_filtrada - v

    prop = 10 * erro

    # Não acumula integral durante o AEB
    if estado_adas["aeb_latched"]:
        integral = 0.0
    else:
        integral += erro * car.dt

    i = 5 * integral
    tot = prop + i

    adas_u, adas_estado = adas_fcw_aeb(
        car, tot, estado_adas
    )

    if adas_estado == "AEB":
        print("AEB: tração cortada!")
    elif adas_estado == "FCW":
        print("FCW: obstáculo à frente!")
    elif adas_estado == "LIBERADO":
        print("Obstáculo removido. Liberando controle.")

    return integral, prop, i, adas_u, ref_filtrada




def adas_fcw_aeb(car, u, estado_adas):
    """
    AEB travado até que:
      1. O obstáculo seja removido;
      2. O veículo esteja praticamente parado.

    Enquanto isso, u = 0.
    """

    A_CD = 0.104
    ATRASO = 0.10
    MARGEM = 0.20
    MARGEM_FCW = 1.0

    D_LIVRE = 3.8       # distância que indica caminho livre [m]
    V_PARADO = 0.03     # tolerância de velocidade [m/s]
    N_LEITURAS = 10     # leituras consecutivas de caminho livre

    d, _ = car.get_distance()
    v, _ = car.get_vel()

    # Se o AEB estiver travado, não permite aceleração
    if estado_adas["aeb_latched"]:

        # Confirma que o obstáculo foi removido
        if d is not None and float(d) >= D_LIVRE:
            estado_adas["clear_count"] += 1
        else:
            estado_adas["clear_count"] = 0

        obstaculo_removido = (
            estado_adas["clear_count"] >= N_LEITURAS
        )

        velocidade_zero = (
            v is not None and abs(float(v)) <= V_PARADO
        )

        # Só libera quando as DUAS condições forem satisfeitas
        if obstaculo_removido and velocidade_zero:
            estado_adas["aeb_latched"] = False
            estado_adas["clear_count"] = 0
            car.set_u(0.0)
            return 0.0, "LIBERADO"

        # Continua cortando a tração
        car.set_u(0.0)
        return 0.0, "AEB"

    # Proteção para leituras inválidas
    if d is None or v is None:
        car.set_u(0.0)
        return 0.0, "SENSOR_INVALIDO"

    d = float(d)
    v = max(0.0, float(v))

    # Distâncias dinâmicas de atuação
    d_stop = v**2 / (2.0 * A_CD)
    d_aeb = d_stop + v * ATRASO + MARGEM
    d_fcw = d_aeb + MARGEM_FCW
    
    print(f"Distancia para parar: {d_stop:.2f}")

    if d <= d_aeb:
        estado_adas["aeb_latched"] = True
        estado_adas["clear_count"] = 0
        car.set_u(0.0)
        return 0.0, "AEB"

    elif d <= d_fcw:
        car.set_u(u)
        return u, "FCW"

    car.set_u(u)
    return u, "CRUISE"
def control_OnOff(car, vel):
	v, w = car.get_vel()
	if v < vel:
		car.set_u(0.5)
	else:
		car.set_u(0.0)
		
########################################
# thread de visão
########################################
def vision_func(car):
		
	# pega imagem
	image = car.get_image(gray=False)
	
	# ultrasom
	dist, _ = car.get_distance()
	#print(f'Ultrasonic distance: {dist:.1f}')
	
	return image
				
########################################
# main program
########################################
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
		tau = 0.8

		control_params = []
		estado_adas = {
    "aeb_latched": False,
    "clear_count": 0
}

		# main loop
		while car.t <= parameters['ts']:
			
			# lê senores
			car.step()
						
			# funcao de controle
			integral, prop, i, tot, ref_filtrada = control_func(car, integral, ref_filtrada, tau, estado_adas)
			# control_OnOff(car, 1.5)
			# if car.t < 2.0:
			# 	car.set_u(0.5)
			# elif car.t > 2.0 and car.t < 5.0:
			# 	car.set_u(1.0)
			# else:
			# 	car.set_u(0.0)
		

			# data = {"t": car.t, "p": prop, "i": i, "tot": tot}
			# control_params.append(data)
			# car.set_u(1.0)
			
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
			t_control = [data["t"] for data in control_params]
			p = [data["p"] for data in control_params]
			i = [data["i"] for data in control_params]
			tot = [data["tot"] for data in control_params]
			plt.plot(t_control, p, label='P', color='red')
			plt.plot(t_control, i, label='I')
			plt.plot(t_control, tot, label='PI')
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
