# -*- coding: utf-8 -*-
# Disciplina: Tópicos em Engenharia de Controle e Automação IV (ENG075): 
# Fundamentos de Veículos Autônomos - 2026/1
# Professores: Armando Alves Neto e Leonardo A. Mozelli
# Cursos: Engenharia de Controle e Automação
# DELT – Escola de Engenharia
# Universidade Federal de Minas Gerais
########################################
import class_car as cp
import numpy as np
import os
os.environ["QT_QPA_PLATFORM"] = "xcb"
import matplotlib.pyplot as plt
plt.rcParams['figure.figsize'] = (10,10)

# Globais
parameters = {	
				'ts'		: 20.0, 			# tempo da simulacao
				'save'		: True,
				'logfile'	: 'logs/',
			}

SET_VEL = 1.0

# Rx = -0.0931
	
########################################
# thread de controle de velocidade
########################################
def control_func(car, integral):
		
	v, w = car.get_vel()
	erro = SET_VEL - v
	integral = integral + erro*car.dt
	prop = 10 * erro
	i = 0.1 * integral
	tot = prop + i
	# Controlador PI: o total antes da saturacao mostra as duas contribuicoes
	car.set_u(tot)

	return integral, prop, i, tot

		
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
	
	plt.figure(1, figsize=(10, 15))
	plt.ion()
	
	# cria comunicação com o carrinho
	car = cp.Car(parameters)

	integral = 0

	control_params = []
	
	try:
		# começa a simulação
		car.start_mission()

		# main loop
		while car.t <= parameters['ts']:
			
			# lê senores
			car.step()
			
			# funcao de controle
			integral, prop, i, tot = control_func(car, integral)

			data = {"t": car.t, "p": prop, "i": i, "tot": tot}
			control_params.append(data)
			
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
			plt.ylabel('u')
			plt.xlabel('t[s]')
			plt.legend()
			
			plt.tight_layout()
			
			plt.show()
			plt.pause(0.01)

		# salva
		if parameters['save']:
			car.save()
			
	finally:
		car.close()
