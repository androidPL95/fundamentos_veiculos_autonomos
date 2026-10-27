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
from collections import deque
import cv2
import os
os.environ["QT_QPA_PLATFORM"] = "xcb"
import matplotlib.pyplot as plt
plt.rcParams['figure.figsize'] = (6,8)

# Globais
parameters = {	
				'ts'		: 20.0, 			# tempo da simulacao
				'save'		: True,
				'logfile'	: 'logs/',
				'beep'		: True,
			}

SET_VEL = 1.0

#Rx = -0.0931
	
########################################
# thread de controle de velocidade
########################################
def control_func(car, integral, erro_anterior, ref_filtrada, tau):

	ref_filtrada = ref_filtrada + (car.dt / tau) * (SET_VEL - ref_filtrada)

	
	v, w = car.get_vel()
	erro = ref_filtrada - v
	integral = integral + erro*car.dt
	# derivada = (erro - erro_anterior) / car.dt if car.dt > 0.0 else 0.0
	prop = 13.4 * erro
	i = 10.1 * integral
	# d = 3.1 * derivada
	# tot = prop + i + d
	tot = prop + i
	# Controlador PID: o total antes da saturacao mostra as tres contribuicoes
	car.set_u(tot)

	# return integral, erro, prop, i, d, tot, ref_filtrada
	return integral, erro, prop, i, tot, ref_filtrada
		
########################################
# thread de visão
########################################
def vision_func(car):
		
	# pega imagem
	# image = car.get_image(gray=False)
	image = car.get_image(gray=False)

	image_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

	image_hsv = cv2.bilateralFilter(image_hsv, 9, 75, 75)
	
	# ultrasom
	dist, _ = car.get_distance()
	#print(f'Ultrasonic distance: {dist:.1f}')
	
	return image, image_hsv
				
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
		erro_anterior = 0.0
		ref_filtrada = 0.0
		tau = 0.8

		control_params = []

		dq_rho = deque(maxlen=20)
		dq_theta = deque(maxlen=20)

		# main loop
		while car.t <= parameters['ts']:
			
			# lê senores
			car.step()
						
			# funcao de controle
			# integral, erro_anterior, prop, i, d, tot, ref_filtrada = control_func(
			# 	car, integral, erro_anterior, ref_filtrada, tau
			# )
			integral, erro_anterior, prop, i, tot, ref_filtrada = control_func(
				car, integral, erro_anterior, ref_filtrada, tau
			)

			# data = {"t": car.t, "p": prop, "i": i, "d": d, "tot": tot}
			data = {"t": car.t, "p": prop, "i": i, "tot": tot}
			control_params.append(data)
			
			# funcao de visao
			image, image_hsv = vision_func(car)

			lower_yellow = np.array([20,50,50])
			upper_yellow = np.array([40,255,255])
		
			# Threshold the HSV image to get only yellow colors
			mask = cv2.inRange(image_hsv, lower_yellow, upper_yellow)

			canny = cv2.Canny(mask, 50, 200)

			image_copy = image.copy()
			
			lines = cv2.HoughLines(canny,1,np.pi/180,80)

			if lines is not None:
				for line in lines:
					rho,theta = line[0]
					a = np.cos(theta)
					b = np.sin(theta)
					x0 = a*rho
					y0 = b*rho
					x1 = int(x0 + 1000*(-b))
					y1 = int(y0 + 1000*(a))
					x2 = int(x0 - 1000*(-b))
					y2 = int(y0 - 1000*(a))
				
					cv2.line(image_copy,(x1,y1),(x2,y2),(0,0,255),2)

			if lines is not None:
				rho_mean = float(np.mean(lines[:,0,0]))
				theta_mean = float(np.mean(lines[:,0,1]))

				dq_rho.append(rho_mean)
				dq_theta.append(theta_mean)

				rho_filtrado = np.mean(dq_rho)
				theta_filtrado = np.mean(dq_theta)

				# if abs(rho_filtrado - rho_mean) > 10:
				# 	rho_filtrado = rho_mean 

				a = np.cos(theta_filtrado)
				b = np.sin(theta_filtrado)
				x0 = a*rho_filtrado
				y0 = b*rho_filtrado
				x1 = int(x0 + 1000*(-b))
				y1 = int(y0 + 1000*(a))
				x2 = int(x0 - 1000*(-b))
				y2 = int(y0 - 1000*(a))
				cv2.line(image_copy, (x1,y1), (x2,y2), (0,255,0), 2)
			
			########################################
			# plota	
			plt.subplot(311)
			plt.cla()
			plt.gca().imshow(image_copy, cmap='gray')
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
			# d = [data["d"] for data in control_params]
			tot = [data["tot"] for data in control_params]
			plt.plot(t_control, p, label='P', color='red')
			plt.plot(t_control, i, label='I')
			# plt.plot(t_control, d, label='D')
			plt.plot(t_control, tot, label='PID')
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
