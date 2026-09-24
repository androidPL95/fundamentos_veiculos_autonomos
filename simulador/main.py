# -*- coding: utf-8 -*-
# Disciplina: Tópicos em Engenharia de Controle e Automação IV (ENG075): 
# Fundamentos de Veículos Autônomos - 2026/1
# Professores: Armando Alves Neto e Leonardo A. Mozelli
# Cursos: Engenharia de Controle e Automação
# DELT – Escola de Engenharia
########################################
from fva_car import Car
import numpy as np
import os
import matplotlib
matplotlib.use("QtAgg")
import matplotlib.pyplot as plt
plt.rcParams['figure.figsize'] = (6,8)

SET_VEL = 1.0
integral = 0.0

parameters = {	
				'ts'		: 20.0,
				'save'		: True,
				'logfile'	: 'logs/',
				'beep'		: True,
			}

########################################
# thread de visão
########################################
def vision_func(car):
		
	image = car.get_image(gray=False)
	
	dist, _ = car.get_distance()
	
	return image
				
########################################
# main program
########################################
if __name__ == "__main__":
	
	plt.figure(1)
	plt.ion()
	
	car = Car(parameters)
	
	try:
		car.start_mission()
		ref_filtrada = 0.0
		tau = 0.8
		while car.t <= parameters['ts']:
			
			car.step()
			ref_filtrada = ref_filtrada + (car.dt / tau) * (SET_VEL - ref_filtrada)

			v, w = car.get_vel()
			# erro = SET_VEL - v
			erro = ref_filtrada - v

			integral = integral + erro * car.dt
			prop = 12 * erro
			i = 5 * integral
			tot = prop + i

			car.set_u(tot)

			print(f'Velocidade: {v:.2f} m/s, Controle: {tot:.2f}, Erro: {erro:.2f}, Integral: {integral:.2f}')

			plt.subplot(211)
			plt.cla()
			plt.axis('off')
			plt.title(f'Telemetria em t={car.t:.1f}s velocidade={car.get_vel()[0]:.2f} m/s')
			
			plt.subplot(212)
			plt.cla()
			t = [traj['t'] for traj in car.traj]
			v = [traj['v'] for traj in car.traj]
			plt.plot(t,v)
			plt.ylabel('v[m/s]')
			plt.xlabel('t[s]')
			
			plt.show()
			plt.pause(0.01)

		if parameters['save']:
			car.save()
			
	finally:
		car.close()