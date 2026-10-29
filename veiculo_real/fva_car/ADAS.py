from fva_car.filter import Kalman1D
import numpy as np
class ADAS: 
    def __init__(self):
        self.d_ant = 0.0
        self.kalman_dist = Kalman1D(initial=4.0)
        self.kalman_vel = Kalman1D(initial=0.0)
        self.erro_ant = 0.0
        self.integral = 0.0
        self.int_deltaD = 0.0
        
    def PID(self, car,  ref, y):
        kp = 0.1
        ki = 0.01
        kd = 0.01

        dt = car.dt
        erro = ref - y

        # Proporcional
        P = kp * erro

        # Integral: acumula o erro ao longo do tempo
        self.integral += erro * dt
        I = ki * self.integral

        # Derivativo: variação do erro
        D = kd * (erro - self.erro_ant) / dt

        self.erro_ant = erro

        u = P + I + D
        return u

    def ACC(self, car, setpoint_d, setpoint_v):
        ##########################
        #Adaptative Cruise Control
        ##########################
        
        d, d_valid = car.get_distance()
        v = car.get_vel()[0]
        
        ########### caso o veiculo esteja perto demais #############
        if(d < setpoint_d*0.9):
            print("Veiculo muito próximo, reduzindo velocidade")
            self.int_deltaD += d - self.d_ant # resultado em cm
            
            #calcula o novo setpoint de velocidade
            
            delta_v = self.int_deltaD / car.dt #resultado em cm/s
            delta_v = delta_v / 100.0 #resultado em m/s
            
            delta_v = np.clip(delta_v, -setpoint_v, 0)
            
            
            setpoint = setpoint_v + delta_v
            
            
        elif(d > setpoint_d*1.1):
            print("Veiculo muito distante, aumentando velocidade")
            self.int_deltaD += d - self.d_ant # resultado em cm
            
            #calcula o novo setpoint de velocidade
            
            delta_v = self.int_deltaD / car.dt #resultado em cm/s
            delta_v = delta_v / 100.0 #resultado em m/s
            
            delta_v = np.clip(delta_v, 0, setpoint_v)
            
            setpoint = setpoint_v + delta_v
            
        else:
            setpoint = setpoint_v
            
        self.d_ant = d
        
        print(f"Setpoint ACC: {setpoint:.2f} m/s")
        return setpoint
            
            
            
            
        