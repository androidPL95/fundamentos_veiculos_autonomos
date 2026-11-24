"""Experimento independente de condução por cones no CoppeliaSim.

Abra a cena simulador_cones_real.ttt e rode python -B main_cones.py.
A pose do simulador serve para projetar o piso e compensar o movimento
entre quadros. A trajetória é construída a partir dos cones da câmera.
"""

import argparse
from collections import Counter, deque
import csv
from dataclasses import dataclass
import math
from pathlib import Path
from time import perf_counter

import cv2
import numpy as np

from fva_car import Car
from fva_car.car import CAR
from visao_cones import MemoriaCones, ProjecaoSolo, detectar, planejar, projetar


# Ajustes do ensaio. As opções de terminal sobrescrevem estes valores.
PARAMETROS = {
    # Controle de velocidade
    "velocidade": 0.5,           # m/s com duas bordas
    "fator_uma_borda": 0.70,     # fração da velocidade com uma borda
    "kp_vel": 0.50,              # ganho do erro de velocidade
    "kd_vel": 0.25,              # ganho de amortecimento pela aceleração
    "tau_vel": 0.45,             # s, constante de tempo da referência de velocidade

    # Controle lateral
    "ganho_direcao": 1.0,        # multiplica o ângulo calculado pelo Pure Pursuit
    "limite_direcao": 5.0,       # graus
    "taxa_direcao": 25.0,        # graus/s
    "sinal_esterco": -1,         # convenção do atuador do simulador
    "antecipacao": 3.0,          # m, distância nominal do alvo
    "confirmacao_alvo": 0.25,    # s de alvo válido antes de partir ou retomar

    # Pista, segmentação e memória
    "largura": 4.25,             # m entre as bordas
    "esquerda": "vermelho",
    "piso_z": 0.0,               # m no referencial do CoppeliaSim
    "alcance": 13.5,             # m, alcance máximo da detecção
    "saturacao_min": 65,         # canal S do HSV (0 a 255)
    "brilho_min": 24,            # canal V do HSV (0 a 255)
    "area_min": 12,              # pixels, área mínima de um componente
    "tempo_memoria": 7.0,        # s, validade de uma detecção guardada
    "raio_associacao": 0.65,     # m, tolerância para atualizar um cone conhecido
    "peso_medicao": 0.45,        # peso do cone recém-detectado na atualização

    # Freio de emergência pelo ultrassom
    "aeb_margem": 0.5,           # m, margem somada à distância de frenagem
    "aeb_coef_vel": 3.0,         # s²/m, coeficiente do quadrado da velocidade

    # Execução e debug
    "duracao": 180.0,            # s simulados
    "salvar_frames": 0,          # salva um painel a cada N passos (0 desativa)
    "sem_janela": False,
}

PASTA_LOGS = Path(__file__).resolve().parent / "logs" / "cones"
JANELA = "FVA | Conducao por cones"
CORES = {"vermelho": (80, 95, 255), "verde": (90, 230, 120)}
AZUL = (255, 216, 75)
CINZA = (205, 205, 205)
CAMPOS = (
    "t_s", "x_m", "y_m", "rumo_graus", "vel_m_s", "vel_ref_m_s", "u",
    "modo", "motivo_visao", "estado_controle", "vermelhos", "verdes",
    "vermelhos_memoria", "verdes_memoria", "alvo_frente_m", "alvo_esquerda_m",
    "direcao_bruta_graus", "direcao_cmd_graus", "direcao_api_graus", "ultrassom_m", "visao_ms",
)


class GeometriaCamera:
    def __init__(self, car, piso_z):
        self.car = car
        self.piso_z = piso_z
        sim = car.sim
        self.traseiro = sim.getObject("/Car/axis_B", {"noError": True})
        self.dianteiro = sim.getObject("/Car/axis_F", {"noError": True})
        if -1 in (self.traseiro, self.dianteiro):
            raise RuntimeError("A cena precisa conter /Car/axis_B e /Car/axis_F.")
        if not sim.getObjectInt32Param(car.cam, sim.visionintparam_perspective_operation):
            raise RuntimeError("Configure /Car/Vision_sensor como sensor em perspectiva.")
        self.frustum = sim.getObjectFloatArrayParam(car.cam, sim.visionfarrayparam_viewfrustum)

    def calcular(self, formato):
        car, sim = self.car, self.car.sim
        mundo = sim.handle_world
        traseiro = sim.getObjectPosition(self.traseiro, mundo)
        dianteiro = sim.getObjectPosition(self.dianteiro, mundo)
        camera = sim.getObjectMatrix(car.cam, mundo)
        return ProjecaoSolo.calcular(formato, camera, self.frustum, traseiro, dianteiro, self.piso_z)


@dataclass
class Comando:
    direcao_bruta: float | None
    direcao: float
    direcao_api: float
    velocidade: float
    distancia: float
    estado: str


def frear(car):
    car.set_u(-CAR["ACCELMAX"])
    # Mantém os motores em velocidade zero durante a frenagem.
    for motor in (car.motorL, car.motorR):
        car.sim.setJointTargetVelocity(motor, 0.0)


class Controle:
    def __init__(self, args):
        self.args = args
        self.direcao = 0.0
        self.ref_filtrada = 0.0
        self.historico_ultra = deque(maxlen=5)
        self.aeb = False
        self.alvo_valido_por = 0.0
        self.limite = math.radians(args.limite_direcao)
        self.taxa_direcao = math.radians(args.taxa_direcao)

    def atualizar(self, car, plano):
        args = self.args
        dt = max(0.0, car.dt)
        distancia, valido = car.get_distance()
        if valido and math.isfinite(distancia):
            self.historico_ultra.append(distancia)
            distancia = sum(self.historico_ultra) / len(self.historico_ultra)
            if distancia > 3.8:
                self.aeb = False
            elif distancia <= args.aeb_coef_vel * car.v**2 + args.aeb_margem:
                self.aeb = True

        if plano.alvo is None:
            bruta = None
            velocidade = 0.0
            estado = "Freando por falta de alvo"
            self.alvo_valido_por = 0.0
        else:
            x, y = plano.alvo
            bruta = args.ganho_direcao * math.atan2(2 * CAR["L"] * y, x*x + y*y)
            self.alvo_valido_por = min(args.confirmacao_alvo, self.alvo_valido_por + dt)
            if self.alvo_valido_por < args.confirmacao_alvo:
                velocidade = 0.0
                estado = "Confirmando alvo"
            else:
                fator = args.fator_uma_borda if plano.modo == "uma_borda" else 1.0
                velocidade = args.velocidade * fator
                estado = "Seguindo cones"
        if self.aeb:
            velocidade = 0.0
            estado = "Freando pelo ultrassom"
            self.alvo_valido_por = 0.0

        alvo = float(np.clip(bruta, -self.limite, self.limite)) if velocidade > 0 else 0.0
        variacao = self.taxa_direcao * dt
        self.direcao += float(np.clip(alvo - self.direcao, -variacao, variacao))
        direcao_api = args.sinal_esterco * self.direcao
        car.set_steer(direcao_api)
        car.vref = velocidade
        if velocidade == 0:
            self.ref_filtrada = 0.0
            frear(car)
        else:
            alpha = -math.expm1(-dt / args.tau_vel)
            self.ref_filtrada += alpha * (velocidade - self.ref_filtrada)
            u = args.kp_vel * (self.ref_filtrada - car.v) - args.kd_vel * car.a
            car.set_u(u)
        return Comando(bruta, self.direcao, direcao_api, velocidade, distancia, estado)


def px(ponto):
    return tuple(int(round(v)) for v in ponto)


def texto(img, linhas, origem=(12, 22), cor=CINZA):
    x, y = origem
    for i, linha in enumerate(linhas):
        cv2.putText(img, linha, (x, y + 22*i), cv2.FONT_HERSHEY_SIMPLEX, 0.49, cor, 1, cv2.LINE_AA)


def desenhar_camera(rgb, projecao, cones, plano):
    img = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
    for cone in cones:
        x, y, w, h = cone.caixa
        cor = CORES[cone.cor]
        cv2.rectangle(img, (x, y), (x+w-1, y+h-1), cor, 1)
        cv2.circle(img, px(cone.base), 2, cor, -1, cv2.LINE_AA)
    caminho = plano.caminho
    visivel = projecao.em_frente_da_camera(caminho)
    pontos = projetar(projecao.imagem, caminho[visivel])
    if len(pontos) >= 2:
        cv2.polylines(img, [np.rint(pontos).astype(np.int32)], False, AZUL, 2, cv2.LINE_AA)
    if plano.alvo is not None and projecao.em_frente_da_camera(np.asarray([plano.alvo]))[0]:
        alvo = projetar(projecao.imagem, [plano.alvo])[0]
        cv2.drawMarker(img, px(alvo), AZUL, cv2.MARKER_CROSS, 16, 2, cv2.LINE_AA)
    return img


def desenhar_mapa(altura, largura, memoria, plano, direcao, alcance):
    mapa = np.full((altura, largura, 3), (28, 28, 28), np.uint8)
    origem = np.array([largura / 2, altura - 38])
    escala = min((altura - 72) / alcance, (largura - 34) / 14)

    def xy(ponto):
        x, y = ponto
        return px(origem - escala * np.array([y, x]))

    for metros in range(0, int(alcance) + 1, 2):
        _, y = xy((metros, 0))
        cv2.line(mapa, (14, y), (largura-12, y), (58, 58, 58), 1)
        texto(mapa, [f"{metros} m"], (8, y-3), (155, 155, 155))
    centro, _ = xy((0, 0))
    cv2.line(mapa, (centro, 28), xy((0, 0)), (78, 78, 78), 1)

    for cone in memoria:
        if cone.pos[0] > -0.7:
            cv2.circle(mapa, xy(cone.pos), 4, CORES[cone.cor], -1 if cone.atual else 1, cv2.LINE_AA)
    if plano.modo == "duas_bordas":
        for a, b in plano.apoio:
            cv2.line(mapa, xy(a), xy(b), (100, 100, 100), 1, cv2.LINE_AA)
    elif plano.modo == "uma_borda":
        cv2.polylines(mapa, [np.array([xy(p) for p in plano.apoio], np.int32)],
                      False, (165, 145, 80), 1, cv2.LINE_AA)

    if plano.alvo is not None:
        cv2.polylines(mapa, [np.array([xy(p) for p in plano.caminho], np.int32)],
                      False, AZUL, 2, cv2.LINE_AA)
        cv2.drawMarker(mapa, xy(plano.alvo), AZUL, cv2.MARKER_CROSS, 12, 2, cv2.LINE_AA)
    # Arco do comando limitado, medido a partir do eixo traseiro.
    k = math.tan(direcao) / CAR["L"]
    s = np.linspace(0, 2.0, 16)
    arco = np.column_stack((s, np.zeros_like(s))) if abs(k) < 1e-5 else np.column_stack((
        np.sin(k*s) / k, (1 - np.cos(k*s)) / k,
    ))
    cv2.polylines(mapa, [np.array([xy(p) for p in arco], np.int32)],
                  False, (185, 145, 235), 1, cv2.LINE_AA)
    cv2.drawMarker(mapa, xy((0, 0)), CINZA, cv2.MARKER_TRIANGLE_UP, 9, 1, cv2.LINE_AA)
    texto(mapa, ["Piso | eixo traseiro", "Cheio = atual | Vazio = memoria"], (8, 17))
    return mapa


def painel(rgb, projecao, cones, mascaras, memoria, plano, comando, car, alcance, ms):
    camera = desenhar_camera(rgb, projecao, cones, plano)
    h, w = camera.shape[:2]
    lateral = max(300, w // 2)
    visao = np.full((h, lateral, 3), (33, 33, 33), np.uint8)
    mask = np.zeros_like(camera)
    for cor, mascara in mascaras.items():
        mask[mascara > 0] = CORES[cor]
    visao[:h//2] = cv2.resize(mask, (lateral, h//2), interpolation=cv2.INTER_NEAREST)
    texto(visao, ["Mascaras HSV"], (8, 18))
    atuais = Counter(c.cor for c in cones)
    rastros = Counter(c.cor for c in memoria if not c.atual)
    alvo = "-" if plano.alvo is None else f"{plano.alvo[0]:.2f} m, {plano.alvo[1]:+.2f} m"
    bruto = "-" if comando.direcao_bruta is None else f"{math.degrees(comando.direcao_bruta):+.2f}"
    texto(visao, [
        f"Atuais     VERM {atuais['vermelho']}  VERD {atuais['verde']}",
        f"Memoria    VERM {rastros['vermelho']}  VERD {rastros['verde']}",
        f"Alvo (frente, esquerda) {alvo}",
        f"Direcao (graus) {bruto} -> {math.degrees(comando.direcao):+.2f}",
        f"Comando API {math.degrees(comando.direcao_api):+.2f} graus",
        f"Ultrassom {comando.distancia:.2f} m | Visao {ms:.1f} ms",
        "Azul = alvo/caminho | Roxo = comando",
    ], (8, h//2 + 23))
    mapa = desenhar_mapa(h, lateral, memoria, plano, comando.direcao, alcance)
    header = np.full((52, w+2*lateral, 3), (24, 24, 24), np.uint8)
    modo = plano.modo.replace("_", " ").upper()
    cor = (145, 195, 255) if comando.velocidade == 0.0 else CINZA
    texto(header, [
        f"t {car.t:.1f} s | v {car.v:.2f}/{comando.velocidade:.2f} m/s | {comando.estado}",
        f"{modo} | {plano.motivo}     (Q / Esc encerra)",
    ], (12, 18), cor)
    return np.vstack((header, np.hstack((camera, visao, mapa))))


def registrar(log, car, projecao, cones, memoria, plano, comando, ms):
    atuais = Counter(c.cor for c in cones)
    rastros = Counter(c.cor for c in memoria if not c.atual)
    x, y = plano.alvo if plano.alvo is not None else ("", "")
    bruta = "" if comando.direcao_bruta is None else math.degrees(comando.direcao_bruta)
    rumo = math.degrees(math.atan2(projecao.eixos[1, 0], projecao.eixos[0, 0]))
    log.writerow((
        car.t, *projecao.origem, rumo, car.v, comando.velocidade, car.u,
        plano.modo, plano.motivo, comando.estado, atuais["vermelho"], atuais["verde"],
        rastros["vermelho"], rastros["verde"], x, y, bruta, math.degrees(comando.direcao),
        math.degrees(comando.direcao_api), comando.distancia, ms,
    ))


def argumentos():
    parser = argparse.ArgumentParser(
        description="Seguidor experimental de cones no CoppeliaSim.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    velocidade = parser.add_argument_group("controle de velocidade")
    velocidade.add_argument("--velocidade", type=float, help="Referência com duas bordas (m/s).")
    velocidade.add_argument("--fator-uma-borda", type=float, help="Fração da velocidade com uma borda.")
    velocidade.add_argument("--kp-vel", type=float, help="Ganho do erro de velocidade.")
    velocidade.add_argument("--kd-vel", type=float, help="Ganho de amortecimento pela aceleração.")
    velocidade.add_argument("--tau-vel", type=float, help="Constante de tempo da referência (s).")

    direcao = parser.add_argument_group("controle lateral")
    direcao.add_argument("--ganho-direcao", type=float, help="Multiplicador do ângulo do Pure Pursuit.")
    direcao.add_argument("--limite-direcao", type=float, help="Limite de esterçamento (graus).")
    direcao.add_argument("--taxa-direcao", type=float, help="Limite de variação do esterçamento (graus/s).")
    direcao.add_argument("--sinal-esterco", type=int, choices=(-1, 1), help="Convenção do atuador.")
    direcao.add_argument("--antecipacao", type=float, help="Distância nominal do alvo (m).")
    direcao.add_argument("--confirmacao-alvo", type=float, help="Tempo de alvo válido para partir ou retomar (s).")

    visao = parser.add_argument_group("pista, visão e memória")
    visao.add_argument("--largura", type=float, help="Largura nominal entre as bordas (m).")
    visao.add_argument("--esquerda", choices=CORES, help="Cor da borda esquerda.")
    visao.add_argument("--piso-z", type=float, help="Altura do piso no simulador (m).")
    visao.add_argument("--alcance", type=float, help="Distância máxima para aceitar cones (m).")
    visao.add_argument("--saturacao-min", type=int, help="Saturação mínima no HSV (0 a 255).")
    visao.add_argument("--brilho-min", type=int, help="Brilho mínimo no HSV (0 a 255).")
    visao.add_argument("--area-min", type=int, help="Área mínima de um componente (pixels).")
    visao.add_argument("--tempo-memoria", type=float, help="Validade dos cones guardados (s).")
    visao.add_argument("--raio-associacao", type=float, help="Tolerância para atualizar um cone conhecido (m).")
    visao.add_argument("--peso-medicao", type=float, help="Peso da detecção atual na memória (0 a 1).")

    freio = parser.add_argument_group("freio de emergência")
    freio.add_argument("--aeb-margem", type=float, help="Margem na distância de frenagem (m).")
    freio.add_argument("--aeb-coef-vel", type=float, help="Coeficiente do quadrado da velocidade na frenagem.")

    parser.add_argument("--duracao", type=float, help="Tempo de simulação (s).")
    janela = parser.add_mutually_exclusive_group()
    janela.add_argument("--sem-janela", action="store_true", help="Dispensa o painel ao vivo.")
    janela.add_argument("--janela", dest="sem_janela", action="store_false", help="Exibe o painel ao vivo.")
    parser.add_argument("--salvar-frames", type=int, metavar="N", help="Salva o painel a cada N passos.")
    parser.set_defaults(**PARAMETROS)
    args = parser.parse_args()

    positivos = ("duracao", "velocidade", "largura", "antecipacao", "alcance", "limite_direcao",
                 "tau_vel", "taxa_direcao", "raio_associacao")
    for nome in positivos:
        valor = getattr(args, nome)
        if not math.isfinite(valor) or valor <= 0:
            parser.error(f"--{nome.replace('_','-')} precisa ser positivo e finito")
    nao_negativos = ("kp_vel", "kd_vel", "ganho_direcao", "confirmacao_alvo", "tempo_memoria", "aeb_coef_vel")
    for nome in nao_negativos:
        valor = getattr(args, nome)
        if not math.isfinite(valor) or valor < 0:
            parser.error(f"--{nome.replace('_','-')} precisa ser finito e maior ou igual a zero")
    if not 0 < args.fator_uma_borda <= 1 or not 0 <= args.peso_medicao <= 1:
        parser.error("--fator-uma-borda deve estar em (0, 1] e --peso-medicao em [0, 1]")
    if not 0 <= args.aeb_margem <= 3.8:
        parser.error("--aeb-margem deve estar entre 0 e 3.8 m")
    if any(not 0 <= valor <= 255 for valor in (args.saturacao_min, args.brilho_min)):
        parser.error("--saturacao-min e --brilho-min devem estar entre 0 e 255")
    if args.area_min < 1:
        parser.error("--area-min deve ser de pelo menos 1 pixel")
    if args.sinal_esterco not in (-1, 1) or args.esquerda not in CORES:
        parser.error("confira sinal_esterco e esquerda em PARAMETROS")
    if args.velocidade > CAR["VELMAX"]:
        parser.error(f"--velocidade aceita ate {CAR['VELMAX']} m/s")
    if args.limite_direcao > math.degrees(CAR["STEERMAX"]):
        parser.error(f"--limite-direcao aceita ate {math.degrees(CAR['STEERMAX']):g} graus")
    if args.alcance < 1.3 or args.alcance < args.antecipacao:
        parser.error("--alcance precisa ser pelo menos 1.3 m e cobrir a antecipacao")
    if not math.isfinite(args.piso_z) or args.salvar_frames < 0:
        parser.error("--piso-z precisa ser finito e --salvar-frames deve ser zero ou positivo")
    return args


def executar(car, args, pasta, log):
    geometria = GeometriaCamera(car, args.piso_z)
    memoria = MemoriaCones(args.tempo_memoria, args.raio_associacao, args.peso_medicao)
    controle = Controle(args)
    abriu_janela = False
    ultima_situacao = None

    car.start_mission()
    passo = 0
    while car.t < args.duracao and car.step():
        if car.dt <= 0:
            break
        rgb = car.get_image(gray=False)
        projecao = geometria.calcular(rgb.shape)
        inicio = perf_counter()
        cones, mascaras = detectar(rgb, projecao, args.alcance, saturacao_min=args.saturacao_min,
                                   brilho_min=args.brilho_min, area_min=args.area_min)
        rastros = memoria.atualizar(cones, projecao, car.t, args.alcance)
        plano = planejar(cones, rastros, args.esquerda, args.largura, args.antecipacao)
        ms = 1000 * (perf_counter() - inicio)
        comando = controle.atualizar(car, plano)
        registrar(log, car, projecao, cones, rastros, plano, comando, ms)

        situacao = (plano.modo, plano.motivo, comando.estado)
        if situacao != ultima_situacao:
            print(f"{car.t:6.2f} s | {comando.estado} | {plano.motivo}", flush=True)
            ultima_situacao = situacao

        salvar = args.salvar_frames > 0 and passo % args.salvar_frames == 0
        if not args.sem_janela or salvar:
            imagem = painel(rgb, projecao, cones, mascaras, rastros, plano, comando, car, args.alcance, ms)
        if salvar:
            ok, png = cv2.imencode(".png", imagem)
            if not ok:
                raise RuntimeError("Falha ao codificar o painel de debug.")
            png.tofile(pasta / f"quadro_{passo:05d}.png")
        if not args.sem_janela:
            if abriu_janela:
                try:
                    if cv2.getWindowProperty(JANELA, cv2.WND_PROP_VISIBLE) < 1:
                        break
                except cv2.error:
                    break
            cv2.imshow(JANELA, imagem)
            abriu_janela = True
            if cv2.waitKey(1) & 0xFF in (27, ord("q"), ord("Q")):
                break
        passo += 1


def main():
    args = argumentos()
    cv2.setNumThreads(2)
    if not args.sem_janela:
        cv2.namedWindow(JANELA, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(JANELA, 1240, 525)
    car = None
    try:
        car = Car({"ts": args.duracao, "logfile": str(PASTA_LOGS), "save": True, "beep": False})
        pasta = Path(car.logfile)
        print(f"Debug: {pasta}", flush=True)
        with (pasta / "cones.csv").open("w", encoding="utf-8", newline="") as arquivo:
            log = csv.writer(arquivo)
            log.writerow(CAMPOS)
            executar(car, args, pasta, log)
    except KeyboardInterrupt:
        print("\nEnsaio interrompido.")
    finally:
        try:
            if car is not None:
                try:
                    if car.sim.getSimulationState() != car.sim.simulation_stopped:
                        try:
                            frear(car)
                        finally:
                            car.sim.stopSimulation()
                finally:
                    if hasattr(car, "traj"):
                        car.save()
        finally:
            if not args.sem_janela:
                cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
