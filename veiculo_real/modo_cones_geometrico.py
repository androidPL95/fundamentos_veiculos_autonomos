"""Executa o modo geométrico do veículo real. Os ajustes ficam em main.py."""

import csv
from dataclasses import dataclass
import json
import math
from pathlib import Path
import threading
import time

import cv2
import numpy as np

from visao_cones_geometrico import ConeLocal, MemoriaCones, ProjecaoSolo, detectar, planejar, projetar


CORES = {"vermelho": (80, 95, 255), "verde": (90, 230, 120)}
AZUL = (255, 216, 75)
CAMPOS = (
    "t_s", "vel_m_s", "vel_ref_m_s", "estado_controle", "plano", "motivo_visao",
    "idade_quadro_s", "visao_ms", "vermelhos", "verdes", "memoria", "alvo_frente_m",
    "alvo_esquerda_m", "esterco_graus", "ultrassom_m",
)


def configurar(padrao, ajustes=(), velocidade=None):
    config = padrao.copy()
    inteiros = ("sinal_esterco", "saturacao_min", "brilho_min", "area_min")
    for ajuste in ajustes:
        nome, separador, valor = ajuste.partition("=")
        if not separador or nome not in padrao or not valor.strip():
            raise ValueError(f"Ajuste inválido {ajuste!r}. Use um nome de PARAMETROS_CONES, como velocidade=0.25.")
        try:
            valor = valor.strip()
            if nome == "esquerda":
                config[nome] = valor
            elif nome in inteiros:
                config[nome] = int(valor)
            else:
                config[nome] = float(valor)
        except ValueError as exc:
            raise ValueError(f"Valor inválido para {nome}: {valor!r}.") from exc
    if velocidade is not None:
        config["velocidade"] = velocidade
    for chave in ("velocidade", "tau_vel", "limite_direcao", "taxa_direcao", "idade_max_quadro_s",
                  "antecipacao_m", "alcance_m", "raio_associacao_m"):
        valor = config[chave]
        if not math.isfinite(valor) or valor <= 0:
            raise ValueError(f"{chave} deve ser positivo e finito.")
    for chave in ("kp_vel", "kd_vel", "ganho_direcao", "alvo_valido_s", "memoria_s"):
        valor = config[chave]
        if not math.isfinite(valor) or valor < 0:
            raise ValueError(f"{chave} deve ser finito e maior ou igual a zero.")
    if not 0 < config["fator_uma_borda"] <= 1 or not 0 <= config["peso_medicao"] <= 1:
        raise ValueError("Confira fator_uma_borda e peso_medicao.")
    if config["sinal_esterco"] not in (-1, 1) or config["esquerda"] not in CORES:
        raise ValueError("Confira sinal_esterco e esquerda.")
    if not all(type(config[k]) is int and 0 <= config[k] <= 255 for k in ("saturacao_min", "brilho_min")):
        raise ValueError("Saturação e brilho mínimos devem ser inteiros de 0 a 255.")
    if type(config["area_min"]) is not int or config["area_min"] < 1:
        raise ValueError("A área mínima deve ser um inteiro positivo.")
    return config


@dataclass(frozen=True)
class Calibracao:
    resolucao: tuple
    largura: float
    alcance: float
    imagem: np.ndarray
    solo: np.ndarray
    erro_px: float

    @classmethod
    def carregar(cls, caminho, config):
        dados = json.loads(caminho.read_text(encoding="utf-8-sig"))
        try:
            w, h = dados["resolucao"]
            largura = float(dados["largura_m"])
            marcas = np.asarray(dados["pontos"], dtype=float)
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError("Informe resolucao, largura_m e pontos [u, v, x, y].") from exc
        if not all(type(v) is int and v > 0 for v in (w, h)):
            raise ValueError("A resolução deve conter largura e altura inteiras em pixels.")
        if not math.isfinite(largura) or largura <= 0:
            raise ValueError("A largura do corredor deve ser positiva.")
        if marcas.ndim != 2 or marcas.shape[1] != 4 or len(marcas) < 4 or not np.isfinite(marcas).all():
            raise ValueError("São necessários pelo menos quatro pares finitos [u, v, x, y].")
        uv, xy = marcas[:, :2], marcas[:, 2:]
        if (uv < 0).any() or (uv >= (w, h)).any() or (xy[:, 0] <= 0).any():
            raise ValueError("Use pixels dentro da imagem e marcas à frente do eixo traseiro.")
        if any(np.linalg.matrix_rank(p - p.mean(axis=0)) < 2 for p in (uv, xy)):
            raise ValueError("Distribua as marcas no chão em profundidade e lateralmente.")
        imagem, _ = cv2.findHomography(xy, uv, 0)
        if imagem is None or not np.isfinite(imagem).all() or np.linalg.matrix_rank(imagem) < 3:
            raise ValueError("Os pontos não definem uma homografia válida.")
        profundidades = xy @ imagem[2, :2] + imagem[2, 2]
        if not (np.all(profundidades > 0) or np.all(profundidades < 0)):
            raise ValueError("As marcas precisam estar no mesmo lado do plano da câmera.")
        imagem /= np.median(profundidades)
        erro = float(np.linalg.norm(projetar(imagem, xy) - uv, axis=1).max())
        if not math.isfinite(erro) or erro > 8.0:
            raise ValueError(f"Erro de projeção de {erro:.1f} px. Confira a ordem e a posição das marcas.")

        x_perto, x_longe = xy[:, 0].min(), xy[:, 0].max()
        perto, longe, esquerda, direita = projetar(
            imagem, [[x_perto, 0], [x_longe, 0], [x_perto, 0.1], [x_perto, -0.1]]
        )
        if longe[1] >= perto[1] or esquerda[0] >= direita[0]:
            raise ValueError("Confira os eixos da calibração. X é à frente e Y é positivo à esquerda.")
        alcance = min(config["alcance_m"], x_longe)
        if alcance < config["antecipacao_m"]:
            raise ValueError("As marcas mais distantes devem alcançar pelo menos a antecipação configurada.")
        return cls((w, h), largura, float(alcance), imagem, np.linalg.inv(imagem), erro)


@dataclass(frozen=True)
class Pose:
    origem: tuple = (0.0, 0.0)
    rumo: float = 0.0
    medida_em: float | None = None

    @property
    def eixos(self):
        c, s = math.cos(self.rumo), math.sin(self.rumo)
        return np.array([[c, -s], [s, c]])


@dataclass(frozen=True)
class Resultado:
    capturado: float
    modo: str
    motivo: str
    alvo_mundo: tuple | None = None
    vermelhos: int = 0
    verdes: int = 0
    memoria: int = 0
    ms: float = 0.0


class Visao:
    def __init__(self, calibracao, config, usar_memoria=True):
        self.calibracao = calibracao
        self.config = config
        self.memoria = MemoriaCones(config["memoria_s"] if usar_memoria else 0.0,
                                    config["raio_associacao_m"], config["peso_medicao"])

    def processar(self, frame, pose, capturado, estado):
        inicio = time.monotonic()
        cal, cfg = self.calibracao, self.config
        if frame.shape[1::-1] != cal.resolucao:
            motivo = f"Resolucao {frame.shape[1::-1]} difere da calibracao {cal.resolucao}"
            return Resultado(capturado, "sem_alvo", motivo), anotar(frame, (), (), motivo, estado)
        if pose.medida_em is not None and capturado - pose.medida_em > cfg["idade_max_quadro_s"]:
            motivo = "Odometria atrasada na captura"
            return Resultado(capturado, "sem_alvo", motivo), anotar(frame, (), (), motivo, estado)
        if not all(math.isfinite(v) for v in (*pose.origem, pose.rumo)):
            motivo = "Odometria invalida"
            return Resultado(capturado, "sem_alvo", motivo), anotar(frame, (), (), motivo, estado)
        projecao = ProjecaoSolo(cal.imagem, cal.solo, np.asarray(pose.origem), pose.eixos)
        cones, _ = detectar(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB), projecao, cal.alcance,
                            saturacao_min=cfg["saturacao_min"], brilho_min=cfg["brilho_min"],
                            area_min=cfg["area_min"])
        rastros = self.memoria.atualizar(cones, projecao, capturado, cal.alcance)

        # As distâncias internas do planejador vieram da pista simulada.
        escala = cal.largura / 4.25
        locais = [ConeLocal(c.cor, c.pos / escala, c.atual) for c in rastros]
        plano = planejar(cones, locais, cfg["esquerda"], 4.25, cfg["antecipacao_m"] / escala)
        alvo_mundo = None
        pontos_imagem = ()
        if plano.alvo is not None:
            alvo = escala * np.asarray(plano.alvo)
            if 0.25 <= alvo[0] <= cal.alcance and projecao.em_frente_da_camera(alvo[None, :])[0]:
                alvo_mundo = tuple(projecao.origem + projecao.eixos @ alvo)
                caminho = escala * plano.caminho
                visivel = projecao.em_frente_da_camera(caminho)
                pontos_imagem = projetar(cal.imagem, np.vstack((caminho[visivel], alvo)))
        modo = plano.modo if alvo_mundo is not None else "sem_alvo"
        motivo = plano.motivo
        if plano.modo == "uma_borda":
            motivo = f"{motivo} | largura {cal.largura:.2f} m"
        if plano.alvo is not None and alvo_mundo is None:
            motivo = "Alvo fora do alcance calibrado"
        resultado = Resultado(
            capturado, modo, motivo, alvo_mundo, sum(c.cor == "vermelho" for c in cones),
            sum(c.cor == "verde" for c in cones), sum(not c.atual for c in rastros),
            1000 * (time.monotonic() - inicio),
        )
        status = f"{modo} | {motivo} | R{resultado.vermelhos} G{resultado.verdes} M{resultado.memoria}"
        return resultado, anotar(frame, cones, pontos_imagem, status, estado)


def anotar(frame, cones, pontos, status, estado):
    img = frame.copy()
    for cone in cones:
        x, y, w, h = cone.caixa
        cor = CORES[cone.cor]
        cv2.rectangle(img, (x, y), (x+w-1, y+h-1), cor, 1)
        cv2.circle(img, tuple(round(v) for v in cone.base), 2, cor, -1)
    if len(pontos):
        pixels = np.rint(np.clip(pontos, -5000, 5000)).astype(np.int32)
        if len(pixels) > 2:
            cv2.polylines(img, [pixels[:-1]], False, AZUL, 2, cv2.LINE_AA)
        cv2.drawMarker(img, tuple(pixels[-1]), AZUL, cv2.MARKER_CROSS, 15, 2)
    cv2.rectangle(img, (0, 0), (img.shape[1], 43), (24, 24, 24), -1)
    for i, texto in enumerate((status, estado)):
        cv2.putText(img, texto, (8, 17+19*i), cv2.FONT_HERSHEY_SIMPLEX, .45,
                    (230, 230, 230), 1, cv2.LINE_AA)
    return img


class CanalVisao:
    def __init__(self):
        self.lock = threading.Lock()
        self.stop = threading.Event()
        self.pose = Pose()
        self.estado = "Aguardando controle"
        self.resultado = None
        self.erro = None

    def executar(self, camera, visao):
        try:
            while not self.stop.is_set():
                with self.lock:
                    pose, estado = self.pose, self.estado
                # Mede idade desde o início da leitura, inclusive se a webcam travar.
                capturado = time.monotonic()
                frame = camera.get_image(gray=False)
                if frame is None:
                    resultado = Resultado(capturado, "sem_alvo", "Falha de captura")
                    anotado = None
                else:
                    resultado, anotado = visao.processar(frame, pose, capturado, estado)
                with self.lock:
                    self.resultado = resultado
                if anotado is not None:
                    camera.publish_frame(anotado)
                if frame is None:
                    self.stop.wait(.02)
        except Exception as exc:
            with self.lock:
                self.erro = exc
                self.resultado = None
            self.stop.set()


class Controle:
    def __init__(self, config, entre_eixos, freio_max):
        self.cfg = config
        self.entre_eixos = entre_eixos
        self.freio_max = freio_max
        self.direcao = 0.0
        self.ref_filtrada = 0.0
        self.primeiro = self.ultimo = None
        self.aeb = False

    def aplicar(self, car, pose, resultado, agora):
        cfg, dt = self.cfg, max(0.0, car.dt)
        idade = agora - resultado.capturado if resultado else math.inf
        distancia, valida = car.get_distance()
        ref, bruta, alvo = 0.0, 0.0, None
        telemetria_valida = all(math.isfinite(v) for v in (*pose.origem, pose.rumo, car.v, car.a, car.dt))
        if valida and math.isfinite(distancia):
            if distancia > 3.8:
                self.aeb = False
            elif telemetria_valida and distancia <= 3.0 * car.v**2 + 0.5:
                self.aeb = True

        if car.emergencia:
            estado = "Emergencia"
        elif not (car.sel_direcao and car.sel_tracao):
            estado = "Freio | chaves fora de AUTO"
        elif getattr(car.gear, "value", None) != "forward":
            estado = "Freio | marcha fora de frente"
        elif not telemetria_valida:
            estado = "Freio | telemetria invalida"
        elif not valida or not math.isfinite(distancia):
            estado = "Freio | ultrassom invalido"
        elif self.aeb:
            estado = "Freio | ultrassom"
        elif idade > cfg["idade_max_quadro_s"]:
            estado = "Freio | quadro ausente ou atrasado"
        elif resultado.alvo_mundo is None:
            estado = "Freio | sem corredor valido"
        else:
            alvo = pose.eixos.T @ (np.asarray(resultado.alvo_mundo) - pose.origem)
            if not np.isfinite(alvo).all() or alvo[0] < 0.20:
                estado = "Freio | alvo ultrapassado"
            else:
                capturado = resultado.capturado
                if self.ultimo is None or capturado - self.ultimo > cfg["idade_max_quadro_s"]:
                    self.primeiro = capturado
                self.ultimo = capturado
                if capturado - self.primeiro < cfg["alvo_valido_s"]:
                    estado = "Confirmando alvo"
                else:
                    estado = "Seguindo cones"
                    x, y = alvo
                    bruta = cfg["sinal_esterco"] * cfg["ganho_direcao"] * math.atan2(
                        2 * self.entre_eixos * y, x*x + y*y
                    )
                    fator = cfg["fator_uma_borda"] if resultado.modo == "uma_borda" else 1.0
                    ref = cfg["velocidade"] * fator

        if estado != "Seguindo cones" and estado != "Confirmando alvo":
            self.primeiro = self.ultimo = None
        limite = math.radians(cfg["limite_direcao"])
        passo = math.radians(cfg["taxa_direcao"]) * dt
        destino = float(np.clip(bruta, -limite, limite))
        self.direcao += float(np.clip(destino - self.direcao, -passo, passo))
        car.vref = ref
        if ref <= 0:
            self.ref_filtrada = 0.0
            car.set_u(-self.freio_max)
        else:
            self.ref_filtrada += -math.expm1(-dt / cfg["tau_vel"]) * (ref - self.ref_filtrada)
            car.set_u(cfg["kp_vel"] * (self.ref_filtrada - car.v) - cfg["kd_vel"] * car.a)
        car.set_steer(self.direcao)
        return estado, idade, alvo, distancia


def capturar_calibracao(Camera, caminho):
    camera = Camera()
    try:
        quadro = None
        for _ in range(15):
            obtido = camera.get_image(gray=False)
            if obtido is not None:
                quadro = obtido
        if quadro is None:
            raise RuntimeError("A câmera não entregou um quadro.")
        ok, imagem = cv2.imencode(".png", quadro)
        if not ok:
            raise RuntimeError("Falha na codificação da imagem.")
        with caminho.open("xb") as arq:
            arq.write(imagem.tobytes())
        print(f"Imagem {quadro.shape[1]} x {quadro.shape[0]} salva em {caminho}", flush=True)
    finally:
        camera.close()


def previa(Camera, calibracao, config):
    camera = Camera()
    canal = CanalVisao()
    canal.estado = "PREVIA | somente camera"
    try:
        camera.start_stream()
        print("Prévia em http://<IP-do-carro>:8080/stream. Ctrl+C encerra.", flush=True)
        canal.executar(camera, Visao(calibracao, config, usar_memoria=False))
        if canal.erro is not None:
            raise RuntimeError("Falha na visão da prévia.") from canal.erro
    except KeyboardInterrupt:
        pass
    finally:
        canal.stop.set()
        camera.close()


def executar(Car, caracteristicas, parametros, calibracao, config):
    if config["velocidade"] > caracteristicas["VELMAX"]:
        raise ValueError("A velocidade de referência excede o limite do carro.")
    if math.radians(config["limite_direcao"]) > caracteristicas["STEERMAX"]:
        raise ValueError("O limite de esterçamento excede o limite do carro.")
    car = Car({**parametros, "camera": True})
    canal, worker = CanalVisao(), None
    freio_max = caracteristicas["ACCELMAX"]
    try:
        car.start_mission()
        car.cam.start_stream()
        controle = Controle(config, caracteristicas["L"], freio_max)
        canal.pose = Pose(tuple(car.p), float(car.th), time.monotonic())
        worker = threading.Thread(target=canal.executar, name="visao-cones",
                                  args=(car.cam, Visao(calibracao, config)), daemon=True)
        worker.start()
        acompanhar(car, parametros, canal, controle)
    except KeyboardInterrupt:
        print("\nMissão de cones interrompida.", flush=True)
    finally:
        canal.stop.set()
        # Freia antes de esperar pela câmera, que pode estar bloqueada em read().
        car.emergencia = True
        try:
            car.set_u(-freio_max)
            car.set_steer(0.0)
        finally:
            if worker is not None:
                worker.join(timeout=1.0)
                if worker.is_alive():
                    print("Visão bloqueada. Os atuadores serão encerrados.", flush=True)
                    # O encerramento do processo libera a câmera se a leitura ficar presa.
                    car.parameters["camera"] = False
            try:
                if parametros["save"] and hasattr(car, "traj"):
                    car.save()
            finally:
                car.close()


def acompanhar(car, parametros, canal, controle):
    arquivo = None
    try:
        if parametros["save"]:
            caminho = Path(car.logfile) / "cones_geometrico.csv"
            arquivo = caminho.open("w", newline="", encoding="utf-8")
            log = csv.writer(arquivo)
            log.writerow(CAMPOS)
            print(f"Log de cones em {caminho}", flush=True)
        anterior = None
        while car.t < parametros["ts"] and not canal.stop.is_set() and car.step():
            if canal.stop.is_set():
                break
            pose = Pose(tuple(car.p), float(car.th), time.monotonic())
            with canal.lock:
                canal.pose = pose
                resultado = canal.resultado
            estado, idade, alvo, distancia = controle.aplicar(car, pose, resultado, time.monotonic())
            with canal.lock:
                canal.estado = estado
            if estado != anterior:
                print(f"{car.t:6.2f}s | {estado}", flush=True)
                anterior = estado
            if arquivo is not None:
                dados = (resultado.modo, resultado.motivo, resultado.ms, resultado.vermelhos,
                         resultado.verdes, resultado.memoria) if resultado else ("", "", "", 0, 0, 0)
                log.writerow((car.t, car.v, car.vref, estado, *dados[:2], idade, *dados[2:],
                              *(alvo if alvo is not None else ("", "")), math.degrees(car.st), distancia))
        if canal.erro is not None:
            raise RuntimeError("A thread de visão falhou.") from canal.erro
    finally:
        if arquivo is not None:
            arquivo.close()
