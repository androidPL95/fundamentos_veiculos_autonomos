# -*- coding: utf-8 -*-
########################################
# Disciplina: Topicos em Engenharia de Controle e Automacao IV (ENG075):
# Fundamentos de Veiculos Autonomos - 2026/2
# Professores: Armando Alves Neto e Leonardo A. Mozelli
# Cursos: Engenharia de Controle e Automacao
# DELT - Escola de Engenharia
# Universidade Federal de Minas Gerais
########################################
# Versao SIMULADOR do main do carro real:
# mesma logica de ADAS, controle de velocidade e controle lateral por visao,
# mas executada de forma sincrona (sem threads), no formato do main do simulador.
########################################

from collections import deque

from fva_car import Car
import numpy as np
import cv2
import csv
import os
import time
import matplotlib

matplotlib.use("QtAgg")

import matplotlib.pyplot as plt

plt.rcParams["figure.figsize"] = (6, 8)


# ============================================================
# PARAMETROS GERAIS
# ============================================================

parameters = {
    "ts": 60.0,
    "save": True,
    "logfile": "logs/",
    "beep": True,
}

# Formato de cor da imagem do simulador.
# Se a mascara amarela nao detectar a linha, troque para False (BGR).
IMAGEM_RGB = True

SET_VEL = 1.5
historico_d = deque(maxlen=5)
AEB_LATCHED = False

USAR_VISAO_NO_LATERAL = True

SET_Y = -9.0
SET_THETA = np.pi

# Ganhos do controle de velocidade (mesmos do carro real)
KP_VEL = 0.5
KD_VEL = 0.25

# Ganhos da referencia manual
K_STANLEY = 1.5
K_HEADING = 1.0

# Ganhos da visao
K_LATERAL_VISAO = 1.2
K_HEADING_VISAO = 0.6
V_SOFT = 0.5                      # suavizacao em baixa velocidade
K_DAMPING_W = 0.10                # amortecimento pela velocidade angular

SINAL_OFFSET_VISAO = -1.0
SINAL_HEADING_VISAO = -1.0

MAX_STEER = np.deg2rad(5.0)
MAX_STEER_RATE = np.deg2rad(25.0)

AMORTECIMENTO_THETA = 0.15
DISTANCIA_ENTRE_EIXOS = 0.302

LINHA_TOPO = 0.25        # faixa mais alta (fracao da altura): 0.25 -> ~2.6 m a frente
LINHA_BASE = 0.85        # faixa mais baixa: 0.85 -> ~0.4 m a frente (mesma de antes)
MIN_PIXELS = 15          # pixels amarelos minimos na faixa para valer um ponto

PLOT_A_CADA = 3          # atualiza a tela a cada N passos (plotar e lento)
NOMES_ETAPAS = ["1 entrada", "2 amarelo realcado (linear, anti-reflexo)", "3 score amarelo",
                "4 mascara (branco=cor, magenta=recuperada)", "5 identificacao da linha", "6 resultado"]
BLUR = (5, 5)            # kernel do blur gaussiano (impar); (1, 1) desliga

LIMIAR_AMARELO = 60      # score minimo de amarelo (fita ~40-90, piso <= ~26)
HOUGH_VOTOS = 50         # votos minimos do HoughLines (so no metodo "hough")

# CAMADA 2 - recuperacao sob reflexo
LIMIAR_SNR = 5.0         # quanto a fita precisa se destacar do piso (1e9 desliga)
HORIZONTE_FRAC = 0.106   # calibracao da camera: linha do horizonte / altura (51/480)
K_LARGURA = 0.084        # calibracao: largura da fita (px) = K_LARGURA * (linha - horizonte)

MODO_CONTROLE = "cones"  # "linha" ou "cones"

H_CONE_VERMELHO = 0.0    # matiz dos cones vermelhos (OpenCV 0..180)
H_CONE_VERDE = 72.0      # matiz dos cones verdes
LIMIAR_CONE = 40         # score minimo de cor do cone (mesmo para as duas cores)
AREA_MIN_CONE = 15       # pixels minimos para um blob contar como cone

# CAMADA 3 - segurar a ultima reta
TEMPO_SEGURA = 0.5       # s (-1 desliga)

# ============================================================
# DEBUG / LOG
# ============================================================
LOG_DEBUG = True
SALVAR_FRAMES_A_CADA = 20      # salva imagem + mascara a cada N passos (0 = nunca)
PASTA_DEBUG = "logs/debug/"
DEBUG = {}                     # preenchido por vision_func e control_func a cada passo




# ============================================================
# ADAS: FCW + AEB (igual ao carro real)
# ============================================================

def adas(car, setpoint):
    global AEB_LATCHED
    v = car.get_vel()[0]
    d, valid = car.get_distance()

    historico_d.append(d)
    d = sum(historico_d) / len(historico_d)

    if AEB_LATCHED and d <= 3.8:
        return 0.0

    if d > 3.8:
        AEB_LATCHED = False
        return setpoint

    d_stop = (v ** 2) * 3.0
    d_aeb = d_stop + 0.5
    d_fcw = d_aeb + 1.0

    if d <= d_aeb:
        AEB_LATCHED = True
        return 0.0
    elif d <= d_fcw:
        return setpoint  # FCW: so alerta
    else:
        return setpoint


# ============================================================
# CONTROLE: velocidade + lateral
# ============================================================

def control_func(car, ref_filtrada, theta_filtrado, delta_anterior, tau,
                 erro_visao, theta_visao):

    # ---------------- longitudinal ----------------
    setpoint = adas(car, SET_VEL)
    car.vref = setpoint

    ref_filtrada = ref_filtrada + (car.dt / tau) * (setpoint - ref_filtrada)

    v, w = car.get_vel()
    erro = ref_filtrada - v

    a = car.get_accel()
    tot = (KP_VEL * erro) - KD_VEL * a
    tot = np.clip(tot, -1.0, 1.0)

    if setpoint == 0.0 and v <= 0.05:
        car.set_u(-1.0)
    else:
        car.set_u(tot)

    # ---------------- lateral ----------------
    v_lateral = max(float(v), 0.1)

    # Primeiro segundo sem comando lateral
    if car.t < 1.0:
        if not USAR_VISAO_NO_LATERAL:
            theta_filtrado = float(car.th)
        delta_lat = 0.0
    else:
        if USAR_VISAO_NO_LATERAL:
            if erro_visao is None or theta_visao is None:
                # Mantem o ultimo esterçamento se perder a linha
                delta_alvo = delta_anterior
            else:
                erro_lateral = SINAL_OFFSET_VISAO * erro_visao
                erro_heading = SINAL_HEADING_VISAO * theta_visao

                termo_heading = K_HEADING_VISAO * erro_heading
                termo_lateral = np.arctan2(K_LATERAL_VISAO * erro_lateral, max(float(v), 0.0) + V_SOFT)
                termo_amort = K_DAMPING_W * float(w)

                delta_alvo = termo_heading + termo_lateral + termo_amort
                DEBUG["termo_heading"] = float(termo_heading)
                DEBUG["termo_lateral"] = float(termo_lateral)
        else:
            # Referencia manual, filtrando a orientacao
            diferenca = np.arctan2(np.sin(car.th - theta_filtrado),
                                   np.cos(car.th - theta_filtrado))
            theta_filtrado += AMORTECIMENTO_THETA * diferenca

            erro_heading = np.arctan2(np.sin(theta_filtrado - SET_THETA),
                                      np.cos(theta_filtrado - SET_THETA))

            # Aproximacao do eixo dianteiro
            y_dianteiro = float(car.p[1]) + DISTANCIA_ENTRE_EIXOS * np.sin(theta_filtrado - np.pi)
            erro_lateral = y_dianteiro - SET_Y

            termo_heading = K_HEADING * erro_heading
            termo_lateral = np.arctan2(K_STANLEY * erro_lateral, v_lateral)

            delta_alvo = termo_heading + termo_lateral

        DEBUG["delta_bruto"] = float(delta_alvo)          # antes da saturacao
        delta_alvo = float(np.clip(delta_alvo, -MAX_STEER, MAX_STEER))
        DEBUG["saturado"] = int(abs(DEBUG["delta_bruto"]) > MAX_STEER)

        passo_max = MAX_STEER_RATE * car.dt
        delta_lat = float(np.clip(delta_alvo,
                                  delta_anterior - passo_max,
                                  delta_anterior + passo_max))
        DEBUG["rate_limitado"] = int(abs(delta_lat - delta_alvo) > 1e-6)

    DEBUG["delta_cmd"] = float(delta_lat)
    car.set_steer(delta_lat)

    return ref_filtrada, theta_filtrado, delta_lat, setpoint


# ============================================================
# VISAO (igual ao carro real, executada no loop principal)
# ============================================================

LUT_LINEAR = (((np.arange(256) / 255.0) ** 2.2) * 255.0).astype(np.float32)   # desfaz o gama da camera
ESTADO_VISAO = {"modo": None, "a": None, "b": None, "t": -1e9, "rho": deque(maxlen=5), "theta": deque(maxlen=5)}

def _amarelamento_linear(suave):
    """(R+G)/2 - B em luz linear: o reflexo soma branco igual nos 3 canais e nao muda este valor."""
    lin = LUT_LINEAR[suave]
    return (lin[..., 0] + lin[..., 1]) * 0.5 - lin[..., 2]


def _recupera_fita(car, Y, mask, h, w):
    """CAMADA 2: estima onde a fita deveria estar (ultima reta recente, senao ajuste robusto
    da mascara de cor) e, em cada linha, procura perto dessa referencia um pico de amarelamento
    com a largura esperada da fita; marca como fita se destacar LIMIAR_SNR vezes do ruido do piso.
    Retorna (extra, ref)."""
    # ---- referencia: ultima reta recente ou ajuste robusto da mascara de cor ----
    ref = None
    if ESTADO_VISAO["a"] is not None and car.t - ESTADO_VISAO["t"] <= TEMPO_SEGURA:
        ref = (ESTADO_VISAO["a"], ESTADO_VISAO["b"])
    else:
        ys, xs = [], []
        for y in np.linspace(LINHA_TOPO, LINHA_BASE, 6):
            y = int(h * y)
            cols = np.where(mask[max(0, y - 4):y + 5] > 0)[1]
            if len(cols) > MIN_PIXELS:
                ys.append(y); xs.append(float(np.median(cols)))
        if len(ys) >= 3:
            ys = np.asarray(ys, float); xs = np.asarray(xs, float)
            usar = np.ones(len(ys), bool)
            for _ in range(3):
                if usar.sum() < 2:
                    break
                a, b = np.polyfit(ys[usar], xs[usar], 1)
                res = np.abs(xs - (a * ys + b))
                novo = res <= max(15.0, 3.0 * np.median(res[usar]))
                if (novo == usar).all():
                    break
                usar = novo
            if usar.sum() >= 2:
                ref = np.polyfit(ys[usar], xs[usar], 1)

    extra = np.zeros((h, w), np.uint8)
    if ref is None:
        return extra, ref

    # ---- completa a mascara perto da referencia ----
    a, b = ref
    y0 = max(0, int(h * LINHA_TOPO) - 5)
    linhas = np.arange(y0, h)
    k = np.maximum(3, np.round(K_LARGURA * (linhas - HORIZONTE_FRAC * h))).astype(int)   # largura esperada
    Yr = Y[y0:]
    cs = np.concatenate([np.zeros((len(linhas), 1), np.float32), np.cumsum(Yr, axis=1)], axis=1)
    xx = np.arange(w)[None, :]
    lo = np.clip(xx - (k // 2)[:, None], 0, w)
    hi = np.clip(xx + (k - k // 2)[:, None], 0, w)
    sm = (np.take_along_axis(cs, hi, 1) - np.take_along_axis(cs, lo, 1)) / np.maximum(hi - lo, 1)
    fundo = np.median(sm, axis=1, keepdims=True)
    ruido = 1.4826 * np.median(np.abs(sm - fundo), axis=1, keepdims=True) + 1e-3
    z = (sm - fundo) / ruido
    x_ref = a * linhas + b
    janela = np.maximum(15, 2 * k)
    perto = np.abs(xx - x_ref[:, None]) <= janela[:, None]
    zg = np.where(perto, z, -np.inf)
    j = np.argmax(zg, axis=1)
    ok = zg[np.arange(len(linhas)), j] >= LIMIAR_SNR
    ja_tem = (mask[y0:] > 0) & perto                 # linha onde a cor ja achou a fita
    ok &= ~ja_tem.any(axis=1)
    for dx in (-1, 0, 1):                            # marca so o eixo da fita (3 px)
        extra[y0 + np.where(ok)[0], np.clip(j[ok] + dx, 0, w - 1)] = 255
    return extra, ref


def _linha_hough(mask_final, h, w, img_dbg):
    """Logica do backup: Canny -> HoughLines -> media de rho/theta (flip theta>90) -> media dos 5 ultimos.
    Acrescimos: so o piso (abaixo de LINHA_TOPO) e descarta retas a mais de 70 graus da vertical."""
    roi = mask_final.copy()
    roi[:int(h * LINHA_TOPO)] = 0
    bordas = cv2.Canny(roi, 50, 200)
    img_dbg[bordas > 0] = (255, 255, 255)
    linhas = cv2.HoughLines(bordas, 1, np.pi / 180, HOUGH_VOTOS)
    n = 0
    if linhas is not None:
        rt = linhas.reshape(-1, linhas.shape[-1])[:, :2].astype(np.float64)
        rhos, thetas = rt[:, 0].copy(), rt[:, 1].copy()
        flip = thetas > np.pi / 2
        thetas[flip] -= np.pi
        rhos[flip] *= -1
        ok = np.abs(thetas) <= np.radians(70)
        for r_, t_ in zip(rhos[ok], thetas[ok]):
            c_, s_ = np.cos(t_), np.sin(t_)
            x0_, y0_ = c_ * r_, s_ * r_
            cv2.line(img_dbg, (int(x0_ - 1000 * s_), int(y0_ + 1000 * c_)),
                     (int(x0_ + 1000 * s_), int(y0_ - 1000 * c_)), (255, 0, 0), 1)
        n = int(ok.sum())
        if n:
            ESTADO_VISAO["rho"].append(float(np.mean(rhos[ok])))
            ESTADO_VISAO["theta"].append(float(np.mean(thetas[ok])))
    DEBUG["n_pontos"] = n
    DEBUG["xs"] = DEBUG["ys"] = ""
    if n == 0 or len(ESTADO_VISAO["rho"]) == 0:
        return None
    rho_f = float(np.mean(ESTADO_VISAO["rho"]))
    theta_f = float(np.mean(ESTADO_VISAO["theta"]))
    c_, s_ = np.cos(theta_f), np.sin(theta_f)
    if abs(c_) < 1e-3:
        return None
    return -s_ / c_, rho_f / c_                  # x = a*y + b

def _tira_manchas(mask, h):
    """Apaga, linha a linha, trechos amarelos mais largos que a fita pode ser naquela linha.
    Largura esperada da fita = K_LARGURA*(linha - horizonte) (calibracao da camera); aceita ate 2.5x
    (fita inclinada ate ~66 graus da vertical). Mancha e larga; fita e estreita."""
    limpa = mask.copy()
    for y in range(int(h * LINHA_TOPO) - 5, h):
        lim = 2.5 * max(3.0, K_LARGURA * (y - HORIZONTE_FRAC * h))
        lin = np.concatenate([[0], (mask[y] > 0).astype(np.int8), [0]])
        d = np.diff(lin)
        ini, fim = np.where(d == 1)[0], np.where(d == -1)[0]
        for a_, b_ in zip(ini, fim):
            if b_ - a_ > lim:
                limpa[y, a_:b_] = 0
    return limpa

def linha_func(rgb, car, etapas):
    """MODO LINHA: tudo da linha amarela. Devolve a reta (a, b) de x = a*y + b, ou None."""
    h, w, _ = rgb.shape
    suave = cv2.GaussianBlur(rgb, BLUR, 0)
    hsv = cv2.cvtColor(suave, cv2.COLOR_RGB2HSV)
    H = hsv[..., 0].astype(np.float32)
    S = hsv[..., 1].astype(np.float32)

    # score de amarelo = saturacao x proximidade da matiz da fita (H=26, cai a zero em +-12)
    score = np.clip(1.0 - np.abs(H - 26.0) / 12.0, 0.0, 1.0) * S
    score[hsv[..., 2] < 40] = 0                         # pixels escuros: matiz nao confiavel
    score = np.clip(score, 0, 255).astype(np.uint8)
    etapas["3 score amarelo"] = np.clip(score.astype(np.float32) * 3, 0, 255).astype(np.uint8)  # x3 so para ver
    mask = ((score >= LIMIAR_AMARELO) * 255).astype(np.uint8)
    mask = _tira_manchas(mask, h)

    # ---- CAMADA 2: completa a mascara onde o reflexo apagou a fita ----
    Y = _amarelamento_linear(suave)
    etapas["2 amarelo realcado (linear, anti-reflexo)"] = np.clip(Y * 6, 0, 255).astype(np.uint8)
    extra, ref = _recupera_fita(car, Y, mask, h, w)
    mask_final = cv2.bitwise_or(mask, extra)
    m_rgb = cv2.cvtColor(mask, cv2.COLOR_GRAY2RGB)
    m_rgb[extra > 0] = (255, 0, 255)
    etapas["4 mascara (branco=cor, magenta=recuperada)"] = m_rgb
    DEBUG["recuperados_px"] = int(np.count_nonzero(extra))
    DEBUG["mask"] = mask_final

    # ---- identificacao da linha ----
    dbg = cv2.cvtColor(mask_final, cv2.COLOR_GRAY2RGB) // 2
    reta = _linha_hough(mask_final, h, w, dbg)
    etapas["5 identificacao da linha"] = dbg
    return reta


def cones_func(rgb, car, etapas):
    """MODO CONES: cada cone pesa pela sua distancia (linha da base na imagem),
    nao pela quantidade de pixels: e = (g - r) / (g + r)."""
    h, w, _ = rgb.shape
    hsv = cv2.cvtColor(cv2.GaussianBlur(rgb, BLUR, 0), cv2.COLOR_RGB2HSV)
    H = hsv[..., 0].astype(np.float32)
    S = hsv[..., 1].astype(np.float32)
    y_hor = HORIZONTE_FRAC * h
    vis_img = rgb.copy()
    mask_total = np.zeros((h, w), np.uint8)
    soma = {}
    for nome, h_alvo, cor in (("r", H_CONE_VERMELHO, (255, 0, 0)), ("g", H_CONE_VERDE, (0, 255, 0))):
        dh = np.abs(H - h_alvo)
        dh = np.minimum(dh, 180.0 - dh)                  # vermelho fica em 0 e em 180
        score = np.clip(1.0 - dh / 20.0, 0.0, 1.0) * S
        score[hsv[..., 2] < 40] = 0
        mask = ((score >= LIMIAR_CONE) * 255).astype(np.uint8)
        mask[:int(y_hor) + 1] = 0                        # so o chao
        mask_total |= mask
        n, _, est, _ = cv2.connectedComponentsWithStats(mask)
        soma[nome] = 0.0
        for x, y, bw, bh, area in est[1:]:
            if area < AREA_MIN_CONE:
                continue
            y_base = y + bh - 1                          # onde o cone toca o chao
            soma[nome] += (y_base - y_hor)         # cone perto pesa mais (~1/distancia^2)
            cv2.circle(vis_img, (int(x + bw / 2), int(y_base)), 5, cor, -1)

    DEBUG["mask"] = mask_total
    DEBUG["n_pontos"] = int(soma["r"] > 0) + int(soma["g"] > 0)
    total = soma["r"] + soma["g"]
    if total == 0:
        return vis_img, None
    e = (soma["g"] - soma["r"]) / total                  # carro real: (soma["r"] - soma["g"])
    cv2.putText(vis_img, f"r={soma['r']:.0f} g={soma['g']:.0f} e={e:+.2f}", (8, h - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
    return vis_img, e

def vision_func(car, numero_tracos=6):
    image = car.get_image(gray=False)
    if image is None:
        return None, None, None

    rgb = image if IMAGEM_RGB else cv2.cvtColor(image, cv2.COLOR_BGR2RGB)   # trabalha sempre em RGB
    h, w, _ = rgb.shape
    centro_x = w / 2.0
    etapas = {"1 entrada": rgb}

    modo = MODO_CONTROLE                       # le uma vez por quadro
    if modo != ESTADO_VISAO["modo"]:           # trocou de modo: esquece a reta antiga
        ESTADO_VISAO.update(modo=modo, a=None, b=None, t=-1e9)
        ESTADO_VISAO["rho"].clear(); ESTADO_VISAO["theta"].clear()

    match modo:
        case "linha":
            reta = linha_func(rgb, car, etapas)
        case "cones":
            vis_img, e = cones_func(rgb, car, etapas)
            return _fim(vis_img, etapas, e, None if e is None else 0.0)   # theta = 0: so o erro lateral
        case _:
            raise ValueError(f"MODO_CONTROLE invalido: {modo}")

    # ---- CAMADA 3: segura a ultima reta ----
    DEBUG["segurando"] = 0
    if reta is not None:
        ESTADO_VISAO["a"], ESTADO_VISAO["b"] = reta
        ESTADO_VISAO["t"] = car.t
    elif ESTADO_VISAO["a"] is not None and car.t - ESTADO_VISAO["t"] <= TEMPO_SEGURA:
        reta = (ESTADO_VISAO["a"], ESTADO_VISAO["b"])
        DEBUG["segurando"] = 1

    DEBUG["img_w"] = w
    vis_img = rgb.copy()
    cv2.line(vis_img, (int(centro_x), int(h * LINHA_TOPO)), (int(centro_x), h - 1), (255, 255, 255), 1)

    if reta is None:
        return _fim(vis_img, etapas, None, None)

    a, b = reta
    y_perto, y_longe = int(h * LINHA_BASE), int(h * LINHA_TOPO)
    x_perto, x_longe = a * y_perto + b, a * y_longe + b
    erro_lat_norm = (centro_x - x_perto) / (w / 2.0)
    theta_e = float(np.arctan2(-(x_longe - x_perto), y_perto - y_longe))
    cor = (255, 140, 0) if DEBUG["segurando"] else (0, 255, 255)        # laranja = reta segurada
    cv2.line(vis_img, (int(round(x_perto)), y_perto), (int(round(x_longe)), y_longe), cor, 2)
    cv2.circle(vis_img, (int(round(x_perto)), y_perto), 6, (255, 0, 0), -1)
    cv2.putText(vis_img, f"e={erro_lat_norm:+.2f} th={np.degrees(theta_e):+.1f}", (8, h - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
    DEBUG["x_perto"] = float(x_perto)
    DEBUG["x_longe"] = float(x_longe)
    return _fim(vis_img, etapas, erro_lat_norm, theta_e)

def _fim(vis_img, etapas, e, th):
    etapas["6 resultado"] = vis_img
    DEBUG["etapas"] = etapas
    saida = vis_img if IMAGEM_RGB else cv2.cvtColor(vis_img, cv2.COLOR_RGB2BGR)
    return saida, e, th
# ============================================================
# PROGRAMA PRINCIPAL
# ============================================================

if __name__ == "__main__":

    plt.ion()

    # cria comunicacao com o carrinho
    car = Car(parameters)
    arq_log = None

    try:
        # comeca a simulacao
        car.start_mission()

        ref_filtrada = 0.0
        theta_filtrado = 0.0
        delta_anterior = 0.0
        tau = 0.45

        hist_t, hist_v, hist_vref, hist_delta = [], [], [], []
                # ---------------- tela: etapas da visao + graficos ----------------
        fig, axs = plt.subplots(3, 3, figsize=(13, 9), num=1)
        arts = {}
        ax_v, ax_d, ax_e = axs.flat[6], axs.flat[7], axs.flat[8]
        hist_e, hist_th = [], []

        # ---------------- log CSV ----------------
        CAMPOS_LOG = [
            "t", "v", "x_real", "y_real", "th_real", "e_real_m", "th_real_err",
            "steer_real", "erro_visao", "theta_visao_deg",
            "termo_heading_deg", "termo_lateral_deg", "delta_bruto_deg",
            "delta_cmd_deg", "saturado", "rate_limitado",
            "n_pontos", "contagem_min", "largura_max_px", "residuo_fit_px",
            "x_perto", "x_longe", "img_w", "xs", "ys",
        ]
        arq_log = None
        escritor = None
        passo = 0
        if LOG_DEBUG:
            os.makedirs(PASTA_DEBUG, exist_ok=True)
            nome_log = os.path.join(PASTA_DEBUG, time.strftime("lateral_%Y%m%d_%H%M%S.csv"))
            arq_log = open(nome_log, "w", newline="")
            escritor = csv.DictWriter(arq_log, fieldnames=CAMPOS_LOG)
            escritor.writeheader()
            print(f"Log de debug: {nome_log}")

        # main loop
        while car.t <= parameters["ts"]:

            # le sensores
            car.step()

            # funcao de visao (antes do controle, para usar a imagem atual)
            frame, erro_visao, theta_visao = vision_func(car)

            # funcao de controle
            ref_filtrada, theta_filtrado, delta_anterior, setpoint = control_func(
                car, ref_filtrada, theta_filtrado, delta_anterior, tau,
                erro_visao, theta_visao,
            )

            v_atual = car.get_vel()[0]
            hist_t.append(car.t)
            hist_v.append(v_atual)
            hist_vref.append(setpoint)
            hist_delta.append(np.rad2deg(delta_anterior))

            # ---------------- grava log ----------------
            if LOG_DEBUG:
                def _num(x):
                    try:
                        return float(x)
                    except (TypeError, ValueError):
                        return float("nan")

                # Verdade do simulador (se existir): pose real do carro
                try:
                    x_real, y_real = float(car.p[0]), float(car.p[1])
                except Exception:
                    x_real = y_real = float("nan")
                th_real = _num(getattr(car, "th", None))
                th_err = np.arctan2(np.sin(th_real - SET_THETA), np.cos(th_real - SET_THETA))

                escritor.writerow({
                    "t": f"{car.t:.3f}",
                    "v": f"{v_atual:.3f}",
                    "x_real": f"{x_real:.4f}",
                    "y_real": f"{y_real:.4f}",
                    "th_real": f"{th_real:.4f}",
                    "e_real_m": f"{y_real - SET_Y:.4f}",       # so vale se a linha for y = SET_Y
                    "th_real_err": f"{np.rad2deg(th_err):.3f}",
                    "steer_real": f"{np.rad2deg(_num(getattr(car, 'st', None))):.3f}",
                    "erro_visao": f"{_num(erro_visao):.4f}",
                    "theta_visao_deg": f"{np.rad2deg(_num(theta_visao)):.3f}",
                    "termo_heading_deg": f"{np.rad2deg(_num(DEBUG.get('termo_heading'))):.3f}",
                    "termo_lateral_deg": f"{np.rad2deg(_num(DEBUG.get('termo_lateral'))):.3f}",
                    "delta_bruto_deg": f"{np.rad2deg(_num(DEBUG.get('delta_bruto'))):.3f}",
                    "delta_cmd_deg": f"{np.rad2deg(_num(DEBUG.get('delta_cmd'))):.3f}",
                    "saturado": DEBUG.get("saturado", 0),
                    "rate_limitado": DEBUG.get("rate_limitado", 0),
                    "n_pontos": DEBUG.get("n_pontos", 0),
                    "contagem_min": DEBUG.get("contagem_min", 0),
                    "largura_max_px": DEBUG.get("largura_max_px", -1),
                    "residuo_fit_px": f"{_num(DEBUG.get('residuo_fit_px')):.2f}",
                    "x_perto": f"{_num(DEBUG.get('x_perto')):.1f}",
                    "x_longe": f"{_num(DEBUG.get('x_longe')):.1f}",
                    "img_w": DEBUG.get("img_w", -1),
                    "xs": DEBUG.get("xs", ""),
                    "ys": DEBUG.get("ys", ""),
                })

                # Salva imagem anotada + mascara lado a lado
                if (SALVAR_FRAMES_A_CADA > 0 and passo % SALVAR_FRAMES_A_CADA == 0
                        and frame is not None and "mask" in DEBUG):
                    img_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR) if IMAGEM_RGB else frame
                    mask_bgr = cv2.cvtColor(DEBUG["mask"], cv2.COLOR_GRAY2BGR)
                    cv2.imwrite(os.path.join(PASTA_DEBUG, f"frame_{passo:05d}_t{car.t:06.2f}.png"),
                                np.hstack([img_bgr, mask_bgr]))

                # Limpa os campos do passo para nao repetir valores antigos
                for k in ("termo_heading", "termo_lateral", "delta_bruto",
                          "x_perto", "x_longe", "residuo_fit_px"):
                    DEBUG.pop(k, None)
            passo += 1

                        ########################################
            # plota as etapas da visao + graficos
            hist_e.append(100 * erro_visao if erro_visao is not None else np.nan)
            hist_th.append(np.degrees(theta_visao) if theta_visao is not None else np.nan)

            if passo % PLOT_A_CADA == 0:
                etapas = DEBUG.get("etapas", {})
                for ax, nome in zip(axs.flat[:6], NOMES_ETAPAS):
                    im = etapas.get(nome)
                    if im is None:
                        continue
                    if im.ndim == 2:
                        im = np.dstack([im, im, im])
                    if nome not in arts:
                        ax.axis("off")
                        ax.set_title(nome, fontsize=9)
                        arts[nome] = ax.imshow(im)
                    else:
                        arts[nome].set_data(im)

                linha = "OK" if erro_visao is not None else "PERDIDA"
                fig.suptitle(f"t = {car.t:.2f} s   v = {v_atual:.2f} m/s   u = {car.u:.2f}   "
                             f"linha: {linha}{' (SEGURANDO)' if DEBUG.get('segurando') else ''}   "
                             f"retas: {DEBUG.get('n_pontos', 0)}   recuperados: {DEBUG.get('recuperados_px', 0)} px",
                             fontsize=10)
                ax_v.cla()
                ax_v.plot(hist_t, hist_v, label="v")
                ax_v.plot(hist_t, hist_vref, "--", color="green", label="vref (ADAS)")
                ax_v.set_title("velocidade [m/s]", fontsize=9)
                ax_v.legend(fontsize=7)

                ax_d.cla()
                ax_d.plot(hist_t, hist_delta, color="red")
                ax_d.axhline(0.0, color="green", linestyle="--")
                ax_d.set_title("estercamento [graus]", fontsize=9)

                ax_e.cla()
                ax_e.plot(hist_t, hist_e, label="e lateral [% meia largura]")
                ax_e.plot(hist_t, hist_th, label="theta [graus]")
                ax_e.axhline(0.0, color="gray", linestyle="--")
                ax_e.set_title("erros da visao", fontsize=9)
                ax_e.legend(fontsize=7)
                ax_e.set_xlabel("t [s]")

                fig.tight_layout()
                plt.pause(0.001)

    except KeyboardInterrupt:
        print("\nMissao interrompida pelo usuario.")

    finally:
        if LOG_DEBUG and arq_log is not None:
            arq_log.close()
        try:
            if parameters["save"]:
                car.save()
        finally:
            car.close()

    print("Terminou...")