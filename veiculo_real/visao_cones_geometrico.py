"""Visão clássica e planejamento local por cones para o veículo real FVA."""

from collections import Counter
from dataclasses import dataclass

import cv2
import numpy as np


# Matiz no OpenCV vai de 0 a 179. O vermelho cruza o início da escala.
FAIXAS_MATIZ = {
    "vermelho": ((0, 12), (168, 179)),
    "verde": ((45, 96),),
}
FECHAMENTO = cv2.getStructuringElement(cv2.MORPH_CROSS, (3, 3))
PONTOS_VAZIOS = np.empty((0, 2))


def projetar(matriz, pontos):
    pontos = np.asarray(pontos, dtype=np.float64).reshape(-1, 1, 2)
    if len(pontos) == 0:
        return PONTOS_VAZIOS
    return cv2.perspectiveTransform(pontos, matriz).reshape(-1, 2)


@dataclass
class ProjecaoSolo:
    imagem: np.ndarray
    solo: np.ndarray
    origem: np.ndarray
    eixos: np.ndarray

    def em_frente_da_camera(self, pontos):
        return pontos @ self.imagem[2, :2] + self.imagem[2, 2] > 1e-9


@dataclass
class Cone:
    cor: str
    caixa: tuple
    base: tuple
    pos: np.ndarray


def detectar(rgb, projecao, alcance, *, saturacao_min=65, brilho_min=24, area_min=12, perto=0.25):
    hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
    altura, largura = hsv.shape[:2]
    candidatos, mascaras = [], {}
    for cor, faixas in FAIXAS_MATIZ.items():
        mascara = np.zeros((altura, largura), np.uint8)
        for h_min, h_max in faixas:
            minimo = (h_min, saturacao_min, brilho_min)
            maximo = (h_max, 255, 255)
            mascara |= cv2.inRange(hsv, minimo, maximo)
        mascara = cv2.morphologyEx(mascara, cv2.MORPH_CLOSE, FECHAMENTO)
        mascaras[cor] = mascara
        _, rotulos, estatisticas, _ = cv2.connectedComponentsWithStats(mascara, connectivity=8)
        for rotulo, (x, y, w, h, area) in enumerate(estatisticas[1:], start=1):
            if area < area_min or h < 4 or w < 3 or not 0.36 <= w / h <= 1.9:
                continue
            if x == 0 or x + w >= largura or y + h >= altura:
                continue
            ultimas = max(2, min(4, h // 5))
            _, colunas = np.nonzero(rotulos[y + h - ultimas:y + h, x:x + w] == rotulo)
            base = (float(x + np.median(colunas)), int(y + h - 1))
            candidatos.append((cor, (int(x), int(y), int(w), int(h)), base))

    pontos = projetar(projecao.solo, [base for _, _, base in candidatos])
    visiveis = projecao.em_frente_da_camera(pontos)
    cones = []
    for (cor, caixa, base), pos, visivel in zip(candidatos, pontos, visiveis):
        x, y = pos
        if visivel and perto <= x <= alcance and abs(y) <= 4.5 and 18 <= caixa[3] * x <= 225:
            cones.append(Cone(cor, caixa, base, pos))
    return cones, mascaras


@dataclass
class Rastro:
    cor: str
    mundo: np.ndarray
    visto_em: float


@dataclass
class ConeLocal:
    cor: str
    pos: np.ndarray
    atual: bool


@dataclass
class Plano:
    modo: str
    motivo: str
    caminho: np.ndarray
    alvo: tuple | None
    apoio: np.ndarray


class MemoriaCones:
    def __init__(self, segundos=7.0, raio_associacao=0.65, peso_medicao=0.45):
        self.segundos = segundos
        self.raio_associacao = raio_associacao
        self.peso_medicao = peso_medicao
        self.rastros = []

    def atualizar(self, cones, projecao, tempo, alcance):
        origem, eixos = projecao.origem, projecao.eixos
        self.rastros = [
            r for r in self.rastros
            if tempo - r.visto_em <= self.segundos and np.linalg.norm(r.mundo - origem) <= alcance + 3
        ]
        usados = set()
        for cone in cones:
            mundo = origem + eixos @ cone.pos
            dist, i = min(
                ((np.linalg.norm(r.mundo - mundo), i) for i, r in enumerate(self.rastros)
                 if r.cor == cone.cor and i not in usados),
                default=(np.inf, -1),
            )
            if dist < self.raio_associacao:
                rastro = self.rastros[i]
                rastro.mundo = (1 - self.peso_medicao) * rastro.mundo + self.peso_medicao * mundo
                rastro.visto_em = tempo
            else:
                i = len(self.rastros)
                self.rastros.append(Rastro(cone.cor, mundo, tempo))
            usados.add(i)

        locais = []
        for rastro in self.rastros:
            pos = eixos.T @ (rastro.mundo - origem)
            if -3 < pos[0] < alcance + 0.5 and abs(pos[1]) < 7:
                locais.append(ConeLocal(rastro.cor, pos, rastro.visto_em == tempo))
        return locais


def triangulos(pontos):
    """Delaunay dos pontos locais com o OpenCV já usado pelo simulador."""
    pontos = np.asarray(pontos, np.float32)
    minimo = np.floor(pontos.min(axis=0)).astype(int) - 1
    tamanho = np.ceil(pontos.max(axis=0)).astype(int) - minimo + 2
    subdiv = cv2.Subdiv2D(tuple(map(int, (*minimo, *tamanho))))
    indices = {}
    for i, ponto in enumerate(pontos):
        chave = tuple(map(float, ponto))
        if chave not in indices:
            subdiv.insert(chave)
            indices[chave] = i
    for triangulo in subdiv.getTriangleList():
        vertices = [indices.get(tuple(ponto)) for ponto in triangulo.reshape(3, 2)]
        if None not in vertices:
            yield vertices


def caminho_duas_bordas(cones, cor_esquerda, largura, antecipacao):
    if len(cones) < 3 or len({c.cor for c in cones}) < 2:
        return None
    pontos = np.array([c.pos for c in cones])
    esquerda = [c.cor == cor_esquerda for c in cones]
    portoes, ligacoes = {}, {}
    for a, b, c in triangulos(pontos):
        travessias = []
        for i, j in ((a, b), (a, c), (b, c)):
            if esquerda[i] == esquerda[j]:
                continue
            l, r = (i, j) if esquerda[i] else (j, i)
            lado = pontos[l] - pontos[r]
            medida = np.linalg.norm(lado)
            if not 0.54 * largura <= medida <= 1.34 * largura:
                continue
            chave = (l, r)
            centro = (pontos[l] + pontos[r]) * 0.5
            tangente = np.array([lado[1], -lado[0]]) / medida
            portoes[chave] = (centro, tangente, medida)
            ligacoes.setdefault(chave, set())
            travessias.append(chave)
        if len(travessias) == 2:
            primeiro, segundo = travessias
            ligacoes[primeiro].add(segundo)
            ligacoes[segundo].add(primeiro)

    adiante = {}
    for chave, vizinhos in ligacoes.items():
        centro, direcao, _ = portoes[chave]
        adiante[chave] = []
        for vizinho in sorted(vizinhos):
            destino, rumo, _ = portoes[vizinho]
            passo = destino - centro
            d = np.linalg.norm(passo)
            if 0.12 < d < 3.1 and min(passo @ direcao, passo @ rumo) > 0.30 * d:
                adiante[chave].append(vizinho)

    opcoes = []
    for chave, (centro, direcao, medida) in portoes.items():
        x, y = centro
        if not (-0.8 <= x <= 8.4 and abs(y) <= 2.7 and direcao[0] > 0.42):
            continue
        sequencia = [chave]
        while len(sequencia) < 14:
            seguintes = [v for v in adiante[sequencia[-1]] if v not in sequencia]
            if not seguintes:
                break
            sequencia.append(min(seguintes, key=lambda v: abs(portoes[v][2] - medida)))
        caminho = np.array([portoes[v][0] for v in sequencia])
        extensao = np.linalg.norm(np.diff(caminho, axis=0), axis=1).sum()
        if extensao > 0.5:
            custo = abs(x) + 1.15 * abs(y) + 1.8 * (1 - direcao[0]) - 0.23 * min(extensao, 6)
            opcoes.append((custo, sequencia, caminho))
    if not opcoes:
        return None
    _, sequencia, caminho = min(opcoes, key=lambda item: item[0])
    alvo = alvo_no_caminho(caminho, antecipacao - np.linalg.norm(caminho[0]))
    if alvo[0] < 0.45:
        return None
    return Plano("duas_bordas", "Portoes vermelho/verde", caminho, alvo, pontos[sequencia])


def alvo_no_caminho(caminho, distancia):
    passos = np.linalg.norm(np.diff(caminho, axis=0), axis=1)
    estacoes = np.r_[0.0, np.cumsum(passos)]
    return tuple(float(np.interp(distancia, estacoes, eixo)) for eixo in caminho.T)


def caminho_uma_borda(cones, contagem_atual, cor_esquerda, largura, antecipacao):
    opcoes = []
    for cor, quantidade in contagem_atual.items():
        # Exige observações atuais dessa borda. A memória completa o segmento.
        if quantidade < 2:
            continue
        interior = -1 if cor == cor_esquerda else 1
        pontos = np.array([c.pos for c in cones if c.cor == cor])
        if len(pontos) < 2:
            continue
        vetores = pontos[None, :, :] - pontos[:, None, :]
        distancias = np.linalg.norm(vetores, axis=2)
        for i, (x, y) in enumerate(pontos):
            if not (-2.9 < x < 6.5 and -4.7 < interior * y < -0.25):
                continue
            for j in np.flatnonzero((distancias[i] >= 0.65) & (distancias[i] <= 3.65)):
                primeiro_rumo = vetores[i, j] / distancias[i, j]
                if primeiro_rumo[0] < 0.43:
                    continue
                sequencia = encadear_borda(i, j, vetores, distancias)
                borda = pontos[sequencia]
                segmentos = np.diff(borda, axis=0)
                segmentos /= np.linalg.norm(segmentos, axis=1, keepdims=True)
                rumos = np.vstack((segmentos[0], segmentos[:-1] + segmentos[1:], segmentos[-1]))
                rumos /= np.linalg.norm(rumos, axis=1, keepdims=True)
                normais = np.column_stack((-rumos[:, 1], rumos[:, 0]))
                centro = borda + interior * largura * 0.5 * normais
                extensao = np.linalg.norm(np.diff(centro, axis=0), axis=1).sum()
                if extensao < 1.3:
                    continue
                alvo = alvo_no_caminho(centro, antecipacao - centro[0, 0])
                if 0.5 < alvo[0] < 7.5 and abs(alvo[1]) <= 4.5:
                    cx, cy = centro[0]
                    custo = abs(cx) + 0.9 * abs(cy) + 1.4 * (1 - primeiro_rumo[0]) - 0.25 * min(7, extensao)
                    opcoes.append((custo, Plano("uma_borda", f"Borda {cor}", centro, alvo, borda)))
    if not opcoes:
        return None
    return min(opcoes, key=lambda item: item[0])[1]


def encadear_borda(i, j, vetores, distancias):
    sequencia = [i, j]
    while len(sequencia) < 8:
        anterior, atual = sequencia[-2:]
        rumo = vetores[anterior, atual] / distancias[anterior, atual]
        vizinhos = np.flatnonzero((distancias[atual] >= 0.65) & (distancias[atual] <= 3.65))
        opcoes = []
        for proximo in vizinhos:
            if proximo in sequencia:
                continue
            d = distancias[atual, proximo]
            alinhamento = vetores[atual, proximo] @ rumo / d
            if alinhamento > 0.53:
                opcoes.append((d + 1.1 * (1 - alinhamento), proximo))
        if not opcoes:
            break
        sequencia.append(min(opcoes)[1])
    return sequencia


def planejar(cones, memoria, cor_esquerda, largura, antecipacao):
    contagem = Counter(c.cor for c in cones)
    if not cones:
        return Plano("sem_alvo", "Nenhum cone aceito neste quadro", PONTOS_VAZIOS, None, PONTOS_VAZIOS)
    plano = caminho_duas_bordas(memoria, cor_esquerda, largura, antecipacao)
    if plano is None:
        plano = caminho_uma_borda(memoria, contagem, cor_esquerda, largura, antecipacao)
    if plano is not None:
        return plano
    motivo = "Menos de 2 cones em cada borda" if max(contagem.values()) < 2 else "Geometria sem corredor valido"
    return Plano("sem_alvo", motivo, PONTOS_VAZIOS, None, PONTOS_VAZIOS)
