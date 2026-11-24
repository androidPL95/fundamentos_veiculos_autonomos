# Modo geométrico de cones

Rode os comandos a partir da pasta `veiculo_real` na Raspberry. O argumento
`--modo` seleciona a condução ao iniciar. O comando sem argumentos continua
usando o valor de `MODO_CONTROLE` em `main.py`.

```bash
python3 -u main.py --modo linha
python3 -u main.py --modo cones
```

## Calibrar a câmera

1. Mantenha o carro parado, com a câmera na posição de rodagem. Marque no piso
   pelo menos quatro pontos distribuídos em ambas as direções, cobrindo a região
   onde o carro vai procurar cones. Meça `x` para frente e `y` para a esquerda
   a partir da projeção do centro do eixo traseiro no chão, em metros.
2. Capture um PNG na resolução nativa, com as marcas visíveis.

   ```bash
   python3 -u main.py --modo cones_geometrico --capturar-calibracao piso.png
   ```

3. Leia as coordenadas em pixels no PNG original. `u` cresce para a direita e
   `v` para baixo, a partir do canto superior esquerdo. Crie
   `calibracao_cones.json` na pasta `veiculo_real` com estes campos.

   | Campo | Valor |
   | --- | --- |
   | `resolucao` | Par `[largura, altura]` do PNG em pixels |
   | `largura_m` | Distância medida entre as bordas do corredor, em metros |
   | `pontos` | Lista com uma entrada numérica `[u, v, x, y]` por marca |

O programa calcula a homografia a partir dessas correspondências. O alcance
fica limitado pela marca de maior `x` e pelo ajuste `alcance_m`. Recalibre ao
mudar a posição da câmera ou a resolução.

## Conferir a projeção e dirigir

```bash
python3 -u main.py --modo cones_geometrico --somente-visao
```

A prévia abre só a webcam e publica o vídeo em `http://<IP-do-carro>:8080/stream`.
Use a câmera ao vivo da GUI para ver as caixas dos cones aceitos, o caminho e
o alvo. Encerre a prévia antes de abrir o teste de condução.

```bash
python3 -u main.py --modo cones_geometrico --velocidade-cones 0.25
```

Use `--calibracao caminho.json` para selecionar outro arquivo. Velocidade,
ganhos, antecipação e validade da memória ficam em `PARAMETROS` em
`modo_cones_geometrico.py`. O padrão usa verde à esquerda e vermelho à direita.
Confira o sinal do esterçamento com as rodas suspensas antes do teste no piso.

Nesse modo, as duas chaves precisam estar em AUTO para liberar a tração.
O controle freia com quadro atrasado, sem alvo válido ou com leitura inválida do
ultrassom. Uma só borda reduz a referência de velocidade. O CSV
`cones_geometrico.csv`, na pasta do ensaio, registra o estado do controle,
a idade do quadro, o alvo, o esterçamento e o motivo da percepção.
