# Modo geométrico de cones

## Enviar e executar pela GUI

1. Se o carro ainda estiver com uma versão anterior das bibliotecas, use
   **Adicionar pasta** na aba de envio e selecione `veiculo_real`. A GUI envia
   o conteúdo para a pasta remota configurada, por padrão
   `/home/alunos/Desktop/fva`.
2. Nas atualizações seguintes, selecione `veiculo_real/main.py` na GUI desta
   versão. Ela inclui os módulos `visao_cones_geometrico.py` e
   `modo_cones_geometrico.py`, ao lado do `main.py` no carro. Se enviar por
   outro meio, copie os três arquivos para a mesma pasta.
3. Use o campo **Comandos** da GUI. A chamada padrão `python3 -u main.py`
   executa o `MODO_CONTROLE` definido no topo do arquivo, inicialmente
   `cones`. `--modo` seleciona o modo para uma única execução.

Os comandos também funcionam no terminal da Raspberry na pasta onde
`main.py` foi instalado.

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
   `calibracao_cones.json` ao lado do `main.py` instalado com estes campos.
   Envie esse JSON para o carro em que as marcas foram medidas.

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

Use `--calibracao caminho.json` para selecionar outro arquivo. O padrão usa
verde à esquerda e vermelho à direita. Confira o sinal do esterçamento com
as rodas suspensas antes do teste no piso.

Nesse modo, as duas chaves precisam estar em AUTO para liberar a tração.
O controle freia com quadro atrasado, sem alvo válido ou com leitura inválida do
ultrassom. Uma só borda reduz a referência de velocidade. O CSV
`cones_geometrico.csv`, na pasta do ensaio, registra o estado do controle,
a idade do quadro, o alvo, o esterçamento e o motivo da percepção.

## Ajustar para cada ensaio

Edite o bloco `PARAMETROS_CONES` no topo do `main.py` para mudar os padrões
do modo geométrico. A duração da missão fica em `parameters["ts"]`. Para um
ensaio, use `--ajuste nome=valor` no campo **Comandos** da GUI, uma vez por
parâmetro. A opção `--mostrar-ajustes` imprime os valores efetivos e encerra
antes de abrir a câmera ou o carro.

```bash
python3 -u main.py --modo cones_geometrico --ajuste velocidade=0.20 --ajuste antecipacao_m=0.8 --mostrar-ajustes
python3 -u main.py --modo cones_geometrico --ajuste velocidade=0.20 --ajuste antecipacao_m=0.8
```

`--velocidade-cones` continua disponível e tem precedência se as duas formas
de definir a velocidade forem usadas juntas.

| Ajuste | O que muda |
| --- | --- |
| `velocidade` e `fator_uma_borda` | Referência de tração com duas bordas e fração usada com uma |
| `antecipacao_m` | Distância nominal do ponto que o carro tenta alcançar no caminho |
| `ganho_direcao`, `limite_direcao` e `taxa_direcao` | Intensidade, limite e rapidez do esterçamento |
| `kp_vel`, `kd_vel` e `tau_vel` | Resposta e suavização do controle de velocidade |
| `esquerda`, `alcance_m` e limiares de imagem | Convenção das bordas e cones aceitos pela percepção |
| `memoria_s` e `idade_max_quadro_s` | Validade de um cone guardado e idade máxima de um quadro usado para comandar |

## Onde conferir a lógica

- `main.py` recebe os parâmetros e seleciona o modo de missão.
- `visao_cones_geometrico.py` separa os cones por cor, projeta suas bases
  no piso, atualiza a memória e monta o caminho pelo centro do corredor.
- `modo_cones_geometrico.py` carrega a homografia e executa a missão. A thread
  de visão publica o alvo com a hora da captura. A thread principal usa o
  Pure Pursuit para calcular o esterçamento e é a única que comanda o carro.
