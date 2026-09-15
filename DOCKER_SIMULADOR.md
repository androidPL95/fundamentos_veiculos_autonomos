# Executar o simulador com Docker

Este guia usa Docker para isolar as dependências Python do diretório
`simulador/`. O CoppeliaSim continua sendo executado no computador host:
o contêiner executa o cliente Python, conecta-se ao servidor ZeroMQ do
CoppeliaSim e abre a janela de telemetria na tela do host.

## Pré-requisitos

- Linux com Docker Engine e o plugin Docker Compose instalados;
- CoppeliaSim instalado no host, com suporte à ZeroMQ Remote API;
- uma sessão gráfica X11 ativa (por exemplo, GNOME em Xorg ou XWayland).

O arquivo `docker-compose.yml` usa `network_mode: host`, pois a aplicação
Python atual conecta-se a `localhost`. Essa configuração é própria para
Docker no Linux; para Docker Desktop em macOS ou Windows seria preciso adaptar
o endereço do cliente e a configuração de rede/X11.

## Preparar o CoppeliaSim

1. Abra o CoppeliaSim no host.
2. Carregue uma cena compatível, como
   `simulador/coppeliasim/simulador_rampa.ttt` ou
   `simulador/coppeliasim/simulador_cones.ttt`. A cena deve conter o objeto
   `/Car` e seus sensores/atuadores, que são os nomes usados pelo código.
3. Inicie o servidor da ZeroMQ Remote API: em **Modules -> Connectivity -> ZMQ
   remote API server**. Ele deve indicar **`(running)`**. Se ainda não estiver
   ativo, clique na opção para iniciá-lo.

Nas versões atuais do CoppeliaSim, esse servidor é fornecido pelo add-on da
ZeroMQ Remote API. No lado Python, a imagem instala o cliente oficial
`coppeliasim-zmqremoteapi-client`; as dependências ZeroMQ e CBOR são instaladas
automaticamente junto com ele, conforme a
[documentação oficial](https://manual.coppeliarobotics.com/en/zmqRemoteApiOverview.htm).

## Autorizar a janela gráfica (X11)

Antes de executar o contêiner, no terminal da mesma sessão gráfica, permita o
acesso local do seu usuário ao servidor X:

```bash
xhost +SI:localuser:$(id -un)
```

O Compose monta `/tmp/.X11-unix` e repassa `DISPLAY`; por isso a janela do
Matplotlib aberta pela aplicação aparece no computador host. Para remover
essa autorização ao terminar, execute:

```bash
xhost -SI:localuser:$(id -un)
```

## Construir e executar

Na raiz do repositório, com a cena aberta e o servidor ZeroMQ em execução:

```bash
docker compose up --build
```

O comando inicia automaticamente `simulador/main.py`. Os diretórios do
simulador são montados em `/workspace/simulador`, portanto mudanças no código
Python local são refletidas na próxima execução sem reconstruir a imagem:

```bash
docker compose up
```

Os resultados são salvos em `simulador/logs/`. Para interromper, use `Ctrl+C`.

## Verificação de conectividade

Antes de diagnosticar o Docker, confirme no CoppeliaSim que o servidor está
ativo em **Modules -> Connectivity -> ZMQ remote API server**: deve aparecer
**`(running)`**. Em seguida, execute `docker compose up`. Se a conexão estiver
correta, o terminal imprime `Carro pronto!` e a janela de telemetria é aberta.

Se o processo não alcançar o CoppeliaSim, confirme que ele foi iniciado no
mesmo computador e que a cena correta está aberta. Como a aplicação conecta
em `localhost`, não é necessário expor portas no Compose quando se usa Docker
no Linux com a rede do host.

## Comandos úteis

```bash
# Executar novamente após uma alteração no código-fonte
docker compose up

# Reconstruir somente quando requirements-simulador.txt ou Dockerfile mudar
docker compose build

# Remover os recursos criados pelo Compose
docker compose down
```
