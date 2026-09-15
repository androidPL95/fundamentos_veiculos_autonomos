# Imagem apenas para as dependências de execução. O código é montado pelo
# docker-compose.yml, para que alterações locais não exijam novo build.
FROM python:3.12-slim

ENV PYTHONDONTWRITEBYTECODE=1 \
    PYTHONUNBUFFERED=1 \
    XDG_CACHE_HOME=/tmp/xdg-cache \
    MPLCONFIGDIR=/tmp/matplotlib

RUN mkdir -p /tmp/xdg-cache/fontconfig /tmp/matplotlib \
    && chmod -R 1777 /tmp/xdg-cache /tmp/matplotlib

# Bibliotecas necessárias para o backend gráfico Tk/Matplotlib acessar o X11.
RUN apt-get update \
    && apt-get install --no-install-recommends -y \
        libgl1 \
        libx11-6 \
        libxext6 \
        libxrender1 \
        python3-tk \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace/simulador

COPY requirements-simulador.txt /tmp/requirements-simulador.txt
RUN pip install --no-cache-dir -r /tmp/requirements-simulador.txt

CMD ["python", "main.py"]
