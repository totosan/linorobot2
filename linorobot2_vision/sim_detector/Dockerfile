# CPU simulation twin of the Jetson .71 detector. Weights are mounted, not baked in.
FROM python:3.11-slim
ENV PIP_NO_CACHE_DIR=1 PYTHONUNBUFFERED=1
RUN apt-get update && apt-get install -y --no-install-recommends libglib2.0-0 libgl1 \
    && rm -rf /var/lib/apt/lists/*
WORKDIR /app
COPY requirements.txt .
RUN pip install --extra-index-url https://download.pytorch.org/whl/cpu torch torchvision \
    && pip install -r requirements.txt
COPY server.py .
EXPOSE 5555 5556
ENTRYPOINT ["python", "server.py"]
