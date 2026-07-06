FROM ardupilot/ardupilot-dev-base:latest

ARG ARDUPILOT_TAG=Rover-4.6.3

USER root

# sim_vehicle.py використовує ip для перевiрки мережi та cleanup.
RUN apt-get update \
    && apt-get install -y --no-install-recommends iproute2 \
    && rm -rf /var/lib/apt/lists/*

# Офiцiйний dev-base мiстить toolchain, але не мiстить source та готових binary.
RUN git clone --depth 1 --branch "${ARDUPILOT_TAG}" --recurse-submodules \
      https://github.com/ArduPilot/ardupilot.git /ardupilot \
    && cd /ardupilot \
    && ./waf configure --board sitl \
    && ./waf rover

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

WORKDIR /ardupilot
ENTRYPOINT ["/entrypoint.sh"]
