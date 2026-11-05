# Linux x86_64 environment for the software-in-the-loop simulation
# (Zephyr native_sim runs only on x86/x86_64 Linux hosts).
# Zephyr is fetched from code/west.yml, so it matches the firmware build.
#
# From the repository root:
#   docker build --platform linux/amd64 -f code/sim/Dockerfile -t viikate-sil code
#   docker run --rm --platform linux/amd64 -v "$PWD":/work viikate-sil \
#       python3 code/sim/sil.py test

ARG BASE_IMAGE=ubuntu:24.04
FROM ${BASE_IMAGE}

ARG DEBIAN_FRONTEND=noninteractive
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        ca-certificates cmake device-tree-compiler file g++ gcc git make \
        ninja-build python3 python3-venv \
    && rm -rf /var/lib/apt/lists/*

RUN python3 -m venv /opt/venv
ENV PATH=/opt/venv/bin:$PATH
RUN pip install --no-cache-dir west

# native_sim needs Zephyr and the CMSIS modules, not the vendor HALs.
WORKDIR /zephyrproject
COPY west.yml manifest/west.yml
RUN west init -l manifest \
    && west config manifest.project-filter -- \
        '-.*,+zephyr,+cmsis,+cmsis_6,+cmsis-dsp' \
    && west update --narrow -o=--depth=1 \
    && pip install --no-cache-dir -r zephyr/scripts/requirements-base.txt

ENV ZEPHYR_BASE=/zephyrproject/zephyr \
    ZEPHYR_TOOLCHAIN_VARIANT=host
WORKDIR /work
