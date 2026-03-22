FROM python:3.11-bookworm

ENV PYTHONDONTWRITEBYTECODE=1 \
    PYTHONUNBUFFERED=1 \
    PIP_DISABLE_PIP_VERSION_CHECK=1 \
    CXXFLAGS="-include iterator"

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    ninja-build \
    pkg-config \
    git \
    curl \
    ca-certificates \
    libzmq3-dev \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    libsdl2-dev \
    libjpeg-dev \
    libusb-1.0-0-dev \
    libxtensor-dev \
    xtl-dev \
    libxsimd-dev \
    && rm -rf /var/lib/apt/lists/*

RUN set -eux; \
    arch="$(uname -m)"; \
    case "$arch" in \
      x86_64) cmake_arch="x86_64" ;; \
      aarch64) cmake_arch="aarch64" ;; \
      *) echo "Unsupported architecture: $arch" >&2; exit 1 ;; \
    esac; \
    cmake_ver="3.31.10"; \
    curl -fsSL "https://github.com/Kitware/CMake/releases/download/v${cmake_ver}/cmake-${cmake_ver}-linux-${cmake_arch}.tar.gz" -o /tmp/cmake.tgz; \
    tar -xzf /tmp/cmake.tgz -C /opt; \
    mv "/opt/cmake-${cmake_ver}-linux-${cmake_arch}" /opt/cmake; \
    ln -sf /opt/cmake/bin/cmake /usr/local/bin/cmake; \
    ln -sf /opt/cmake/bin/ctest /usr/local/bin/ctest; \
    ln -sf /opt/cmake/bin/cpack /usr/local/bin/cpack; \
    rm -f /tmp/cmake.tgz

RUN useradd --create-home --uid 10001 appuser

WORKDIR /app

# Install build tools into system Python (needed for compiling C extensions)
RUN pip install --no-cache-dir --upgrade pip setuptools wheel ninja

# Create a venv owned by appuser so the LLM can pip install at runtime
RUN python -m venv /app/.venv
ENV PATH="/app/.venv/bin:$PATH"

COPY requirements-mcp.txt /app/requirements-mcp.txt
RUN pip install --no-cache-dir -r /app/requirements-mcp.txt

COPY . /app
RUN mkdir -p /app/modules && chown -R appuser:appuser /app

USER appuser

CMD ["python", "-m", "server.mcp_server"]
