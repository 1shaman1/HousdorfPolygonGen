# Build from repository root: docker compose build polygon-generator
# (needs contracts/ and orchestrator/cmake/)

FROM ubuntu:22.04 AS builder

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake ninja-build git curl zip unzip tar pkg-config \
    ca-certificates linux-libc-dev python3 \
  && rm -rf /var/lib/apt/lists/*

ENV VCPKG_ROOT=/opt/vcpkg
ENV VCPKG_BUILD_TYPE=release
ENV VCPKG_MAX_CONCURRENCY=4
RUN git clone --depth 1 https://github.com/microsoft/vcpkg.git ${VCPKG_ROOT} \
    && ${VCPKG_ROOT}/bootstrap-vcpkg.sh -disableMetrics

WORKDIR /build

COPY HousdorfPolygonGen/vcpkg.json ./HousdorfPolygonGen/
RUN cd HousdorfPolygonGen && ${VCPKG_ROOT}/vcpkg install --triplet x64-linux

COPY contracts/ ./contracts/
COPY orchestrator/cmake/ ./orchestrator/cmake/
COPY HousdorfPolygonGen/ ./HousdorfPolygonGen/

RUN rm -rf HousdorfPolygonGen/build \
  && cd HousdorfPolygonGen && cmake -B build -G Ninja \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_TOOLCHAIN_FILE=${VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake \
    -DHAUSDORF_PROTO_ROOT=/build/contracts/proto \
  && cmake --build build --parallel

FROM ubuntu:22.04

RUN apt-get update && apt-get install -y --no-install-recommends \
    ca-certificates \
  && rm -rf /var/lib/apt/lists/*

COPY --from=builder /build/HousdorfPolygonGen/build/vcpkg_installed/x64-linux/lib/ /usr/local/lib/
RUN ldconfig

COPY --from=builder /build/HousdorfPolygonGen/build/polygon_gen /usr/local/bin/polygon_gen
COPY HousdorfPolygonGen/appConfig.cfg /app/appConfig.cfg

WORKDIR /app

EXPOSE 50051

ENTRYPOINT ["polygon_gen", "appConfig.cfg"]
