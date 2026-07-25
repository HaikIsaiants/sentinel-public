FROM ubuntu:24.04

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends build-essential cmake ninja-build protobuf-compiler libprotobuf-dev libgtest-dev libeigen3-dev python3 python3-protobuf && rm -rf /var/lib/apt/lists/*

WORKDIR /sentinel

COPY . .

RUN python3 tools/generate_seeds.py --check && cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release && cmake --build build --parallel && ctest --test-dir build --output-on-failure

CMD ["ctest", "--test-dir", "build", "--output-on-failure"]
