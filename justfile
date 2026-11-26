set shell := ["bash", "-eu", "-o", "pipefail", "-c"]

default: check

configure:
    cmake --preset dev

sdk:
    #!/usr/bin/env bash
    set -euo pipefail
    git submodule update --init -- third_party/OrbbecSDK
    sdk_home="$(sed -n 's/^CMAKE_HOME_DIRECTORY:INTERNAL=//p' build/_sdk/src/CMakeCache.txt 2>/dev/null || true)"
    if [[ -n "$sdk_home" && "$sdk_home" != "$PWD/third_party/OrbbecSDK" ]]; then
        rm -rf build/_sdk/src
    fi
    cmake -S third_party/OrbbecSDK -B build/_sdk/src -G Ninja \
        -DCMAKE_BUILD_TYPE=Release \
        -DOB_BUILD_EXAMPLES=OFF \
        -DOB_BUILD_TOOLS=OFF \
        -DOB_BUILD_TESTS=OFF \
        -DOB_BUILD_DOCS=OFF \
        -DOB_INSTALL_EXAMPLES_SOURCE=OFF \
        -DCMAKE_INSTALL_PREFIX="$PWD/build/_sdk/install"
    cmake --build build/_sdk/src --parallel
    cmake --install build/_sdk/src

build: configure
    cmake --build --preset dev --parallel

test: build
    ctest --preset dev

run *args: build
    ./build/dev/rgbd_demo {{args}}

viewer *args: build
    ./build/dev/rgbd_viewer {{args}}

replay *args: build
    ./build/dev/rgbd_replay {{args}}

format:
    find apps include src tests -type f \( -name '*.cpp' -o -name '*.hpp' \) -print0 \
        | xargs -0 clang-format -i

format-check:
    find apps include src tests -type f \( -name '*.cpp' -o -name '*.hpp' \) -print0 \
        | xargs -0 clang-format --dry-run --Werror

cpplint:
    python3 -m cpplint --recursive apps include src tests

lint: cpplint

tidy:
    cmake -S . -B build/tidy -G Ninja \
        -DCMAKE_BUILD_TYPE=Debug \
        -DBUILD_TESTING=ON \
        -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
        -DCMAKE_CXX_CLANG_TIDY=clang-tidy
    cmake --build build/tidy --parallel

hooks:
    git config core.hooksPath .githooks
    chmod +x .githooks/commit-msg .githooks/pre-commit .githooks/check-commit-range

commit-check range="HEAD":
    .githooks/check-commit-range '{{range}}'

check: test cpplint format-check

clean:
    rm -rf build

install: build
    cmake --install build/dev --prefix "$PWD/build/stage"

submodules:
    #!/usr/bin/env bash
    set -euo pipefail
    git submodule update --init --recursive
    if [[ ! -f third_party/librealsense/CMakeLists.txt ]]; then
        git submodule add -f https://github.com/IntelRealSense/librealsense.git third_party/librealsense
    fi
    if [[ ! -f third_party/mcap/README.md ]]; then
        git submodule add -f https://github.com/foxglove/mcap.git third_party/mcap
    fi

gemini305 uri="gemini305://?width=640&height=480&fps=30" *args: sdk
    cmake --preset gemini305
    cmake --build --preset gemini305 --parallel
    ./build/gemini305/rgbd_viewer "{{uri}}" {{args}}

lichtblick-setup:
    #!/usr/bin/env bash
    set -euo pipefail
    mkdir -p data
    if [[ -f data/lichtblick-web/index.html ]]; then
        exit 0
    fi
    if [[ -f ../neo-nav/data/lichtblick-web/index.html ]]; then
        ln -s ../../neo-nav/data/lichtblick-web data/lichtblick-web
        exit 0
    fi
    curl -L --fail --retry 3 -o data/lichtblick-web.tar.gz \
        "https://github.com/lichtblick-suite/lichtblick/releases/download/v1.31.0/lichtblick-web.tar.gz"
    mkdir -p data/lichtblick-web
    tar -xzf data/lichtblick-web.tar.gz -C data/lichtblick-web
    rm data/lichtblick-web.tar.gz

gemini305-lichtblick *args: sdk lichtblick-setup
    git submodule update --init --depth 1 -- third_party/mcap
    cmake --preset gemini305
    cmake --build --preset gemini305 --parallel
    ./build/gemini305/rgbd_lichtblick {{args}}

d435i uri="d435i://?width=640&height=480&fps=30" *args:
    #!/usr/bin/env bash
    set -euo pipefail
    if [[ ! -f third_party/librealsense/CMakeLists.txt ]]; then
        git submodule add -f https://github.com/IntelRealSense/librealsense.git third_party/librealsense
    fi
    git submodule update --init --recursive -- third_party/librealsense
    cmake --preset d435i
    cmake --build --preset d435i --parallel
    ./build/d435i/rgbd_viewer "{{uri}}" {{args}}
