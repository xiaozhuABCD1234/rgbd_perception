set shell := ["bash", "-eu", "-o", "pipefail", "-c"]

default: check

configure:
    cmake --preset dev

build: configure
    cmake --build --preset dev --parallel

test: build
    ctest --preset dev

run *args: build
    ./build/dev/rgbd_demo {{args}}

release:
    cmake --preset release
    cmake --build --preset release --parallel
    ctest --preset release

sanitizer:
    cmake --preset asan
    cmake --build --preset asan --parallel
    ctest --preset asan

lint-setup:
    python3 -m venv .venv
    .venv/bin/python -m pip install --upgrade pip
    .venv/bin/python -m pip install -r requirements-dev.txt

format:
    find apps include src tests -type f \( -name '*.cpp' -o -name '*.hpp' -o -name '*.in' \) -print0 \
        | xargs -0 .venv/bin/clang-format -i

format-check:
    find apps include src tests -type f \( -name '*.cpp' -o -name '*.hpp' -o -name '*.in' \) -print0 \
        | xargs -0 .venv/bin/clang-format --dry-run --Werror

lint:
    .venv/bin/python -m cpplint --recursive apps include src tests

tidy:
    cmake -S . -B build/tidy -G Ninja \
        -DCMAKE_BUILD_TYPE=Debug \
        -DBUILD_TESTING=ON \
        -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
        -DCMAKE_CXX_CLANG_TIDY=clang-tidy
    cmake --build build/tidy --parallel

hooks:
    bash scripts/install_hooks.sh

commit-check range="HEAD":
    python3 scripts/check_commit_range.py '{{range}}'

check: lint format-check test

clean:
    rm -rf build

install: build
    cmake --install build/dev --prefix "$PWD/build/stage"
