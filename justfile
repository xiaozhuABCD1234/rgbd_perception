set shell := ["bash", "-eu", "-o", "pipefail", "-c"]

default: check

# Configure the Debug development build.
configure:
    cmake --preset dev

# Build all targets.
build: configure
    cmake --build --preset dev --parallel

# Build and run unit tests.
test: build
    ctest --preset dev

# Run the synthetic-camera demonstration.
run *args: build
    ./build/dev/rgbd_demo {{args}}

# Build and test an optimized Release configuration.
release:
    cmake --preset release
    cmake --build --preset release --parallel
    ctest --preset release

# Build and test with ASan/UBSan (GCC or Clang).
sanitizer:
    cmake --preset asan
    cmake --build --preset asan --parallel
    ctest --preset asan

# Format all first-party C++ sources.
format:
    find apps include src tests -type f \( -name '*.cpp' -o -name '*.hpp' -o -name '*.in' \) -print0 \
        | xargs -0 clang-format -i

# Verify formatting without modifying files.
format-check:
    find apps include src tests -type f \( -name '*.cpp' -o -name '*.hpp' -o -name '*.in' \) -print0 \
        | xargs -0 clang-format --dry-run --Werror

# Run clang-tidy in a separate build directory.
tidy:
    cmake -S . -B build/tidy -G Ninja \
        -DCMAKE_BUILD_TYPE=Debug \
        -DBUILD_TESTING=ON \
        -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
        -DCMAKE_CXX_CLANG_TIDY=clang-tidy
    cmake --build build/tidy --parallel

# Install the repository-local Git hooks.
hooks:
    bash scripts/install_hooks.sh

# Check the latest commit, or a range: just commit-check origin/main..HEAD
commit-check range="HEAD":
    python3 scripts/check_commit_range.py '{{range}}'

# Local checks corresponding to CI.
check: format-check test

# Remove generated build files.
clean:
    rm -rf build

# Install to a local staging directory.
install: build
    cmake --install build/dev --prefix "$PWD/build/stage"
