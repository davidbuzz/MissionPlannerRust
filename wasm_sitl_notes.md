#
# ref https://github.com/ArduPilot/ardupilot/blob/master/.github/workflows/test_wasm_plane.yml
# sudo apt install emscripten - not this one too old 3.1.6

# this is a dep, 6.0.8 
source "$HOME/emsdk/emsdk_env.sh"

# using this folder 
cd ~/ardupilot

./waf configure --board wasm
./waf build --target bin/arduplane -j12
./waf build --target bin/arducopter -j12
./waf build --target bin/ardurover -j12
./waf build --target bin/arduheli -j12


# test -f build/wasm/bin/arduplane.js
# test -f build/wasm/bin/arduplane.wasm
# node Tools/autotest/wasm_plane_smoke_test.mjs build/wasm/bin/arduplane.js