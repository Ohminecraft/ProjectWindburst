CXX ?= g++
CXXFLAGS = -std=c++17 -O2 -Wall -Wextra -Wno-unused-parameter
# Every program is one .cpp that includes headers; the compiler writes the list of those headers to build/<name>.d (-MMD), so a
# changed header always rebuilds exactly the programs that use it (a hand-kept list once let a stale binary pass a mutation test).
STUB = -DARDUINO=1 -Itest/arduino_stub
# `make test`      : every PC test of the firmware (no hardware, no Python needed)
# `make sim`       : build/wb_sim - the firmware on a virtual board, the app connects with socket://127.0.0.1:5760
# `make apptest`   : the Python client against the simulator (needs python3, pyserial, pytest)
# `make proto`     : regenerate src/proto/wbp_schema.h after editing protocol/wbp_schema.json
# `make mutate`    : mutation test of the protocol / link / receiver / calibration / app glue (deliberate bugs; every one must be caught - takes hours, see docs/HANDOFF.md). GUI: python3 tools/mutate_gui.py
test: build/test_core build/test_ekf build/test_control build/test_nav build/test_nav_est build/test_nav_ctl build/test_mission build/test_flight build/test_accgyro build/test_manager build/test_motor build/test_baro build/test_range build/test_flowsensor build/test_msp_sensors build/test_msp_config build/test_gps build/test_mag build/test_dshot build/test_esc4way build/test_port build/test_proto build/test_cli build/test_can build/test_mavlink build/test_osd build/test_rx build/test_power build/test_blackbox build/test_app build/test_fw build/test_threads
	./build/test_core
	./build/test_ekf
	./build/test_control
	./build/test_nav
	./build/test_nav_est
	./build/test_nav_ctl
	./build/test_mission
	./build/test_flight
	./build/test_accgyro
	./build/test_manager
	./build/test_motor
	./build/test_baro
	./build/test_range
	./build/test_flowsensor
	./build/test_msp_sensors
	./build/test_msp_config
	./build/test_gps
	./build/test_mag
	./build/test_dshot
	./build/test_esc4way
	./build/test_port
	./build/test_proto
	./build/test_cli
	./build/test_can
	./build/test_mavlink
	./build/test_osd
	./build/test_rx
	./build/test_power
	./build/test_blackbox
	./build/test_app
	./build/test_fw
	./build/test_threads
sim: build/wb_sim
apptest: build/wb_sim build/ui_reference
	cd app && python3 -m pytest -q tests
proto:
	python3 tools/gen_proto.py
mutate:
	python3 tools/mutate.py
# ---- flags per program (STUB = compiled against the Arduino stubs, with ARDUINO defined so the delay hooks are exercised)
FLAGS_test_manager = $(STUB)
FLAGS_test_baro = $(STUB)
FLAGS_test_range = $(STUB)
FLAGS_test_flowsensor = $(STUB)
FLAGS_test_gps = $(STUB)
FLAGS_test_mag = $(STUB)
FLAGS_test_proto = -Wno-sign-compare
FLAGS_test_cli = -Wno-sign-compare
FLAGS_test_blackbox = -pthread
FLAGS_test_app = -pthread
# firmware-in-the-loop: the real FlightApp + drivers + core + WBP server on a virtual board (chip simulators + plant)
FLAGS_test_fw = $(STUB)
# the firmware with its two threads (flight task, sensor thread) and the link task, under ThreadSanitizer: any access the lock does not cover fails the test
FLAGS_test_threads = $(STUB) -pthread -fsanitize=thread -g -DWB_STUB_ATOMIC_CLOCK
# the firmware on the virtual board as a program the app connects to
FLAGS_wb_sim = $(STUB) -pthread

build/test_%: test/test_%.cpp
	@mkdir -p build
	$(CXX) $(CXXFLAGS) $(WIN) $(FLAGS_test_$*) -MMD -MP -MT $@ -MF build/test_$*.d $< -o $@ -lm
# the firmware's own numbers for what the GUI previews (mixer rows, rate / throttle curves); compared with the page's JS by app/tests/test_gui_math.py
build/ui_reference: tools/ui_reference.cpp
	@mkdir -p build
	$(CXX) $(CXXFLAGS) $(WIN) -MMD -MP -MT $@ -MF build/ui_reference.d $< -o $@ -lm
ifeq ($(OS),Windows_NT)
LIBS_wb_sim = -lws2_32                 # (winsock)
# MinGW's math.h hides M_PI under -std=c++17 (strict ANSI) unless asked for it; kept out of CXXFLAGS so that overriding those keeps it
WIN = -D_USE_MATH_DEFINES
endif
build/wb_sim: tools/wb_sim.cpp
	@mkdir -p build
	$(CXX) $(CXXFLAGS) $(WIN) $(FLAGS_wb_sim) -MMD -MP -MT $@ -MF build/wb_sim.d $< -o $@ -lm $(LIBS_wb_sim)
# (-MT $@: the target in a .d file is the name make knows - g++ on Windows would write build/x.exe, and a changed header
#  would then never rebuild anything)
# the schema is not a header: the generated wbp_schema.h is (run `make proto` after editing the json)
-include $(wildcard build/*.d)
