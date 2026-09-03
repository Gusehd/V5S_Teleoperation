# Builds the C++ tools. The Python side needs no build step (pip install -e .).
#
#   make            build everything whose source changed
#   make bridge     the bridge only
#   make clean
#
# WARNING: editing a source without rebuilding leaves the old binary running.
#   It looks like it is working, which makes it hard to spot -- we lost time to
#   exactly this once, after adding per-hand sockets without rebuilding, so the
#   right-hand port was never opened.
#
# The MANUS SDK is not in this repository (redistribution is not permitted).
# The default path follows docs/SETUP.md; if it is unpacked elsewhere, pass it:
#
#   make MANUS_SDK=/path/to/ManusSDK

MANUS_SDK ?= external/ManusSDK_v3.1.1/SDKClient_Linux/ManusSDK

CXX      ?= g++
CXXFLAGS ?= -std=c++17 -O2 -Wall
SDKFLAGS  = -I$(MANUS_SDK)/include -L$(MANUS_SDK)/lib \
            -lManusSDK_Integrated -Wl,-rpath,$(MANUS_SDK)/lib -lpthread

BRIDGE = bridge_cpp/manus_bridge
TOOLS  = tools/manus_diag tools/manus_nodes tools/manus_apply_mcal
ALL    = $(BRIDGE) $(TOOLS)

.PHONY: all bridge tools clean check-sdk test

all: check-sdk $(ALL)
	@echo "built $(words $(ALL)) targets"

bridge: check-sdk $(BRIDGE)
tools:  check-sdk $(TOOLS)

check-sdk:
	@test -d "$(MANUS_SDK)/include" || { \
	  echo "MANUS SDK not found at: $(MANUS_SDK)"; \
	  echo "See docs/SETUP.md -- unpack it under external/ or pass MANUS_SDK=<path>."; \
	  exit 1; }

# Only the bridge additionally needs zmq.
$(BRIDGE): bridge_cpp/manus_bridge.cpp
	$(CXX) $(CXXFLAGS) $< $(SDKFLAGS) -lzmq -o $@

tools/%: tools/%.cpp
	$(CXX) $(CXXFLAGS) $< $(SDKFLAGS) -o $@

# Python checks. Needs neither the SDK nor hardware -- a few seconds.
PYTHON ?= .venv/bin/python
test:
	$(PYTHON) tests/smoke.py
	$(PYTHON) tools/check_lr_symmetry.py

clean:
	rm -f $(ALL)
