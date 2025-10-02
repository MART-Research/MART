# Compiler and flags
CXX       := g++
CXXFLAGS  := -std=c++17 -Wall \
               -I../../modules/utils/Networks \
               -I../../modules/utils/Networks/Topologies \
               -I../../modules/Router/include

# Add helper sources for topology quadrant random node selection
TOPOLOGY_HELPERS := \
  ../../modules/utils/networks/topologies/helper_function/MeshHelper.cpp \
  ../../modules/utils/networks/topologies/helper_function/RingHelper.cpp \
  ../../modules/utils/networks/topologies/helper_function/TorusHelper.cpp

# Shared sources (used by all targets)
COMMON_SRCS := \
  ../../modules/utils/nodes/Node.cpp \
  ../../modules/utils/nodes/Router_node.cpp \
  ../../modules/Router/src/MinimalOblivious.cpp \
  ../../modules/Router/src/XorTag.cpp \
  ../../modules/Router/src/DestinationTag.cpp \
  ../../modules/Router/src/DimensionOrder.cpp \
  $(TOPOLOGY_HELPERS)

# Targets
TARGETS := test_dor test_dtr test_xor

# Source files per target
test_dor_SRCS       := test_dor.cpp $(COMMON_SRCS)
test_dtr_SRCS       := test_dtr.cpp $(COMMON_SRCS)
test_xor_SRCS       := test_xor.cpp $(COMMON_SRCS)


# Object files per target
test_dor_OBJS       := $(test_dor_SRCS:.cpp=.o)
test_dtr_OBJS       := $(test_dtr_SRCS:.cpp=.o)
test_xor_OBJS       := $(test_xor_SRCS:.cpp=.o)


# Default: build all targets
all: $(TARGETS)

# Individual build rules
test_dor: $(test_dor_OBJS)
	$(CXX) $(CXXFLAGS) $^ -o $@

test_dtr: $(test_dtr_OBJS)
	$(CXX) $(CXXFLAGS) $^ -o $@

test_xor: $(test_xor_OBJS)
	$(CXX) $(CXXFLAGS) $^ -o $@

# Generic compile rule
%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Run targets
run_test_dor: test_dor
	./test_dor

run_test_dtr: test_dtr
	./test_dtr

run_test_xor: test_xor
	./test_xor
# Clean
clean:
	rm -f $(test_dor_OBJS) $(test_dtr_OBJS) $(test_xor_OBJS) $(TARGETS)
	rm -f *.png *.dot

.PHONY: all clean \
	run_test_dor run_test_dtr run_test_xor
