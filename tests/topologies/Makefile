# Compiler and flags
CXX       := g++
CXXFLAGS  := -std=c++17 -Wall \
               -I../../modules/utils/Networks \
               -I../../modules/utils/Networks/Topologies \
               -I../../modules/utils/Networks/Topologies/helper_function \
               -I../../modules/Router/include

HELPER_SRCS := $(wildcard ../../modules/utils/Networks/Topologies/helper_function/*Helper.cpp)

# Shared sources (used by all targets)
COMMON_SRCS := \
  ../../modules/utils/nodes/Node.cpp \
  ../../modules/utils/nodes/Router_node.cpp \
  ../../modules/Router/src/MinimalOblivious.cpp \
  ../../modules/Router/src/XorTag.cpp \
  ../../modules/Router/src/DestinationTag.cpp \
  ../../modules/Router/src/DimensionOrder.cpp \
  ../../modules/Router/src/LoadBalancedOblivious.cpp \
  ../../modules/Router/src/Valiant.cpp \
  $(HELPER_SRCS)

# Topology test targets only
TARGETS := mesh_test tree_test butterfly_test dragonfly_test ring_test torus_test hypercube_test

# Source files per target
mesh_test_SRCS      := mesh_test.cpp $(COMMON_SRCS)
tree_test_SRCS      := tree_test.cpp $(COMMON_SRCS)
butterfly_test_SRCS := butterfly_test.cpp $(COMMON_SRCS)
dragonfly_test_SRCS := dragonfly_test.cpp $(COMMON_SRCS)
ring_test_SRCS      := ring_test.cpp $(COMMON_SRCS)
torus_test_SRCS     := torus_test.cpp $(COMMON_SRCS)
hypercube_test_SRCS := hypercube_test.cpp $(COMMON_SRCS)

# Object files per target
mesh_test_OBJS      := $(mesh_test_SRCS:.cpp=.o)
tree_test_OBJS      := $(tree_test_SRCS:.cpp=.o)
butterfly_test_OBJS := $(butterfly_test_SRCS:.cpp=.o)
dragonfly_test_OBJS := $(dragonfly_test_SRCS:.cpp=.o)
ring_test_OBJS      := $(ring_test_SRCS:.cpp=.o)
torus_test_OBJS     := $(torus_test_SRCS:.cpp=.o)
hypercube_test_OBJS := $(hypercube_test_SRCS:.cpp=.o)

# Default: build all targets
all: $(TARGETS)

# Individual build rules
mesh_test: $(mesh_test_OBJS)
	$(CXX) $(CXXFLAGS) $^ -o $@

tree_test: $(tree_test_OBJS)
	$(CXX) $(CXXFLAGS) $^ -o $@

butterfly_test: $(butterfly_test_OBJS)
	$(CXX) $(CXXFLAGS) $^ -o $@

dragonfly_test: $(dragonfly_test_OBJS)
	$(CXX) $(CXXFLAGS) $^ -o $@

ring_test: $(ring_test_OBJS)
	$(CXX) $(CXXFLAGS) $^ -o $@

torus_test: $(torus_test_OBJS)
	$(CXX) $(CXXFLAGS) $^ -o $@

hypercube_test: $(hypercube_test_OBJS)
	$(CXX) $(CXXFLAGS) $^ -o $@

# Generic compile rule
%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Run targets
run_mesh_test: mesh_test
	./mesh_test

run_tree_test: tree_test
	./tree_test

run_butterfly_test: butterfly_test
	./butterfly_test

run_dragonfly_test: dragonfly_test
	./dragonfly_test

run_ring_test: ring_test
	./ring_test

run_torus_test: torus_test
	./torus_test

run_hypercube_test: hypercube_test
	./hypercube_test

# Clean
clean:
	rm -f $(mesh_test_OBJS) $(tree_test_OBJS) $(butterfly_test_OBJS) \
	      $(dragonfly_test_OBJS) $(ring_test_OBJS) $(torus_test_OBJS) $(hypercube_test_OBJS)
	-$(RM) $(TARGETS)
	rm -f *.png *.dot

.PHONY: all clean \
	run_mesh_test run_tree_test run_butterfly_test run_dragonfly_test \
	run_ring_test run_torus_test run_hypercube_test
