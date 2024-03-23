# Define Gurobi directory
GUROBI_HOME := $(shell echo $$GUROBI_HOME)
# Include directory for Gurobi
GUROBI_INCLUDE := $(GUROBI_HOME)/include


# Library directory for Gurobi
#GUROBI_LIB := -L$(GUROBI_HOME)/lib -lgurobi_c++ -lgurobi110
GUROBI_LIB := -L$(GUROBI_HOME)/lib -lgurobi110 -lgurobi_c++
# Compiler
CXX = g++

# Compiler flags
CXXFLAGS = -m64 -g -std=c++11 -I$(GUROBI_INCLUDE) -mcmodel=medium

LDFLAGS += -Wl,--no-relax -Wl,-rpath,$(GUROBI_HOME)/lib

# Target executable name
TARGET = my_gurobi_program

# Source files
SRC = MDHODBRP_model.cpp

# Object files
OBJ = $(SRC:.cpp=.o)

# Make rule for the target
$(TARGET): $(OBJ)
	$(CXX) $(CXXFLAGS) -o $@ $^ $(GUROBI_LIB) $(LDFLAGS)

# Clean rule
clean:
	rm -f *.o $(TARGET)

# Phony targets
.PHONY: clean