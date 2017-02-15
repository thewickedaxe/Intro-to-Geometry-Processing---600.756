MESH_IO_TARGET=MeshIO
MESH_IO_SOURCE=MeshIO.cpp

 COMPILER = gcc
#COMPILER = clang

ifeq ($(COMPILER),gcc)
	CFLAGS += -fopenmp -Wno-deprecated -Wno-write-strings -std=c++11 -fopenmp -fpermissive
	LFLAGS += -lgomp -lstdc++
else
# 	CFLAGS += -fopenmp=libiomp5 -Wno-deprecated -Wno-write-strings -std=c++14 -Wno-invalid-offsetof
# 	LFLAGS += -liomp5 -lstdc++
	CFLAGS += -Wno-deprecated -Wno-write-strings -std=c++11 -Wno-invalid-offsetof
	LFLAGS += -lstdc++
endif

CFLAGS_DEBUG = -DDEBUG -g3
LFLAGS_DEBUG =

CFLAGS_RELEASE = -O3 -DRELEASE -funroll-loops -ffast-math
LFLAGS_RELEASE = -O3 

SRC = MeshIO/
BIN = Bin/Linux/
INCLUDE = .

ifeq ($(COMPILER),gcc)
	CC=gcc
	CXX=g++
else
	CC=clang-3.8
	CXX=clang++-3.8
endif

MD=mkdir

MESH_IO_OBJECTS=$(addprefix $(BIN), $(addsuffix .o, $(basename $(MESH_IO_SOURCE))))


all: CFLAGS += $(CFLAGS_RELEASE)
all: LFLAGS += $(LFLAGS_RELEASE)
all: $(BIN)
all: $(BIN)$(MESH_IO_TARGET)
all:
	@echo "Done"

debug: CFLAGS += $(CFLAGS_DEBUG)
debug: LFLAGS += $(LFLAGS_DEBUG)
debug: $(BIN)
debug: $(BIN)$(MESH_IO_TARGET)

MeshIO: CFLAGS += $(CFLAGS_RELEASE)
MeshIO: LFLAGS += $(LFLAGS_RELEASE)
MeshIO: $(BIN)
MeshIO: $(BIN)$(MESH_IO_TARGET)

clean:
	rm -rf $(BIN)$(MESH_IO_TARGET)
	rm -rf $(MESH_IO_OBJECTS)
	@echo "Done"

$(BIN):
	$(MD) -p $(BIN)

$(BIN)$(MESH_IO_TARGET): $(MESH_IO_OBJECTS)
	$(CXX) -o $@ $(MESH_IO_OBJECTS) $(LFLAGS)
	
$(BIN)%.o: $(SRC)%.c
	mkdir -p $(BIN)
	$(CC) -c -o $@ $(CFLAGS) -I$(INCLUDE) $<

$(BIN)%.o: $(SRC)%.cpp
	mkdir -p $(BIN)
	$(CXX) -c -o $@ $(CFLAGS) -I$(INCLUDE) $<
