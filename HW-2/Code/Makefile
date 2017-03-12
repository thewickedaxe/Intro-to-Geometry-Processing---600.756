MESH_IO_TARGET=MeshIO
MESH_IO_SOURCE=MeshIO/MeshIO.cpp
CURVATURE_TARGET=Curvature
CURVATURE_SOURCE=Curvature/Curvature.cpp
LIC_TARGET=LIC
LIC_SOURCE=LIC/LIC.cpp

 COMPILER = gcc
#COMPILER = clang

ifeq ($(COMPILER),gcc)
	CFLAGS += -fopenmp -Wno-deprecated -Wno-write-strings -std=c++11
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

MESH_IO_OBJECTS=$(addsuffix .o, $(basename $(MESH_IO_SOURCE)))
CURVATURE_OBJECTS=$(addsuffix .o, $(basename $(CURVATURE_SOURCE)))
LIC_OBJECTS=$(addsuffix .o, $(basename $(LIC_SOURCE)))


all: CFLAGS += $(CFLAGS_RELEASE)
all: LFLAGS += $(LFLAGS_RELEASE)
all: $(BIN)
all: $(BIN)$(MESH_IO_TARGET)
all: $(BIN)$(CURVATURE_TARGET)
all: $(BIN)$(LIC_TARGET)

debug: CFLAGS += $(CFLAGS_DEBUG)
debug: LFLAGS += $(LFLAGS_DEBUG)
debug: $(BIN)
debug: $(BIN)$(MESH_IO_TARGET)
debug: $(BIN)$(CURVATURE_TARGET)
debug: $(BIN)$(LIC_TARGET)

MeshIO: CFLAGS += $(CFLAGS_RELEASE)
MeshIO: LFLAGS += $(LFLAGS_RELEASE)
MeshIO: $(BIN)
MeshIO: $(BIN)$(MESH_IO_TARGET)

Curvature: CFLAGS += $(CFLAGS_RELEASE)
Curvature: LFLAGS += $(LFLAGS_RELEASE)
Curvature: $(BIN)
Curvature: $(BIN)$(CURVATURE_TARGET)

LIC: CFLAGS += $(CFLAGS_RELEASE)
LIC: LFLAGS += $(LFLAGS_RELEASE)
LIC: $(BIN)
LIC: $(BIN)$(LIC_TARGET)

clean:
	rm -rf $(BIN)$(MESH_IO_TARGET)
	rm -rf $(MESH_IO_OBJECTS)
	rm -rf $(BIN)$(CURVATURE_TARGET)
	rm -rf $(CURVATURE_OBJECTS)
	rm -rf $(BIN)$(LIC_TARGET)
	rm -rf $(LIC_OBJECTS)

$(BIN):
	$(MD) -p $(BIN)

$(BIN)$(MESH_IO_TARGET): $(MESH_IO_OBJECTS)
	mkdir -p $(BIN)
	$(CXX) -o $@ $(MESH_IO_OBJECTS) $(LFLAGS)

$(BIN)$(CURVATURE_TARGET): $(CURVATURE_OBJECTS)
	mkdir -p $(BIN)
	$(CXX) -o $@ $(CURVATURE_OBJECTS) $(LFLAGS)
	
$(BIN)$(LIC_TARGET): $(LIC_OBJECTS)
	mkdir -p $(BIN)
	$(CXX) -o $@ $(LIC_OBJECTS) $(LFLAGS)

%.o: %.c
	$(CC) -c -o $@ $(CFLAGS) -I$(INCLUDE) $<

%.o: %.cpp
	$(CXX) -c -o $@ $(CFLAGS) -I$(INCLUDE) $<

