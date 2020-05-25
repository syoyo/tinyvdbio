CC=clang
CXX=clang++

CXXFLAGS += -std=c++11 -I../../src

CXXFLAGS += -DTINYVDBIO_USE_BLOSC -I../../third_party/c-blosc/blosc
LDFLAGS += ../../third_party/c-blosc/build/blosc/libblosc.a -lz -pthread

# BLOSC_INC=-I./third_party/c-blosc/blosc
# BLOSC_LIB=./third_party/c-blosc/build/blosc/libblosc.a -lz -llz4 -lsnappy

all: vdb2vtk

vdb2vtk: vdb2vtk.o miniz.o
	$(CXX) -o $@ $^ $(LDFLAGS)

vdb2vtk.o: vdb2vtk.cc ../../src/tinyvdbio.h
	$(CXX) -c -o $@ $(CXXFLAGS) $<

miniz.o: ../../src/miniz.c
	$(CC) -c -o $@ $(CFLAGS) $<

.PHONY: clean

clean:
	rm miniz.o vdb2vtk.o
