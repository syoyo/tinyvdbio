CC=clang
CXX=clang++

CXXFLAGS += -std=c++11 -I../../src

all: vdb2vtk

vdb2vtk: vdb2vtk.o miniz.o
	$(CXX) -o $@ $^

vdb2vtk.o: vdb2vtk.cc ../../src/tinyvdbio.h
	$(CXX) -c -o $@ $(CXXFLAGS) $<

miniz.o: ../../src/miniz.c
	$(CC) -c -o $@ $(CFLAGS) $<

.PHONY: clean

clean:
	rm miniz.o vdb2vtk.o
