# BLOSC master
BLOSC_INC=-I./third_party/c-blosc/blosc
BLOSC_LIB=./third_party/c-blosc/build/blosc/libblosc.a

# BLOSC v1.5
# BLOSC_INC=-I./third_party/c-blosc/blosc
# BLOSC_LIB=./third_party/c-blosc/build/blosc/libblosc.a -lz -llz4 -lsnappy


## NODEP version
#all:
#	clang -Wno-#pragma-messages -c -g -O2 miniz.c
#	clang++ -o loader_test -fsanitize=address -O2 -g -Weverything -Werror -Wno-c++98-compat loader_test.cc miniz.o

# BLOSC(static lib) version

all:
	clang -Wno-#pragma-messages -c -g -O1 src/miniz.c
	clang++ -o loader_test -DTINYVDBIO_USE_BLOSC $(BLOSC_INC) -I./src -O3 -g -Weverything -Werror -Wno-c++98-compat tests/loader_test.cc miniz.o $(BLOSC_LIB) -pthread
