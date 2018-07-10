BLOSC_INC=-I./third_party/c-blosc/
BLOSC_LIB=./third_party/c-blosc/build/blosc/libblosc.a


# # NODEP version
# all:
# 	clang -Wno-#pragma-messages -c miniz.c
# 	clang++ -o loader_test -fsanitize=address -g -Weverything -Werror -Wno-c++98-compat loader_test.cc miniz.o

# BLOSC(static lib) version

all:
	clang -Wno-#pragma-messages -c miniz.c
	clang++ -o loader_test -DTINYVDBIO_USE_BLOSC $(BLOSC_INC) -fsanitize=address -g -Weverything -Werror -Wno-c++98-compat loader_test.cc miniz.o $(BLOSC_LIB)
