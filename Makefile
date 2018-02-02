all:
	clang++ -o loader_test -fsanitize=address  -Weverything -Werror loader_test.cc
