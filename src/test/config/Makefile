
SRC_DIR = ../../main
LIB_DIR = ../../../lib/main
CFLAGS = -g -Wall -Wextra -ggdb3 -O0 -std=gnu99 -I $(SRC_DIR) -I $(LIB_DIR)/boost_preprocessor/include -MMD -MP


VPATH := $(SRC_DIR) $(SRC_DIR)/config

%.o: %.c Makefile
	$(CC) -c -o $@ $(CFLAGS) $<

config_test : main.o config_parser.o
	$(CC) $(LD_FLAGS) $^ -o $@

-include main.d config_parser.d
