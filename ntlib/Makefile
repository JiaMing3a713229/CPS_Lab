CC = gcc
CFLAGS = -Wall -Wextra -I Ntlib

SRC_DIR = src
NT_DIR = Ntlib

SRCS = $(wildcard $(SRC_DIR)/*.c) $(wildcard $(NT_DIR)/*.c)
OBJS = $(SRCS:.c=.o)
TARGET = main.exe
# TARGET = main

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CC) $(CFLAGS) $^ -o $@

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

clean:
	del $(SRC_DIR)\*.o
	del $(NT_DIR)\*.o
	del $(TARGET)