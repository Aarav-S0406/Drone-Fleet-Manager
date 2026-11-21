CC = gcc
CFLAGS = -Wall -Wextra -Wpedantic -std=c11 -O2
LDLIBS = -lm

SRC = main.c simulator.c algorithms.c
TARGET = drone_planner

.PHONY: all clean run benchmark
all: $(TARGET)

$(TARGET): $(SRC) drone_system.h algorithms.h
	$(CC) $(CFLAGS) $(SRC) -o $@ $(LDLIBS)

run: $(TARGET)
	./$(TARGET)

benchmark: $(TARGET)
	./$(TARGET) --benchmark

clean:
	rm -f $(TARGET) *.o
