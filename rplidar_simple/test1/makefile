CC = gcc
CFLAGS = -Wall -Wextra -g -lm # -lm でmathライブラリをリンク
TARGET = rplidar_reader

all: $(TARGET)

$(TARGET): main.c
	$(CC) $(CFLAGS) main.c -o $(TARGET)

clean:
	rm -f $(TARGET) *.o