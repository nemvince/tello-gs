CC      = gcc
CFLAGS  = -O2 -Wall -Wextra \
          -Iinclude \
          $(shell pkg-config --cflags sdl2 SDL2_ttf libavcodec libavutil libswscale libonnxruntime)
LDFLAGS = $(shell pkg-config --libs sdl2 SDL2_ttf libavcodec libavutil libswscale libonnxruntime) \
          -lpthread -lm

TARGET  = tello_gs
SRCDIR  = src
BUILDDIR= build

SRCS    = $(wildcard $(SRCDIR)/*.c)
OBJS    = $(patsubst $(SRCDIR)/%.c, $(BUILDDIR)/%.o, $(SRCS))
DEPS    = $(OBJS:.o=.d)

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CC) -o $@ $^ $(LDFLAGS)

$(BUILDDIR)/%.o: $(SRCDIR)/%.c | $(BUILDDIR)
	$(CC) $(CFLAGS) -MMD -MP -c -o $@ $<

$(BUILDDIR):
	mkdir -p $(BUILDDIR)

-include $(DEPS)

clean:
	rm -rf $(BUILDDIR) $(TARGET)

.PHONY: all clean
