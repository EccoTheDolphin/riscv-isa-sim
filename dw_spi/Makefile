CXX ?= c++

CPPFLAGS += -I../riscv
CXXFLAGS += -std=c++2a -fPIC -Wall -Wextra

TARGET := libdw_spi.so
SRCS := dw_spi.cc spi_nor.cc
OBJS := $(SRCS:.cc=.o)

.PHONY: all clean

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CXX) $(LDFLAGS) -shared -o $@ $^ $(LDLIBS)

%.o: %.cc
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -c -o $@ $<

clean:
	rm -f $(TARGET) $(OBJS)
