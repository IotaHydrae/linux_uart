CSRCS:=$(shell find -L tests -name "*.c")
#OBJS :=$(patsubst %.c,%.o,$(CSRCS))
OUTPUT:=$(basename $(CSRCS))
include libso.mk

#INSTALL PATH
PATH_INSTALL_BIN:=/usr/local/bin
PATH_INSTALL_HEAD:=/usr/local/include
PATH_INSTALL_LIB:=/usr/local/lib

# OUTPUT NAME
OUTPUT_LIB_NAME:=libuartutils.so

WARNING:=
CFLAGS:=-fPIC --shared
LDFLAGS:=
INCLUDE:=

cc:=gcc $(WARNING) $(CFLAGS) $(LDFLAGS) $(INCLUDE)

all:
	@echo "Building library..."
	$(cc) $(obj-c) -o $(OUTPUT_LIB_NAME)
	@echo "Building tests"
	@make -C tests

.PHONY:clean install uninstall distclean

install:
	@echo "Installing..."
	sudo cp ./linux-uart.conf /etc/ld.so.conf.d/
	sudo cp ./libuartutils.so $(PATH_INSTALL_LIB)
	sudo ldconfig
	sudo cp ./tests/serial_rw $(PATH_INSTALL_BIN)

uninstall:
	@echo "Uninstalling..."
	sudo rm -f /etc/ld.so.conf.d/linux-uart.conf
	sudo rm -f $(PATH_INSTALL_LIB)libuartutils.so
	sudo ldconfig
	sudo rm -f $(PATH_INSTALL_BIN)serial_rw

clean:
	@echo "cleaning..."
	rm -rf $(OUTPUT)

distclean:
	@echo "distcleaning..."

