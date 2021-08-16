CSRCS:=$(shell find -L `pwd` -name "*.c")
OBJS :=$(patsubst %.c,%.o,$(CSRCS))
OUTPUT :=$(basename $(CSRCS))

#INSTALL PATH
PATH_INSTALL_HEAD:=/usr/local/include
PATH_INSTALL_LIB:=/usr/local/lib

WARNING:=-fPIC
CFLAGS:=
LDFLAGS:=
INCLUDE:=

cc:=gcc $(WARNING) $(CFLAGS) $(LDFLAGS) $(INCLUDE)

all:
	@echo $(CSRCS)
	@echo $(OUTPUT)
	make -C tests
	$(cc) uart_utils.c -shared -o libuart.utils.so

.PHONY:clean install uninstall distclean

install:
	cp 

uninstall:

clean:
	rm -rf ./*.o $(OUTPUT)
