CSRCS:=$(shell find -L `pwd` -name *.c)
OBJS :=$(patsubst %.c,%.o,$(CSRCS))
OUTPUT :=$(basename $(CSRCS))

all:
	@echo $(OUTPUT)
	gcc $(foreach c,$(CSRCS),$(c)) -o $(foreach e,$(OUTPUT),$(e))

.PHONY:clean

clean:
	rm -rf ./*.o $(OUTPUT)
