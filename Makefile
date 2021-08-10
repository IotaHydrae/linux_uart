CSRCS:=$(shell find -L `pwd` -name "*.c")
OBJS :=$(patsubst %.c,%.o,$(CSRCS))
OUTPUT :=$(basename $(CSRCS))

all:
	@echo $(CSRCS)
	@echo $(OUTPUT)
	$(foreach c,$(CSRCS),$(shell gcc $(c) -o $(basename $(c))))

.PHONY:clean

clean:
	rm -rf ./*.o $(OUTPUT)
