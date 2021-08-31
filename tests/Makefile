
all:
	$(foreach c,$(obj-c),$(shell gcc -L.. -luartutils $(c) -o $(basename $(c))))

obj-c+=serial_rw.c
