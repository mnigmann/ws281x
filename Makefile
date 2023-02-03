# obj-m += hello-1.o
obj-m += ws281x.o
# obj-m += dma_test.o

all:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules

clean:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) clean

