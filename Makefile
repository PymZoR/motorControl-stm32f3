include ./make/build.mk

LDFLAGS += -u _printf_float

upload: build/new-motorController.elf
	openocd -f board/st_nucleo_f3.cfg -c "program $^ verify reset exit"
