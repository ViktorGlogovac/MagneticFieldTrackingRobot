SHELL=cmd
CC = xc32-gcc
OBJCPY = xc32-bin2hex
ARCH = -mprocessor=32MX130F064B
OBJ = Tone.o
PORTN=$(shell type COMPORT.inc)

Tone.elf: $(OBJ)
	$(CC) $(ARCH) -o Tone.elf Tone.o -mips16 -DXPRJ_default=default -legacy-libc -Wl,-Map=Tone.map
	$(OBJCPY) Tone.elf
	@echo Success!
   
Tone.o: Tone.c
	$(CC) -mips16 -g -x c -c $(ARCH) -MMD -o Tone.o Tone.c -DXPRJ_default=default -legacy-libc

clean:
	@del *.o *.elf *.hex *.map *.d 2>NUL
	
LoadFlash:
	@taskkill /f /im putty.exe /t /fi "status eq running" > NUL
	pro32 -p Tone.hex
	cmd /c start putty.exe -serial $(PORTN) -sercfg 115200,8,n,1,N

putty:
	@taskkill /f /im putty.exe /t /fi "status eq running" > NUL
	cmd /c start putty.exe -serial $(PORTN) -sercfg 115200,8,n,1,N

dummy: Tone.hex Tone.map
	$(CC) --version
