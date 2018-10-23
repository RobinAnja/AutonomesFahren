################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
..\dbsct.c \
..\hwsetup.c \
..\intprg.c \
..\kit12_rx62t.c \
..\resetprg.c \
..\sbrk.c \
..\vecttbl.c 

OBJS += \
./dbsct.obj \
./hwsetup.obj \
./intprg.obj \
./kit12_rx62t.obj \
./resetprg.obj \
./sbrk.obj \
./vecttbl.obj 

C_DEPS += \
./dbsct.d \
./hwsetup.d \
./intprg.d \
./kit12_rx62t.d \
./resetprg.d \
./sbrk.d \
./vecttbl.d 


# Each subdirectory must supply rules for building sources it contributes
%.obj: ../%.c %.d
	@echo 'Building file: $<'
	@echo 'Invoking: Compiler'
	ccrx -output=obj=$(dir $@)$(basename $(notdir $@)).obj -include=C:\Renesas\e2studio\Tools\Renesas\RX\1_2_1\include -debug -nologo -section=L=C -nologo -section=L=C -cpu=rx600 -endian=big  -lang=c "$<"
	@echo 'Finished building: $<'
	@echo.


# Each subdirectory must supply rules for scanning sources it contributes
%.d: ../%.c
	@echo 'Scanning started: $<'
	scandep1 -MM -MP -MF  "$(@:%.obj=%.d)" -MT  "$(@:%.obj=%.d)"  -IC:\Renesas\e2studio\Tools\Renesas\RX\1_2_1\include -D__RX600=1 -D__BIG=1   -D__FPU=1  -D__RON=1 -D__UCHAR=1 -D__DBL4=1 -D__UBIT=1 -D__BITRIGHT=1 -D__DOFF=1   -D__RENESAS__=1 -D__RENESAS_VERSION__=0x010201 -D__RX=1   -E -quiet -I. -C $<
	@echo 'Finished scanning: $<'
	@echo.

