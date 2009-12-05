#
# This file is subject to the terms and conditions of the GNU General Public
# License.  See the file "COPYING" in the main directory of this archive
# for more details.
#
# Copyright (C) 1995, 1998, 2001, 2002 by Ralf Baechle
# Copyright (C) 2004  Maciej W. Rozycki
#

#
# Some DECstations need all possible sections of an ECOFF executable
#
ifdef CONFIG_MACH_DECSTATION
  E2EFLAGS = -a
else
  E2EFLAGS =
endif

#
# Drop some uninteresting sections in the kernel.
# This is only relevant for ELF kernels but doesn't hurt a.out
#
drop-sections	= .reginfo .mdebug .comment .note .pdr .options .MIPS.options
strip-flags	= $(addprefix --remove-section=,$(drop-sections))

VMLINUX = vmlinux

all: vmlinux.ecoff vmlinux.srec addinitrd

vmlinux.ecoff: $(obj)/elf2ecoff $(VMLINUX)
	$(obj)/elf2ecoff $(VMLINUX) vmlinux.ecoff $(E2EFLAGS)

$(obj)/elf2ecoff: $(obj)/elf2ecoff.c
	$(HOSTCC) -o $@ $^

vmlinux.bin: $(VMLINUX)
	$(OBJCOPY) -O binary $(strip-flags) $(VMLINUX) $(obj)/vmlinux.bin

vmlinux.srec: $(VMLINUX)
	$(OBJCOPY) -S -O srec $(strip-flags) $(VMLINUX) $(obj)/vmlinux.srec

$(obj)/vmlinux.bin.lzma: vmlinux.bin
	lzma -9f $(obj)/vmlinux.bin

$(obj)/ih.o: $(obj)/ih_temp.c $(obj)/vmlinux.bin.lzma
	$(CC) -o $(obj)/ih.o -c $(obj)/ih_temp.c -DIH_TIME=$(shell date -d 'now +8 hour' +'%s') -DIH_RUN=0x80000400 \
		 -DIH_SIZE=$(shell stat -c%s $(obj)/vmlinux.bin.lzma) -DIH_MID=0x1234 -DIH_VER=0x010201

$(obj)/ih.bin: $(obj)/ih.o
	$(OBJCOPY) -j .data -O binary $(obj)/ih.o $(obj)/ih.bin
	rm -f $(obj)/ih.o

kernel.img: $(obj)/ih.bin $(obj)/vmlinux.bin.lzma
	cat $(obj)/ih.bin $(obj)/vmlinux.bin.lzma > kernel.img
		
$(obj)/addinitrd: $(obj)/addinitrd.c
	$(HOSTCC) -o $@ $^

clean-files += addinitrd \
	       elf2ecoff \
	       vmlinux.bin \
	       vmlinux.ecoff \
	       vmlinux.srec
