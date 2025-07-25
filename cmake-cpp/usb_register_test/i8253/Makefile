# Makefile for the linux kernel.
#
 
extra-y                := head_$(BITS).o head$(BITS).o head.o vmlinux.lds
 
CPPFLAGS_vmlinux.lds += -U$(UTS_MACHINE)
 
ifdef CONFIG_FUNCTION_TRACER
# Do not profile debug and lowlevel utilities
CFLAGS_REMOVE_tsc.o = -pg
CFLAGS_REMOVE_paravirt-spinlocks.o = -pg
CFLAGS_REMOVE_pvclock.o = -pg
CFLAGS_REMOVE_kvmclock.o = -pg
CFLAGS_REMOVE_ftrace.o = -pg
CFLAGS_REMOVE_early_printk.o = -pg
endif
 
KASAN_SANITIZE_head$(BITS).o := n
KASAN_SANITIZE_dumpstack.o := n
KASAN_SANITIZE_dumpstack_$(BITS).o := n
 
CFLAGS_irq.o := -I$(src)/../include/asm/trace
 
obj-y                   := process_$(BITS).o signal.o entry_$(BITS).o
obj-y                   += traps.o irq.o irq_$(BITS).o dumpstack_$(BITS).o
obj-y                   += time.o ioport.o ldt.o dumpstack.o nmi.o
obj-y                   += setup.o x86_init.o i8259.o irqinit.o jump_label.o
obj-$(CONFIG_IRQ_WORK)  += irq_work.o
obj-y                   += probe_roms.o
obj-$(CONFIG_X86_32)    += i386_ksyms_32.o
obj-$(CONFIG_X86_64)    += sys_x86_64.o x8664_ksyms_64.o
obj-$(CONFIG_X86_64)    += mcount_64.o
obj-y                   += syscall_$(BITS).o vsyscall_gtod.o
obj-$(CONFIG_IA32_EMULATION)    += syscall_32.o
obj-$(CONFIG_X86_VSYSCALL_EMULATION)    += vsyscall_64.o vsyscall_emu_64.o
obj-$(CONFIG_X86_ESPFIX64)      += espfix_64.o
obj-$(CONFIG_SYSFS)     += ksysfs.o
obj-y                   += bootflag.o e820.o
obj-y                   += pci-dma.o quirks.o topology.o kdebugfs.o
obj-y                   += alternative.o i8253.o pci-nommu.o hw_breakpoint.o
obj-y                   += tsc.o tsc_msr.o io_delay.o rtc.o
obj-y                   += pci-iommu_table.o
obj-y                   += resource.o
 
obj-y                           += process.o
obj-y                           += i387.o xsave.o
obj-y                           += ptrace.o
obj-$(CONFIG_X86_32)            += tls.o
obj-$(CONFIG_IA32_EMULATION)    += tls.o
obj-y                           += step.o
obj-$(CONFIG_INTEL_TXT)         += tboot.o
obj-$(CONFIG_ISA_DMA_API)       += i8237.o
obj-$(CONFIG_STACKTRACE)        += stacktrace.o
obj-y                           += cpu/
obj-y                           += acpi/
obj-y                           += reboot.o
obj-$(CONFIG_X86_MSR)           += msr.o
obj-$(CONFIG_X86_CPUID)         += cpuid.o
obj-$(CONFIG_PCI)               += early-quirks.o
apm-y                           := apm_32.o
obj-$(CONFIG_APM)               += apm.o
obj-$(CONFIG_SMP)               += smp.o
obj-$(CONFIG_SMP)               += smpboot.o
obj-$(CONFIG_SMP)               += tsc_sync.o
obj-$(CONFIG_SMP)               += setup_percpu.o
obj-$(CONFIG_X86_MPPARSE)       += mpparse.o
obj-y                           += apic/
obj-$(CONFIG_X86_REBOOTFIXUPS)  += reboot_fixups_32.o
obj-$(CONFIG_DYNAMIC_FTRACE)    += ftrace.o
obj-$(CONFIG_LIVEPATCH)         += livepatch.o
obj-$(CONFIG_FUNCTION_GRAPH_TRACER) += ftrace.o
obj-$(CONFIG_FTRACE_SYSCALLS)   += ftrace.o
obj-$(CONFIG_X86_TSC)           += trace_clock.o
obj-$(CONFIG_KEXEC)             += machine_kexec_$(BITS).o
obj-$(CONFIG_KEXEC)             += relocate_kernel_$(BITS).o crash.o
obj-$(CONFIG_KEXEC_FILE)        += kexec-bzimage64.o
obj-$(CONFIG_CRASH_DUMP)        += crash_dump_$(BITS).o
obj-y                           += kprobes/
obj-$(CONFIG_MODULES)           += module.o
obj-$(CONFIG_DOUBLEFAULT)       += doublefault.o
obj-$(CONFIG_KGDB)              += kgdb.o
obj-$(CONFIG_VM86)              += vm86_32.o
obj-$(CONFIG_EARLY_PRINTK)      += early_printk.o
 
obj-$(CONFIG_HPET_TIMER)        += hpet.o
obj-$(CONFIG_APB_TIMER)         += apb_timer.o
 
obj-$(CONFIG_AMD_NB)            += amd_nb.o
obj-$(CONFIG_DEBUG_RODATA_TEST) += test_rodata.o
obj-$(CONFIG_DEBUG_NX_TEST)     += test_nx.o
obj-$(CONFIG_DEBUG_NMI_SELFTEST) += nmi_selftest.o
 
obj-$(CONFIG_KVM_GUEST)         += kvm.o kvmclock.o
obj-$(CONFIG_PARAVIRT)          += paravirt.o paravirt_patch_$(BITS).o
obj-$(CONFIG_PARAVIRT_SPINLOCKS)+= paravirt-spinlocks.o
obj-$(CONFIG_PARAVIRT_CLOCK)    += pvclock.o
obj-$(CONFIG_X86_PMEM_LEGACY)   += pmem.o
 
obj-$(CONFIG_PCSPKR_PLATFORM)   += pcspeaker.o
obj-$(CONFIG_I8253_REFRESH)     += i8253_ref_setup.o
 
obj-$(CONFIG_X86_CHECK_BIOS_CORRUPTION) += check.o
 
obj-$(CONFIG_SWIOTLB)                   += pci-swiotlb.o
obj-$(CONFIG_OF)                        += devicetree.o
obj-$(CONFIG_UPROBES)                   += uprobes.o
obj-y                                   += sysfb.o
obj-$(CONFIG_X86_SYSFB)                 += sysfb_simplefb.o
obj-$(CONFIG_EFI)                       += sysfb_efi.o
 
obj-$(CONFIG_PERF_EVENTS)               += perf_regs.o
obj-$(CONFIG_TRACING)                   += tracepoint.o
obj-$(CONFIG_IOSF_MBI)                  += iosf_mbi.o
obj-$(CONFIG_PMC_ATOM)                  += pmc_atom.o
 
###
# 64 bit specific files
ifeq ($(CONFIG_X86_64),y)
        obj-$(CONFIG_AUDIT)             += audit_64.o
 
        obj-$(CONFIG_GART_IOMMU)        += amd_gart_64.o aperture_64.o
        obj-$(CONFIG_CALGARY_IOMMU)     += pci-calgary_64.o tce_64.o
 
        obj-$(CONFIG_PCI_MMCONFIG)      += mmconf-fam10h_64.o
        obj-y                           += vsmp_64.o
endif