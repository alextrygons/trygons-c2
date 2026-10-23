/*
 * memplace.h - Where data shared between the cores lives (TC375).
 *
 * Data that one core writes and another reads (the CPU1 monitor block, the
 * log queues, the logger status, the timing statistics) must not sit in a
 * CACHED memory: the reader could see stale values. The LMU (cpu0/1/2_dlmu)
 * is linked at its cached address (0x9...), the cores' data scratchpads
 * (DSPR) are never cached. So shared data goes explicitly into core 0's
 * DSPR (section bss_cpu0, Lcf_Tasking_Tricore_Tc.lsl / Lcf_Gnuc_Tricore_Tc.lsl);
 * the other cores reach it through its global address (0x7...).
 * Only zero-initialised data (bss). Checked by tools/mapreport.py (objects:
 * the section of every shared symbol; ADS map: never at a 0x9... address).
 * The TASKING pragma spelling is the one of the black box in logger.c.
 */
#ifndef MEMPLACE_H
#define MEMPLACE_H

#if defined(__TASKING__)
#define SHARED_BEGIN() _Pragma("section farbss bss_cpu0")
#define SHARED_END()   _Pragma("section farbss restore")
#elif defined(__tricore__)
#define SHARED_BEGIN() _Pragma("section .bss_cpu0 aw 4")
#define SHARED_END()   _Pragma("section")
#else                                   /* PC simulation: one address space */
#define SHARED_BEGIN()
#define SHARED_END()
#endif

#endif /* MEMPLACE_H */
