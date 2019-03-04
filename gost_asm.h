/*
 * $Id: gost_asm.h,v 1.2 2007/04/19 20:01:15 andrew_belov Exp $
 * ---------------------------------------------------------------------------
 * Interface to the assembly module, GOST_ASM.ASM
 *
 */

#ifndef GOST_ASM_INCLUDED
#define GOST_ASM_INCLUDED

/* Prototypes */

uint32_t gost_term_32(uint32_t *src, uint32_t *dest, uint32_t *key);

#endif

