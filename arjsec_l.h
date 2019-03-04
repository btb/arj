/*
 * $Id: arjsec_l.h,v 1.2 2007/04/19 20:01:11 andrew_belov Exp $
 * ---------------------------------------------------------------------------
 * Prototypes of the functions located in ARJSEC_L.C are declared here.
 *
 */

#ifndef ARJSEC_L_INCLUDED
#define ARJSEC_L_INCLUDED

/* Prototypes */

int create_envelope(FILE *stream, unsigned long offset, int iter);

void arjsec_term(uint32_t *block, uint32_t *dest, int iter);
void arjsec_xor(uint32_t *dest, uint32_t *src);
void arjsec_newblock(uint32_t *dest);
void arjsec_invert(uint32_t *block);
void arjsec_crcterm(uint32_t *block, unsigned char c);
void arjsec_read(uint32_t *block, FILE *stream, unsigned long len);

void rev_arjsec_term(uint32_t *block, uint32_t *dest, int iter);
void arjsec_revert(uint32_t *block);

#endif

