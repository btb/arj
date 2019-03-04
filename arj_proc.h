/*
 * $Id: arj_proc.h,v 1.7 2007/04/19 18:42:09 andrew_belov Exp $
 * ---------------------------------------------------------------------------
 * Prototypes of the functions located in ARJ_PROC.C are declared here.
 *
 */

#ifndef ARJ_PROC_INCLUDED
#define ARJ_PROC_INCLUDED

#include "c99int.h"

/* Helper macros */

#define mget_byte(p) (*(uint8_t FAR *)(p)&0xFF)
#define mput_byte(c, p) *(uint8_t FAR *)(p)=(uint8_t)(c)
#if !defined(ALIGN_POINTERS) && !defined(WORDS_BIGENDIAN)
#define mget_word(p) (*(uint16_t *)(p)&0xFFFF)
#define mput_word(w,p) (*(uint16_t *)(p)=(uint16_t)(w))
#define mget_dword(p) (*(uint32_t *)(p))
#define mput_dword(w,p) (*(uint32_t *)(p)=(uint32_t)(w))
#endif

/* Prototypes */

void garble_encode_stub(char *data, int len);
void garble_decode_stub(char *data, int len);
void append_curtime_proc();
void add_pathsep(char *path);
void trim_pathsep(char *path, int leave_root_paths);
void convert_time_limits();
void copy_bytes(unsigned long nbytes);
int translate_path(char *name);
void restart_proc(char *dest);
int search_for_extension(char *name, char *ext_list);
unsigned long get_volfree(unsigned long increment);
unsigned int check_multivolume(unsigned int increment);
void store();
void hollow_encode();
void convert_strtime(struct timestamp *dest, char *str);
int check_integrity(char *name);
void name_to_hdr(char *name);
char *format_filename(char *name);
void analyze_arg(char *sw);
void init();
int is_switch(char *arg);
#if SFX_LEVEL>=ARJSFXV||defined(REARJ)
int split_name(char *name, char *pathname, char *filename);
#else
int split_name(char *name);
#endif
int subclass_errors(FMSG *errmsg);
void file_seek(FILE *stream, long offset, int whence);
void search_setup();
int calc_percentage(unsigned long partial, unsigned long total);
void smart_seek(unsigned long position, FILE *stream);
void alltrim(char *cmd);
void unstore(int action);
void hollow_decode(int action);
void pack_mem(struct mempack *mempack);
void unpack_mem(struct mempack *mempack);
void strip_lf(char *str);
char *ltrim(char *str);
#if defined(ALIGN_POINTERS) || defined(WORDS_BIGENDIAN)
uint16_t mget_word(char FAR *p);
uint32_t mget_dword(char FAR *p);
void mput_word(uint16_t w, char FAR *p);
void mput_dword(uint32_t d, char FAR *p);
#endif

#endif

