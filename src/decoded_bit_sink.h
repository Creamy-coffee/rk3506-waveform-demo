#ifndef V32_DECODED_BIT_SINK_H
#define V32_DECODED_BIT_SINK_H
/* 0/1 are actual recovered symbols; -1 is an erasure or continuity break. */
typedef void (*decoded_bit_sink_t)(void *context,int bit);
#endif
