#ifndef RC_SNIFF_H
#define RC_SNIFF_H
#include <stdint.h>
void RcSniff_Init(void);
extern volatile uint32_t g_sniff[520];
#endif
