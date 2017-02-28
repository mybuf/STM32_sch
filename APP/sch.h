
#include "stm32f10x.h"

struct TASK{
    void (*fp)() ;  // 函数
    u32 td ;       // 用于记录时间
};

#define MAXTASKS 32 

void settimer(char *lc,char line,char tmrid,int d) ;
void dectimers(void) ;
void runtasks(void) ;
