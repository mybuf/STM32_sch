
#include "stm32f10x.h"

struct TASK{
    u32 td ;       // ���ڼ�¼ʱ��
    void (*fp)() ;  // ����
};

#define MAXTASKS 32 

void settimer(char *lc,char line,char tmrid,int d) ;
void dectimers(void) ;
void runtasks(void) ;