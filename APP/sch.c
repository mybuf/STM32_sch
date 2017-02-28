
#include "sch.h"


struct TASK tasks[MAXTASKS] ;

// ���ö�ʱ��
void settimer(char *lc,char line,char tmrid,int d)
{
    *lc = line;
    tasks[tmrid].td = d ;
}

// ��ʱ������
void dectimers(void)
{
    u32 i ;
    for(i=0;i<MAXTASKS;i++) {
        if(tasks[i].td>0) tasks[i].td--;
    }
}

// ����ִ��
void runtasks(struct TASK tasks[])
{
    u32 i ;
    for(i=0;i<MAXTASKS;i++)
    {
        if(tasks[i].fp!=0)
        {
            if(tasks[i].td==0)
                tasks[i].fp() ;
        }
    }
}
/**********************************************************/
