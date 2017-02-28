
#include "sch.h"


struct TASK tasks[MAXTASKS] ;

// 设置定时器
void settimer(char *lc,char line,char tmrid,int d)
{
    *lc = line;
    tasks[tmrid].td = d ;
}

// 定时器处理
void dectimers(void)
{
    u32 i ;
    for(i=0;i<MAXTASKS;i++) {
        if(tasks[i].td>0) tasks[i].td--;
    }
}

// 任务执行
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
