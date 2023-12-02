#ifndef TASK_H
#define TASK_H
#include "main.h"

// 任务结构
typedef struct _TASK_COMPONENTS
{
 uint32_t Run;                 // 程序运行标记�??0-不运行，1运行
 uint32_t Timer;              // 计时�??
 uint32_t ItvTime;              // 任务运行间隔时间
    void (*TaskHook)(void);    // 要运行的任务函数
} TASK_COMPONENTS;       // 任务定义

// 任务清单
typedef enum _TASK_LIST
{
	TASK_SEND_DATA,            // 显示时钟
    TAST_KEY_SAN,             // 按键扫描
    TASK_DISP_WS,             // 工作状�?�显�??
     // 这里添加你的任务。�?��?��??
     TASKS_MAX                                           // 总的可供分配的定时任务数�??
} TASK_LIST;

//任务函数直接写在main函数里
void TaskDispStatus(void);
void TaskKeySan(void);
void TaskSendData(void);



void TaskRemarks(void);//这个函数在1ms的定时器中断里
void TaskProcess(void);//这个函数在while循环里


#endif


