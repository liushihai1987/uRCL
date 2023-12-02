#include "task.h"
#include "stm32f3xx_hal.h"




//void TaskDispStatus(void){}//预留函数，使用时要放在main函数里，这里删除
void TaskKeySan(void){}
void TaskSendData(void){}

/**************************************************************************************
* Variable definition ����������
* 任务列表，列出每个任务函数名，写出运行周期。
* Run;Timer; ItvTime;
**************************************************************************************/
static TASK_COMPONENTS TaskComps[] =
{
   {0, 200, 200, TaskSendData},            // ��ʾʱ��
   {0, 20, 20, TaskKeySan},               // ����ɨ��
   {0, 500, 500, TaskDispStatus},            // ��ʾ����״???

    // ��������������?????????
};



/**************************************************************************************
* FunctionName   : TaskRemarks()
* Description    : �����־����
* EntryParameter : None
* ReturnValue    : None
**************************************************************************************/
void TaskRemarks(void)
{
 uint32_t i;

   for (i=0; i<TASKS_MAX; i++)          // �������ʱ�䴦��
   {
        if (TaskComps[i].Timer)          // ʱ�䲻Ϊ0
       {
           TaskComps[i].Timer--;         // ��ȥ???����???
           if (TaskComps[i].Timer == 0)       // ʱ�����???
           {
                TaskComps[i].Timer = TaskComps[i].ItvTime;       // �ָ���ʱ��???��������һ???
                TaskComps[i].Run = 1;           // �����������
           }
       }
  }
}

/**************************************************************************************
* FunctionName   : TaskProcess()
* Description    : ������
* EntryParameter : None
* ReturnValue    : None
**************************************************************************************/
void TaskProcess(void)
{
 uint32_t i;

   for (i=0; i<TASKS_MAX; i++)           // �������ʱ�䴦��
   {
        if (TaskComps[i].Run)           // ʱ�䲻Ϊ0
       {
            TaskComps[i].TaskHook();         // ��������
            TaskComps[i].Run = 0;          // ��־???0
       }
   }
}
