/*-----------------------------------------------------------------------
|                            FILE DESCRIPTION                           |
-----------------------------------------------------------------------*/
/*----------------------------------------------------------------------
  - File name     : tmt.c
  - Author        : zeweni
  - Update date   : 2020.09.28                  
  -	Copyright(c)  : 2020-2021 zeweni. All rights reserved.
-----------------------------------------------------------------------*/
/*------------------------------------------------------------------------
|                            COPYRIGHT NOTICE                            |
------------------------------------------------------------------------*/
/*
 * Copyright (C) 2021, zeweni (17870070675@163.com)

 * This file is part of task management time slice framework.
 * Abbreviated as TMT.
 
 * Task management time slice framework is free software: you can redist-
 * ribute it and/or modify it under the terms of the Apache-2.0 License.

 * Task management time slice framework is distributed in the hope that it 
 * will be useful,but WITHOUT ANY WARRANTY; without even the implied  
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the 
 * Apache-2.0 License for more details.

 * You should have received a copy of the Apache-2.0 License.Task management 
 * time slice framework. If not, see <http://www.apache.org/licenses/>.
**/
/*-----------------------------------------------------------------------
|                               INCLUDES                                |
-----------------------------------------------------------------------*/
#include "tmt.h"
/*-----------------------------------------------------------------------
|                                 DATA                                  |
-----------------------------------------------------------------------*/

/**
 * @brief     最大任务数量宏，再封装一层。
 * @details   The maximum number of tasks is a macro, 
 *            and then one layer is encapsulated.
**/
#define  TASKS_MAX       TMT_TASK_NUM


/**
 * @brief   定义TMT相关操作函数结构体。
 * @details Define the structure of the TMT related operation functions.
**/
tmt_class_public tmt;


/**
 * @brief   TMT的类，包含公共和私有两个属性。
 * @details Class of TMT, containing both pbl and pvt properties.
**/
static struct class
{
	tmt_class_public pbl;  /*!< 公共部分，对外开放的属性，具体定义在头文件。*/
	
	struct pvt   /*!< 私有部分，仅限本文件内调用。*/
	{
		struct
		{
            void (* entry) (void); /*!< 任务函数入口指针。*/
            uint16_t tim_cnt;     /*!< 任务调度时间计数器。*/
            uint16_t set_time;      /*!< 任务设置的调度时间。*/
            uint8_t state;         /*!< 任务运行状态。*/
		}comp[TASKS_MAX];
		
		uint8_t num;   /*!< 已经创建的任务数量。*/
	}pvt;
	
}object;

/*-----------------------------------------------------------------------
|                               FUNCTION                                |
-----------------------------------------------------------------------*/

/**
 * @brief     TMT更新任务调度tick时间。
 * @details   TMT update task schedule tick time.
 * @param[in] None.
 * @return    None.
**/
static void tmt_update_tick(void)
{
	uint8_t i;
	for (i = 0; i< object.pvt.num; i++) {
		 /* If time arrives */
        if (object.pvt.comp[i].tim_cnt > 0) {
            object.pvt.comp[i].tim_cnt--;
        }
	}	
}

/**
 * @brief     TMT运行函数，是任务调度器。
 * @details   TMT runs the function, which is the task scheduler.
 * @param[in] None.
 * @return    None.
**/
static void tmt_run(void)
{
	uint8_t index;
	for (index = 0; index < object.pvt.num; index++) {
		 /* If time arrives */
		if (object.pvt.comp[index].tim_cnt == 0 && object.pvt.comp[index].state != task_stop) {
			object.pvt.comp[index].tim_cnt = object.pvt.comp[index].set_time;
			
			/* 
				To prevent process conflict, 
				only one process can be executed in the same time period.
			*/
			if (object.pvt.comp[index].entry != (void*)(0)) {
				object.pvt.comp[index].entry();  /* Run task */
			}
		}
	}
}


/**
 * @brief     TMT创建一个任务函数。
 * @details   TMT creates a task function.
 * @param[in] entry 任务函数入口指针.
 *            A pointer function without formal parameters.
 * @param[in] set_time 任务调度的时间 task run time (ticks)
 * @return    [tmt_ok]创建成功.Create successfully.
 * @return    [tmt_nok]创建失败.	Create failed.
**/	
static tmt_fun_t tmt_create(void (*entry) (void),uint16_t set_time)
{	
	if (object.pvt.num < TASKS_MAX) {
		object.pvt.comp[object.pvt.num].entry = entry;
		object.pvt.comp[object.pvt.num].set_time = set_time;
		object.pvt.comp[object.pvt.num].tim_cnt = set_time;
		object.pvt.comp[object.pvt.num].state = task_continue;
		object.pvt.num++;
		return tmt_ok;
	} else {
		return tmt_nok;	
	}		
}


/**
 * @brief     TMT删除一个任务函数。
 * @details   TMT creates a task function.
 * @param[in] entry 任务函数入口指针.
 *            A pointer function without formal parameters.
 * @return    [tmt_ok]删除成功.Delete successfully.
 * @return    [tmt_nok]删除失败.	Delete failed.
**/	
static tmt_fun_t tmt_delete(void (*entry) (void))
{	
    uint8_t index;
    if (object.pvt.num > 0) {
        ENTER_TMT_CRI_AREA();
		
        for(index = 0; index < object.pvt.num; index++) {
            if(object.pvt.comp[index].entry == entry) {
                    object.pvt.num--;
                    object.pvt.comp[index].entry = object.pvt.comp[object.pvt.num].entry;
                    object.pvt.comp[index].set_time = object.pvt.comp[object.pvt.num].set_time;
                    object.pvt.comp[index].tim_cnt = object.pvt.comp[object.pvt.num].tim_cnt;
                    object.pvt.comp[index].state = object.pvt.comp[object.pvt.num].state;
                    EXTI_TMT_CRI_AREA();
                    return tmt_ok;
            }
        }
    }
    EXTI_TMT_CRI_AREA();
    return tmt_nok;
}


/**
 * @brief     TMT控制一个任务运行状态函数。
 * @details   TMT controls a task running state function.
 * @param[in] entry 任务函数入口指针.
 *            A pointer function without formal parameters.
 * @param[in] state 任务运行状态.Task running status
 * @return    [tmt_ok]修改成功.The modification was successful.
 * @return    [tmt_nok]修改失败.The modification was failed.
**/	
static tmt_fun_t tmt_run_ctrl(void (*entry)(void),task_type state)
{
	uint8_t index;
	for (index = 0; index<object.pvt.num; index++) {
		if (object.pvt.comp[index].entry == entry) {
		    object.pvt.comp[index].state = state;
			return tmt_ok;
		}
	}
	return tmt_nok;
}


/**
 * @brief     TMT控制一个任务的调度时间函数。
 * @details   The TMT controls the scheduling time function of a task.
 * @param[in] *taskFunc (void) 任务函数入口指针.
 *            A pointer function without formal parameters.
 * @param[in] set_time 任务调度的时间 task run time (ticks)
 * @return    [tmt_ok]修改成功.The modification was successful.
 * @return    [tmt_nok]修改失败.The modification was failed.
**/	
static tmt_fun_t tmt_time_ctrl(void (*entry) (void),uint16_t set_time)
{	
	uint8_t index;
	for(index = 0; index< object.pvt.num; index++) {
		if (object.pvt.comp[index].entry == entry) {
			object.pvt.comp[index].tim_cnt = set_time;
			object.pvt.comp[index].set_time = set_time;
			return tmt_ok;
		}
	}
	return tmt_nok;
}


/**
 * @brief   TMT初始化函数，注册相关元素。
 * @details The TMT initializes the function 
 *          and registers the associated elements.
 * @param   None.
 * @return  None.
**/
void tmt_init(void)
{
	object.pbl.run = tmt_run;
	object.pbl.tick = tmt_update_tick;
	object.pbl.create = tmt_create;
	object.pbl.delete = tmt_delete;
	object.pbl.time_ctrl = tmt_time_ctrl;
	object.pbl.run_ctrl = tmt_run_ctrl;
	object.pvt.num = 0;
	tmt = object.pbl;
}

/*-----------------------------------------------------------------------
|                   END OF FLIE.  (C) COPYRIGHT zeweni                  |
-----------------------------------------------------------------------*/


