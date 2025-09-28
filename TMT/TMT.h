/*-----------------------------------------------------------------------
|                            FILE DESCRIPTION                           |
-----------------------------------------------------------------------*/
/*----------------------------------------------------------------------
  - File name     : tmt.h
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
|                              TRANSPLANT                               |
-----------------------------------------------------------------------*/
#ifndef __TMT_H_
#define __TMT_H_

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief     硬件平台相关的头文件，提供硬件运行的基本环境，一般是寄存器头文件。
 * @details   The header files related to the hardware platform provide 
 *            the basic environment for hardware operation, generally 
 *            register header files..
**/
#include     "tim.h"  //STM32版
//#include     "STC8Ax_REG.h"  //51版本

/**
 * @brief     进入TMT临界区宏函数，需要关闭相关定时器的中断。
 * @details   Enter the TMT critical section macro function.
 *            It is necessary to turn off the interrupt of the related timer.
 * @note      需要根据硬件平台，移植合适的关定时器中断代码。
 *            It is necessary to transplant the appropriate off-timer 
 *            interrupt code according to the hardware platform.
**/
#define       ENTER_TMT_CRI_AREA()    do{ __HAL_TIM_DISABLE_IT(&htim14,TIM_IT_UPDATE); }while(0)  //32版本
//#define       ENTER_TMT_CRI_AREA()    do{ IT0 = 0; }while(0) //51版本

/**
 * @brief     退出TMT临界区宏函数，需要打开相关定时器的中断。
 * @details   Enter the TMT critical section macro function.
 *            It is necessary to turn on the interrupt of the related timer.
 * @note      需要根据硬件平台，移植合适的开定时器中断代码。
 *            It is necessary to transplant the appropriate on-timer 
 *            interrupt code according to the hardware platform.
**/
#define       EXTI_TMT_CRI_AREA()     do{ __HAL_TIM_ENABLE_IT(&htim14,TIM_IT_UPDATE); }while(0) //32版本
//#define       EXTI_TMT_CRI_AREA()     do{ IT0 = 1; }while(0) //51版本


/**
 * @brief     TMT的变量类型重定义，如果有冲突，请配置为0。
 * @details   The variable type of TMT is redefined. 
 *            If there is a conflict, please configure it to 0..
**/
#if 		  (0)

	typedef  unsigned char   uint8_t; /*!<  8 bits */
	typedef  unsigned int   uint16_t; /*!< 16 bits */

#endif

/*-----------------------------------------------------------------------
|                                CONFIG                                 |
-----------------------------------------------------------------------*/

/**
 * @brief     任务数量控制宏，用来确定TMT需要调度的最大任务数量。
 * @details   The number of tasks control macro is used to 
 *            determine the maximum number of tasks that TMT
 *            needs to schedule.
**/
#define 	  TMT_TASK_NUM  	 (8)


/*-----------------------------------------------------------------------
|                                 DATA                                  |
-----------------------------------------------------------------------*/


/**
 * @brief     任务的执行状态枚举体。
 * @details   The execution status enumerator of the task.
**/
typedef enum
{
    task_stop = 1,   /*!< 停止运行状态。*/
	task_continue = !task_stop /*!< 继续运行状态。*/
}task_type;


/**
  * @brief   TMT函数自检枚举体，指示函数是否正常执行。
  *          TMT function self checks enumerator. 
  *          indicating whether the function is normally executed.
***/
typedef enum
{
	tmt_nok= 1,     /* Function self checking failed */
	tmt_ok = !tmt_nok /* Function self checking successfed */ 
}tmt_fun_t; 


/*-----------------------------------------------------------------------
|                             API FUNCTION                              |
-----------------------------------------------------------------------*/

/**
 * @brief   TMT的API操作函数结构体。
 * @details The TMT API manipulates the function structure.
**/
typedef struct
{
	/**
	 * @brief     TMT更新任务调度tick时间。
	 * @details   TMT update task schedule tick time.
	 * @param[in] None.
	 * @return    None.
	 * @note      在定时器中断服务函数中调用。
	 *            Called in the timer interrupt service function.
	 * @example   tmt.tick();
	**/
	void (*tick) (void);
	
	
	/**
	 * @brief     TMT运行函数，是任务调度器。
	 * @details   TMT runs the function, which is the task scheduler.
	 * @param[in] None.
	 * @return    None.
	 * @note      在main函数的无限循环中调用。
	 *            Called in the infinite loop of main.
	 * @example   tmt.run();
	**/
	void (*run) (void);
	
	
	/**
	 * @brief     TMT创建一个任务函数。
	 * @details   TMT creates a task function.
	 * @param[in] entry 任务函数入口指针.
	 *            A pointer function without formal parameters.
	 * @param[in] setTime 任务调度的时间 task run time (ticks)
	 * @return    [tmt_ok]创建成功.Create successfully.
	 * @return    [tmt_nok]创建失败.	Create failed.
	 * @note      在main函数的无限循环中调用。
	 *            Called in the infinite loop of main.
	 * @example   tmt.create(enrty,500); //定时entry任务为500ticks调度一次
	**/	
	tmt_fun_t (*create) (void (*entry) (void), uint16_t set_time);
	
	
	/**
	 * @brief     TMT删除一个任务函数。
	 * @details   TMT creates a task function.
	 * @param[in] entry 任务函数入口指针.
	 *            A pointer function without formal parameters.
	 * @return    [tmt_ok]删除成功.Delete successfully.
	 * @return    [tmt_nok]删除失败.	Delete failed.
	 * @example   tmt.delete(enrty); //删除entry任务
	**/		
	tmt_fun_t (*delete) (void (*entry) (void));
	
	
	/**
	 * @brief     TMT控制一个任务的调度时间函数。
	 * @details   The TMT controls the scheduling time function of a task.
	 * @param[in] entry 任务函数入口指针.
	 *            A pointer function without formal parameters.
	 * @param[in] setTime 任务调度的时间 task run time (ticks)
	 * @return    [tmt_ok]修改成功.The modification was successful.
	 * @return    [tmt_nok]修改失败.The modification was failed.
	 * @example   tmt.time_ctrl(enrty,100); //修改entry任务为100ticks调度一次
	**/	
	tmt_fun_t (*time_ctrl) (void (*entry) (void), uint16_t set_time);
	
	
	/**
	 * @brief     TMT控制一个任务运行状态函数。
	 * @details   TMT controls a task running state function.
	 * @param[in] entry 任务函数入口指针.
	 *            A pointer function without formal parameters.
	 * @param[in] state 任务运行状态.Task running status
	 * @return    [tmt_ok]修改成功.The modification was successful.
	 * @return    [tmt_nok]修改失败.The modification was failed.
	 * @example   tmt.run_ctrl(enrty,task_stop); //修改entry任务为停止运行.
	**/		
	tmt_fun_t (*run_ctrl) (void (*entry) (void), task_type state);
}tmt_class_public;


/**
 * @brief   TMT初始化函数，注册相关元素。
 * @details The TMT initializes the function 
 *          and registers the associated elements.
 * @param   None.
 * @return  None.
**/
extern void tmt_init(void);

/**
 * @brief   对外声明的TMT操作函数结构体。
 * @details TMT operation function structure declared externally.
 * @note    所有需要被外部调用的函数,都包含在这个结构体中.
 *          All functions that need to be called externally 
 *          are included in this structure.
**/
extern tmt_class_public tmt;

#ifdef __cplusplus
}
#endif

#endif
/*-----------------------------------------------------------------------
|          END OF FLIE        (C) COPYRIGHT zeweni                      | 
-----------------------------------------------------------------------*/

