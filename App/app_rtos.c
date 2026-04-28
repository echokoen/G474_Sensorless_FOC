/*
 * RTOS 应用层。
 */
#include "app_rtos.h"

#include "FreeRTOS.h"
#include "task.h"

#include "app_debug.h"
#include "app_foc.h"
#include "bsp_key.h"

/*
 * 任务优先级。
 */
#define FOC_MEDIUM_TASK_PRIORITY    (6u)    /* FOC 中频任务优先级最高，负责 1ms 状态机和启动流程推进。 */
#define KEY_TASK_PRIORITY           (3u)    /* 按键任务优先级中等，负责按键扫描和启停请求处理。 */
#define DEBUG_TASK_PRIORITY         (1u)    /* 调试任务优先级最低，避免 printf 影响控制任务。 */

/*
 * 任务栈大小。
 */
#define FOC_MEDIUM_TASK_STACK_WORDS (768u)  /* FOC 中频任务栈，单位是 word，768 words 约等于 3072 bytes。 */
#define KEY_TASK_STACK_WORDS        (256u)  /* 按键任务栈，单位是 word，256 words 约等于 1024 bytes。 */
#define DEBUG_TASK_STACK_WORDS      (1024u) /* 调试任务栈，单位是 word，1024 words 约等于 4096 bytes。 */
#define IDLE_TASK_STACK_WORDS       (configMINIMAL_STACK_SIZE) /* FreeRTOS 空闲任务栈，使用内核最小栈配置。 */

/*
 * 任务控制块和栈。
 */
static StaticTask_t s_focMediumTaskTcb;                                      /* FOC 中频任务控制块，用于静态创建 FocMediumTask。 */
static StackType_t s_focMediumTaskStack[FOC_MEDIUM_TASK_STACK_WORDS];        /* FOC 中频任务栈空间，由 FreeRTOS 调度器使用。 */

static StaticTask_t s_keyTaskTcb;                                            /* 按键任务控制块，用于静态创建 KeyTask。 */
static StackType_t s_keyTaskStack[KEY_TASK_STACK_WORDS];                     /* 按键任务栈空间，用于按键扫描和启停请求处理。 */

static StaticTask_t s_debugTaskTcb;                                          /* 调试任务控制块，用于静态创建 DebugTask。 */
static StackType_t s_debugTaskStack[DEBUG_TASK_STACK_WORDS];                 /* 调试任务栈空间，预留较大栈给 printf 和浮点格式化。 */

static StaticTask_t s_idleTaskTcb;                                           /* FreeRTOS IdleTask 控制块，静态内存模式下必须由用户提供。 */
static StackType_t s_idleTaskStack[IDLE_TASK_STACK_WORDS];                   /* FreeRTOS IdleTask 栈空间，静态内存模式下必须由用户提供。 */

/*
 * 任务函数声明。
 */
static void FocMediumTask(void *argument);                                   /* FOC 中频任务函数，周期 1ms。 */
static void KeyTask(void *argument);                                         /* 按键任务函数，周期 5ms。 */
static void DebugTask(void *argument);                                       /* 调试任务函数，周期 20ms。 */

/*
 * 启动 RTOS。
 */
void AppRtos_Start(void)
{
  (void)xTaskCreateStatic(FocMediumTask,                                     /* 创建 FOC 中频任务。 */
                          "FocMedium",                                      /* 任务名称，方便调试器或 RTOS 运行信息查看。 */
                          FOC_MEDIUM_TASK_STACK_WORDS,                      /* 任务栈大小，单位是 word，不是 byte。 */
                          NULL,                                             /* 任务参数，当前 FOC 中频任务不需要参数。 */
                          FOC_MEDIUM_TASK_PRIORITY,                         /* 任务优先级，当前用户任务中最高。 */
                          s_focMediumTaskStack,                             /* 静态任务栈空间。 */
                          &s_focMediumTaskTcb);                             /* 静态任务控制块。 */

  (void)xTaskCreateStatic(KeyTask,                                           /* 创建按键任务。 */
                          "Key",                                            /* 任务名称，方便调试和栈溢出定位。 */
                          KEY_TASK_STACK_WORDS,                             /* 任务栈大小，单位是 word。 */
                          NULL,                                             /* 任务参数，当前按键任务不需要参数。 */
                          KEY_TASK_PRIORITY,                                /* 任务优先级，低于 FOC 中频任务，高于调试任务。 */
                          s_keyTaskStack,                                   /* 静态任务栈空间。 */
                          &s_keyTaskTcb);                                   /* 静态任务控制块。 */

  (void)xTaskCreateStatic(DebugTask,                                         /* 创建调试任务。 */
                          "Debug",                                          /* 任务名称，方便调试器查看任务状态。 */
                          DEBUG_TASK_STACK_WORDS,                           /* 任务栈大小，调试打印可能使用较多栈。 */
                          NULL,                                             /* 任务参数，当前调试任务不需要参数。 */
                          DEBUG_TASK_PRIORITY,                              /* 任务优先级最低，避免影响控制任务。 */
                          s_debugTaskStack,                                 /* 静态任务栈空间。 */
                          &s_debugTaskTcb);                                 /* 静态任务控制块。 */

  vTaskStartScheduler();                                                     /* 启动 FreeRTOS 调度器，正常情况下该函数不会返回。 */
}

/*
 * FOC 中频任务。
 */
static void FocMediumTask(void *argument)
{
  TickType_t lastWakeTime;                                                   /* 周期任务基准 tick，用于 vTaskDelayUntil() 保持 1ms 固定节拍。 */

  (void)argument;                                                            /* 当前任务未使用参数，避免编译器未使用参数警告。 */

  AppFoc_Start();                                                            /* 保持移植前行为：RTOS 启动后自动进入 FOC 启动流程。 */

  lastWakeTime = xTaskGetTickCount();                                        /* 获取当前 tick，作为后续 1ms 周期调度的起始基准。 */

  for (;;)
  {
    AppFoc_MediumFreqTask();                                                 /* FOC 中频状态机任务，负责启动流程、状态切换、故障处理和速度环节拍。 */

    vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(1u));                       /* 固定 1ms 周期运行，避免 vTaskDelay() 带来的累计误差。 */
  }
}

/*
 * 按键任务。
 */
static void KeyTask(void *argument)
{
  TickType_t lastWakeTime;                                                   /* 按键任务周期基准 tick，用于保持 5ms 扫描周期。 */

  (void)argument;                                                            /* 当前任务未使用参数，避免编译器未使用参数警告。 */

  lastWakeTime = xTaskGetTickCount();                                        /* 获取当前 tick，作为按键任务周期运行的起始基准。 */

  for (;;)
  {
    BSP_KEY_Task();                                                          /* 普通按键扫描任务，例如目标速度增加、目标速度降低等。 */

    if (BSP_KEY_ConsumeStartStopRequest() != 0u)                             /* 读取并清除 EXTI 中断上报的 Start/Stop 请求。 */
    {
      if (AppFoc_GetState() == FOC_STATE_IDLE)                               /* 当前处于 IDLE 状态，说明电机未运行，可以执行启动。 */
      {
        AppFoc_Start();                                                      /* 在任务上下文中启动 FOC，避免在 EXTI 中断里直接操作 ADC/TIM/PWM。 */
      }
      else
      {
        AppFoc_Stop();                                                       /* 当前非 IDLE 状态，说明电机已启动或运行中，执行停止流程。 */
      }
    }

    vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(5u));                       /* 固定 5ms 周期运行，适合按键扫描和请求处理。 */
  }
}

/*
 * 调试任务。
 */
static void DebugTask(void *argument)
{
  TickType_t lastWakeTime;                                                   /* 调试任务周期基准 tick，用于保持 20ms 调用周期。 */

  (void)argument;                                                            /* 当前任务未使用参数，避免编译器未使用参数警告。 */

  lastWakeTime = xTaskGetTickCount();                                        /* 获取当前 tick，作为调试任务周期运行的起始基准。 */

  for (;;)
  {
    AppDebug_Task();                                                         /* 调试输出任务，内部会根据开关和节流周期决定是否真正打印。 */

    vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(20u));                      /* 固定 20ms 周期调用调试任务，实际打印频率由 AppDebug_Task 内部控制。 */
  }
}

/*
 * Idle 静态内存。
 */
void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer,
                                   StackType_t **ppxIdleTaskStackBuffer,
                                   uint32_t *pulIdleTaskStackSize)
{
  *ppxIdleTaskTCBBuffer = &s_idleTaskTcb;                                    /* 返回 IdleTask 的静态任务控制块地址。 */

  *ppxIdleTaskStackBuffer = s_idleTaskStack;                                 /* 返回 IdleTask 的静态任务栈地址。 */

  *pulIdleTaskStackSize = IDLE_TASK_STACK_WORDS;                             /* 返回 IdleTask 栈大小，单位是 word，不是 byte。 */
}

/*
 * 栈溢出钩子。
 */
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
  (void)xTask;                                                               /* 当前未直接使用任务句柄，调试时可查看该变量定位溢出任务。 */
  (void)pcTaskName;                                                          /* 当前未直接使用任务名，调试时可查看该变量定位溢出任务名称。 */

  __disable_irq();                                                           /* 栈溢出属于严重错误，立即关闭全局中断，避免系统继续异常运行。 */

  while (1)
  {
  }                                                                          /* 停机等待调试器接入，可查看 pcTaskName 和 xTask 定位问题任务。 */
}