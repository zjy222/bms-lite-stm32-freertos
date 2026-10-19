#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

/*============================================================
 * FreeRTOSConfig.h
 * Target: STM32F103C8T6 / Cortex-M3
 * Toolchain: Keil MDK5 / ARM Compiler 5 (armcc)
 * Kernel source: current FreeRTOS-Kernel main branch
 *
 * M3-P1 configuration:
 * - 72 MHz CPU
 * - 1 ms RTOS tick
 * - preemptive scheduling + time slicing
 * - dynamic allocation via heap_4.c
 * - mutex/counting semaphore enabled for later M3 steps
 * - software timers/event groups/stream buffers disabled for now
 *============================================================*/

/* ---------- Clock / tick ---------- */
#define configCPU_CLOCK_HZ                    ( 72000000UL )
#define configTICK_RATE_HZ                    ( 1000U )

/* Current FreeRTOS-Kernel requires one tick-width definition. */
#define configTICK_TYPE_WIDTH_IN_BITS         TICK_TYPE_WIDTH_32_BITS

/* ---------- Scheduler ---------- */
#define configUSE_PREEMPTION                  1
#define configUSE_TIME_SLICING                1
#define configUSE_PORT_OPTIMISED_TASK_SELECTION 0
#define configUSE_TICKLESS_IDLE               0
#define configMAX_PRIORITIES                  5
#define configMINIMAL_STACK_SIZE              128
#define configMAX_TASK_NAME_LEN               16
#define configIDLE_SHOULD_YIELD               1

/* ---------- Memory ---------- */
#define configSUPPORT_DYNAMIC_ALLOCATION      1
#define configSUPPORT_STATIC_ALLOCATION       0
#define configTOTAL_HEAP_SIZE                 ( 8U * 1024U )

/* ---------- Hooks / diagnostics ---------- */
#define configUSE_IDLE_HOOK                   0
#define configUSE_TICK_HOOK                   0
#define configCHECK_FOR_STACK_OVERFLOW        2
#define configUSE_MALLOC_FAILED_HOOK          1
#define configUSE_TRACE_FACILITY              0
#define configGENERATE_RUN_TIME_STATS         0
#define configUSE_STATS_FORMATTING_FUNCTIONS  0

/* Keep asserts simple for the first bring-up.
 * If an RTOS invariant fails, execution stops here for debugging. */
#define configASSERT( x )                     do { if( ( x ) == 0 ) { for( ;; ) { } } } while( 0 )

/* ---------- IPC features needed later in M3 ---------- */
#define configUSE_MUTEXES                     1
#define configUSE_RECURSIVE_MUTEXES           0
#define configUSE_COUNTING_SEMAPHORES         1
#define configUSE_QUEUE_SETS                  0

/* Not used in M3-P1. Keeping them off means we do not need
 * timers.c / event_groups.c / stream_buffer.c yet. */
#define configUSE_TIMERS                      0
#define configUSE_EVENT_GROUPS                0
#define configUSE_STREAM_BUFFERS              0
#define configUSE_CO_ROUTINES                 0

/* ---------- Optional API functions ---------- */
#define INCLUDE_vTaskDelay                    1
#define INCLUDE_xTaskDelayUntil               1
#define INCLUDE_vTaskDelete                   1
#define INCLUDE_vTaskSuspend                  1
#define INCLUDE_vTaskPrioritySet              1
#define INCLUDE_uxTaskPriorityGet             1
#define INCLUDE_xTaskGetSchedulerState        1
#define INCLUDE_uxTaskGetStackHighWaterMark   1

/* ---------- Cortex-M3 interrupt priorities ----------
 * STM32F103 implements 4 NVIC priority bits.
 *
 * Numerically smaller = higher hardware interrupt priority.
 * Interrupts that call FreeRTOS FromISR APIs must use a numerical
 * priority >= configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY.
 */
#define configPRIO_BITS                       4
#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY       15
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY  5

#define configKERNEL_INTERRUPT_PRIORITY       \
    ( configLIBRARY_LOWEST_INTERRUPT_PRIORITY << ( 8 - configPRIO_BITS ) )

#define configMAX_SYSCALL_INTERRUPT_PRIORITY  \
    ( configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY << ( 8 - configPRIO_BITS ) )

/* ---------- Map FreeRTOS Cortex-M port handlers to STM32 vector names ----------
 * The startup file already has weak vector entries with these CMSIS names.
 * These macros make port.c provide the actual implementations.
 */
#define vPortSVCHandler                       SVC_Handler
#define xPortPendSVHandler                    PendSV_Handler
#define xPortSysTickHandler                   SysTick_Handler

#endif /* FREERTOS_CONFIG_H */
