
/*-----------------------------------------------------------
 * Implementation of functions defined in portable.h for the ARM CM4F port.
 *----------------------------------------------------------*/

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "gr55xx_hal.h"
#include "gr55xx_sys.h"

/*
 * GLOBAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
extern void ultra_wfi(void);
extern uint32_t get_remain_sleep_dur(void);

/*
 * LOCAL MACRO DEFINITIONS
 *****************************************************************************************
 */
#define SYSTICK_LOAD_VALUE                  ((configCPU_CLOCK_HZ / configTICK_RATE_HZ))
#define TICK_MS_IN_HUS                      (2000)
#define SYS_BLE_SLEEP_ALGO_HUS              (580)
#define SLP_WAKUP_ALGO_LP_CNT               (32)

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static uint32_t s_lp_cnt_when_tick_stop = 0;
static uint32_t s_lp_cnt_when_tick_reload = 0;

/**
  * @brief Function Implementations
  */
TINY_RAM_SECTION uint32_t vPortLocker(void)
{
    uint32_t ret_pri = __get_PRIMASK();
    __set_PRIMASK(1);
    return ret_pri;
}

TINY_RAM_SECTION void vPortUnLocker(uint32_t set_pri)
{
    __set_PRIMASK(set_pri);
}

TINY_RAM_SECTION void SysTickReload(void)
{
    SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;
    SysTick->LOAD = (uint32_t)(SYSTICK_LOAD_VALUE - 1UL); /* set reload register */
    SysTick->VAL = 0UL;                                   /* Load the SysTick Counter Value */
    SysTick->CTRL |= (SysTick_CTRL_ENABLE_Msk | SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk);
}

TINY_RAM_SECTION void pwr_mgmt_sleep_dur_limit(int rtos_idle_ticks)
{
    uint32_t rtos_idle_hus = rtos_idle_ticks*TICK_MS_IN_HUS - SYS_BLE_SLEEP_ALGO_HUS;
    if( get_remain_sleep_dur() > rtos_idle_hus)
    {
        pwr_mgmt_ble_wakeup();
    }
    sys_ble_heartbeat_period_set(rtos_idle_hus);
}

TINY_RAM_SECTION void pwr_mgmt_enter_sleep_with_cond(int rtos_idle_ticks)
{
    /* Limit maximum sleep duration. */
    pwr_mgmt_sleep_dur_limit(rtos_idle_ticks);

    /* To disbale global IRQ. */
    uint32_t pwr_locker = vPortLocker();
    /* Check Whether Device is busy. */
    if (DEVICE_BUSY == pwr_mgmt_dev_suspend())
    {
        ultra_wfi();
        vPortUnLocker(pwr_locker);
        return;
    }

    /* Check BLE Sleep Status. */
    pwr_mgmt_mode_t ble_state = pwr_mgmt_baseband_state_get();
    if (PMR_MGMT_SLEEP_MODE != ble_state)
    {
        if (PMR_MGMT_IDLE_MODE == ble_state)
        {
            ultra_wfi();
        }
        vPortUnLocker(pwr_locker);
        return;
    }

    /* Record Systick Stop Time. */
    s_lp_cnt_when_tick_stop = ll_pwr_get_comm_sleep_duration();

    /* Before the lock, 
     there was an interrupt context that changed the sleep mode and needed to take effect immediately. */
    if (PMR_MGMT_SLEEP_MODE != pwr_mgmt_mode_get())
    {
        vPortUnLocker(pwr_locker);
        return;
    }

    /* Save the context of RTOS. */
    pwr_mgmt_save_context();

    /* Judge the current startup mode mark. */
    if (pwr_mgmt_get_wakeup_flag() == COLD_BOOT)
    {
        /* Shutdown all system power and wait some event to wake up. */
        if (PMR_MGMT_IDLE_MODE == pwr_mgmt_shutdown())
        {
            ultra_wfi();
            vPortUnLocker(pwr_locker);
            return;
        }
        vPortUnLocker(pwr_locker);
    }
    else // wakeup from deep sleep mode
    {
        /* clear wakeup mark ,parpare next time enter-sleep action. */
        pwr_mgmt_set_wakeup_flag(COLD_BOOT);

        /* To disbale global IRQ. */
        uint32_t _local_lock = vPortLocker();

        /* Restart SysTick module. */
        s_lp_cnt_when_tick_reload = ll_pwr_get_comm_sleep_duration();
        SysTickReload();

        /* Calculate Step Ticks With Low Power Counter. */
        uint64_t sleep_lp_cycles = s_lp_cnt_when_tick_reload - s_lp_cnt_when_tick_stop + SLP_WAKUP_ALGO_LP_CNT;
        uint32_t sys_lpcycles_2_hus_err = 0;
        uint64_t ticks_hus = sys_lpcycles_2_hus(sleep_lp_cycles, &sys_lpcycles_2_hus_err);
        uint64_t step_ticks = ticks_hus / TICK_MS_IN_HUS;

        /* Step Systick Increment. */
        vTaskStepTick(step_ticks);

        /* To enable global IRQ. */
        vPortUnLocker(_local_lock);

        extern void warm_boot_second(void);
        warm_boot_second();
    }
}

TINY_RAM_SECTION void vPortEnterDeepSleep(TickType_t xExpectedIdleTime)
{
    /* To support low power mode, the ble stack shall be initiated. */
    if (xExpectedIdleTime < 5)
    {
        ultra_wfi();
        return;
    }

    if (PMR_MGMT_ACTIVE_MODE == pwr_mgmt_mode_get())
    {
        ultra_wfi();
        return;
    }

    pwr_mgmt_enter_sleep_with_cond(xExpectedIdleTime);
}
