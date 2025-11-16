#include "usb_pd_benchmark.h"

#include "ch32x035.h"
#include "ch32x035_usbpd.h"
#include "millis.h"
#include "usb_cdc_print.h"
#include "usb_pd_snk.h"

#define BENCH_SETTLE_DELAY_MS 1000U
#define BENCH_MAX_FRAMES      USB_PD_BENCH_MAX_FRAMES
#define BENCH_MAX_INTERVAL_US USB_PD_BENCH_MAX_INTERVAL_US

#define BENCH_TIMER_CLK  RCC_APB1Periph_TIM2
#define BENCH_TIMER_INST TIM2
#define BENCH_TIMER_IRQn TIM2_UP_IRQn

typedef enum {
    BENCH_STATE_IDLE = 0,
    BENCH_STATE_WAIT_PSRDY,
    BENCH_STATE_WAIT_SETTLE,
    BENCH_STATE_SENDING,
} bench_state_t;

typedef enum {
    BENCH_ERR_NONE = 0,
    BENCH_ERR_TIMEOUT,
    BENCH_ERR_TX,
    BENCH_ERR_ABORT,
} bench_error_t;

#define BENCH_NOTIFY_ARMED  0x01
#define BENCH_NOTIFY_PSRDY  0x02
#define BENCH_NOTIFY_DONE   0x04
#define BENCH_NOTIFY_ERROR  0x08

static struct {
    bench_state_t state;
    uint32_t interval_us;
    uint32_t planned_count;
    uint32_t remaining;
    uint32_t completed_count;
    uint32_t settle_ready_ms;
    uint32_t burst_start_ms;
    uint32_t burst_duration_ms;
    volatile uint8_t notify_flags;
    bench_error_t last_error;
    bool burst_started;
} s_bench = {0};

static bool s_timer_inited = false;
static bool s_timer_running = false;
static volatile uint16_t s_pending_slots = 0;

static inline void bench_flag(uint8_t flag) { s_bench.notify_flags |= flag; }

static void bench_timer_stop(void);
static void bench_timer_apply_interval(uint32_t interval_us);
static void bench_timer_start(uint32_t interval_us);
static bool bench_fire_frame(bench_error_t *err_out);
static bool bench_start_burst(void);
static void bench_drain_pending(void);
static void bench_handle_tick(void);
static void bench_finalize(bool success, bench_error_t err);

static void bench_reset_internal(bool clear_notifications) {
    bench_timer_stop();
    s_bench.state = BENCH_STATE_IDLE;
    s_bench.interval_us = 0;
    s_bench.planned_count = 0;
    s_bench.remaining = 0;
    s_bench.completed_count = 0;
    s_bench.settle_ready_ms = 0;
    s_bench.burst_start_ms = 0;
    s_bench.burst_duration_ms = 0;
    s_bench.last_error = BENCH_ERR_NONE;
    s_bench.burst_started = false;
    s_pending_slots = 0;
    if (clear_notifications) {
        s_bench.notify_flags = 0;
    }
}

static void bench_timer_apply_interval(uint32_t interval_us) {
    uint32_t ticks = interval_us * 48U; /* 48MHz core clock */
    uint32_t prescaler = (ticks + 65535U) / 65536U;
    if (prescaler == 0U) prescaler = 1U;
    if (prescaler > 0x10000U) prescaler = 0x10000U;
    uint32_t reload = (ticks + prescaler - 1U) / prescaler;
    if (reload == 0U) reload = 1U;
    if (reload > 0x10000U) reload = 0x10000U;

    TIM_TimeBaseInitTypeDef timer_cfg = {0};
    timer_cfg.TIM_Period = (uint16_t)(reload - 1U);
    timer_cfg.TIM_Prescaler = (uint16_t)(prescaler - 1U);
    timer_cfg.TIM_ClockDivision = TIM_CKD_DIV1;
    timer_cfg.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(BENCH_TIMER_INST, &timer_cfg);
    TIM_ClearFlag(BENCH_TIMER_INST, TIM_FLAG_Update);
    TIM_ITConfig(BENCH_TIMER_INST, TIM_IT_Update, ENABLE);
}

static void bench_timer_start(uint32_t interval_us) {
    if (!s_timer_inited) {
        RCC_APB1PeriphClockCmd(BENCH_TIMER_CLK, ENABLE);
        NVIC_InitTypeDef nvic = {0};
        nvic.NVIC_IRQChannel = BENCH_TIMER_IRQn;
        nvic.NVIC_IRQChannelPreemptionPriority = 1;
        nvic.NVIC_IRQChannelSubPriority = 1;
        nvic.NVIC_IRQChannelCmd = ENABLE;
        NVIC_Init(&nvic);
        s_timer_inited = true;
    }

    bench_timer_apply_interval(interval_us);
    TIM_SetCounter(BENCH_TIMER_INST, 0);
    TIM_Cmd(BENCH_TIMER_INST, ENABLE);
    s_timer_running = true;
}

static void bench_timer_stop(void) {
    if (!s_timer_running) {
        return;
    }
    TIM_Cmd(BENCH_TIMER_INST, DISABLE);
    TIM_SetCounter(BENCH_TIMER_INST, 0);
    TIM_ClearFlag(BENCH_TIMER_INST, TIM_FLAG_Update);
    s_timer_running = false;
}

static bool bench_fire_frame(bench_error_t *err_out) {
    if (s_bench.remaining == 0) {
        return true;
    }

    if (!usb_pd_snk_is_active()) {
        if (err_out) *err_out = BENCH_ERR_ABORT;
        return false;
    }

    if ((USBPD->CONTROL & PD_TX_EN) != 0) {
        if (err_out) *err_out = BENCH_ERR_TIMEOUT;
        return false;
    }

    if (!usb_pd_snk_send_goodcrc_pd10()) {
        if (err_out) *err_out = BENCH_ERR_TX;
        return false;
    }

    s_bench.remaining--;
    s_bench.completed_count++;
    return true;
}

static void bench_finalize(bool success, bench_error_t err) {
    bench_timer_stop();
    s_pending_slots = 0;
    if (s_bench.burst_started && s_bench.burst_duration_ms == 0U) {
        s_bench.burst_duration_ms = (uint32_t)(millis() - s_bench.burst_start_ms);
    }
    s_bench.state = BENCH_STATE_IDLE;
    s_bench.burst_started = false;
    s_bench.last_error = err;
    s_bench.remaining = 0;
    bench_flag(success ? BENCH_NOTIFY_DONE : BENCH_NOTIFY_ERROR);
}

static bool bench_start_burst(void) {
    s_bench.burst_started = true;
    s_bench.burst_start_ms = millis();
    s_bench.burst_duration_ms = 0;

    bench_error_t err = BENCH_ERR_NONE;
    if (!bench_fire_frame(&err)) {
        bench_finalize(false, err);
        return false;
    }

    if (s_bench.remaining == 0) {
        s_bench.burst_duration_ms = (uint32_t)(millis() - s_bench.burst_start_ms);
        bench_finalize(true, BENCH_ERR_NONE);
        return false;
    }

    bench_timer_start(s_bench.interval_us);
    return true;
}

static void bench_drain_pending(void) {
    while (s_pending_slots > 0 && s_bench.state == BENCH_STATE_SENDING) {
        bench_error_t err = BENCH_ERR_NONE;
        if (!bench_fire_frame(&err)) {
            if (err == BENCH_ERR_TIMEOUT) {
                /* Hardware still transmitting; try again once TX_END fires. */
                return;
            }
            bench_finalize(false, err);
            return;
        }
        s_pending_slots--;
        if (s_bench.remaining == 0) {
            s_bench.burst_duration_ms = (uint32_t)(millis() - s_bench.burst_start_ms);
            bench_finalize(true, BENCH_ERR_NONE);
            return;
        }
    }
}

static void bench_handle_tick(void) {
    if (s_bench.state != BENCH_STATE_SENDING) {
        return;
    }
    if (s_pending_slots != 0xFFFFu) {
        s_pending_slots++;
    }
    bench_drain_pending();
}

bool usb_pd_benchmark_start(uint32_t interval_us, uint32_t count) {
    if (interval_us == 0 || interval_us > BENCH_MAX_INTERVAL_US || count == 0 || count > BENCH_MAX_FRAMES) {
        return false;
    }
    if (s_bench.state != BENCH_STATE_IDLE) {
        return false;
    }

    if (usb_pd_snk_is_active()) {
        usb_pd_snk_exit();
    }
    bench_reset_internal(true);
    usb_pd_snk_set_spec_rev(2);
    usb_pd_snk_enter();

    s_bench.interval_us = interval_us ? interval_us : 1U;
    s_bench.planned_count = count;
    s_bench.remaining = count;
    s_bench.completed_count = 0;
    s_bench.state = BENCH_STATE_WAIT_PSRDY;
    s_bench.last_error = BENCH_ERR_NONE;
    bench_flag(BENCH_NOTIFY_ARMED);

    return true;
}

void usb_pd_benchmark_abort(void) {
    if (s_bench.state == BENCH_STATE_IDLE) {
        bench_reset_internal(true);
        return;
    }

    if (s_bench.state == BENCH_STATE_SENDING && s_bench.burst_started) {
        s_bench.burst_duration_ms = (uint32_t)(millis() - s_bench.burst_start_ms);
    }
    bench_finalize(false, BENCH_ERR_ABORT);
}

void usb_pd_benchmark_on_rx(uint32_t status, const uint8_t *data, uint8_t len) {
    if (s_bench.state != BENCH_STATE_WAIT_PSRDY) return;
    if ((status & MASK_PD_STAT) != PD_RX_SOP0) return;
    if (len < 2) return;

    bool is_extended = (data[1] & 0x80u) != 0;
    uint8_t msg_type = data[0] & 0x1Fu;
    if (!is_extended && msg_type == DEF_TYPE_PS_RDY) {
        s_bench.state = BENCH_STATE_WAIT_SETTLE;
        s_bench.settle_ready_ms = millis() + BENCH_SETTLE_DELAY_MS;
        bench_flag(BENCH_NOTIFY_PSRDY);
    }
}

void usb_pd_benchmark_poll(void) {
    if (s_bench.state == BENCH_STATE_WAIT_SETTLE) {
        if ((int32_t)(millis() - s_bench.settle_ready_ms) >= 0) {
            s_bench.state = BENCH_STATE_SENDING;
        }
    }

    if (s_bench.state == BENCH_STATE_SENDING && !s_bench.burst_started) {
        (void)bench_start_burst();
    }

    bool need_exit = (s_bench.notify_flags & (BENCH_NOTIFY_DONE | BENCH_NOTIFY_ERROR)) != 0;

    if (cdc_acm_is_configured()) {
        uint8_t flags = s_bench.notify_flags;
        if (flags & BENCH_NOTIFY_ARMED) {
            cdc_acm_printf("# benchmark: SNK armed for PD1.0 GoodCRC, interval=%luus, count=%lu\n",
                           (unsigned long)s_bench.interval_us,
                           (unsigned long)s_bench.planned_count);
            s_bench.notify_flags &= (uint8_t)~BENCH_NOTIFY_ARMED;
        }
        if (flags & BENCH_NOTIFY_PSRDY) {
            cdc_acm_printf("# benchmark: PS_RDY detected, waiting %ums before firing\n",
                           (unsigned)BENCH_SETTLE_DELAY_MS);
            s_bench.notify_flags &= (uint8_t)~BENCH_NOTIFY_PSRDY;
        }
        if (flags & BENCH_NOTIFY_DONE) {
            cdc_acm_printf("# benchmark: sent %lu/%lu PD1.0 GoodCRC frames in %lums\n",
                           (unsigned long)s_bench.completed_count,
                           (unsigned long)s_bench.planned_count,
                           (unsigned long)s_bench.burst_duration_ms);
            s_bench.notify_flags &= (uint8_t)~BENCH_NOTIFY_DONE;
        } else if (flags & BENCH_NOTIFY_ERROR) {
            if (s_bench.last_error == BENCH_ERR_TIMEOUT) {
                cdc_acm_printf("# benchmark: aborted after %lu/%lu (bus busy) in %lums\n",
                               (unsigned long)s_bench.completed_count,
                               (unsigned long)s_bench.planned_count,
                               (unsigned long)s_bench.burst_duration_ms);
            } else if (s_bench.last_error == BENCH_ERR_ABORT) {
                cdc_acm_printf("# benchmark: aborted after %lu/%lu (user exit) in %lums\n",
                               (unsigned long)s_bench.completed_count,
                               (unsigned long)s_bench.planned_count,
                               (unsigned long)s_bench.burst_duration_ms);
            } else {
                cdc_acm_printf("# benchmark: aborted after %lu/%lu (TX failure) in %lums\n",
                               (unsigned long)s_bench.completed_count,
                               (unsigned long)s_bench.planned_count,
                               (unsigned long)s_bench.burst_duration_ms);
            }
            s_bench.notify_flags &= (uint8_t)~BENCH_NOTIFY_ERROR;
        }
    }

    if (need_exit && usb_pd_snk_is_active()) {
        usb_pd_snk_exit();
    }
}

void usb_pd_benchmark_on_tx_idle(void) {
    if (s_bench.state != BENCH_STATE_SENDING || !s_bench.burst_started) {
        return;
    }
    bench_drain_pending();
}

void TIM2_UP_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void TIM2_UP_IRQHandler(void) {
    if (TIM_GetITStatus(BENCH_TIMER_INST, TIM_IT_Update) != RESET) {
        TIM_ClearITPendingBit(BENCH_TIMER_INST, TIM_IT_Update);
        bench_handle_tick();
    }
}
