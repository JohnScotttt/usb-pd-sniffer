#include "usb_pd_benchmark.h"

#include "ch32x035_usbpd.h"
#include "debug.h"
#include "millis.h"
#include "usb_cdc_print.h"
#include "usb_pd_snk.h"

#define BENCH_SETTLE_DELAY_MS     1000U
#define BENCH_TX_IDLE_TIMEOUT_MS  20U
#define BENCH_MAX_FRAMES          USB_PD_BENCH_MAX_FRAMES
#define BENCH_MAX_INTERVAL_US     USB_PD_BENCH_MAX_INTERVAL_US

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
    uint8_t notify_flags;
    bench_error_t last_error;
    bool burst_started;
    bool abort_requested;
} s_bench = {0};

static inline void bench_flag(uint8_t flag) { s_bench.notify_flags |= flag; }

static void bench_reset_internal(bool clear_notifications) {
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
    s_bench.abort_requested = false;
    if (clear_notifications) {
        s_bench.notify_flags = 0;
    }
}

static void bench_delay_interval(uint32_t interval_us) {
    if (interval_us >= 1000U) {
        uint32_t ms = interval_us / 1000U;
        uint32_t us = interval_us % 1000U;
        if (ms) {
            Delay_Ms(ms);
        }
        if (us) {
            Delay_Us(us);
        }
    } else if (interval_us > 0U) {
        Delay_Us(interval_us);
    }
}

static void bench_finalize(bool success, bench_error_t err, uint32_t completed) {
    s_bench.state = BENCH_STATE_IDLE;
    s_bench.burst_started = false;
    s_bench.last_error = err;
    s_bench.completed_count = completed;
    s_bench.remaining = 0;
    bench_flag(success ? BENCH_NOTIFY_DONE : BENCH_NOTIFY_ERROR);
}

bool usb_pd_benchmark_start(uint32_t interval_us, uint32_t count) {
    if (interval_us == 0 || interval_us > BENCH_MAX_INTERVAL_US || count == 0 || count > BENCH_MAX_FRAMES) {
        return false;
    }
    if (s_bench.state != BENCH_STATE_IDLE) {
        return false;
    }

    if (interval_us == 0) {
        interval_us = 1;
    }

    if (usb_pd_snk_is_active()) {
        usb_pd_snk_exit();
    }
    bench_reset_internal(true);
    usb_pd_snk_set_spec_rev(2);
    usb_pd_snk_enter();

    s_bench.interval_us = interval_us;
    s_bench.planned_count = count;
    s_bench.remaining = count;
    s_bench.completed_count = 0;
    s_bench.state = BENCH_STATE_WAIT_PSRDY;
    s_bench.burst_started = false;
    s_bench.last_error = BENCH_ERR_NONE;
    s_bench.notify_flags = BENCH_NOTIFY_ARMED;

    return true;
}

void usb_pd_benchmark_abort(void) {
    if (s_bench.state == BENCH_STATE_IDLE) {
        bench_reset_internal(true);
    } else {
        s_bench.abort_requested = true;
    }
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
    if (s_bench.abort_requested && s_bench.state != BENCH_STATE_IDLE) {
        if (s_bench.state != BENCH_STATE_SENDING || !s_bench.burst_started) {
            bench_finalize(false, BENCH_ERR_ABORT, s_bench.completed_count);
            s_bench.abort_requested = false;
        }
    }

    if (s_bench.state == BENCH_STATE_WAIT_SETTLE) {
        if ((int32_t)(millis() - s_bench.settle_ready_ms) >= 0) {
            s_bench.state = BENCH_STATE_SENDING;
        }
    }

    if (s_bench.state == BENCH_STATE_SENDING && !s_bench.burst_started) {
        s_bench.burst_started = true;
        s_bench.burst_start_ms = millis();
        bool success = true;
        bench_error_t err = BENCH_ERR_NONE;
        uint32_t completed = s_bench.planned_count - s_bench.remaining;

        while (s_bench.remaining > 0) {
            if (s_bench.abort_requested || !usb_pd_snk_is_active()) {
                success = false;
                err = BENCH_ERR_ABORT;
                break;
            }
            if (!usb_pd_snk_wait_for_idle(BENCH_TX_IDLE_TIMEOUT_MS)) {
                success = false;
                err = BENCH_ERR_TIMEOUT;
                break;
            }
            if (!usb_pd_snk_send_goodcrc_pd10_blocking()) {
                success = false;
                err = BENCH_ERR_TX;
                break;
            }
            s_bench.remaining--;
            completed++;
            if (s_bench.remaining > 0) {
                bench_delay_interval(s_bench.interval_us);
            }
        }
        s_bench.burst_duration_ms = (uint32_t)(millis() - s_bench.burst_start_ms);
        s_bench.abort_requested = false;
        bench_finalize(success, err, completed);
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
        /* Returning to LISTEN mode mirrors manual 'exit'. */
        usb_pd_snk_exit();
    }
}
