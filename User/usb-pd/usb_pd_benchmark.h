#pragma once

#define USB_PD_BENCH_MAX_FRAMES       1000000UL
#define USB_PD_BENCH_MAX_INTERVAL_US  4000000UL

#include <stdbool.h>
#include <stdint.h>

/* Arm a benchmark run: enter SNK mode, wait for PS_RDY, then issue PD1.0 GoodCRC bursts. */
bool usb_pd_benchmark_start(uint32_t interval_us, uint32_t count);

/* Feed every RX frame (from ISR context) so we can detect PS_RDY. */
void usb_pd_benchmark_on_rx(uint32_t status, const uint8_t *data, uint8_t len);

/* Service deferred logs and run the actual GoodCRC burst once everything is ready. */
void usb_pd_benchmark_poll(void);

/* Called from USBPD TX_END ISR when the transmitter becomes idle. */
void usb_pd_benchmark_on_tx_idle(void);

/* Cancel any pending benchmark state (e.g., when SNK exits manually). */
void usb_pd_benchmark_abort(void);
