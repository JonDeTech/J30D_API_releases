/*
 * j30d_internal.h
 *
 * Driver-private working state. This is declared in a header only because
 * this driver never heap-allocates -- customers must be able to put a
 * j30d_handle_t on the stack, in .bss, or inside their own struct,
 * which means every member it contains needs a compile-time-known size.
 *
 * NOTHING in this file is part of the supported API. Field names and layout
 * here can change in any release, including a PATCH one. That is safe ONLY
 * because this driver ships as source and is always recompiled with its own
 * headers -- a change here does resize j30d_handle_t, so it is a hard break
 * for anyone linking a pre-built object against different headers. There is
 * no ABI guarantee; see the note on j30d_handle_t in j30d.h.
 *
 * Do not read or write these members from application code -- go through
 * the j30d_* functions declared in j30d.h (j30d_get_calib() /
 * j30d_get_config() / j30d_is_initialized() cover the read-only cases).
 */

#ifndef J30D_INTERNAL_H_
#define J30D_INTERNAL_H_

#include <stdint.h>
#include <stdbool.h>
#include "j30d_types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
	j30d_data_t   last_data;        /* last known-good value per field, including stale ones */
	uint8_t       status_reg_raw;   /* last DATA_READY read */
	uint8_t       status_raw_reg_raw; /* last DATA_READY_RAW read */

	/* j30d_get_data()'s sleep-interval pacing (only consulted/updated when
	 * platform.now_ms is non-NULL and sleep is configured -- see
	 * should_skip_wake() in j30d.c). last_get_data_wake_ms is meaningless
	 * until get_data_has_woken is true -- needed because 0 is itself a
	 * valid now_ms() reading (e.g. very shortly after boot), so it can't
	 * double as "never woken yet". */
	uint32_t      last_get_data_wake_ms;
	bool          get_data_has_woken;

	/* Set by j30d_notify_interrupt() from interrupt context, consumed by the
	 * next j30d_get_data(). volatile because the two run in different
	 * contexts; a single bool store needs no locking.
	 *
	 * Bypasses the pacing above: samples arrive one sleep interval plus one
	 * conversion apart, so pacing on the interval alone rejects ready
	 * reads. */
	volatile bool interrupt_pending;
} j30d_internal_t;

#ifdef __cplusplus
}
#endif

#endif /* J30D_INTERNAL_H_ */
