/*
 * j30d_internal.h
 *
 * Driver-private working state. This is declared in a header only because
 * this driver never heap-allocates -- customers must be able to put a
 * j30d_handle_t on the stack, in .bss, or inside their own struct,
 * which means every member it contains needs a compile-time-known size.
 *
 * NOTHING in this file is part of the supported API. Field names and layout
 * here can change in any release, including a PATCH one -- see the note on
 * binary compatibility at j30d_handle_t in j30d.h.
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
	j30d_data_t   last_data;            /* last known-good value per field, including stale ones */
	uint8_t       status_reg_raw;       /* last DATA_READY read */
	uint8_t       status_raw_reg_raw;   /* last DATA_READY_RAW read */

	/* now_ms() at the last sleep command, which restarts the device's
	 * conversion cycle; j30d_get_data()'s pacing counts from it -- see the
	 * sample-timing section in j30d.c. sleep_command_ms means nothing until
	 * sleep_commanded is true, since 0 is itself a valid now_ms() reading. */
	uint32_t      sleep_command_ms;
	bool          sleep_commanded;

	/* Set by j30d_notify_interrupt() from interrupt context, cleared by
	 * each j30d_get_data(). In interrupt mode it lets the pacing above use
	 * its shorter wait. volatile because the two run in different contexts;
	 * a single bool store needs no locking. */
	volatile bool interrupt_pending;

	/* Converting continuously: now_ms() when conversions were last known
	 * to be running, for j30d_get_data()'s stopped-conversion check -- see
	 * j30d.c. converting_ms means nothing until converting_seen is true. */
	uint32_t      converting_ms;
	bool          converting_seen;
} j30d_internal_t;

#ifdef __cplusplus
}
#endif

#endif /* J30D_INTERNAL_H_ */
