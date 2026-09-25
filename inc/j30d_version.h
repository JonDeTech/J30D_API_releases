/*
 * j30d_version.h
 *
 * Semantic version of the J30D driver/API surface itself (this file plus
 * j30d.h, j30d_types.h, j30d_platform.h, j30d_config.h, j30d_calib.h). This
 * is a version for the *API contract*, not the sensor silicon/factory
 * calibration revision (see j30d_unit_version_t / j30d_calib_t for that).
 *
 * Bump MAJOR  - any breaking change: a struct layout change, a function
 *               signature change, a removed/renamed field or enum value.
 * Bump MINOR  - additive, backward compatible: a new optional platform
 *               hook, a field appended to the end of a struct, a new
 *               error code.
 * Bump PATCH  - behavior-only fixes that touch no public declaration.
 *
 * This versions the SOURCE-level API contract only. It is not an ABI
 * version: j30d_handle_t embeds driver-private state by value, so its size
 * and layout can change in any release. Compile the driver's .c files and
 * your application against the same revision of these headers; never link a
 * pre-built j30d object or library against a different revision.
 *
 * Customers linking against this driver can compile-time gate on
 * J30D_API_VERSION if they ever need to support more than one driver
 * revision side by side, or simply log j30d_get_version_string() in a
 * bug report.
 */

#ifndef J30D_VERSION_H_
#define J30D_VERSION_H_

#define J30D_API_VERSION_MAJOR   0
#define J30D_API_VERSION_MINOR   5
#define J30D_API_VERSION_PATCH   0

#define J30D_API_VERSION \
    ((J30D_API_VERSION_MAJOR << 16) | (J30D_API_VERSION_MINOR << 8) | J30D_API_VERSION_PATCH)

#ifdef __cplusplus
extern "C" {
#endif

/* "MAJOR.MINOR.PATCH" as a compile-time string literal. Usable in a static
 * initializer, a banner concatenation ("fw 1.2 / j30d " J30D_API_VERSION_STRING),
 * or anywhere a string constant is wanted -- and, unlike the function below,
 * it costs nothing at runtime. */
#define J30D_STRINGIFY_(x)   #x
#define J30D_STRINGIFY(x)    J30D_STRINGIFY_(x)
#define J30D_API_VERSION_STRING \
    J30D_STRINGIFY(J30D_API_VERSION_MAJOR) "." \
    J30D_STRINGIFY(J30D_API_VERSION_MINOR) "." \
    J30D_STRINGIFY(J30D_API_VERSION_PATCH)

/* Returns the same thing as a static string. No I2C traffic, callable
 * before j30d_init(), and safe to call from any thread -- it hands back a
 * pointer to a const literal and formats nothing at runtime (it used to
 * snprintf() into a shared static buffer, which dragged the whole printf
 * family into the image -- several KB on a Cortex-M0+ -- and raced when two
 * threads called it at once). */
const char *j30d_get_version_string(void);

#ifdef __cplusplus
}
#endif

#endif /* J30D_VERSION_H_ */
