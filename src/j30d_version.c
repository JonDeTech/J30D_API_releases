#include "j30d_version.h"

/* No <stdio.h> on purpose -- see the note on J30D_API_VERSION_STRING in
 * j30d_version.h. This is the whole implementation: a pointer to a literal. */
const char *j30d_get_version_string(void)
{
	return J30D_API_VERSION_STRING;
}
