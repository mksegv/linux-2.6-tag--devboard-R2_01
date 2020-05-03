// Choose Argus2 or Argus3 headers
// Defaults to Argus2.

#include <linux/config.h>

#ifdef CONFIG_ARGUS_3
#include <asm/arch/argus3.h>
#else
#include <asm/arch/argus2.h>
#endif
