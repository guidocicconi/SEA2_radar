
/* Use the defaults for everything else */
#include_next<FreeRTOSConfig.h>

#ifdef configUSE_TICK_HOOK
#undef configUSE_TICK_HOOK
#endif

#define configUSE_TICK_HOOK 1

#ifdef configUSE_DAEMON_TASK_STARTUP_HOOK
#undef configUSE_DAEMON_TASK_STARTUP_HOOK
#endif

#define configUSE_DAEMON_TASK_STARTUP_HOOK 1

#ifdef configUSE_TIMERS
#undef configUSE_TIMERS
#endif

#define configUSE_TIMERS 1

