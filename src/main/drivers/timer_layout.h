// include correct pin layout definition. Define macros to prevent multiple inclusio

#if !defined(TIMER_LAYOUT_HEADER_INCLUDED) || defined(INCLUDE_LAYOUT_BODY)

#if !defined(INCLUDE_LAYOUT_BODY) && !defined(TIMER_LAYOUT_HEADER_INCLUDED)
# define TIMER_LAYOUT_HEADER_INCLUDED
#endif

#if defined(STM32F10X)
# if defined(NAZE)
#  include "timer_layout_naze.h"
# elif defined(CC3D)
#  include "timer_layout_cc3d.h"
# else
// fallback
#  include "timer_layout_naze.h"
# endif
#elif defined(STM32F303)
# if defined(CHEBUZZF3)
#  include "timer_layout_cchebuzzf3.h"
# elif defined(NAZE32PRO)
#  include "timer_layout_naze32pro.h"
# elif defined(STM32F3DISCOVERY)
#  include "timer_layout_f3discovery.h"
# else
// fallback
#  include "timer_layout_f3discovery.h"
# endif
#else
# error "Channel layout was not found for configured device"
#endif

#endif
