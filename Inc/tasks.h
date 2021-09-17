#ifndef _TASKS_H_
#define _TASKS_H_

#include "pt.h"


// Long running tasks
PT_THREAD( uvc_task(struct pt *pt_ptr));
PT_THREAD( SendToPi_task(struct pt *pt_ptr));
PT_THREAD( DetectBadLine_task(struct pt *pt_ptr));



#endif
