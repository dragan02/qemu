/* This file is autogenerated by tracetool, do not edit. */

#ifndef TRACE_HW_ALPHA_GENERATED_TRACERS_H
#define TRACE_HW_ALPHA_GENERATED_TRACERS_H

#include "qemu-common.h"
#include "trace/control.h"

extern TraceEvent _TRACE_ALPHA_PCI_IACK_WRITE_EVENT;
extern uint16_t _TRACE_ALPHA_PCI_IACK_WRITE_DSTATE;
#define TRACE_ALPHA_PCI_IACK_WRITE_ENABLED 1
#include "qemu/log-for-trace.h"


#define TRACE_ALPHA_PCI_IACK_WRITE_BACKEND_DSTATE() ( \
    trace_event_get_state_dynamic_by_id(TRACE_ALPHA_PCI_IACK_WRITE) || \
    false)

static inline void _nocheck__trace_alpha_pci_iack_write(void)
{
    if (trace_event_get_state(TRACE_ALPHA_PCI_IACK_WRITE) && qemu_loglevel_mask(LOG_TRACE)) {
        struct timeval _now;
        gettimeofday(&_now, NULL);
        qemu_log("%d@%zu.%06zu:alpha_pci_iack_write " "" "\n",
                 getpid(),
                 (size_t)_now.tv_sec, (size_t)_now.tv_usec
                 );
    }
}

static inline void trace_alpha_pci_iack_write(void)
{
    if (true) {
        _nocheck__trace_alpha_pci_iack_write();
    }
}
#endif /* TRACE_HW_ALPHA_GENERATED_TRACERS_H */
