/* AUTOMATICALLY GENERATED, DO NOT MODIFY */

/*
 * Schema-defined QAPI/QMP events
 *
 * Copyright (c) 2014 Wenchao Xia
 * Copyright (c) 2015-2018 Red Hat Inc.
 *
 * This work is licensed under the terms of the GNU LGPL, version 2.1 or later.
 * See the COPYING.LIB file in the top-level directory.
 */

#ifndef TEST_QAPI_EVENTS_H
#define TEST_QAPI_EVENTS_H

#include "qapi/util.h"
#include "test-qapi-types.h"


void qapi_event_send_event_a(void);

void qapi_event_send_event_b(void);

void qapi_event_send_event_c(bool has_a, int64_t a, bool has_b, UserDefOne *b, const char *c);

void qapi_event_send_event_d(EventStructOne *a, const char *b, bool has_c, const char *c, bool has_enum3, EnumOne enum3);

void qapi_event_send_event_e(UserDefZero *arg);

void qapi_event_send_event_f(UserDefAlternate *arg);

void qapi_event_send___org_qemu_x_event(__org_qemu_x_Enum __org_qemu_x_member1, const char *__org_qemu_x_member2, bool has_q_wchar_t, int64_t q_wchar_t);

#if defined(TEST_IF_EVT) && defined(TEST_IF_STRUCT)
void qapi_event_send_testifevent(TestIfStruct *foo);
#endif /* defined(TEST_IF_EVT) && defined(TEST_IF_STRUCT) */

typedef enum test_QAPIEvent {
    TEST_QAPI_EVENT_EVENT_A = 0,
    TEST_QAPI_EVENT_EVENT_B = 1,
    TEST_QAPI_EVENT_EVENT_C = 2,
    TEST_QAPI_EVENT_EVENT_D = 3,
    TEST_QAPI_EVENT_EVENT_E = 4,
    TEST_QAPI_EVENT_EVENT_F = 5,
    TEST_QAPI_EVENT___ORG_QEMU_X_EVENT = 6,
    TEST_QAPI_EVENT_TESTIFEVENT = 7,
    TEST_QAPI_EVENT__MAX = 8,
} test_QAPIEvent;

#define test_QAPIEvent_str(val) \
    qapi_enum_lookup(&test_QAPIEvent_lookup, (val))

extern const QEnumLookup test_QAPIEvent_lookup;

#endif /* TEST_QAPI_EVENTS_H */
