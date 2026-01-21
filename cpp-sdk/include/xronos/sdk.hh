// SPDX-FileCopyrightText: Copyright (c) Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

/**
 * @file
 * Main header to include for using the Xronos SDK.
 */

#ifndef XRONOS_SDK_HH
#define XRONOS_SDK_HH

/**
 * @namespace xronos::sdk
 * Main Xronos SDK namespace.
 */

// IWYU pragma: begin_exports
#include "xronos/sdk/context.hh"
#include "xronos/sdk/element.hh"
#include "xronos/sdk/environment.hh"
#include "xronos/sdk/metric.hh"
#include "xronos/sdk/periodic_timer.hh"
#include "xronos/sdk/physical_event.hh"
#include "xronos/sdk/port.hh"
#include "xronos/sdk/programmable_timer.hh"
#include "xronos/sdk/reaction.hh"
#include "xronos/sdk/reactor.hh"
#include "xronos/sdk/time.hh"
#include "xronos/sdk/value_ptr.hh"
// IWYU pragma: end_exports

#endif // XRONOS_SDK_HH
