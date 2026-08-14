// SPDX-FileCopyrightText: Copyright (c) Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

#ifndef XRONOS_ABI_NODE_HH
#define XRONOS_ABI_NODE_HH

#include <cstdint>

// Included for its compile-time toolchain checks: every translation unit
// that touches the ABI must satisfy the contract.
#include "xronos/abi/contract.hh" // IWYU pragma: keep
#include "xronos/abi/types.hh"

// The contract between a node module (a `.so` registered with
// XRONOS_REGISTER_NODE) and the host that loads and runs it (`xronos run`).
// This header is the ENTIRE node/host contract: everything else the two
// sides exchange crosses through the interfaces in backend.hh, and all
// SDK-level types (Context, reactors, ...) remain private to each side's
// own compilation.
//
// The host's protocol: dlopen the module, resolve the three entry points
// (the descriptor and both creators), read the descriptor, apply the
// version gate (accept iff same major and host minor >= module minor --
// see version.hh) BEFORE calling anything else the module defines, resolve
// the node name (an explicit override, or the descriptor's default), then
// create the node against a `Backend` the host keeps alive for the node's
// whole lifetime -- at the top level, or under a host reactor named by its
// uid. Everything else the node needs -- including the hot-path lookup
// surface once a run is prepared -- is reachable from the backend (see
// backend.hh).

namespace xronos::abi::inline v1 {

class Backend;

// Compatibility information a node module advertises, returned by the
// `xronos_node_descriptor` entry point and read by the host before anything
// node-defined executes.
//
// The first two fields carry the version gate itself and are therefore read
// BEFORE the gate: they are frozen forever, across all major versions.
// Everything after them is only read once the gate has passed and follows
// the ordinary append-only growth rule.
struct NodeDescriptor {
  // ABI major/minor version the module was compiled against; the gate.
  std::uint32_t abi_version_major;
  std::uint32_t abi_version_minor;
  // SDK package version the module was compiled against; diagnostics only.
  const char* sdk_version;
  // The node's default name, applied by the host when no override is given.
  const char* default_name;
};

// Pins the ABI 1.0 layout as a tripwire against accidental repacking;
// append-only growth updates this alongside the minor version bump.
// NOLINTNEXTLINE(*-magic-numbers)
static_assert(sizeof(NodeDescriptor) == 24 && alignof(NodeDescriptor) == 8);

// The host's handle on a created node: opaque, owned by the host, destroyed
// with plain `delete` (safe across `.so` boundaries on Linux, see
// backend.hh). Everything the host needs to know about the node beyond its
// lifetime -- its name, its version -- comes from the descriptor and the
// host's own name resolution, so no further operations are needed.
class Node {
public:
  virtual ~Node() = default;
};

// Entry-point contract for hosts resolving the module's symbols via dlsym.
//
// `xronos_node_descriptor` returns the module's descriptor (static storage,
// never null for a well-formed module). The two creators construct the
// registered node with the host-resolved name (never null or empty) against
// the host's backend (never null, outlives the node): `xronos_create_node`
// places it under the host reactor `parent` (a uid the host obtained from
// this same backend), `xronos_create_top_level_node` places it at the top
// level. The pair mirrors the backend's `register_reactor` /
// `register_top_level_reactor`; a well-formed module exports both, and the
// host picks one per placement. Ownership of the returned node transfers to
// the host. Exceptions propagate natively out of the creators, like any
// other cross-boundary exception.
inline constexpr const char* node_descriptor_symbol_name = "xronos_node_descriptor";
inline constexpr const char* create_node_symbol_name = "xronos_create_node";
inline constexpr const char* create_top_level_node_symbol_name = "xronos_create_top_level_node";
using NodeDescriptorFunction = const NodeDescriptor* (*)();
using CreateNodeFunction = Node* (*)(const char* name, Backend* backend, ElementUid parent);
using CreateTopLevelNodeFunction = Node* (*)(const char* name, Backend* backend);

} // namespace xronos::abi::inline v1

#endif // XRONOS_ABI_NODE_HH
