// SPDX-FileCopyrightText: Copyright (c) Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

#ifndef XRONOS_CORE_CONNECTION_GRAPH_HH
#define XRONOS_CORE_CONNECTION_GRAPH_HH

#include <cstdint>
#include <functional>
#include <optional>
#include <ranges>
#include <unordered_map>
#include <vector>

#include "xronos/core/element.hh"
#include "xronos/core/time.hh"

namespace xronos::core {

// How a connection takes part in a boundary crossing. A crossing spans the
// connections between its serializing port and its deserializing ports.
// Between them the value exists only as bytes.
//
// - `None`: a plain connection.
// - `Serializing`: the value is serialized with the serializer of `from_uid`,
//   the serializing port, which opens a crossing.
// - `Deserializing`: the value is deserialized with the serializer of
//   `to_uid`, a deserializing port, which closes the crossing.
// - `Complete`: a crossing that opens and closes on this one connection.
//   The value is serialized with the serializer of `from_uid` and
//   deserialized with the serializer of `to_uid`.
enum class BoundaryCrossing : std::uint8_t {
  None,
  Serializing,
  Deserializing,
  Complete,
};

struct ConnectionProperties {
  ElementID from_uid;
  ElementID to_uid;
  std::optional<Duration> delay;
  BoundaryCrossing crossing{BoundaryCrossing::None};
};

// The connection tree of an origin port holds every connection that carries
// the port's values, whichever reactions write or observe its ports. Its
// leaves are the receiving ports, and its interior vertices are boundary
// crossings, where values are serialized and deserialized.

// A receiving port together with the delay accumulated along the path from
// the origin port.
struct ReceivingPort {
  ElementID port_uid{};
  std::optional<Duration> delay;
};

struct ConnectionSerialization;

// A part of a connection tree between crossings: the receiving ports and
// boundary crossings reachable from one point in the tree without crossing
// a further boundary. A nested segment lists its deserializing port as a
// receiving port.
struct ConnectionSegment {
  std::vector<ReceivingPort> receiving_ports;
  std::vector<ConnectionSerialization> serializations;
};

// One deserializing port of a boundary crossing. The value is deserialized
// with the serializer of the port named by `deserialize_uid` and then
// continues into the nested segment below.
struct ConnectionDeserialization {
  ElementID deserialize_uid{};
  ConnectionSegment segment;
};

// The serializing port of a boundary crossing. The value is serialized once
// with the serializer of the port named by `serialize_uid`, however many
// deserializing ports the crossing has. Every crossing has at least one
// deserializing port.
struct ConnectionSerialization {
  ElementID serialize_uid{};
  std::vector<ConnectionDeserialization> deserializations;
};

struct ConnectionTree {
  ElementID origin_uid{};
  ConnectionSegment root;
};

class ConnectionGraph {
public:
  [[nodiscard]] auto has_incoming_connection(ElementID uid) const noexcept -> bool {
    return connections_.contains(uid);
  }
  [[nodiscard]] auto get_sending_uid(ElementID uid) const noexcept -> std::optional<ElementID>;
  [[nodiscard]] auto get_incoming_connection(ElementID uid) const noexcept
      -> std::optional<std::reference_wrapper<const ConnectionProperties>>;
  auto add_connection(const ConnectionProperties& properties) noexcept -> bool;

  [[nodiscard]] auto connections() const -> auto {
    return connections_ | std::views::transform([](const auto& pair) -> const auto& { return pair.second; });
  }

  // Derives the properties of the end-to-end connection from the origin port
  // to the given receiving port. This accumulates all individual connections
  // found along the way into a single properties object. This assumes that
  // there are no cycles in the connection graph.
  [[nodiscard]] auto get_incoming_end_to_end_connection(ElementID to_uid) const noexcept
      -> std::optional<core::ConnectionProperties>;

  // Computes the connection tree of every origin port with outgoing
  // connections. An origin port is a port without an incoming connection.
  [[nodiscard]] auto compute_connection_trees() const -> std::vector<ConnectionTree>;

private:
  // The to_uid of each connection acts as key. This reflects, that each
  // port may have at most one incoming connection.
  std::unordered_map<ElementID, ConnectionProperties> connections_;
};

} // namespace xronos::core

#endif // XRONOS_CORE_CONNECTION_GRAPH_HH
