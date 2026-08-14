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

// How a connection relates to a node boundary crossing. A crossing spans a
// serialized region of the connection graph. Values inside a region exist
// only in serialized form.
//
// - `None`: a plain connection.
// - `Exit`: the value leaves its node. It is serialized with the serializer
//   of `from_uid`, which opens a serialized region.
// - `Entry`: the value enters a node. It is deserialized with the serializer
//   of `to_uid`, which closes the region.
// - `Both`: a single-connection crossing. The value is serialized with the
//   serializer of `from_uid` and deserialized with the serializer of
//   `to_uid` on the same connection.
enum class BoundaryCrossing : std::uint8_t {
  None,
  Exit,
  Entry,
  Both,
};

struct ConnectionProperties {
  ElementID from_uid;
  ElementID to_uid;
  std::optional<Duration> delay;
  BoundaryCrossing crossing{BoundaryCrossing::None};
};

// The delivery tree of an origin port describes where a value written to
// that port is delivered. Leaves are the receiving ports. Interior nodes are
// node boundary crossings, where values are serialized and deserialized.

// A receiving port together with the delay accumulated along the path from
// the origin port.
struct DeliveryLeaf {
  ElementID port_uid{};
  std::optional<Duration> delay;
};

struct BoundarySerialization;

// The receiving ports and boundary crossings reachable from one point in
// the tree without crossing a further boundary.
struct DeliverySubtree {
  std::vector<DeliveryLeaf> leaves;
  std::vector<BoundarySerialization> serializations;
};

// One deserializing side of a boundary crossing. The value is deserialized
// with the serializer of the port named by `deserialize_uid` and then
// delivered to the subtree below.
struct BoundaryDeserialization {
  ElementID deserialize_uid{};
  DeliverySubtree subtree;
};

// One serializing side of a boundary crossing. The value is serialized once
// with the serializer of the port named by `serialize_uid`, no matter how
// many branches leave the port.
struct BoundarySerialization {
  ElementID serialize_uid{};
  std::vector<BoundaryDeserialization> branches;
};

struct DeliveryTree {
  ElementID origin_uid{};
  DeliverySubtree root;
};

class ConnectionGraph {
public:
  [[nodiscard]] auto has_incoming_connection(ElementID uid) const noexcept -> bool {
    return connections_.contains(uid);
  }
  [[nodiscard]] auto get_upstream_uid(ElementID uid) const noexcept -> std::optional<ElementID>;
  [[nodiscard]] auto get_incoming_connection(ElementID uid) const noexcept
      -> std::optional<std::reference_wrapper<const ConnectionProperties>>;
  auto add_connection(const ConnectionProperties& properties) noexcept -> bool;

  [[nodiscard]] auto connections() const -> auto {
    return connections_ | std::views::transform([](const auto& pair) -> const auto& { return pair.second; });
  }

  // Derives connection properties for end to end connection from its upstream
  // origin to the given downstream port. This accumulates all individual
  // connections found along the way into a single properties object. This
  // assumes that there are no cycles in the connection graph.
  [[nodiscard]] auto get_incoming_end_to_end_connection(ElementID to_uid) const noexcept
      -> std::optional<core::ConnectionProperties>;

  // Computes the delivery tree of every origin port (ports without an
  // incoming connection).
  [[nodiscard]] auto compute_delivery_trees() const -> std::vector<DeliveryTree>;

private:
  // The to_uid of each connection acts as key. This reflects, that each
  // port may have at most one incoming connection.
  std::unordered_map<ElementID, ConnectionProperties> connections_;
};

} // namespace xronos::core

#endif // XRONOS_CORE_CONNECTION_GRAPH_HH
