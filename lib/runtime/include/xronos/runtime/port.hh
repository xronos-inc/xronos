// SPDX-FileCopyrightText: Copyright (c) 2025 Xronos Inc.
// SPDX-FileCopyrightText: Copyright (c) 2019 TU Dresden
// SPDX-License-Identifier: BSD-3-Clause

#ifndef XRONOS_RUNTIME_PORT_HH
#define XRONOS_RUNTIME_PORT_HH

#include <any>
#include <set>
#include <string_view>
#include <vector>

#include "xronos/runtime/connection_properties.hh"
#include "xronos/runtime/fwd.hh"
#include "xronos/runtime/reactor_element.hh"

namespace xronos::runtime {

class Port : public ReactorElement {
private:
  Port* inward_binding_{nullptr};
  std::set<Port*> outward_bindings_{};

  std::set<Reaction*> dependencies_{};
  std::set<Reaction*> triggers_{};
  std::set<Reaction*> anti_dependencies_{};

  std::any current_value_{};

  PortCallback set_callback_{nullptr};
  PortCallback clean_callback_{nullptr};

  void cleanup() noexcept { current_value_.reset(); }

protected:
  void register_dependency(Reaction* reaction, bool is_trigger) noexcept;
  void register_antidependency(Reaction* reaction);

  void invoke_set_callback();
  void invoke_clean_callback();

public:
  using ReactorElement::ReactorElement;

  void set_inward_binding(Port* port) noexcept { inward_binding_ = port; }
  void add_outward_binding(Port* port) noexcept { outward_bindings_.insert(port); }

  void instantiate_connection_to(const ConnectionProperties& properties, const std::vector<Port*>& downstreams);

  [[nodiscard]] auto is_present() const noexcept -> bool;

  [[nodiscard]] auto has_inward_binding() const noexcept -> bool { return inward_binding_ != nullptr; }
  [[nodiscard]] auto has_outward_bindings() const noexcept -> bool { return !outward_bindings_.empty(); }
  [[nodiscard]] auto has_dependencies() const noexcept -> bool { return !dependencies_.empty(); }
  [[nodiscard]] auto has_anti_dependencies() const noexcept -> bool { return !anti_dependencies_.empty(); }

  [[nodiscard]] auto inward_binding() const noexcept -> Port* { return inward_binding_; }
  [[nodiscard]] auto outward_bindings() const noexcept -> const auto& { return outward_bindings_; }

  [[nodiscard]] auto triggers() const noexcept -> const auto& { return triggers_; }
  [[nodiscard]] auto dependencies() const noexcept -> const auto& { return dependencies_; }
  [[nodiscard]] auto anti_dependencies() const noexcept -> const auto& { return anti_dependencies_; }

  void register_set_callback(const PortCallback& callback);
  void register_clean_callback(const PortCallback& callback);

  [[nodiscard]] virtual auto is_input() const -> bool = 0;
  [[nodiscard]] virtual auto is_output() const -> bool = 0;

  [[nodiscard]] auto element_type() const -> std::string_view final { return "port"; };
  void visit(ReactorElementVisitor& visitor) const final { visitor.visit(*this); };

  void startup() final {}
  void shutdown() final {}

  void set(const std::any& value);
  [[nodiscard]] auto get() const noexcept -> const std::any&;

  friend class Reaction;
  friend class Scheduler;
};

class Input : public Port {
public:
  using Port::Port;

  [[nodiscard]] auto is_input() const -> bool final { return true; }
  [[nodiscard]] auto is_output() const -> bool final { return false; }
};

class Output : public Port {
public:
  using Port::Port;

  [[nodiscard]] auto is_input() const -> bool final { return false; }
  [[nodiscard]] auto is_output() const -> bool final { return true; }
};

} // namespace xronos::runtime

#endif // XRONOS_RUNTIME_PORT_HH
