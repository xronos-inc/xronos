// SPDX-FileCopyrightText: Copyright (c) 2025 Xronos Inc.
// SPDX-FileCopyrightText: Copyright (c) 2019 TU Dresden
// SPDX-License-Identifier: BSD-3-Clause

#ifndef XRONOS_RUNTIME_PORT_HH
#define XRONOS_RUNTIME_PORT_HH

#include <cstdint>
#include <set>
#include <vector>

#include "assert.hh"
#include "connection_properties.hh"
#include "fwd.hh"
#include "reactor_element.hh"
#include "value_ptr.hh"

namespace xronos::runtime {

class BasePort : public ReactorElement {
private:
  BasePort* inward_binding_{nullptr};
  std::set<BasePort*> outward_bindings_{};

  std::set<Reaction*> dependencies_{};
  std::set<Reaction*> triggers_{};
  std::set<Reaction*> anti_dependencies_{};

  PortCallback set_callback_{nullptr};
  PortCallback clean_callback_{nullptr};

protected:
  bool present_{false}; // NOLINT cppcoreguidelines-non-private-member-variables-in-classes

  BasePort(std::string_view name, Reactor& container)
      : ReactorElement(name, container) {}

  void register_dependency(Reaction* reaction, bool is_trigger) noexcept;
  void register_antidependency(Reaction* reaction) noexcept;
  virtual void cleanup() = 0;

  void invoke_set_callback() noexcept {
    if (set_callback_ != nullptr) {
      set_callback_(*this);
    }
  }

  void invoke_clean_callback() noexcept {
    if (clean_callback_ != nullptr) {
      clean_callback_(*this);
    }
  }

public:
  void set_inward_binding(BasePort* port) noexcept { inward_binding_ = port; }
  void add_outward_binding(BasePort* port) noexcept { outward_bindings_.insert(port); }

  virtual void instantiate_connection_to(const ConnectionProperties& properties,
                                         const std::vector<BasePort*>& downstreams) = 0;

  [[nodiscard]] auto is_present() const noexcept -> bool {
    if (has_inward_binding()) {
      return inward_binding()->is_present();
    }
    return present_;
  };

  [[nodiscard]] auto has_inward_binding() const noexcept -> bool { return inward_binding_ != nullptr; }
  [[nodiscard]] auto has_outward_bindings() const noexcept -> bool { return !outward_bindings_.empty(); }
  [[nodiscard]] auto has_dependencies() const noexcept -> bool { return !dependencies_.empty(); }
  [[nodiscard]] auto has_anti_dependencies() const noexcept -> bool { return !anti_dependencies_.empty(); }

  [[nodiscard]] auto inward_binding() const noexcept -> BasePort* { return inward_binding_; }
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

  friend class Reaction;
  friend class Scheduler;
};

template <class T> class Port : public BasePort {
private:
  ImmutableValuePtr<T> value_ptr_{nullptr};

  void cleanup() noexcept final {
    value_ptr_ = nullptr;
    present_ = false;
    invoke_clean_callback();
  }

public:
  using value_type = T;

  Port(std::string_view name, Reactor& container)
      : BasePort(name, container) {}

  void instantiate_connection_to(const ConnectionProperties& properties,
                                 const std::vector<BasePort*>& downstream) override;
  [[nodiscard]] auto typed_inward_binding() const noexcept -> Port<T>*;
  [[nodiscard]] auto typed_outward_bindings() const noexcept -> const std::set<Port<T>*>&;

  virtual void set(const ImmutableValuePtr<T>& value_ptr);
  void set(MutableValuePtr<T>&& value_ptr) { set(ImmutableValuePtr<T>(std::move(value_ptr))); }
  void set(const T& value) { set(make_immutable_value<T>(value)); }
  void set(T&& value) { set(make_immutable_value<T>(std::move(value))); }

  // Setting a port to nullptr is not permitted. We use requires to delete
  // set() if it is actually called with nullptr. Without requires, set(0) would
  // be ambiguous as 0 can be implicitly casted to nullptr_t.
  template <typename V>
  void set(V)
    requires(std::is_same_v<V, std::nullptr_t>)
  = delete;

  void startup() final {}
  void shutdown() final {}

  [[nodiscard]] auto get() const noexcept -> const ImmutableValuePtr<T>&;
};

template <> class Port<void> : public BasePort {
private:
  void cleanup() noexcept final {
    present_ = false;
    invoke_clean_callback();
  }

public:
  using value_type = void;

  Port(std::string_view name, Reactor& container)
      : BasePort(name, container) {}

  void instantiate_connection_to(const ConnectionProperties& properties,
                                 const std::vector<BasePort*>& downstream) override;
  [[nodiscard]] auto typed_inward_binding() const noexcept -> Port<void>*;
  [[nodiscard]] auto typed_outward_bindings() const noexcept -> const std::set<Port<void>*>&;

  void set();

  void startup() final {}
  void shutdown() final {}
};

template <class T> class Input : public Port<T> {
public:
  Input(std::string_view name, Reactor& container)
      : Port<T>(name, container) {}

  [[nodiscard]] auto is_input() const -> bool final { return true; }
  [[nodiscard]] auto is_output() const -> bool final { return false; }
};

template <class T> class Output : public Port<T> {
public:
  Output(std::string_view name, Reactor& container)
      : Port<T>(name, container) {}

  [[nodiscard]] auto is_input() const -> bool final { return false; }
  [[nodiscard]] auto is_output() const -> bool final { return true; }
};

} // namespace xronos::runtime

#include "impl/port_impl.hh"

#endif // XRONOS_RUNTIME_PORT_HH
