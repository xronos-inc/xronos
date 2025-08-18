// SPDX-FileCopyrightText: Copyright (c) 2025 Xronos Inc.
// SPDX-FileCopyrightText: Copyright (c) 2019 TU Dresden
// SPDX-License-Identifier: BSD-3-Clause

#ifndef XRONOS_RUNTIME_REACTOR_ELEMENT_HH
#define XRONOS_RUNTIME_REACTOR_ELEMENT_HH

#include <atomic>
#include <cstdint>
#include <functional>
#include <string>
#include <string_view>

#include "xronos/runtime/environment.hh"
#include "xronos/runtime/fwd.hh"

namespace xronos::runtime {

class ReactorElement {
private:
  std::string name_{};
  std::string fqn_{};
  std::uint64_t uid_{};

  // The reactor owning this element
  Reactor* container_{nullptr};
  std::reference_wrapper<Environment> environment_;

  inline static std::atomic<std::uint64_t> uid_counter_{0};

  static auto generate_uid() -> std::uint64_t { return uid_counter_.fetch_add(1, std::memory_order_relaxed); }
  static auto compute_fqn(const Reactor& container, std::string_view name) -> std::string;

public:
  ReactorElement(std::string_view name, Reactor& container);
  ReactorElement(std::string_view name, Environment& environment);
  virtual ~ReactorElement() = default;

  // not copyable, but movable
  ReactorElement(const ReactorElement&) = delete;
  ReactorElement(ReactorElement&&) = default;

  auto operator=(const ReactorElement&) -> ReactorElement& = delete;
  auto operator=(ReactorElement&&) -> ReactorElement& = default;

  [[nodiscard]] auto container() const noexcept -> Reactor* { return container_; }

  [[nodiscard]] auto name() const noexcept -> const std::string& { return name_; }
  [[nodiscard]] auto fqn() const noexcept -> const std::string& { return fqn_; }
  [[nodiscard]] auto uid() const noexcept -> std::uint64_t { return uid_; }
  [[nodiscard]] auto environment() noexcept -> Environment& { return environment_; }
  [[nodiscard]] auto environment() const noexcept -> const Environment& { return environment_; }

  [[nodiscard]] auto is_top_level() const noexcept -> bool { return this->container() == nullptr; }

  virtual void startup() = 0;
  virtual void shutdown() = 0;

  // Get a string that represents a concrete type of reactor element
  [[nodiscard]] virtual auto element_type() const -> std::string_view = 0;

  virtual void visit(ReactorElementVisitor& visitor) const = 0;
};

class ReactorElementVisitor {
public:
  virtual ~ReactorElementVisitor() = default;

  virtual void visit(const Reactor& reactor) = 0;
  virtual void visit(const Reaction& reaction) = 0;
  virtual void visit(const BaseAction& action) = 0;
  virtual void visit(const Port& port) = 0;
  virtual void visit(const Timer& timer) = 0;
  virtual void visit(const StartupTrigger& startup) = 0;
  virtual void visit(const ShutdownTrigger& shutdown) = 0;
  virtual void visit(const MiscElement& misc) = 0;
};

} // namespace xronos::runtime

#endif // XRONOS_RUNTIME_REACTOR_ELEMENT_HH
