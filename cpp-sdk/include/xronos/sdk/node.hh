// SPDX-FileCopyrightText: Copyright (c) Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

/** @file */

#ifndef XRONOS_SDK_NODE_HH
#define XRONOS_SDK_NODE_HH

#include <cstdint>
#include <functional>
#include <memory>
#include <source_location>
#include <string_view>
#include <type_traits>
#include <utility>

#include "xronos/abi/backend.hh"
#include "xronos/abi/node.hh"
#include "xronos/abi/types.hh"
#include "xronos/abi/version.hh" // IWYU pragma: keep (version_major/minor, used by the XRONOS_REGISTER_NODE macro)
#include "xronos/sdk/context.hh"
#include "xronos/sdk/detail/context_access.hh"
#include "xronos/sdk/detail/program_context.hh"
#include "xronos/sdk/detail/source_location.hh"
#include "xronos/sdk/gen/config.hh" // IWYU pragma: keep (config::VERSION, used by the XRONOS_REGISTER_NODE macro)
#include "xronos/sdk/port.hh"
#include "xronos/sdk/reactor.hh"
#include "xronos/sdk/value_type.hh"

namespace xronos::sdk {

/**
 * @brief A Xronos program's entry point, launched with `xronos run`.
 *
 * A node is the root reactor of a program that `xronos run` loads and runs, in
 * place of a hand-written `main()` and Environment. Because a Node is a Reactor,
 * it has the full reactor surface: define its behavior with reactions, build and
 * wire child reactors as data members using context(), and override assemble()
 * — which runs automatically when the program starts.
 *
 * To define a node, derive from Node, add its reactions and any child reactors,
 * and register the class with @ref XRONOS_REGISTER_NODE so `xronos run` can find
 * it:
 *
 * @code
 * #include <iostream>
 *
 * #include <xronos/sdk.hh>
 *
 * namespace sdk = xronos::sdk;
 *
 * class HelloNode final : public sdk::Node {
 * public:
 *   using sdk::Node::Node;
 *
 * private:
 *   class Hello : public sdk::Reaction<HelloNode> {
 *     using sdk::Reaction<HelloNode>::Reaction;
 *     Trigger<void> startup{self().startup(), context()};
 *     void handler() final { std::cout << "Hello, World!\n"; }
 *   };
 *
 *   void assemble() final { add_reaction<Hello>("hello"); }
 * };
 *
 * XRONOS_REGISTER_NODE(HelloNode, "hello_node")
 * @endcode
 *
 * @see XRONOS_REGISTER_NODE
 * @see Reactor
 */
// Node also is the handle the host holds and destroys once the run finishes;
// deriving from abi::Node lets `xronos_create_node` hand it back as that opaque
// handle without the host knowing the concrete SDK type.
class Node : public Reactor, public abi::Node {
public:
  using Reactor::Reactor;

protected:
  /**
   * @brief Declare @p port as part of this node's interface.
   *
   * An exported port is visible outside the node and may be connected to
   * another node's exported port. Ports are internal unless exported.
   * A node's interface is the set of all exported ports.
   *
   * @p port must be one of this node's own ports; exporting anything else
   * throws. To export a child reactor's port, give the node a port
   * of its own and connect the two.
   *
   * This overload exports the port under the encoding named by the
   * serializer the port itself declares, and applies to a valueless port too,
   * which has only one possible encoding. A port left with NoSerializer uses
   * the overload below.
   *
   * Call this from the constructor or from assemble(), like connect(). Each
   * port may be exported once; exporting the same port twice throws.
   *
   * @param port One of this node's own ports.
   */
  template <class T, template <class> class Serializer>
    requires HasValueTypeName<T> && (std::is_same_v<T, void> || IsSerializer<Serializer<T>, T>)
  void export_port(const InputPort<T, Serializer>& port) {
    export_declared<T, Serializer>(port.uid());
  }

  /**
   * @overload
   */
  template <class T, template <class> class Serializer>
    requires HasValueTypeName<T> && (std::is_same_v<T, void> || IsSerializer<Serializer<T>, T>)
  void export_port(const OutputPort<T, Serializer>& port) {
    export_declared<T, Serializer>(port.uid());
  }

  /**
   * @brief Declare @p port with @p serializer as part of this node's interface.
   *
   * For a port that declares no serializer of its own. Everything else matches
   * the overloads above.
   *
   * @p serializer fixes how the port's values are laid out in bytes; its
   * @c encoding name is declared alongside the value type's name from
   * ValueType. Two ports connect only when their value type and encoding names
   * are equal. This means that the caller is responsible for ensuring that the
   * encoding and the value type are uniquely named, and that the schema does
   * not change in an incompatible way without changing the name. Prefer a
   * structurally versioned encoding — such as protocol buffers — that tolerates
   * the type gaining or losing fields.
   *
   * @param port One of this node's own ports, declaring no serializer.
   * @param serializer The serializer defining the encoding the port is exported under.
   */
  template <class T, template <class> class Serializer, class ExportSerializer>
    requires HasValueTypeName<T> && IsSerializer<ExportSerializer, T> && std::is_same_v<Serializer<T>, NoSerializer<T>>
  void export_port(const InputPort<T, Serializer>& port, ExportSerializer serializer) {
    detail::export_port<T>(uid(), port.uid(), context(), std::move(serializer));
  }

  /**
   * @overload
   */
  template <class T, template <class> class Serializer, class ExportSerializer>
    requires HasValueTypeName<T> && IsSerializer<ExportSerializer, T> && std::is_same_v<Serializer<T>, NoSerializer<T>>
  void export_port(const OutputPort<T, Serializer>& port, ExportSerializer serializer) {
    detail::export_port<T>(uid(), port.uid(), context(), std::move(serializer));
  }

private:
  // Shared by the two overloads that export a port under its own declared
  // encoding, which differ only in the port type they accept.
  template <class T, template <class> class Serializer> void export_declared(std::uint64_t port_uid) {
    if constexpr (std::is_same_v<T, void>) {
      detail::export_void_port(uid(), port_uid, context());
    } else {
      detail::export_port<T>(uid(), port_uid, context(), Serializer<T>{});
    }
  }
};

} // namespace xronos::sdk

namespace xronos::sdk::detail {

// The program context a loaded node runs against: unlike DefaultProgramContext,
// it does not own a backend but borrows the one the host supplies to
// xronos_create_node. The host is responsible for keeping the backend alive for
// the node's whole lifetime.
class BorrowedProgramContext final : public ProgramContext {
public:
  explicit BorrowedProgramContext(abi::Backend& backend) noexcept
      : backend_{backend} {}

  [[nodiscard]] auto backend() const noexcept -> abi::Backend& final { return backend_; }

private:
  std::reference_wrapper<abi::Backend> backend_;
};

// Builds NodeClass against the host's backend and returns the opaque abi::Node
// the host owns. The backend is wrapped in a borrowing program context, typed
// as the ProgramContext interface so it is an lvalue the context refers to (not
// a temporary) and co-owned by the node through its Element base.
// XRONOS_REGISTER_NODE's creator entry points are thin wrappers over these
// overloads; the source location defaults to that registration site.
// make_unique + .release() keeps construction exception-safe and hands the
// caller ownership without flagging as an owning-raw-pointer-returning
// function.
//
// This overload builds an environment context, so the node becomes a
// top-level reactor of the host's model.
template <class NodeClass>
[[nodiscard]] auto create_node(const char* name, abi::Backend* backend,
                               std::source_location source_location = std::source_location::current()) -> abi::Node* {
  std::shared_ptr<ProgramContext> program_context = std::make_shared<BorrowedProgramContext>(*backend);
  const Context context{
      ContextAccess::create_environment_context(program_context, SourceLocationView::from_std(source_location))};
  return std::make_unique<NodeClass>(std::string_view{name}, context).release();
}

// This overload builds a reactor context for `parent` (the uid of a reactor
// the host registered against the same backend), so the node becomes a child
// of that reactor, exactly as if it were declared inside it.
template <class NodeClass>
[[nodiscard]] auto create_node(const char* name, abi::Backend* backend, abi::ElementUid parent,
                               std::source_location source_location = std::source_location::current()) -> abi::Node* {
  std::shared_ptr<ProgramContext> program_context = std::make_shared<BorrowedProgramContext>(*backend);
  const Context context{
      ContextAccess::create_reactor_context(program_context, parent, SourceLocationView::from_std(source_location))};
  return std::make_unique<NodeClass>(std::string_view{name}, context).release();
}

} // namespace xronos::sdk::detail

/**
 * @brief Register @p NodeClass as the node that `xronos run` loads from this
 * module.
 *
 * Invoke once at namespace scope, after defining @p NodeClass. This is what
 * turns a compiled `.so` into a runnable Xronos module: `xronos run` finds the
 * registered node, checks that the module is compatible with the CLI, then
 * constructs it under @p default_name (or the name passed to `xronos run
 * --name`) and runs it.
 *
 * @param NodeClass A concrete subclass of xronos::sdk::Node, constructible from
 * a `std::string_view` name and a `const xronos::sdk::Context&` (inheriting
 * Node's constructor with `using Node::Node;` is enough when it has no other
 * members to construct).
 * @param default_name The node's default name, overridable with
 * `xronos run --name`.
 *
 * @see Node
 */
// A macro is required here (rather than a function or template) so the three
// entry points it emits are single, hand-named, extern "C" symbols that
// `xronos run` can resolve by name via dlsym, regardless of NodeClass's own
// (mangled) name.
//
// The three entry points are the module's whole side of the node/host contract:
//   * xronos_node_descriptor advertises the ABI version the module was built
//     against (the host's compatibility gate) plus the SDK version and default
//     name; the host reads it before constructing anything.
//   * xronos_create_top_level_node builds the node against the host's backend
//     as a top-level reactor; xronos_create_node builds it as a child of the
//     host reactor named by `parent`. Both delegate to detail::create_node
//     (which wraps the backend in a borrowing program context the host keeps
//     alive for the node's whole lifetime); the host picks one per placement.
// NOLINTNEXTLINE(cppcoreguidelines-macro-usage)
#define XRONOS_REGISTER_NODE(NodeClass, default_name)                                                                  \
  extern "C" __attribute__((visibility("default"))) auto xronos_node_descriptor()                                      \
      -> const ::xronos::abi::NodeDescriptor* {                                                                        \
    static constexpr ::xronos::abi::NodeDescriptor descriptor{::xronos::abi::version_major,                            \
                                                              ::xronos::abi::version_minor,                            \
                                                              ::xronos::sdk::config::VERSION.data(), default_name};    \
    return &descriptor;                                                                                                \
  }                                                                                                                    \
  extern "C" __attribute__((visibility("default"))) auto xronos_create_node(                                           \
      const char* name, ::xronos::abi::Backend* backend, ::xronos::abi::ElementUid parent) -> ::xronos::abi::Node* {   \
    return ::xronos::sdk::detail::create_node<NodeClass>(name, backend, parent);                                       \
  }                                                                                                                    \
  extern "C" __attribute__((visibility("default"))) auto xronos_create_top_level_node(                                 \
      const char* name, ::xronos::abi::Backend* backend) -> ::xronos::abi::Node* {                                     \
    return ::xronos::sdk::detail::create_node<NodeClass>(name, backend);                                               \
  }

#endif // XRONOS_SDK_NODE_HH
