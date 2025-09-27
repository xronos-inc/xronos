// SPDX-FileCopyrightText: Copyright (c) 2025 Xronos Inc.
// SPDX-FileCopyrightText: Copyright (c) 2019 TU Dresden
// SPDX-License-Identifier: BSD-3-Clause

#include "xronos/runtime/default/impl/environment.hh"

#include <algorithm>
#include <exception>
#include <fstream>
#include <functional>
#include <iterator>
#include <map>
#include <mutex>
#include <set>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "xronos/runtime/default/impl/action.hh"
#include "xronos/runtime/default/impl/assert.hh"
#include "xronos/runtime/default/impl/connection_properties.hh"
#include "xronos/runtime/default/impl/port.hh"
#include "xronos/runtime/default/impl/reaction.hh"
#include "xronos/runtime/default/impl/reactor.hh"
#include "xronos/runtime/default/impl/reactor_element.hh"
#include "xronos/runtime/default/impl/time.hh"
#include "xronos/runtime/interfaces.hh"
#include "xronos/util/assert.hh"
#include "xronos/util/logging.hh"

namespace xronos::runtime::default_::impl {

Environment::Environment(unsigned int num_workers, bool fast_fwd_execution, const Duration& timeout)
    : log_("Environment")
    , num_workers_(num_workers)
    , fast_fwd_execution_(fast_fwd_execution)
    , scheduler_(this)
    , timeout_(timeout) {}

void Environment::register_top_level_reactor(Reactor& reactor) {
  validate(this->phase() == Phase::Construction, "Reactors may only be registered during construction phase!");
  validate(reactor.is_top_level(), "The environment may only contain top level reactors!");
  [[maybe_unused]] bool result = top_level_reactors_.insert(&reactor).second;
  util::assert_(result);
}

void Environment::register_input_action(BaseAction& action) {
  validate(this->phase() == Phase::Construction || this->phase() == Phase::Assembly,
           "Input actions may only be registered during construction or assembly phase!");
  [[maybe_unused]] bool result = input_actions_.insert(&action).second;
  util::assert_(result);
  run_forever_ = true;
}

void Environment::draw_connection(Port& source, Port& sink, ConnectionProperties properties) {
  util::log::debug() << "drawing connection: " << source.fqn() << " --> " << sink.fqn();
  auto existing_source = graph_.source_for(&sink);
  if (existing_source.has_value()) {
    std::stringstream error_message;
    error_message << "multiple sources (both " << existing_source.value()->fqn() << " and " << source.fqn()
                  << ") connected to port " << sink.fqn();
    throw ValidationError(error_message.str());
  }
  graph_.add_edge(&source, &sink, properties);
}

void Environment::optimize() {
  // no optimizations
  optimized_graph_ = graph_;
}

void Environment::assemble() { // NOLINT(readability-function-cognitive-complexity)
  validate(phase_ == Phase::Construction, "assemble() called on a program that is already assembled");
  phase_ = Phase::Assembly;

  util::log::debug() << "start optimization on port graph";
  this->optimize();

  util::log::debug() << "instantiating port graph declaration";
  util::log::debug() << "graph: ";
  util::log::debug() << optimized_graph_;

  auto graph = optimized_graph_.get_edges_grouped_by_properties();
  // this generates the port graph
  for (auto const& [source, sinks] : graph) {

    auto* source_port = source.first;
    auto properties = source.second;

    if (properties.type_ == ConnectionType::Normal) {
      for (auto* const destination_port : sinks) {
        destination_port->set_inward_binding(source_port);
        source_port->add_outward_binding(destination_port);
        util::log::debug() << "from: " << source_port->fqn() << "(" << source_port << ")"
                           << " --> to: " << destination_port->fqn() << "(" << destination_port << ")";
      }
    } else {
      source_port->instantiate_connection_to(properties, sinks);

      util::log::debug() << "from: " << source_port->container()->fqn() << " |-> to: " << sinks.size() << " objects";
    }
  }
}

void Environment::build_dependency_graph(Reactor* reactor) {

  // obtain dependencies from each contained reactor
  for (auto* sub_reactor : reactor->contained_reactors()) {
    build_dependency_graph(sub_reactor);
  }
  // get reactions_ from this reactor; also order reactions_ by their priority
  std::map<int, Reaction*> priority_map;
  for (auto* reaction : reactor->reactions()) {
    reactions_.insert(reaction);
    auto result = priority_map.emplace(reaction->priority(), reaction);
    validate(result.second, "priorities must be unique for all reactions_ of the same reactor");
  }

  // connect all reactions_ this reaction depends on
  for (auto* reaction : reactor->reactions()) {
    for (auto* dependency : reaction->dependencies()) {
      auto* source = dependency;
      while (source->has_inward_binding()) {
        source = source->inward_binding();
      }
      for (auto* antidependency : source->anti_dependencies()) {
        dependencies_.emplace_back(reaction, antidependency);
      }
    }
  }

  // connect reactions_ by priority
  if (priority_map.size() > 1) {
    auto iterator = priority_map.begin();
    auto next = std::next(iterator);
    while (next != priority_map.end()) {
      dependencies_.emplace_back(next->second, iterator->second);
      iterator++;
      next = std::next(iterator);
    }
  }
}

void Environment::sync_shutdown() {
  {
    std::lock_guard<std::mutex> lock{shutdown_mutex_};

    if (phase_ >= Phase::Shutdown) {
      // sync_shutdown() was already called -> abort
      return;
    }

    validate(phase_ == Phase::Execution, "sync_shutdown() may only be called during execution phase!");
    phase_ = Phase::Shutdown;
  }

  // the following will only be executed once
  log_.debug() << "Terminating the execution";

  for (auto* reactor : top_level_reactors_) {
    reactor->shutdown();
  }

  phase_ = Phase::Deconstruction;
  scheduler_.stop();
}

void Environment::async_shutdown() {
  [[maybe_unused]] auto lock_guard = scheduler_.lock();
  sync_shutdown();
}

void Environment::set_exception() {
  std::lock_guard<std::mutex> lock(exception_mutex_);
  auto current_exception = std::current_exception();
  if (exception_ == nullptr) {
    exception_ = current_exception;
  } else {
    try {
      std::rethrow_exception(exception_);
    } catch (const std::exception& ex) {
      util::log::error() << "Exception already set. Dropping exception with message:\n    " << ex.what();
    }
  }
}

void Environment::rethrow_exception_if_any() const {
  std::lock_guard<std::mutex> lock(exception_mutex_);
  if (exception_ != nullptr) {
    std::rethrow_exception(exception_);
  }
}

auto dot_name([[maybe_unused]] ReactorElement* reactor_element) -> std::string {
  std::string fqn{reactor_element->fqn()};
  std::ranges::replace(fqn, '.', '_');
  return fqn;
}

void Environment::export_dependency_graph(const std::string& path) {
  std::ofstream dot;
  dot.open(path);

  // sort all reactions_ by their index
  std::map<unsigned int, std::vector<Reaction*>> reactions_by_index;
  for (auto* reaction : reactions_) {
    reactions_by_index[reaction->index()].push_back(reaction);
  }

  // start the graph
  dot << "digraph {\n";
  dot << "rankdir=LR;\n";

  // place reactions_ of the same index in the same subgraph
  for (auto& index_reactions : reactions_by_index) {
    dot << "subgraph {\n";
    dot << "rank=same;\n";
    for (auto* reaction : index_reactions.second) {
      dot << dot_name(reaction) << " [label=\"" << reaction->fqn() << "\"];\n";
    }
    dot << "}\n";
  }

  // establish an order between subgraphs
  Reaction* reaction_from_last_index = nullptr;
  for (auto& index_reactions : reactions_by_index) {
    Reaction* reaction_from_this_index = index_reactions.second.front();
    if (reaction_from_last_index != nullptr) {
      dot << dot_name(reaction_from_last_index) << " -> " << dot_name(reaction_from_this_index) << " [style=invis];\n";
    }
    reaction_from_last_index = reaction_from_this_index;
  }

  // add all the dependencies
  for (auto dependency : dependencies_) {
    dot << dot_name(dependency.first) << " -> " << dot_name(dependency.second) << '\n';
  }
  dot << "}\n";

  dot.close();

  log_.info() << "Reaction graph was written to " << path;
}

void Environment::calculate_indexes() {
  // build the graph
  std::map<Reaction*, std::set<Reaction*>> graph;
  for (auto* reaction : reactions_) {
    graph[reaction];
  }
  for (auto dependencies : dependencies_) {
    graph[dependencies.first].insert(dependencies.second);
  }

  log_.debug() << "Reactions sorted by index:";
  unsigned int index = 0;
  while (!graph.empty()) {
    // find nodes with degree zero and assign index
    std::set<Reaction*> degree_zero;
    for (auto& edge : graph) {
      if (edge.second.empty()) {
        edge.first->set_index(index);
        degree_zero.insert(edge.first);
      }
    }

    if (degree_zero.empty()) {
      export_dependency_graph("/tmp/reactor_dependency_graph.dot");
      throw ValidationError("There is a loop in the dependency graph. Graph was written to "
                            "/tmp/reactor_dependency_graph.dot");
    }

    auto dbg = log_.debug();
    dbg << index << ": ";
    for (auto* reaction : degree_zero) {
      dbg << reaction->fqn() << ", ";
    }

    // reduce graph
    for (auto* reaction : degree_zero) {
      graph.erase(reaction);
    }
    for (auto& edge : graph) {
      for (auto* reaction : degree_zero) {
        edge.second.erase(reaction);
      }
    }

    index++;
  }

  max_reaction_index_ = index - 1;
}

auto Environment::startup() -> std::thread { return startup(get_physical_time()); }

auto Environment::startup(const TimePoint& start_time) -> std::thread {
  validate(this->phase() == Phase::Assembly, "startup() may only be called during assembly phase!");

  util::log::debug() << "Building the Dependency-Graph";
  for (auto* reactor : top_level_reactors_) {
    build_dependency_graph(reactor);
  }

  calculate_indexes();

  log_.debug() << "Starting the execution";
  phase_ = Phase::Startup;

  this->start_tag_ = Tag::from_physical_time(start_time);
  if (this->timeout_ == Duration::max()) {
    this->timeout_tag_ = Tag::max();
  } else if (this->timeout_ == Duration::zero()) {
    this->timeout_tag_ = this->start_tag_;
  } else {
    this->timeout_tag_ = this->start_tag_.delay(this->timeout_);
  }

  // start up initialize all reactors
  for (auto* reactor : top_level_reactors_) {
    reactor->startup();
  }

  // start processing events
  phase_ = Phase::Execution;

  return std::thread([this]() {
    // start scheduler and wait until it returns
    this->scheduler_.start();
  });
}

// A visitor that walks the reactor instance tree and applies another given
// visitor to all elements.
class TreeWalkVisitor : public ReactorElementVisitor {
private:
  std::reference_wrapper<ReactorElementVisitor> visitor;

public:
  TreeWalkVisitor(ReactorElementVisitor& visitor)
      : visitor(visitor) {}

  void visit(const Reactor& reactor) final {
    reactor.visit(visitor);
    for (auto* elem : reactor.elements()) {
      elem->visit(*this);
    }
  }
  void visit(const Reaction& reaction) final { reaction.visit(visitor); }
  void visit(const BaseAction& action) final { action.visit(visitor); }
  void visit(const Port& port) final { port.visit(visitor); }
  void visit(const Timer& timer) final { timer.visit(visitor); }
  void visit(const StartupTrigger& startup) final { startup.visit(visitor); }
  void visit(const ShutdownTrigger& shutdown) final { shutdown.visit(visitor); }
};

void Environment::visit_all_elements(ReactorElementVisitor& visitor) const {
  TreeWalkVisitor walker{visitor};
  for (auto* reactor : top_level_reactors()) {
    reactor->visit(walker);
  }
}

} // namespace xronos::runtime::default_::impl
