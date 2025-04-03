// SPDX-FileCopyrightText: Copyright (c) 2025 Xronos Inc.
// SPDX-FileCopyrightText: Copyright (c) 2019 TU Dresden
// SPDX-License-Identifier: BSD-3-Clause

#ifndef XRONOS_RUNTIME_ENVIRONMENT_HH
#define XRONOS_RUNTIME_ENVIRONMENT_HH

#include <set>
#include <string>
#include <vector>

#include "connection_properties.hh"
#include "data_logger.hh"
#include "fwd.hh"
#include "graph.hh"
#include "logging.hh"
#include "scheduler.hh"
#include "time.hh"

namespace xronos::runtime {

constexpr unsigned int default_number_worker = 1;
constexpr unsigned int default_max_reaction_index = 0;
constexpr bool default_run_forever = false;
constexpr bool default_fast_fwd_execution = false;

enum class Phase : std::uint8_t {
  Construction = 0,
  Assembly = 1,
  Startup = 2,
  Execution = 3,
  Shutdown = 4,
  Deconstruction = 5
};

class Environment {
private:
  using Dependency = std::pair<Reaction*, Reaction*>;

  std::string name_{};
  log::NamedLogger log_;
  unsigned int num_workers_{default_number_worker};
  unsigned int max_reaction_index_{default_max_reaction_index};
  bool run_forever_{default_run_forever};
  const bool fast_fwd_execution_{default_fast_fwd_execution};

  std::set<Reactor*> top_level_reactors_{};
  /// Set of actions that act as an input to the reactor program in this environment
  std::set<BaseAction*> input_actions_{};
  std::set<Reaction*> reactions_{};
  std::vector<Dependency> dependencies_{};

  /// The environment containing this environment. nullptr if this is the top environment
  Environment* containing_environment_{nullptr};
  /// Set of all environments contained by this environment
  std::set<Environment*> contained_environments_{};
  /// Pointer to the top level environment
  Environment* top_environment_{nullptr};

  Scheduler scheduler_;
  Phase phase_{Phase::Construction};

  /// Timeout as given in the constructor
  const Duration timeout_{};

  /// Exception thrown during a reaction, if that is the reason for the shutdown
  std::mutex exception_mutex_{};
  std::exception_ptr exception_{};

  /// The start tag as determined during startup()
  Tag start_tag_{};
  /// The timeout tag as determined during startup()
  Tag timeout_tag_{};

  Graph<BasePort*, ConnectionProperties> graph_{};
  Graph<BasePort*, ConnectionProperties> optimized_graph_{};

  void build_dependency_graph(Reactor* reactor);
  void calculate_indexes();

  std::mutex shutdown_mutex_{};

  NoopRuntimeDataLogger default_data_logger_{};
  std::reference_wrapper<RuntimeDataLogger> data_logger_{default_data_logger_};

  auto startup(const TimePoint& start_time) -> std::thread;

public:
  explicit Environment(unsigned int num_workers, bool fast_fwd_execution = default_fast_fwd_execution,
                       const Duration& timeout = Duration::max());
  explicit Environment(const std::string& name, Environment* containing_environment);
  virtual ~Environment() = default;

  auto name() -> const std::string& { return name_; }

  // this method draw a connection between two graph elements with some properties
  template <class T> void draw_connection(Port<T>& source, Port<T>& sink, ConnectionProperties properties) {
    this->draw_connection(&source, &sink, properties);
  }

  template <class T> void draw_connection(Port<T>* source, Port<T>* sink, ConnectionProperties properties) {
    if (top_environment_ == nullptr || top_environment_ == this) {
      log::Debug() << "drawing connection: " << source->fqn() << " --> " << sink->fqn();
      auto existing_source = graph_.source_for(sink);
      if (existing_source.has_value()) {
        std::stringstream error_message;
        error_message << "multiple sources (both " << existing_source.value()->fqn() << " and " << source->fqn()
                      << ") connected to port " << sink->fqn();
        throw ValidationError(error_message.str());
      }
      graph_.add_edge(source, sink, properties);
    } else {
      top_environment_->draw_connection(source, sink, properties);
    }
  }

  void optimize();

  void register_top_level_reactor(Reactor& reactor);
  void register_input_action(BaseAction& action);
  void assemble();
  auto startup() -> std::thread;
  void sync_shutdown();
  void async_shutdown();
  void set_exception();
  void rethrow_exception_if_any();

  void export_dependency_graph(const std::string& path);

  [[nodiscard]] auto top_level_reactors() const noexcept -> const auto& { return top_level_reactors_; }
  [[nodiscard]] auto phase() const noexcept -> Phase { return phase_; }
  [[nodiscard]] auto scheduler() const noexcept -> const Scheduler* { return &scheduler_; }

  auto scheduler() noexcept -> Scheduler* { return &scheduler_; }

  [[nodiscard]] auto logical_time() const noexcept -> const LogicalTime& { return scheduler_.logical_time(); }
  [[nodiscard]] auto start_tag() const noexcept -> const Tag& { return start_tag_; }
  [[nodiscard]] auto timeout() const noexcept -> const Duration& { return timeout_; }
  [[nodiscard]] auto timeout_tag() const noexcept -> const Tag& { return timeout_tag_; }

  static auto physical_time() noexcept -> TimePoint { return get_physical_time(); }

  [[nodiscard]] auto num_workers() const noexcept -> unsigned int { return num_workers_; }
  [[nodiscard]] auto fast_fwd_execution() const noexcept -> bool { return fast_fwd_execution_; }
  [[nodiscard]] auto run_forever() const noexcept -> bool { return run_forever_; }
  [[nodiscard]] auto max_reaction_index() const noexcept -> unsigned int { return max_reaction_index_; }

  [[nodiscard]] auto connections() const noexcept -> auto { return graph_.get_edges(); }

  [[nodiscard]] auto data_logger() const noexcept -> auto& { return data_logger_.get(); }
  void set_data_logger(RuntimeDataLogger& data_logger) noexcept { data_logger_ = data_logger; }

  // Walk the reactor element tree and apply the visitor to all elements
  void visit_all_elements(ReactorElementVisitor& visitor) const;

  friend Scheduler;
};
} // namespace xronos::runtime

#endif // XRONOS_RUNTIME_ENVIRONMENT_HH
