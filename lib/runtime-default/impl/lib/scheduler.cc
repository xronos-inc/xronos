// SPDX-FileCopyrightText: Copyright (c) 2025 Xronos Inc.
// SPDX-FileCopyrightText: Copyright (c) 2019 TU Dresden
// SPDX-License-Identifier: BSD-3-Clause

#include "xronos/runtime/default/impl/scheduler.hh"

#include <algorithm>
#include <atomic>
#include <cstddef>
#include <memory>
#include <mutex>
#include <shared_mutex>
#include <sstream>
#include <utility>
#include <vector>

#include "xronos/runtime/default/impl/action.hh"
#include "xronos/runtime/default/impl/assert.hh"
#include "xronos/runtime/default/impl/environment.hh"
#include "xronos/runtime/default/impl/logical_time.hh"
#include "xronos/runtime/default/impl/port.hh"
#include "xronos/runtime/default/impl/reaction.hh"
#include "xronos/runtime/default/impl/time.hh"
#include "xronos/runtime/default/impl/time_barrier.hh"
#include "xronos/util/assert.hh"
#include "xronos/util/logging.hh"

namespace xronos::runtime::default_::impl {

// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
thread_local const Worker* Worker::current_worker = nullptr;

void Worker::work() const {
  // initialize the current worker thread local variable
  current_worker = this;

  log_.debug() << "Starting";

  if (identity_ == 0) {
    log_.debug() << "do the initial scheduling";
    scheduler_->schedule();
  }

  while (true) {
    // wait for a ready reaction
    auto* reaction = scheduler_->ready_queue_.pop();

    // receiving a nullptr indicates that the worker should terminate
    if (reaction == nullptr) {
      break;
    }

    // execute the reaction
    execute_reaction(reaction);

    // was this the very last reaction?
    if (scheduler_->reactions_to_process_.fetch_sub(1, std::memory_order_acq_rel) == 1) {
      // Yes, then schedule. The atomic decrement above ensures that only one
      // thread enters this block.
      scheduler_->schedule();
    }
    // continue otherwise
  }

  log_.debug() << "terminates";
}

void Worker::execute_reaction(Reaction* reaction) const {
  log_.debug() << "execute reaction " << reaction->fqn();

  try {
    reaction->trigger();
  } catch (...) {
    log_.error() << "Exception caught during execution of reaction " << reaction->fqn();
    scheduler_->stop();
    scheduler_->set_exception();
  }
}

void Scheduler::schedule() noexcept {
  bool found_ready_reactions = schedule_ready_reactions();

  while (!found_ready_reactions) {
    if (!continue_execution_ && !found_ready_reactions) {
      // Cleanup and let all workers know that they should terminate.
      cleanup_after_tag();
      terminate_all_workers();
      break;
    }

    log_.debug() << "call next()";
    next();
    reaction_queue_pos_ = 0;

    found_ready_reactions = schedule_ready_reactions();
  }
}

auto ReadyQueue::pop() -> Reaction* {
  auto old_size = size_.fetch_sub(1, std::memory_order_acq_rel);

  // If there is no ready reaction available, wait until there is one.
  while (old_size <= 0) {
    log_.debug() << "Worker " << Worker::current_worker_id() << " now waits for work";
    sem_.acquire();
    log_.debug() << "Worker " << Worker::current_worker_id() << " wakes up";
    old_size = size_.fetch_sub(1, std::memory_order_acq_rel);
    // FIXME: Protect against underflow?
  }

  auto pos = old_size - 1;
  return queue_[pos];
}

void ReadyQueue::fill_up(std::vector<Reaction*>& ready_reactions) {
  // clear the internal queue and swap contents
  queue_.clear();
  queue_.swap(ready_reactions);

  // update the atomic size counter and release the semaphore to wake up
  // waiting worker threads
  auto new_size = static_cast<std::ptrdiff_t>(queue_.size());
  auto old_size = size_.exchange(new_size, std::memory_order_acq_rel);

  // calculate how many workers to wake up. -old_size indicates the number of
  // workers who started waiting since the last update.
  // We want to wake up at most all the waiting workers. If we would release
  // more, other workers that are out of work would not block when acquiring
  // the semaphore.
  // Also, we do not want to wake up more workers than there is work. new_size
  // indicates the number of ready reactions. Since there is always at least
  // one worker running running, new_size - running_workers indicates the
  // number of additional workers needed to process all reactions.
  waiting_workers_ += -old_size;
  std::ptrdiff_t running_workers{num_workers_ - waiting_workers_};
  auto workers_to_wakeup = std::min(waiting_workers_, new_size - running_workers);

  // wakeup other workers_
  if (workers_to_wakeup > 0) {
    waiting_workers_ -= workers_to_wakeup;
    log_.debug() << "Wakeup " << workers_to_wakeup << " workers";
    sem_.release(static_cast<int>(workers_to_wakeup));
  }
}

void EventQueue::fill_action_list_pool() {
  for (std::size_t i{0}; i < action_list_pool_increment_; i++) {
    action_list_pool_.emplace_back(std::make_unique<ActionList>());
  }
}

auto EventQueue::next_tag() const -> Tag {
  util::assert_(!event_queue_.empty());
  return event_queue_.begin()->first;
}

auto EventQueue::extract_next_event() -> ActionListPtr {
  util::assert_(!event_queue_.empty());
  return std::move(event_queue_.extract(event_queue_.begin()).mapped());
}

auto EventQueue::insert_event_at(const Tag& tag) -> const ActionListPtr& {
  auto shared_lock = std::shared_lock<std::shared_mutex>(mutex_);

  auto event_it = event_queue_.find(tag);
  if (event_it == event_queue_.end()) {
    shared_lock.unlock();
    {
      auto unique_lock = std::unique_lock<std::shared_mutex>(mutex_);
      if (action_list_pool_.empty()) {
        fill_action_list_pool();
      }
      const auto& result = event_queue_.try_emplace(tag, std::move(action_list_pool_.back()));
      if (result.second) {
        action_list_pool_.pop_back();
      }
      return result.first->second;
    }
  } else {
    return event_it->second;
  }
}

void EventQueue::return_action_list(ActionListPtr&& action_list) {
  util::assert_(action_list != nullptr);
  action_list_pool_.emplace_back(std::move(action_list));
}

void EventQueue::discard_events_until_tag(const Tag& tag) {
  while (!empty() && next_tag() <= tag) {
    auto actions = extract_next_event();
    return_action_list(std::move(actions));
  }
}

void Scheduler::terminate_all_workers() {
  log_.debug() << "Send termination signal to all workers";
  auto num_workers = environment_->num_workers();
  std::vector<Reaction*> null_reactions{num_workers, nullptr};
  ready_queue_.fill_up(null_reactions);
}

auto Scheduler::schedule_ready_reactions() -> bool {
  // insert any triggered reactions_ into the reaction queue
  for (auto& vec_reaction : triggered_reactions_) {
    for (auto* reaction : vec_reaction) {
      reaction_queue_[reaction->index()].push_back(reaction);
    }
    vec_reaction.clear();
  }

  log_.debug() << "Scanning the reaction queue for ready reactions";

  // continue iterating over the reaction queue
  for (; reaction_queue_pos_ < reaction_queue_.size(); reaction_queue_pos_++) {
    auto& reactions = reaction_queue_[reaction_queue_pos_];

    // any ready reactions of current priority?
    if (!reactions.empty()) {
      log_.debug() << "Process reactions of priority " << reaction_queue_pos_;

      // Make sure that any reaction is only executed once even if it
      // was triggered multiple times.
      std::ranges::sort(reactions);
      auto items_to_erase = std::ranges::unique(reactions);
      reactions.erase(items_to_erase.begin(), items_to_erase.end());

      if constexpr (util::log::debug_enabled) {
        for (auto* reaction : reactions) {
          log_.debug() << "Reaction " << reaction->fqn() << " is ready for execution";
        }
      }

      reactions_to_process_.store(static_cast<std::ptrdiff_t>(reactions.size()), std::memory_order_release);
      ready_queue_.fill_up(reactions);

      // break out of the loop and return
      return true;
    }
  }

  log_.debug() << "Reached end of reaction queue";
  return false;
}

void Scheduler::start() {
  log_.debug() << "Starting the scheduler...";

  {
    // Other schedulers (enclaves or federates) could try to access our logical
    // time and our event queue. Thus, we need to lock the main scheduling mutex
    // in order to avoid data races.
    std::lock_guard<std::mutex> lock_guard(scheduling_mutex_);

    // Initialize our logical time to the value right before the start tag. This
    // is important for usage with enclaves/federates, to indicate, that no events
    // before the start tag can be generated.
    logical_time_.advance_to(environment_->start_tag().decrement());

    // It could happen that another scheduler (enclave or federates) already
    // tried to acquire a tag before our start tag. In that case, we will have
    // empty events on the queue, that are earlier than our startup event. We
    // resolve this simply by deleting all such events. Once we have processed
    // the startup reactions, the start tag will be released, and consequently
    // also all earlier tags are released.
    event_queue_.discard_events_until_tag(Tag::from_logical_time(logical_time_));
  }

  auto num_workers = environment_->num_workers();
  // initialize the reaction queue, set ports vector, and triggered reactions
  // vector
  reaction_queue_.resize(environment_->max_reaction_index() + 1);
  set_ports_.resize(num_workers);
  triggered_reactions_.resize(num_workers);

  // Initialize and start the workers. By resizing the workers vector first,
  // we make sure that there is sufficient space for all the workers and non of
  // them needs to be moved. This is important because a running worker may not
  // be moved.
  workers_.reserve(num_workers);
  for (unsigned i = 0; i < num_workers; i++) {
    std::stringstream stream;
    stream << "Worker " << environment_->name() << " " << i;
    workers_.emplace_back(this, i, stream.str());
    workers_.back().start_thread();
  }

  // join all worker threads
  for (auto& worker : workers_) {
    worker.join_thread();
  }
}

void Scheduler::advance_logical_time_to(const Tag& tag) {
  log_.debug() << "advance logical time to tag " << tag;
  logical_time_.advance_to(tag);
}

void Scheduler::cleanup_after_tag() {
  // Notify other environments and let them know that we finished processing the
  // current tag
  release_current_tag();

  // clean up all actions triggered at the current tag (if there are any remaining)
  if (triggered_actions_ != nullptr) {
    // cleanup all triggered actions
    for (auto* action : *triggered_actions_) {
      action->cleanup();
    }

    triggered_actions_->clear();
  }

  // cleanup all set ports
  for (auto& vec_ports : set_ports_) {
    for (auto& port : vec_ports) {
      port->cleanup();
    }
    vec_ports.clear();
  }
}

void Scheduler::next() { // NOLINT(readability-function-cognitive-complexity)
  // First, clean up after the last tag.
  cleanup_after_tag();

  {
    std::unique_lock<std::mutex> lock{scheduling_mutex_};

    while (triggered_actions_ == nullptr || triggered_actions_->empty()) {
      if (triggered_actions_ != nullptr) {
        event_queue_.return_action_list(std::move(triggered_actions_));
      }
      util::assert_(triggered_actions_ == nullptr);

      // shutdown if there are no more events in the queue
      if (event_queue_.empty() && !stop_) {
        if (environment_->run_forever()) {
          // wait for a new asynchronous event
          cv_schedule_.wait(lock, [this]() { return !event_queue_.empty() || stop_; });
        } else {
          log_.debug() << "No more events in queue_. -> Terminate!";
          environment_->sync_shutdown();
        }
      }

      if (stop_) {
        continue_execution_ = false;
        log_.debug() << "Shutting down the scheduler";
        auto current_tag = Tag::from_logical_time(logical_time_);
        if (current_tag < environment_->start_tag()) {
          // processing the first event initializes the logical time, so we initialize the logical
          // time in case no prior event initialized it
          current_tag = environment_->start_tag();
        }
        Tag t_next = current_tag.delay();
        if (!event_queue_.empty() && t_next == event_queue_.next_tag()) {
          log_.debug() << "Trigger the last round of reactions including all "
                          "shutdown reactions";
          triggered_actions_ = event_queue_.extract_next_event();
          advance_logical_time_to(t_next);
        } else {
          return;
        }
      } else {
        auto t_next = event_queue_.next_tag();

        // synchronize with physical time if not in fast forward mode
        if (!environment_->fast_fwd_execution()) {
          log_.debug() << "acquire tag " << t_next << " from physical time barrier";
          bool result = PhysicalTimeBarrier::acquire_tag(
              t_next, lock, this, [&t_next, this]() { return t_next != event_queue_.next_tag(); });
          // If acquire tag returns false, then a new event was inserted into the queue and we need to start over
          if (!result) {
            log_.debug() << "abort waiting and restart as the event queue was modified";
            continue;
          }
        }

        // Wait until all input actions mark the tag as safe to process.
        bool result{true};
        for (auto* action : environment_->input_actions_) {
          log_.debug() << "acquire tag " << t_next << " from input action " << action->fqn();
          bool inner_result =
              action->acquire_tag(t_next, lock, [&t_next, this]() { return t_next != event_queue_.next_tag(); });
          // If the wait was aborted or if the next tag changed in the meantime,
          // we need to break from the loop and continue with the main loop.
          if (!inner_result || t_next != event_queue_.next_tag()) {
            result = false;
            break;
          }
        }
        // If acquire tag returns false, then a new event was inserted into the queue and we need to start over
        if (!result) {
          log_.debug() << "abort waiting and restart as the event queue was modified";
          continue;
        }

        // Stop execution in case we reach the timeout tag. This checks needs to
        // be done here, after acquiring the check, as only then we are fully
        // commited to executing the tag t_next. Otherwise, we could still get
        // earlier events (e.g., from a physical action).
        if (t_next == environment_->timeout_tag()) {
          continue_execution_ = false;
          log_.debug() << "Shutting down the scheduler due to timeout";
          log_.debug() << "Trigger the last round of reactions including all "
                          "shutdwon reactions";
        }

        // Retrieve all triggered actions at the next tag.
        // We do not need to lock mutex_event_queue_ here, as the lock on
        // scheduling_mutex_ already ensures that no one can write to the event
        // queue.
        triggered_actions_ = event_queue_.extract_next_event();

        advance_logical_time_to(t_next);

        // If there are no triggered actions at the event, then release the
        // current tag and go back to the start of the loop
        if (triggered_actions_->empty()) {
          // It is important to unlock the mutex here. Otherwise we could enter a deadlock as
          // releasing a tag also requires holding the downstream mutex.
          lock.unlock();
          release_current_tag();
          lock.lock();
        }
      }
    }
  } // mutex schedule_

  // iterate over all events/actions, call setup and insert scheduled reactions
  log_.debug() << "events: " << triggered_actions_->size();
  for (auto* action : *triggered_actions_) {
    log_.debug() << "Action " << action->fqn();
    action->setup();
    for (auto* reaction : action->triggers()) {
      // There is no need to acquire the mutex. At this point the scheduler
      // should be the only thread accessing the reaction queue as none of the
      // workers_ are running
      log_.debug() << "insert reaction " << reaction->fqn() << " with index " << reaction->index();
      reaction_queue_[reaction->index()].push_back(reaction);
    }
  }
}

Scheduler::Scheduler(Environment* env)
    : using_workers_(env->num_workers() > 1)
    , environment_(env)
    , log_("Scheduler " + env->name())
    , ready_queue_(log_, env->num_workers()) {}

Scheduler::~Scheduler() = default;

void Scheduler::schedule_sync(BaseAction* action, const Tag& tag) {
  log_.debug() << "Schedule action " << action->fqn() << (action->is_logical() ? " synchronously " : " asynchronously ")
               << " with tag " << tag;
  util::assert_(logical_time_ < tag);

  const auto& action_list = event_queue_.insert_event_at(tag);
  action_list->push_back(action);
}

auto Scheduler::schedule_async(BaseAction* action, const Duration& delay) -> Tag {
  Tag tag;
  {
    std::lock_guard<std::mutex> lock_guard(scheduling_mutex_);
    tag = Tag::from_physical_time(get_physical_time() + delay);
    schedule_sync(action, tag);
  }
  notify();
  return tag;
}

auto Scheduler::schedule_async_at(BaseAction* action, const Tag& tag) -> bool {
  {
    std::lock_guard<std::mutex> lock_guard(scheduling_mutex_);
    if (tag <= logical_time_) {
      return false;
    }
    schedule_sync(action, tag);
  }
  notify();
  return true;
}

auto Scheduler::schedule_empty_async_at(const Tag& tag) -> bool {
  {
    std::lock_guard<std::mutex> lock_guard(scheduling_mutex_);
    if (tag <= logical_time_) {
      // If we try to insert an empty event at the current logical time, then we
      // succeeded because there must be an event at this tag that is currently
      // processed.
      bool result{tag == logical_time_};
      log_.debug() << "try to schedule empty event at tag " << tag << (result ? " -> succeeded" : " -> failed");
      return result;
    }
    event_queue_.insert_event_at(tag);
  }
  notify();
  log_.debug() << "try to schedule empty event at tag " << tag << " -> succeeded";
  return true;
}

auto Scheduler::set_exception() -> void { environment_->set_exception(); }

void Scheduler::set_port(Port* port) {
  log_.debug() << "Set port " << port->fqn();

  // We do not check here if port is already in the list. This means clean()
  // could be called multiple times for a single port. However, calling
  // clean() multiple time is not harmful and more efficient then checking if
  set_ports_[Worker::current_worker_id()].push_back(port);

  // recursively search for triggered reactions
  set_port_helper(port);
}

void Scheduler::set_port_helper(Port* port) {
  // record the port for cleaning it up later
  set_ports_[Worker::current_worker_id()].push_back(port);

  // Call the 'set' callback on the port
  port->invoke_set_callback();

  // Record all triggered reactions
  for (auto* reaction : port->triggers()) {
    triggered_reactions_[Worker::current_worker_id()].push_back(reaction);
  }

  // Continue recursively
  for (auto* binding : port->outward_bindings()) {
    set_port_helper(binding);
  }
}

void Scheduler::stop() {
  stop_ = true;
  notify();
}

void Scheduler::register_release_tag_callback(const ReleaseTagCallback& callback) {
  // Callbacks should only be registered during assembly, which happens strictly
  // sequentially. Therefore, we should be fine accessing the vector directly
  // and do not need to lock.
  validate(environment_->phase() <= Phase::Assembly,
           "registering callbacks is only allowed during construction and assembly");
  release_tag_callbacks_.push_back(callback);
}

void Scheduler::release_current_tag() {
  log_.debug() << "Release tag " << logical_time_;
  for (const auto& callback : release_tag_callbacks_) {
    callback(logical_time_);
  }
}

} // namespace xronos::runtime::default_::impl
