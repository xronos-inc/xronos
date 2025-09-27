// SPDX-FileCopyrightText: Copyright (c) 2025 Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

#include "xronos/telemetry/attribute_manager.hh"

#include <cstdint>
#include <mutex>
#include <string_view>
#include <utility>

namespace xronos::telemetry {

auto AttributeManager::try_emplace_and_get_attribute_map(std::uint64_t uid) -> AttributeMap& {
  // This retrieves the attribute map for the given element or creates an empty attribute map if it does not yet exist
  auto result = attribute_maps_.try_emplace(uid, AttributeMap{});
  auto iterator = result.first; // try_emplace returns a tuple, the first entry is an iterator to the element
  return iterator->second;      // the element is a tuple of key and value; return the value
}

void AttributeManager::set_attribute(std::uint64_t uid, std::string_view key, const AttributeValue& value) {
  std::lock_guard<std::mutex> lock(mutex_);
  auto& current_attributes = try_emplace_and_get_attribute_map(uid);
  current_attributes[std::string{key}] = value;
}

void AttributeManager::set_attributes(std::uint64_t uid, AttributeMap attributes) {
  std::lock_guard<std::mutex> lock(mutex_);

  auto& current_attributes = try_emplace_and_get_attribute_map(uid);
  // Swap the current and the new attributes.
  current_attributes.swap(attributes);
  // Merge all old attributes back in. By swapping first and then merging all
  // old entries back in, we ensure that for any keys that are contained in both
  // maps, the value as given in the new attributes takes precedence.
  current_attributes.merge(attributes);
  // At this point attributes should only contain entries for keys contained in
  // both maps. We simply discard them.
}

auto AttributeManager::add_attribute(std::uint64_t uid, std::string_view key, const AttributeValue& value) -> bool {
  std::lock_guard<std::mutex> lock(mutex_);
  auto& current_attributes = try_emplace_and_get_attribute_map(uid);
  auto result = current_attributes.try_emplace(std::string{key}, value);
  return result.second;
}

auto AttributeManager::add_attributes(std::uint64_t uid, AttributeMap attributes) -> bool {
  std::lock_guard<std::mutex> lock(mutex_);
  auto& current_attributes = try_emplace_and_get_attribute_map(uid);
  current_attributes.merge(attributes);
  return attributes.empty();
}

} // namespace xronos::telemetry
