// SPDX-FileCopyrightText: Copyright (c) 2025 Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

#include "xronos/telemetry/attribute_manager.hh"

#include <mutex>
#include <optional>
#include <string_view>
#include <utility>

#include "xronos/runtime/reactor_element.hh"

namespace xronos::telemetry {

auto AttributeManager::try_emplace_and_get_attribute_map(const runtime::ReactorElement& element) -> AttributeMap& {
  // This retrieves the attribute map for the given element or creates an empty attribute map if it does not yet exist
  auto result = attribute_maps_.try_emplace(element.uid(), AttributeMap{});
  auto iterator = result.first; // try_emplace returns a tuple, the first entry is an iterator to the element
  return iterator->second;      // the element is a tuple of key and value; return the value
}

void AttributeManager::set_attribute(const runtime::ReactorElement& element, std::string_view key,
                                     AttributeValue value) {
  std::lock_guard<std::mutex> lock(mutex_);
  auto& current_attributes = try_emplace_and_get_attribute_map(element);
  current_attributes[std::string{key}] = std::move(value);
}

void AttributeManager::set_attributes(const runtime::ReactorElement& element, AttributeMap attributes) {
  std::lock_guard<std::mutex> lock(mutex_);

  auto& current_attributes = try_emplace_and_get_attribute_map(element);
  // Swap the current and the new attributes.
  current_attributes.swap(attributes);
  // Merge all old attributes back in. By swapping first and then merging all
  // old entries back in, we ensure that for any keys that are contained in both
  // maps, the value as given in the new attributes takes precedence.
  current_attributes.merge(attributes);
  // At this point attributes should only contain entries for keys contained in
  // both maps. We simply discard them.
}

auto AttributeManager::add_attribute(const runtime::ReactorElement& element, std::string_view key,
                                     const AttributeValue& value) -> bool {
  std::lock_guard<std::mutex> lock(mutex_);
  auto& current_attributes = try_emplace_and_get_attribute_map(element);
  auto result = current_attributes.try_emplace(std::string{key}, value);
  return result.second;
}

auto AttributeManager::add_attributes(const runtime::ReactorElement& element, AttributeMap attributes) -> bool {
  std::lock_guard<std::mutex> lock(mutex_);
  auto& current_attributes = try_emplace_and_get_attribute_map(element);
  current_attributes.merge(attributes);
  return attributes.empty();
}

auto AttributeManager::get_attributes(const runtime::ReactorElement& element) const -> std::optional<AttributeMap> {
  std::lock_guard<std::mutex> lock(mutex_);
  auto iterator = attribute_maps_.find(element.uid());
  if (iterator != attribute_maps_.end()) {
    return iterator->second;
  }
  return std::nullopt;
}

} // namespace xronos::telemetry
