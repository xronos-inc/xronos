// SPDX-FileCopyrightText: Copyright (c) Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

#include <concepts>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <span>
#include <string_view>
#include <utility>
#include <vector>

#include "xronos/sdk.hh"
#include "gtest/gtest.h"

namespace xronos::sdk::test {

namespace {

// A serializer: converts, and names the layout it produces. It can therefore
// sit on a port, carry an export from that port, or be supplied at an export
// site.
template <class T> struct CountSerializer {
  static constexpr std::string_view encoding = "count.v1";

  static auto serialize(const T& value) -> std::vector<std::byte> {
    std::vector<std::byte> bytes(sizeof(value));
    for (std::size_t i = 0; i < sizeof(value); ++i) {
      bytes[i] = static_cast<std::byte>(value >> (i * 8U));
    }
    return bytes;
  }

  static auto deserialize(std::span<const std::byte> data) -> T {
    T value{0};
    for (std::size_t i = 0; i < data.size(); ++i) {
      value |= static_cast<T>(data[i]) << (i * 8U);
    }
    return value;
  }
};

// Converts, but names no encoding, so it is not a serializer: naming the
// layout is part of what a serializer is, not an extra earned by ports that
// happen to be exported.
template <class T> struct UnnamedSerializer {
  static auto serialize(const T& value) -> std::vector<std::byte> { return CountSerializer<T>::serialize(value); }
  static auto deserialize(std::span<const std::byte> data) -> T { return CountSerializer<T>::deserialize(data); }
};

// Carries configuration, and is therefore neither copyable nor default
// constructible. A port could not declare it — a port constructs its own
// serializer and has nothing to configure it with — but an export site
// supplies an instance, so this is exactly what the concept promises to allow.
struct ConfiguredSerializer {
  static constexpr std::string_view encoding = "count.v1";

  std::unique_ptr<std::uint64_t> bias;

  explicit ConfiguredSerializer(std::uint64_t bias_value)
      : bias{std::make_unique<std::uint64_t>(bias_value)} {}

  [[nodiscard]] auto serialize(const std::uint64_t& value) const -> std::vector<std::byte> {
    return CountSerializer<std::uint64_t>::serialize(value + *bias);
  }

  [[nodiscard]] auto deserialize(std::span<const std::byte> data) const -> std::uint64_t {
    return CountSerializer<std::uint64_t>::deserialize(data) - *bias;
  }
};

static_assert(IsSerializer<CountSerializer<std::uint64_t>, std::uint64_t>);
static_assert(!IsSerializer<UnnamedSerializer<std::uint64_t>, std::uint64_t>);
// The tag stands for the absence of serialization, so it is not one either.
static_assert(!IsSerializer<NoSerializer<std::uint64_t>, std::uint64_t>);

// Default constructibility is asked of a port's serializer parameter, not of
// serializers as such, so a configured serializer is still a serializer.
static_assert(IsSerializer<ConfiguredSerializer, std::uint64_t>);
static_assert(!std::default_initializable<ConfiguredSerializer>);
static_assert(!std::copy_constructible<ConfiguredSerializer>);

} // namespace

TEST(exported_ports, ExportsPortsThatDeclareNoSerializer) {
  class Exporting : public Node {
  public:
    Exporting(std::string_view name, const Context& context)
        : Node{name, context} {
      export_port(input_, CountSerializer<std::uint64_t>{});
      export_port(output_, CountSerializer<std::uint64_t>{});
      export_port(trigger_);
    }

  private:
    InputPort<std::uint64_t> input_{"input", context()};
    OutputPort<std::uint64_t> output_{"output", context()};
    InputPort<void> trigger_{"trigger", context()};

    void assemble() final {}
  };

  TestEnvironment env{};
  EXPECT_NO_THROW(Exporting("node", env.context()));
}

TEST(exported_ports, ExportsWithASerializerCarryingConfiguration) {
  // The serializer is moved through to the adapter rather than copied, so a
  // move-only one works and its configuration is not duplicated.
  class Exporting : public Node {
  public:
    Exporting(std::string_view name, const Context& context)
        : Node{name, context} {
      export_port(output_, ConfiguredSerializer{100});
    }

  private:
    OutputPort<std::uint64_t> output_{"output", context()};

    void assemble() final {}
  };

  TestEnvironment env{};
  EXPECT_NO_THROW(Exporting("node", env.context()));
}

TEST(exported_ports, ExportsPortsUnderTheirDeclaredEncoding) {
  // A port whose own serializer names an encoding is exported without
  // repeating it at the export site.
  class Exporting : public Node {
  public:
    Exporting(std::string_view name, const Context& context)
        : Node{name, context} {
      export_port(input_);
      export_port(output_);
    }

  private:
    InputPort<std::uint64_t, CountSerializer> input_{"input", context()};
    OutputPort<std::uint64_t, CountSerializer> output_{"output", context()};

    void assemble() final {}
  };

  TestEnvironment env{};
  EXPECT_NO_THROW(Exporting("node", env.context()));
}

TEST(exported_ports, ThrowOnExportingAPortOfAChildReactor) {
  class Child : public Reactor {
  public:
    using Reactor::Reactor;

    [[nodiscard]] auto output() const noexcept -> auto& { return output_; }

  private:
    OutputPort<std::uint64_t> output_{"output", context()};

    void assemble() final {}
  };

  class Exporting : public Node {
  public:
    Exporting(std::string_view name, const Context& context)
        : Node{name, context} {
      export_port(child_.output(), CountSerializer<std::uint64_t>{});
    }

  private:
    Child child_{"child", context()};

    void assemble() final {}
  };

  TestEnvironment env{};
  EXPECT_THROW(Exporting("node", env.context()), ValidationError);
}

TEST(exported_ports, ThrowOnExportingTheSamePortTwice) {
  class Exporting : public Node {
  public:
    Exporting(std::string_view name, const Context& context)
        : Node{name, context} {
      export_port(output_, CountSerializer<std::uint64_t>{});
      export_port(output_, CountSerializer<std::uint64_t>{});
    }

  private:
    OutputPort<std::uint64_t> output_{"output", context()};

    void assemble() final {}
  };

  TestEnvironment env{};
  EXPECT_THROW(Exporting("node", env.context()), ValidationError);
}

TEST(exported_ports, ExportsFromAssemble) {
  class Exporting : public Node {
  public:
    using Node::Node;

  private:
    OutputPort<std::uint64_t> output_{"output", context()};

    void assemble() final { export_port(output_, CountSerializer<std::uint64_t>{}); }
  };

  TestEnvironment env{};
  Exporting node{"node", env.context()};
  EXPECT_NO_THROW(env.execute());
}

} // namespace xronos::sdk::test
