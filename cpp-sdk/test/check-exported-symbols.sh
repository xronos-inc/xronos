#!/usr/bin/env bash
# Regression guard for the exported surface of libxronos-sdk.so, which a
# version script (xronos-sdk.map) restricts to an allowlist. Assert both
# halves of that allowlist:
#
#   * Positive -- the compiled SDK API is present, and so is the RTTI of the
#     exception types thrown inside the library and caught by type across the
#     boundary. The abi exceptions live in `inline namespace v1`; if the map
#     spells them without the v1 the export silently no-ops and cross-DSO
#     catch degrades to name comparison (invisible to tests, broken on
#     macOS/libc++). Pinning the typeinfo here makes that failure loud.
#   * Negative -- NOTHING else under xronos:: is exported. Every internal
#     symbol (the whole runtime implementation, the context/provider
#     vtables and typeinfos, template instantiations) must be hidden; a
#     single leaked xronos:: name that is not the SDK API or one of the two
#     boundary-crossing abi exceptions fails the check.
#
# Usage: check-exported-symbols.sh <path-to-libxronos-sdk.so>
set -euo pipefail

so="${1:?usage: check-exported-symbols.sh <shared-object>}"

if ! command -v nm >/dev/null 2>&1; then
  echo "check-exported-symbols: 'nm' not found; cannot verify exported symbols" >&2
  exit 1
fi

# Defined, exported (dynamic) symbols, demangled, one name per line. Captured
# into a single string so membership tests use a here-string (a plain grep
# with no upstream process, avoiding pipefail/SIGPIPE surprises).
exported_names="$(nm --dynamic --demangle --defined-only "$so" | sed -E 's/^[0-9a-fA-F]+ [A-Za-z] //')"

fail=0

require_demangled() {
  if ! grep -Fq -- "$1" <<<"$exported_names"; then
    echo "FAIL: expected exported symbol '$1' was not found" >&2
    fail=1
  fi
}

# Positive: the compiled SDK API. The factory is what every program's inline
# Environment constructors link against; losing it breaks consumers at link
# time only after installation, so pin it here.
require_demangled 'xronos::sdk::detail::create_default_program_context()'
require_demangled 'xronos::sdk::Environment::execute()'
require_demangled 'xronos::sdk::Environment::enable_telemetry('
# The compiled low-level constructor the inline public constructors delegate to.
require_demangled 'xronos::sdk::Environment::Environment(bool,'
require_demangled 'xronos::sdk::DefaultRuntimeProvider::version() const'
# DefaultRuntimeProvider is constructed in consumer code (the `xronos run`
# version probe) while its vtable lives only here, so it must stay exported.
require_demangled 'vtable for xronos::sdk::DefaultRuntimeProvider'
# RTTI of the boundary-crossing exceptions -- the v1 spelling is the trap.
require_demangled 'typeinfo for xronos::sdk::VersionMismatchError'
require_demangled 'typeinfo for xronos::abi::v1::ValidationError'
require_demangled 'typeinfo for xronos::abi::v1::InvalidNameError'

# Negative (the core invariant): the ONLY xronos:: symbols exported are the
# SDK API/RTTI and the RTTI of the two boundary-crossing abi exceptions.
# Demangle every defined dynamic symbol, strip the address/type columns and
# any "vtable for " / "typeinfo [name] for " decorator, then anything under
# xronos:: that is not xronos::sdk:: nor one of those abi exceptions is a leak.
leaked="$(printf '%s\n' "$exported_names" \
  | sed -E 's/^(typeinfo name|typeinfo|vtable) for //' \
  | grep -E '^xronos::' \
  | grep -vE '^xronos::sdk::' \
  | grep -vE '^xronos::abi::v1::(ValidationError|InvalidNameError)$' \
  | sort -u || true)"

if [[ -n "$leaked" ]]; then
  echo "FAIL: unexpected internal xronos symbols are exported from $so:" >&2
  echo "$leaked" >&2
  fail=1
fi

if [[ "$fail" -ne 0 ]]; then
  echo "check-exported-symbols: $so exports an unexpected symbol set (see above)." >&2
  exit 1
fi

echo "check-exported-symbols: OK ($so exports only its public surface)"
