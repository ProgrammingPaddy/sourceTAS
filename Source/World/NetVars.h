#pragma once

#include <cstdint>

// Runtime netvar resolver. Walks client.dll's networked class tree once per
// unique field and caches the resulting byte offset, so we never hardcode a
// field offset (those move between game builds; the network tree does not lie).
// This only reads the layout Valve already publishes - it changes nothing.
namespace NetVars {
	// Byte offset of 'field' within an entity instance. Searches the named
	// top-level table first, then falls back to a global search (inherited
	// fields share one offset, so the first match is safe). Returns 0 if
	// unresolved, which callers treat as "don't read".
	int Offset(const char* table, const char* field);

	// Typed read of a field at a resolved offset from an entity base pointer.
	template <typename T>
	inline T& Get(void* entity, int offset) {
		return *reinterpret_cast<T*>(reinterpret_cast<uintptr_t>(entity) + offset);
	}
}
