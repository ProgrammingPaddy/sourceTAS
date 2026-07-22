#include "NetVars.h"

#include <cstring>
#include <string>
#include <unordered_map>

#include <cstrike/sdk.h>
#include <cstrike/Classes/ClientClass.h>
#include <cstrike/Classes/RecvTable.h>

namespace {
	// Depth-first search for 'field' within a receive table, accumulating the
	// offsets of any nested data tables along the way. Returns -1 when absent.
	int ResolveInTable(RecvTable* table, const char* field) {
		for (int i = 0; i < table->m_nProps; ++i) {
			RecvProp* prop = &table->m_pProps[i];
			const char* name = prop->m_pVarName;

			if (name && std::strcmp(name, field) == 0)
				return prop->m_Offset;

			if (prop->m_RecvType == DPT_DataTable && prop->m_pDataTable) {
				int sub = ResolveInTable(prop->m_pDataTable, field);
				if (sub >= 0)
					return prop->m_Offset + sub;
			}
		}
		return -1;
	}
}

int NetVars::Offset(const char* table, const char* field) {
	static std::unordered_map<std::string, int> cache;

	const std::string key = std::string(table) + "->" + field;
	const auto it = cache.find(key);
	if (it != cache.end())
		return it->second;

	int result = 0;

	if (clientdll) {
		// 1) Preferred: resolve within the named top-level class table.
		for (ClientClass* cc = clientdll->GetAllClasses(); cc; cc = cc->m_pNext) {
			RecvTable* rt = cc->m_pRecvTable;
			if (rt && rt->m_pNetTableName && std::strcmp(rt->m_pNetTableName, table) == 0) {
				const int off = ResolveInTable(rt, field);
				if (off > 0) { result = off; break; }
			}
		}

		// 2) Fallback: any networked class that exposes the field.
		if (result == 0) {
			for (ClientClass* cc = clientdll->GetAllClasses(); cc; cc = cc->m_pNext) {
				RecvTable* rt = cc->m_pRecvTable;
				if (!rt) continue;
				const int off = ResolveInTable(rt, field);
				if (off > 0) { result = off; break; }
			}
		}
	}

	cache[key] = result;
	return result;
}
