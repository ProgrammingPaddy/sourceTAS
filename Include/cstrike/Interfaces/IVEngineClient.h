#pragma once

class IVEngineClient {
	public:
		int GetLocalPlayer() {
			return GetVirtualFunction<int(*)(IVEngineClient*)>(this, 12)(this);
		}

		void GetViewAngles(QAngle& angles) {
			return GetVirtualFunction<void(*)(IVEngineClient*, QAngle&)>(this, 19)(this, angles);
		}

		void SetViewAngles(QAngle& angles) {
			return GetVirtualFunction<void(*)(IVEngineClient*, QAngle&)>(this, 20)(this, angles);
		}

		bool IsInGame() {
			return GetVirtualFunction<bool(*)(IVEngineClient*)>(this, 26)(this);
		}

		// player_info_t out-buffer (name[32] first). Standard VEngineClient013
		// slot; RUNTIME-VALIDATED before first trust (DemoCap checks the local
		// player's name is printable and disables the call otherwise).
		bool GetPlayerInfo(int ent_num, void* pinfo) {
			return GetVirtualFunction<bool(*)(IVEngineClient*, int, void*)>(
				this, 8)(this, ent_num, pinfo);
		}

		const char* GetLevelName() {
			return GetVirtualFunction<const char*(*)(IVEngineClient*)>(this, 51)(this);
		}

		void ClientCmd_Unrestricted(const char* command) {
			return GetVirtualFunction<void(*)(IVEngineClient*, const char*)>(this, 106)(this, command);
		}
};

extern IVEngineClient* engine;