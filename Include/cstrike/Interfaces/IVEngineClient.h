#pragma once

class IVEngineClient {
	public:
		int GetLocalPlayer() {
			return GetVirtualFunction<int(__thiscall*)(IVEngineClient*)>(this, 12)(this);
		}

		void SetViewAngles(QAngle& angles) {
			return GetVirtualFunction<void(__thiscall*)(IVEngineClient*, QAngle&)>(this, 20)(this, angles);
		}

		bool IsInGame() {
			return GetVirtualFunction<bool(__thiscall*)(IVEngineClient*)>(this, 26)(this);
		}

		const char* GetLevelName() {
			return GetVirtualFunction<const char*(__thiscall*)(IVEngineClient*)>(this, 51)(this);
		}

		void ClientCmd_Unrestricted(const char* command) {
			return GetVirtualFunction<void(__thiscall*)(IVEngineClient*, const char*)>(this, 106)(this, command);
		}
};

extern IVEngineClient* engine;