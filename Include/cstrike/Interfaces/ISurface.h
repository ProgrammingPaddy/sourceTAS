#pragma once

class ISurface {
	public:
		void SetCursorAlwaysVisible(bool visible) {
			return GetVirtualFunction<void(__thiscall*)(ISurface*, bool)>(this, 52)(this, visible);
		}
};

extern ISurface* matsurface;