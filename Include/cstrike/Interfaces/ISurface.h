#pragma once

class ISurface {
	public:
		void SetCursorAlwaysVisible(bool visible) {
			return GetVirtualFunction<void(*)(ISurface*, bool)>(this, 52)(this, visible);
		}
};

extern ISurface* matsurface;