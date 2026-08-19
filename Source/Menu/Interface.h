#pragma once

#include <cstrike/sdk.h>
#include <renderer/renderer.h>

class BasehookInterface: public DX9RenderMgr {
	private:
		bool is_menu_visible = false;
	protected:
		BasehookInterface(void) {};
		~BasehookInterface(void) {};
	public:
		BasehookInterface(BasehookInterface const&) = delete;
		BasehookInterface& operator=(BasehookInterface const&) = delete;

		inline static BasehookInterface& GetInstance() {
			static BasehookInterface instance;
			return instance;
		}

		virtual void OnInitialize();
		virtual void OnEndScene();
		virtual bool OnInputMessage(UINT, WPARAM, LPARAM);

		// The HUD + menu body, split out so OnEndScene can run it under a
		// hook-level SEH shell (crash fix 2026-08-16).
		void MenuHudBody();
};