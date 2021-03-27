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

		virtual void OnEndScene();
		virtual bool OnInputMessage(UINT, WPARAM, LPARAM);
};