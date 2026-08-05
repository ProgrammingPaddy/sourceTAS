#pragma once

// Shared visual language for every sourceTAS panel: one theme function plus a
// handful of small helpers (headings, primary/danger buttons, tabs) and the
// success/warning/error palette. Header-only inline so both the menu and the
// editor share exactly one style. Targets the vendored ImGui 1.50 API.

#include <imgui/imgui.h>

namespace Theme {

	// ---- palette --------------------------------------------------------
	// Dark graphite base, pink accent, and consistent semantic colors.
	static const ImVec4 Pink   (0.93f, 0.29f, 0.60f, 1.00f);  // accent / selected tab
	static const ImVec4 PinkHi (1.00f, 0.39f, 0.68f, 1.00f);
	static const ImVec4 Blue   (0.26f, 0.55f, 0.96f, 1.00f);  // primary action
	static const ImVec4 BlueHi (0.36f, 0.63f, 1.00f, 1.00f);
	static const ImVec4 Red    (0.84f, 0.24f, 0.28f, 1.00f);  // destructive action
	static const ImVec4 RedHi  (0.92f, 0.34f, 0.38f, 1.00f);
	static const ImVec4 Success(0.40f, 0.82f, 0.47f, 1.00f);  // green
	static const ImVec4 Warning(0.98f, 0.72f, 0.26f, 1.00f);  // amber
	static const ImVec4 Error  (0.96f, 0.42f, 0.42f, 1.00f);  // red text
	static const ImVec4 Muted  (0.56f, 0.56f, 0.62f, 1.00f);

	// ---- runtime menu opacity ------------------------------------------
	// One knob (exposed on the Rendering tab) that makes the menu's surfaces -
	// window/child/frame backgrounds, buttons, headers - translucent so the game
	// shows through the panel. Text, checkmarks and slider grips stay fully
	// opaque so readings remain crisp at any opacity. 1.0 = solid.
	inline float& MenuOpacity() { static float v = 0.80f; return v; }

	// The opaque palette captured by Apply(); the fixed base ApplyOpacity()
	// scales from each frame (scaling the live colors would compound and decay).
	inline ImVec4* BasePalette() { static ImVec4 base[ImGuiCol_COUNT] = {}; return base; }
	inline bool& BaseCaptured() { static bool v = false; return v; }

	// ---- one theme function --------------------------------------------
	inline void Apply() {
		ImGuiStyle& s = ImGui::GetStyle();
		s.WindowRounding      = 6.f;
		s.ChildWindowRounding = 6.f;
		s.FrameRounding       = 4.f;
		s.GrabRounding        = 4.f;
		s.ScrollbarRounding   = 6.f;
		s.WindowPadding       = ImVec2(12.f, 10.f);
		s.FramePadding        = ImVec2(9.f, 5.f);
		s.ItemSpacing         = ImVec2(8.f, 7.f);
		s.ItemInnerSpacing    = ImVec2(7.f, 5.f);
		s.IndentSpacing       = 18.f;
		s.ScrollbarSize       = 13.f;
		s.GrabMinSize         = 9.f;
		s.WindowTitleAlign    = ImVec2(0.5f, 0.5f);
		s.AntiAliasedLines    = true;
		s.AntiAliasedShapes   = true;

		ImVec4* c = s.Colors;
		const ImVec4 bg   (0.085f, 0.088f, 0.10f, 1.00f);
		const ImVec4 child(0.11f,  0.115f, 0.13f, 1.00f);
		const ImVec4 frame(0.16f,  0.165f, 0.19f, 1.00f);
		c[ImGuiCol_Text]                 = ImVec4(0.90f, 0.91f, 0.94f, 1.00f);
		c[ImGuiCol_TextDisabled]         = Muted;
		c[ImGuiCol_WindowBg]             = bg;
		c[ImGuiCol_ChildWindowBg]        = child;
		c[ImGuiCol_PopupBg]              = ImVec4(0.10f, 0.10f, 0.12f, 0.98f);
		c[ImGuiCol_Border]               = ImVec4(0.26f, 0.26f, 0.30f, 0.55f);
		c[ImGuiCol_BorderShadow]         = ImVec4(0.00f, 0.00f, 0.00f, 0.00f);
		c[ImGuiCol_FrameBg]              = frame;
		c[ImGuiCol_FrameBgHovered]       = ImVec4(0.21f, 0.215f, 0.25f, 1.00f);
		c[ImGuiCol_FrameBgActive]        = ImVec4(0.25f, 0.255f, 0.30f, 1.00f);
		c[ImGuiCol_TitleBg]              = ImVec4(0.10f, 0.10f, 0.12f, 1.00f);
		c[ImGuiCol_TitleBgActive]        = ImVec4(0.15f, 0.15f, 0.18f, 1.00f);
		c[ImGuiCol_TitleBgCollapsed]     = ImVec4(0.10f, 0.10f, 0.12f, 0.80f);
		c[ImGuiCol_MenuBarBg]            = ImVec4(0.12f, 0.12f, 0.14f, 1.00f);
		c[ImGuiCol_ScrollbarBg]          = bg;
		c[ImGuiCol_ScrollbarGrab]        = ImVec4(0.26f, 0.26f, 0.30f, 1.00f);
		c[ImGuiCol_ScrollbarGrabHovered] = ImVec4(0.34f, 0.34f, 0.40f, 1.00f);
		c[ImGuiCol_ScrollbarGrabActive]  = Pink;
		c[ImGuiCol_ComboBg]              = ImVec4(0.13f, 0.13f, 0.155f, 1.00f);
		c[ImGuiCol_CheckMark]            = Pink;
		c[ImGuiCol_SliderGrab]           = ImVec4(0.85f, 0.30f, 0.58f, 0.85f);
		c[ImGuiCol_SliderGrabActive]     = Pink;
		c[ImGuiCol_Button]               = ImVec4(0.20f, 0.205f, 0.24f, 1.00f);
		c[ImGuiCol_ButtonHovered]        = ImVec4(0.26f, 0.265f, 0.31f, 1.00f);
		c[ImGuiCol_ButtonActive]         = ImVec4(0.30f, 0.305f, 0.35f, 1.00f);
		c[ImGuiCol_Header]               = ImVec4(0.20f, 0.205f, 0.24f, 1.00f);
		c[ImGuiCol_HeaderHovered]        = ImVec4(0.93f, 0.29f, 0.60f, 0.30f);
		c[ImGuiCol_HeaderActive]         = ImVec4(0.93f, 0.29f, 0.60f, 0.45f);
		c[ImGuiCol_Column]               = c[ImGuiCol_Border];
		c[ImGuiCol_ColumnHovered]        = ImVec4(0.93f, 0.29f, 0.60f, 0.55f);
		c[ImGuiCol_ColumnActive]         = Pink;
		c[ImGuiCol_ResizeGrip]           = ImVec4(0.30f, 0.30f, 0.35f, 0.60f);
		c[ImGuiCol_ResizeGripHovered]    = ImVec4(0.93f, 0.29f, 0.60f, 0.60f);
		c[ImGuiCol_ResizeGripActive]     = Pink;
		c[ImGuiCol_PlotLines]            = ImVec4(0.75f, 0.75f, 0.80f, 1.00f);
		c[ImGuiCol_PlotHistogram]        = Pink;
		c[ImGuiCol_PlotHistogramHovered] = BlueHi;
		c[ImGuiCol_TextSelectedBg]       = ImVec4(0.93f, 0.29f, 0.60f, 0.35f);

		// Remember the opaque base so ApplyOpacity() can scale from it.
		for (int i = 0; i < ImGuiCol_COUNT; ++i)
			BasePalette()[i] = c[i];
		BaseCaptured() = true;
	}

	// Scale the alpha of the menu's surface colors (window/child/frame
	// backgrounds, buttons, headers, scrollbar/title/popup/combo) by `op`,
	// leaving text, checkmarks and accents opaque so the panel stays readable
	// while you see the game through it. op == 1 restores the solid palette.
	// No-op until Apply() has captured the base palette.
	inline void ApplyOpacity(float op) {
		if (!BaseCaptured())
			return;
		ImVec4* c = ImGui::GetStyle().Colors;
		const ImVec4* b = BasePalette();
		// PopupBg/ComboBg stay OPAQUE on purpose: dropdown lists and tooltips
		// float over other text, which bleeds through and makes the options
		// unreadable when translucent.
		static const int surf[] = {
			ImGuiCol_WindowBg, ImGuiCol_ChildWindowBg,
			ImGuiCol_FrameBg, ImGuiCol_FrameBgHovered, ImGuiCol_FrameBgActive,
			ImGuiCol_TitleBg, ImGuiCol_TitleBgActive, ImGuiCol_TitleBgCollapsed,
			ImGuiCol_MenuBarBg, ImGuiCol_ScrollbarBg,
			ImGuiCol_Button, ImGuiCol_ButtonHovered, ImGuiCol_ButtonActive,
			ImGuiCol_Header,
		};
		for (int i : surf)
			c[i].w = b[i].w * op;
	}

	// ---- small helpers --------------------------------------------------
	// Larger bold face for section headings (loaded in OnInitialize alongside
	// the base font; null falls back to the base font).
	inline ImFont*& HeadingFont() { static ImFont* f = nullptr; return f; }

	// Forward: defined below, used by Heading's optional help marker.
	inline void Help(const char* text);

	// A colored section heading with a thin accent underline - replaces the
	// repetitive gray Separator()/Text() pairs. Optional help text becomes a
	// hover "(?)" marker at the end of the heading line.
	inline void Heading(const char* label, const char* help = nullptr) {
		ImGui::Spacing();
		if (HeadingFont()) ImGui::PushFont(HeadingFont());
		ImGui::TextColored(Pink, "%s", label);
		if (HeadingFont()) ImGui::PopFont();
		if (help) Help(help);
		const ImVec2 p = ImGui::GetCursorScreenPos();
		const float w = ImGui::GetContentRegionAvailWidth();
		ImGui::GetWindowDrawList()->AddLine(
			ImVec2(p.x, p.y + 1.f), ImVec2(p.x + w, p.y + 1.f),
			ImGui::ColorConvertFloat4ToU32(ImVec4(Pink.x, Pink.y, Pink.z, 0.35f)), 1.0f);
		ImGui::Dummy(ImVec2(0.f, 4.f));
	}

	// A button in an arbitrary accent color (three shades pushed).
	inline bool ColorButton(const char* label, const ImVec4& base,
	                        const ImVec4& hi, const ImVec2& size) {
		const float a = MenuOpacity();   // accent buttons go translucent too
		ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(base.x, base.y, base.z, base.w * a));
		ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(hi.x, hi.y, hi.z, hi.w * a));
		ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4(hi.x, hi.y, hi.z, hi.w * a));
		ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.f, 1.f, 1.f, 1.f));
		const bool r = ImGui::Button(label, size);
		ImGui::PopStyleColor(4);
		return r;
	}

	// Primary (blue) action button.
	inline bool Primary(const char* label, const ImVec2& size = ImVec2(0, 0)) {
		return ColorButton(label, Blue, BlueHi, size);
	}
	// Accent (pink) action button - the record/run headline actions.
	inline bool Accent(const char* label, const ImVec2& size = ImVec2(0, 0)) {
		return ColorButton(label, Pink, PinkHi, size);
	}
	// Destructive (red) action button.
	inline bool Danger(const char* label, const ImVec2& size = ImVec2(0, 0)) {
		return ColorButton(label, Red, RedHi, size);
	}

	// A tab button: pink (the accent) when selected, neutral otherwise.
	inline bool Tab(const char* label, bool selected, const ImVec2& size) {
		if (selected) {
			const float a = MenuOpacity();
			ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(Pink.x, Pink.y, Pink.z, Pink.w * a));
			ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(PinkHi.x, PinkHi.y, PinkHi.z, PinkHi.w * a));
			ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4(PinkHi.x, PinkHi.y, PinkHi.z, PinkHi.w * a));
		}
		const bool r = ImGui::Button(label, size);
		if (selected)
			ImGui::PopStyleColor(3);
		return r;
	}

	// A small "(?)" marker appended to the current row; the verbose description
	// lives in its hover tooltip instead of the panel. Deliberate hover only -
	// nothing pops up while just mousing around the controls.
	inline void Help(const char* text) {   // (forward-declared above Heading)
		ImGui::SameLine();
		ImGui::TextDisabled("(?)");
		if (ImGui::IsItemHovered()) {
			ImGui::BeginTooltip();
			ImGui::PushTextWrapPos(340.f);
			ImGui::TextUnformatted(text);
			ImGui::PopTextWrapPos();
			ImGui::EndTooltip();
		}
	}

}   // namespace Theme
