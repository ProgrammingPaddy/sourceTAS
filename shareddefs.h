#pragma once

#include <vector>
#include <string>

#include "Include/cstrike/Structures/Vector.h"   // QAngle, used by CUserCmd
#include "Include/cstrike/Classes/CUserCmd.h"

// One tick of captured input.
struct Frame {
	float viewangles[2];
	float forwardmove;
	float sidemove;
	float upmove;
	int buttons;
	unsigned char impulse;
	short mousedx;
	short mousedy;

	Frame() = default;

	Frame(CUserCmd* cmd) {
		this->viewangles[0] = cmd->viewangles.X;
		this->viewangles[1] = cmd->viewangles.Y;
		this->forwardmove = cmd->forwardmove;
		this->sidemove = cmd->sidemove;
		this->upmove = cmd->upmove;
		this->buttons = cmd->buttons;
		this->impulse = cmd->impulse;
		this->mousedx = cmd->mousedx;
		this->mousedy = cmd->mousedy;
	}

	void Replay(CUserCmd* cmd) const {
		cmd->viewangles.X = this->viewangles[0];
		cmd->viewangles.Y = this->viewangles[1];
		cmd->forwardmove = this->forwardmove;
		cmd->sidemove = this->sidemove;
		cmd->upmove = this->upmove;
		cmd->buttons = this->buttons;
		cmd->impulse = this->impulse;
		cmd->mousedx = this->mousedx;
		cmd->mousedy = this->mousedy;
	}
};

// A run is an ordered list of segments; a segment is a contiguous capture.
using Segment = std::vector<Frame>;

// Absolute starting state of a run, in world coordinates. Captured on the first
// recorded tick (or set as the editor's anchor), so a run is pinned to a fixed
// point in virtual space and can be re-simulated/replayed across sessions.
struct StartState {
	bool valid = false;
	Vector origin;
	Vector velocity;
	float pitch = 0.f;
	float yaw = 0.f;
	bool ducked = false;
	float stamina = 0.f;
};

struct Run {
	std::string name;
	std::string filepath;                    // on-disk .tas file ("" if unsaved)
	StartState start;                        // absolute anchor (v2 files; may be invalid)
	std::vector<Segment> segments;

	size_t FrameCount() const {
		size_t total = 0;
		for (const Segment& segment : segments)
			total += segment.size();
		return total;
	}

	bool Empty() const {
		return FrameCount() == 0;
	}
};

enum class TasState {
	Idle,        // run mode: nothing capturing or replaying, hotkeys live
	Recording,   // a continuous capture session is active
	Playback     // replaying the selected run
};

// Owns the recording/playback state machine, mirroring the hardware TAS tool:
//
//   - Recording is ONE continuous session. "Save Segment" commits a boundary and
//     keeps recording; "Stop & Save" finalizes the whole run into the library.
//   - Segment edits (save / overwrite / delete-previous) only apply while
//     recording, and never touch finalized runs (which are immutable).
//   - Overwrite discards the *current* (active, uncommitted) segment and restarts
//     it. Delete removes the most recently committed segment, repeatably.
//   - Empty segments are never saved.
//   - Frames are per-tick, so the run is inherently gapless: deciding what to do
//     between captures records nothing, and segments replay back to back.
//   - Playback only writes the CUserCmd; hotkeys come from the keyboard hook, so
//     replayed input can never drive the engine. Emergency stop always wins.
//
// Finalized runs persist to disk (Documents\sourceTAS\recordings\*.tas) and are
// reloaded on startup. Disk methods live in Source/Recording/RecordingStore.cpp.
class TasEngine {
private:
	TasState state = TasState::Idle;

	// Active recording session (separate from the immutable library until saved).
	std::vector<Segment> session_segments;   // committed boundaries this session
	Segment active_segment;                  // current, not-yet-committed capture

	// One anchor per segment, captured on each segment's FIRST tick. The run's
	// anchor is the first surviving segment's - so overwriting or deleting back
	// to an empty session re-captures instead of keeping a stale start.
	std::vector<StartState> session_starts;  // parallel to session_segments
	StartState active_start;                 // anchor for the active segment

	std::vector<Run> library;                // finalized, immutable runs
	int selected = -1;                       // run used by playback
	int run_counter = 0;                     // for auto-naming

	int playback_segment = 0;
	size_t playback_frame = 0;
	size_t playback_pos = 0;                  // flattened progress, for display
	size_t playback_total = 0;

	// Editor test playback: a scratch run played without touching the library.
	Run scratch;
	bool scratch_active = false;
	int playback_delay = 0;                   // ticks to wait before feeding input
	                                          // (lets a teleport-to-anchor settle)

	std::string status = "Idle.";

	Run* SelectedRun() {
		if (selected >= 0 && selected < static_cast<int>(library.size()))
			return &library[selected];
		return nullptr;
	}

	void SkipEmptyPlaybackSegments(const Run& run) {
		while (playback_segment < static_cast<int>(run.segments.size())
			&& playback_frame >= run.segments[playback_segment].size()) {
			playback_segment++;
			playback_frame = 0;
		}
	}

	void FinishPlayback(const char* message) {
		state = TasState::Idle;
		scratch_active = false;
		playback_delay = 0;
		status = message;
	}

public:
	// --- queries ---------------------------------------------------------
	TasState State() const { return state; }
	bool IsRecording() const { return state == TasState::Recording; }
	bool IsPlaying() const { return state == TasState::Playback; }
	const char* Status() const { return status.c_str(); }
	const std::vector<Run>& Library() const { return library; }
	int Selected() const { return selected; }
	size_t ActiveSegmentSize() const { return active_segment.size(); }
	size_t SessionSegmentCount() const { return session_segments.size(); }
	size_t PlaybackPosition() const { return playback_pos; }
	size_t PlaybackTotal() const { return playback_total; }
	int PlaybackSegment() const { return playback_segment; }

	// --- per-segment anchor capture (called from the CreateMove hook) ----
	// True exactly when the next recorded frame starts a segment, so the hook
	// captures the world state that precedes that segment's first input.
	bool NeedsStartCapture() const {
		return state == TasState::Recording && active_segment.empty();
	}
	void SetPendingStart(const StartState& s) {
		if (state == TasState::Recording && active_segment.empty())
			active_start = s;
	}

	// --- editor export ----------------------------------------------------
	// Add a finalized run to the library and select it (run mode only). The
	// caller persists it with PersistSelected(). Returns the index or -1.
	int AddRun(Run run) {
		if (state != TasState::Idle) { status = "Can't add a run now."; return -1; }
		if (run.segments.empty()) return -1;
		if (run.name.empty())
			run.name = "Run " + std::to_string(++run_counter);
		library.push_back(std::move(run));
		selected = static_cast<int>(library.size()) - 1;
		status = "Added " + library.back().name + " ("
			+ std::to_string(library.back().FrameCount()) + " frames).";
		return selected;
	}

	// --- persistence (defined in RecordingStore.cpp) --------------------
	void LoadFromDisk();          // populate the library from disk on startup
	void PersistSelected();       // write the selected run to disk
	bool RenameSelected(const char* newName);
	void DeleteSelected();

	// --- selection (run mode only) --------------------------------------
	void Select(int index) {
		if (state != TasState::Idle) { status = "Can't change selection now."; return; }
		if (index >= 0 && index < static_cast<int>(library.size())) {
			selected = index;
			status = "Selected " + library[index].name + ".";
		}
	}

	void SelectNext() {
		if (state != TasState::Idle) return;
		if (library.empty()) { status = "No recordings to select."; return; }
		selected = (selected + 1) % static_cast<int>(library.size());
		status = "Selected " + library[selected].name + ".";
	}

	// --- recording session ----------------------------------------------
	void StartRecording() {
		if (state != TasState::Idle) return;
		session_segments.clear();
		active_segment.clear();
		session_starts.clear();
		active_start = StartState();
		state = TasState::Recording;
		status = "Recording... (Save Segment to split, Stop & Save to finish)";
	}

	void SaveSegment() {
		if (state != TasState::Recording) return;
		if (active_segment.empty()) {
			status = "Active segment is empty - nothing saved.";
			return;
		}
		status = "Segment saved (" + std::to_string(active_segment.size()) + " frames); new segment started.";
		session_segments.push_back(active_segment);
		session_starts.push_back(active_start);
		active_segment.clear();
		active_start = StartState();   // next segment re-captures on its first tick
	}

	void OverwriteCurrentSegment() {
		if (state != TasState::Recording) return;
		const size_t discarded = active_segment.size();
		active_segment.clear();
		active_start = StartState();   // restart means a fresh anchor too
		status = "Overwriting current segment (discarded " + std::to_string(discarded) + " frames).";
	}

	void DeletePreviousSegment() {
		if (state != TasState::Recording) return;
		if (session_segments.empty()) { status = "No previous segment to delete."; return; }
		const size_t removed = session_segments.back().size();
		session_segments.pop_back();
		session_starts.pop_back();
		status = "Deleted previous segment (" + std::to_string(removed) + " frames; "
			+ std::to_string(session_segments.size()) + " left).";
	}

	// Finalize the session into an immutable run and select it. Returns true if a
	// run was created (the caller then persists it to disk).
	bool StopRecordingAndSave() {
		if (state != TasState::Recording) return false;

		if (!active_segment.empty()) {
			session_segments.push_back(active_segment);
			session_starts.push_back(active_start);
			active_segment.clear();
			active_start = StartState();
		}

		Run run;
		for (size_t i = 0; i < session_segments.size(); ++i) {
			if (session_segments[i].empty())
				continue;
			if (run.segments.empty())
				run.start = session_starts[i];   // anchor of the first surviving segment
			run.segments.push_back(session_segments[i]);
		}
		session_segments.clear();
		session_starts.clear();
		state = TasState::Idle;

		if (run.segments.empty()) {
			status = "Stopped - nothing captured, no run saved.";
			return false;
		}

		run.name = "Run " + std::to_string(++run_counter);
		status = "Saved " + run.name + " (" + std::to_string(run.segments.size()) + " seg, "
			+ std::to_string(run.FrameCount()) + " frames).";
		library.push_back(run);
		selected = static_cast<int>(library.size()) - 1;
		return true;
	}

	// --- playback --------------------------------------------------------
	void PlaySelected() {
		if (state != TasState::Idle) return;
		Run* run = SelectedRun();
		if (!run || run->Empty()) { status = "No recording selected to play."; return; }

		playback_segment = 0;
		playback_frame = 0;
		playback_pos = 0;
		playback_total = run->FrameCount();
		SkipEmptyPlaybackSegments(*run);

		state = TasState::Playback;
		status = "Playing " + run->name + "...";
	}

	// Editor test playback: play a scratch run that never enters the library.
	// delay_ticks holds off input feeding so a teleport-to-anchor can settle.
	bool PlayEphemeral(Run run, int delay_ticks) {
		if (state != TasState::Idle || run.Empty()) return false;

		scratch = std::move(run);
		scratch_active = true;
		playback_segment = 0;
		playback_frame = 0;
		playback_pos = 0;
		playback_total = scratch.FrameCount();
		SkipEmptyPlaybackSegments(scratch);
		playback_delay = delay_ticks > 0 ? delay_ticks : 0;

		state = TasState::Playback;
		status = "Test playing (editor run)...";
		return true;
	}

	bool IsTestPlayback() const { return scratch_active; }

	void EmergencyStop() {
		const char* message =
			state == TasState::Playback ? "Emergency stop - playback aborted." :
			state == TasState::Recording ? "Emergency stop - recording discarded." :
			"Released all.";
		session_segments.clear();
		active_segment.clear();
		session_starts.clear();
		active_start = StartState();
		scratch_active = false;
		playback_delay = 0;
		state = TasState::Idle;
		status = message;
	}

	// --- per-tick (called from CreateMove) ------------------------------
	void RecordFrame(CUserCmd* cmd) {
		if (state == TasState::Recording)
			active_segment.push_back(Frame(cmd));
	}

	bool ReplayFrame(CUserCmd* cmd) {
		if (state != TasState::Playback)
			return false;

		// Grace period after a test-play teleport: pass user input through
		// until the setpos has settled.
		if (playback_delay > 0) {
			playback_delay--;
			return false;
		}

		Run* run = scratch_active ? &scratch : SelectedRun();
		if (!run || playback_segment >= static_cast<int>(run->segments.size())) {
			FinishPlayback("Playback complete.");
			return false;
		}

		run->segments[playback_segment][playback_frame].Replay(cmd);

		playback_frame++;
		playback_pos++;
		SkipEmptyPlaybackSegments(*run);

		if (playback_segment >= static_cast<int>(run->segments.size()))
			FinishPlayback("Playback complete.");

		return true;
	}
};

extern TasEngine g_tas;
