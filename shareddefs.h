#pragma once

#include <vector>
#include <string>

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

struct Run {
	std::string name;
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
//     it. Delete removes the most recently committed segment, repeatedly.
//   - Empty segments are never saved.
//   - Frames are per-tick, so the run is inherently gapless: deciding what to do
//     between captures records nothing, and segments replay back to back.
//   - Playback only writes the CUserCmd; hotkeys come from the keyboard hook, so
//     replayed input can never drive the engine. Emergency stop always wins.
class TasEngine {
private:
	TasState state = TasState::Idle;

	// Active recording session (separate from the immutable library until saved).
	std::vector<Segment> session_segments;   // committed boundaries this session
	Segment active_segment;                  // current, not-yet-committed capture

	std::vector<Run> library;                // finalized, immutable runs
	int selected = -1;                       // run used by playback
	int run_counter = 0;                     // for auto-naming

	int playback_segment = 0;
	size_t playback_frame = 0;
	size_t playback_pos = 0;                  // flattened progress, for display
	size_t playback_total = 0;

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
		state = TasState::Recording;
		status = "Recording... (Save Segment to split, Stop & Save to finish)";
	}

	// Commit the active segment as a boundary and keep recording.
	void SaveSegment() {
		if (state != TasState::Recording) return;
		if (active_segment.empty()) {
			status = "Active segment is empty - nothing saved.";
			return;
		}
		status = "Segment saved (" + std::to_string(active_segment.size()) + " frames); new segment started.";
		session_segments.push_back(active_segment);
		active_segment.clear();
	}

	// Discard the current (uncommitted) segment and start it over; committed
	// segments are untouched, so the replacement lands at the same boundary.
	void OverwriteCurrentSegment() {
		if (state != TasState::Recording) return;
		const size_t discarded = active_segment.size();
		active_segment.clear();
		status = "Overwriting current segment (discarded " + std::to_string(discarded) + " frames).";
	}

	// Remove the most recently committed segment; repeatable.
	void DeletePreviousSegment() {
		if (state != TasState::Recording) return;
		if (session_segments.empty()) { status = "No previous segment to delete."; return; }
		const size_t removed = session_segments.back().size();
		session_segments.pop_back();
		status = "Deleted previous segment (" + std::to_string(removed) + " frames; "
			+ std::to_string(session_segments.size()) + " left).";
	}

	// Finalize the session into an immutable run and select it.
	void StopRecordingAndSave() {
		if (state != TasState::Recording) return;

		if (!active_segment.empty()) {
			session_segments.push_back(active_segment);
			active_segment.clear();
		}

		Run run;
		for (const Segment& segment : session_segments)
			if (!segment.empty())
				run.segments.push_back(segment);
		session_segments.clear();
		state = TasState::Idle;

		if (run.segments.empty()) {
			status = "Stopped - nothing captured, no run saved.";
			return;
		}

		run.name = "Run " + std::to_string(++run_counter);
		status = "Saved " + run.name + " (" + std::to_string(run.segments.size()) + " seg, "
			+ std::to_string(run.FrameCount()) + " frames).";
		library.push_back(run);
		selected = static_cast<int>(library.size()) - 1;
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

	// Always wins: abort whatever is happening and release held virtual input.
	void EmergencyStop() {
		const char* message =
			state == TasState::Playback ? "Emergency stop - playback aborted." :
			state == TasState::Recording ? "Emergency stop - recording discarded." :
			"Released all.";
		session_segments.clear();
		active_segment.clear();
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

		Run* run = SelectedRun();
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
