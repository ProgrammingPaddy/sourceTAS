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
	Recording,   // actively capturing the active segment
	Playback     // replaying the selected run
};

// Owns the recording/playback state machine. The rules deliberately favour
// predictable behaviour: no recording during playback, no playback while
// recording, and emergency stop always wins. Playback only writes into the
// CUserCmd; it never feeds hotkeys, so replayed input cannot drive the engine.
class TasEngine {
private:
	TasState state = TasState::Idle;

	std::vector<Run> library;        // saved runs
	int selected = -1;               // index used by playback / appended to
	int run_counter = 0;             // for auto-naming new runs

	Segment active_segment;          // captured while Recording

	int playback_segment = 0;        // position within the selected run
	size_t playback_frame = 0;
	size_t playback_pos = 0;         // flattened progress, for display
	size_t playback_total = 0;

	std::string status = "Idle.";

	Run* SelectedRun() {
		if (selected >= 0 && selected < static_cast<int>(library.size()))
			return &library[selected];
		return nullptr;
	}

	// Advance the playback cursor past any empty segments.
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
	size_t PlaybackPosition() const { return playback_pos; }
	size_t PlaybackTotal() const { return playback_total; }

	// --- selection (run mode only) --------------------------------------
	void NewRun() {
		if (state != TasState::Idle) { status = "Finish the current action first."; return; }
		Run run;
		run.name = "Run " + std::to_string(++run_counter);
		library.push_back(run);
		selected = static_cast<int>(library.size()) - 1;
		status = "Created " + run.name + ".";
	}

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

	// --- transitions -----------------------------------------------------
	void StartRecording() {
		if (state != TasState::Idle) return;
		if (selected < 0)
			NewRun();                       // record into a fresh run
		active_segment.clear();
		state = TasState::Recording;
		status = "Recording into " + SelectedRun()->name + "...";
	}

	void StopSegment() {
		if (state != TasState::Recording) return;

		if (active_segment.empty()) {
			status = "Empty segment - nothing saved.";
		} else if (Run* run = SelectedRun()) {
			status = "Saved segment (" + std::to_string(active_segment.size()) + " frames, "
				+ std::to_string(run->segments.size() + 1) + " total).";
			run->segments.push_back(active_segment);
		}

		active_segment.clear();
		state = TasState::Idle;
	}

	void DeletePreviousSegment() {
		if (state != TasState::Idle) return;
		Run* run = SelectedRun();
		if (!run || run->segments.empty()) { status = "No segment to delete."; return; }

		const size_t removed = run->segments.back().size();
		run->segments.pop_back();
		status = "Deleted segment (" + std::to_string(removed) + " frames, "
			+ std::to_string(run->segments.size()) + " left).";
	}

	void OverwritePreviousSegment() {
		if (state != TasState::Idle) return;
		Run* run = SelectedRun();
		if (!run || run->segments.empty()) { status = "No segment to overwrite."; return; }

		// Discard the target and record its replacement at the same boundary;
		// the decision time spent here is never captured, so no dead gap forms.
		run->segments.pop_back();
		active_segment.clear();
		state = TasState::Recording;
		status = "Overwriting last segment - recording replacement...";
	}

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
		const bool was_busy = (state != TasState::Idle);
		active_segment.clear();              // drop any in-progress capture
		state = TasState::Idle;
		status = was_busy ? "Emergency stop - released all." : "Released all.";
	}

	// --- per-tick (called from CreateMove) ------------------------------
	void RecordFrame(CUserCmd* cmd) {
		if (state == TasState::Recording)
			active_segment.push_back(Frame(cmd));
	}

	// Writes the next recorded frame into cmd. Returns true if a frame was
	// applied (the caller then commits the view angles).
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
