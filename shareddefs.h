#pragma once
#include "Include/cstrike/Classes/CUserCmd.h"

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

	void Replay(CUserCmd* cmd) {
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



class Recorder {
private:
	bool is_recording_active = false;
	
public:
	void StartRecording() {
		this->is_recording_active = true;
	}

	void StopRecording() {
		this->is_recording_active = false;
	}

	bool IsRecordingActive() const {
		return this->is_recording_active;
	}

};

class Playback {
private:
	bool is_playback_active = false;
	size_t current_frame = 0;
	
public:
	void StartPlayback() {
		this->is_playback_active = true;

	};

	void StopPlayback() {
		this->is_playback_active = false;
		this->current_frame = 0;
	};

	bool IsPlaybackActive() const {
		return this->is_playback_active;
	}

	size_t GetCurrentFrame() const {
		return this->current_frame;
	};

	void SetCurrentFrame(size_t frame) {
		this->current_frame = frame;
	};


};

class RecordedFrames {
public:
	static Recorder recorder;
	static Playback playback;
	static std::vector<Frame> Frames;
};

