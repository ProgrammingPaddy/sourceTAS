// Disk persistence for finalized runs. Runs are written as small binary .tas
// files under Documents\sourceTAS\recordings\ and reloaded on startup. The
// display name is the file name, so renaming a run renames its file.
#include "../../shareddefs.h"

#include <windows.h>
#include <shlobj.h>
#include <fstream>
#include <cstring>
#include <cstdint>

namespace {
	constexpr char kMagic[4] = { 'S', 'T', 'A', 'S' };
	// v1: segments only. v2: adds the StartState anchor block after the version.
	constexpr uint32_t kVersion = 2;

	// Documents\sourceTAS\recordings (created if missing); "" on failure.
	std::string Directory() {
		char documents[MAX_PATH];
		if (FAILED(SHGetFolderPathA(nullptr, CSIDL_PERSONAL, nullptr, SHGFP_TYPE_CURRENT, documents)))
			return {};

		std::string base = std::string(documents) + "\\sourceTAS";
		CreateDirectoryA(base.c_str(), nullptr);

		std::string directory = base + "\\recordings";
		CreateDirectoryA(directory.c_str(), nullptr);
		return directory;
	}

	// Make a string safe to use as a file name.
	std::string Sanitize(const std::string& name) {
		std::string out;
		for (char c : name) {
			if (static_cast<unsigned char>(c) < 32)
				continue;
			out += std::strchr("\\/:*?\"<>|", c) ? '_' : c;
		}

		const size_t first = out.find_first_not_of(" .");
		const size_t last = out.find_last_not_of(" .");
		if (first == std::string::npos)
			return {};
		return out.substr(first, last - first + 1);
	}

	// File name without directory or .tas extension.
	std::string FileStem(const std::string& path) {
		const size_t slash = path.find_last_of("\\/");
		std::string file = (slash == std::string::npos) ? path : path.substr(slash + 1);
		const size_t dot = file.find_last_of('.');
		if (dot != std::string::npos)
			file = file.substr(0, dot);
		return file;
	}

	// First "<base>.tas" / "<base> (N).tas" in dir not already taken (allowing self).
	std::string UniquePath(const std::string& directory, const std::string& base, const std::string& self) {
		std::string path = directory + "\\" + base + ".tas";
		for (int n = 2; GetFileAttributesA(path.c_str()) != INVALID_FILE_ATTRIBUTES && path != self; ++n)
			path = directory + "\\" + base + " (" + std::to_string(n) + ").tas";
		return path;
	}

	bool WriteRun(Run& run) {
		const std::string directory = Directory();
		if (directory.empty())
			return false;

		if (run.filepath.empty()) {
			std::string base = Sanitize(run.name);
			if (base.empty())
				base = "run";
			run.filepath = UniquePath(directory, base, {});
			run.name = FileStem(run.filepath);
		}

		std::ofstream out(run.filepath, std::ios::binary | std::ios::trunc);
		if (!out)
			return false;

		out.write(kMagic, sizeof(kMagic));
		out.write(reinterpret_cast<const char*>(&kVersion), sizeof(kVersion));

		// v2 anchor block.
		const uint8_t start_valid = run.start.valid ? 1 : 0;
		const uint8_t start_ducked = run.start.ducked ? 1 : 0;
		out.write(reinterpret_cast<const char*>(&start_valid), sizeof(start_valid));
		out.write(reinterpret_cast<const char*>(&run.start.origin), sizeof(float) * 3);
		out.write(reinterpret_cast<const char*>(&run.start.velocity), sizeof(float) * 3);
		out.write(reinterpret_cast<const char*>(&run.start.pitch), sizeof(run.start.pitch));
		out.write(reinterpret_cast<const char*>(&run.start.yaw), sizeof(run.start.yaw));
		out.write(reinterpret_cast<const char*>(&start_ducked), sizeof(start_ducked));
		out.write(reinterpret_cast<const char*>(&run.start.stamina), sizeof(run.start.stamina));

		const uint32_t segment_count = static_cast<uint32_t>(run.segments.size());
		out.write(reinterpret_cast<const char*>(&segment_count), sizeof(segment_count));

		for (const Segment& segment : run.segments) {
			const uint32_t frame_count = static_cast<uint32_t>(segment.size());
			out.write(reinterpret_cast<const char*>(&frame_count), sizeof(frame_count));

			for (const Frame& frame : segment) {
				out.write(reinterpret_cast<const char*>(frame.viewangles), sizeof(frame.viewangles));
				out.write(reinterpret_cast<const char*>(&frame.forwardmove), sizeof(frame.forwardmove));
				out.write(reinterpret_cast<const char*>(&frame.sidemove), sizeof(frame.sidemove));
				out.write(reinterpret_cast<const char*>(&frame.upmove), sizeof(frame.upmove));
				out.write(reinterpret_cast<const char*>(&frame.buttons), sizeof(frame.buttons));
				out.write(reinterpret_cast<const char*>(&frame.impulse), sizeof(frame.impulse));
				out.write(reinterpret_cast<const char*>(&frame.mousedx), sizeof(frame.mousedx));
				out.write(reinterpret_cast<const char*>(&frame.mousedy), sizeof(frame.mousedy));
			}
		}

		return static_cast<bool>(out);
	}

	bool ReadRun(const std::string& path, Run& out) {
		std::ifstream in(path, std::ios::binary);
		if (!in)
			return false;

		char magic[4];
		uint32_t version = 0;
		if (!in.read(magic, sizeof(magic)) || std::memcmp(magic, kMagic, sizeof(magic)) != 0)
			return false;
		if (!in.read(reinterpret_cast<char*>(&version), sizeof(version)) || version < 1 || version > kVersion)
			return false;

		Run run;
		run.filepath = path;
		run.name = FileStem(path);

		if (version >= 2) {
			uint8_t start_valid = 0, start_ducked = 0;
			if (!in.read(reinterpret_cast<char*>(&start_valid), sizeof(start_valid)) ||
				!in.read(reinterpret_cast<char*>(&run.start.origin), sizeof(float) * 3) ||
				!in.read(reinterpret_cast<char*>(&run.start.velocity), sizeof(float) * 3) ||
				!in.read(reinterpret_cast<char*>(&run.start.pitch), sizeof(run.start.pitch)) ||
				!in.read(reinterpret_cast<char*>(&run.start.yaw), sizeof(run.start.yaw)) ||
				!in.read(reinterpret_cast<char*>(&start_ducked), sizeof(start_ducked)) ||
				!in.read(reinterpret_cast<char*>(&run.start.stamina), sizeof(run.start.stamina)))
				return false;
			run.start.valid = start_valid != 0;
			run.start.ducked = start_ducked != 0;
		}

		uint32_t segment_count = 0;
		if (!in.read(reinterpret_cast<char*>(&segment_count), sizeof(segment_count)) || segment_count > 1000000)
			return false;

		run.segments.reserve(segment_count);

		for (uint32_t s = 0; s < segment_count; ++s) {
			uint32_t frame_count = 0;
			if (!in.read(reinterpret_cast<char*>(&frame_count), sizeof(frame_count)) || frame_count > 100000000)
				return false;

			Segment segment;
			segment.resize(frame_count);
			for (uint32_t f = 0; f < frame_count; ++f) {
				Frame& frame = segment[f];
				if (!in.read(reinterpret_cast<char*>(frame.viewangles), sizeof(frame.viewangles)) ||
					!in.read(reinterpret_cast<char*>(&frame.forwardmove), sizeof(frame.forwardmove)) ||
					!in.read(reinterpret_cast<char*>(&frame.sidemove), sizeof(frame.sidemove)) ||
					!in.read(reinterpret_cast<char*>(&frame.upmove), sizeof(frame.upmove)) ||
					!in.read(reinterpret_cast<char*>(&frame.buttons), sizeof(frame.buttons)) ||
					!in.read(reinterpret_cast<char*>(&frame.impulse), sizeof(frame.impulse)) ||
					!in.read(reinterpret_cast<char*>(&frame.mousedx), sizeof(frame.mousedx)) ||
					!in.read(reinterpret_cast<char*>(&frame.mousedy), sizeof(frame.mousedy)))
					return false;
			}
			run.segments.push_back(std::move(segment));
		}

		out = std::move(run);
		return true;
	}
}

void TasEngine::LoadFromDisk() {
	library.clear();

	const std::string directory = Directory();
	if (!directory.empty()) {
		WIN32_FIND_DATAA find = {};
		const std::string pattern = directory + "\\*.tas";

		HANDLE handle = FindFirstFileA(pattern.c_str(), &find);
		if (handle != INVALID_HANDLE_VALUE) {
			do {
				if (find.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY)
					continue;
				Run run;
				if (ReadRun(directory + "\\" + find.cFileName, run))
					library.push_back(std::move(run));
			} while (FindNextFileA(handle, &find));
			FindClose(handle);
		}
	}

	selected = library.empty() ? -1 : 0;
	run_counter = static_cast<int>(library.size());
	status = library.empty() ? "No saved recordings." : (std::to_string(library.size()) + " recording(s) loaded.");
}

void TasEngine::PersistSelected() {
	if (selected < 0 || selected >= static_cast<int>(library.size()))
		return;
	if (!WriteRun(library[selected]))
		status = "Run kept in memory, but writing to disk failed.";
}

bool TasEngine::RenameSelected(const char* newName) {
	if (state != TasState::Idle || selected < 0 || selected >= static_cast<int>(library.size()))
		return false;

	Run& run = library[selected];
	const std::string directory = Directory();
	const std::string base = Sanitize(newName ? newName : "");
	if (directory.empty() || base.empty()) {
		status = "Invalid name.";
		return false;
	}

	const std::string target = UniquePath(directory, base, run.filepath);

	if (!run.filepath.empty() && GetFileAttributesA(run.filepath.c_str()) != INVALID_FILE_ATTRIBUTES) {
		if (!MoveFileA(run.filepath.c_str(), target.c_str())) {
			status = "Rename failed.";
			return false;
		}
	}

	run.filepath = target;
	run.name = FileStem(target);
	status = "Renamed to " + run.name + ".";
	return true;
}

void TasEngine::DeleteSelected() {
	if (state != TasState::Idle || selected < 0 || selected >= static_cast<int>(library.size()))
		return;

	Run& run = library[selected];
	if (!run.filepath.empty())
		DeleteFileA(run.filepath.c_str());

	const std::string name = run.name;
	library.erase(library.begin() + selected);

	if (library.empty())
		selected = -1;
	else if (selected >= static_cast<int>(library.size()))
		selected = static_cast<int>(library.size()) - 1;

	status = "Deleted " + name + ".";
}
