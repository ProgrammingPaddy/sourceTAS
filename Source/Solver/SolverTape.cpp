#include "SolverTape.h"

#include <cstdint>
#include <cstring>
#include <fstream>

namespace Solver {

	namespace {
		bool Fail(std::string* err, const char* msg) {
			if (err) *err = msg;
			return false;
		}

		template <typename T>
		bool ReadT(std::ifstream& in, T* v) {
			return static_cast<bool>(in.read(reinterpret_cast<char*>(v), sizeof(T)));
		}
	}

	bool LoadTas(const std::string& path, Tape& out, std::string* err) {
		std::ifstream in(path, std::ios::binary);
		if (!in)
			return Fail(err, "cannot open .tas file");

		char magic[4];
		uint32_t version = 0;
		if (!in.read(magic, 4) || std::memcmp(magic, "STAS", 4) != 0)
			return Fail(err, "bad magic (not a STAS file)");
		if (!ReadT(in, &version) || version < 1 || version > 3)
			return Fail(err, "unsupported .tas version");
		out.version = static_cast<int>(version);

		if (version >= 2) {
			uint8_t valid = 0, ducked = 0;
			float origin[3], velocity[3];
			if (!ReadT(in, &valid) ||
				!in.read(reinterpret_cast<char*>(origin), 12) ||
				!in.read(reinterpret_cast<char*>(velocity), 12) ||
				!ReadT(in, &out.start.pitch) ||
				!ReadT(in, &out.start.yaw) ||
				!ReadT(in, &ducked) ||
				!ReadT(in, &out.start.stamina))
				return Fail(err, "truncated anchor block");
			out.start.valid = valid != 0;
			out.start.ducked = ducked != 0;
			out.start.origin = Vec3(origin[0], origin[1], origin[2]);
			out.start.velocity = Vec3(velocity[0], velocity[1], velocity[2]);
		}

		if (version >= 3) {
			uint32_t map_len = 0;
			if (!ReadT(in, &map_len) || map_len > 256)
				return Fail(err, "bad map-name length");
			if (map_len) {
				out.map.resize(map_len);
				if (!in.read(&out.map[0], map_len))
					return Fail(err, "truncated map name");
			}
		}

		uint32_t nseg = 0;
		if (!ReadT(in, &nseg) || nseg > 1000000)
			return Fail(err, "bad segment count");
		for (uint32_t s = 0; s < nseg; ++s) {
			uint32_t nframes = 0;
			if (!ReadT(in, &nframes) || nframes > 100000000)
				return Fail(err, "bad frame count");
			out.segment_starts.push_back(static_cast<int>(out.frames.size()));
			for (uint32_t f = 0; f < nframes; ++f) {
				// On-disk frame layout (RecordingStore::WriteRun):
				//   float viewangles[2], float fwd/side/up, int buttons,
				//   u8 impulse, s16 mousedx, s16 mousedy.
				float va[2], fmove, smove, umove;
				int32_t buttons;
				uint8_t impulse;
				int16_t mdx, mdy;
				if (!in.read(reinterpret_cast<char*>(va), 8) ||
					!ReadT(in, &fmove) || !ReadT(in, &smove) || !ReadT(in, &umove) ||
					!ReadT(in, &buttons) || !ReadT(in, &impulse) ||
					!ReadT(in, &mdx) || !ReadT(in, &mdy))
					return Fail(err, "truncated frame data");
				TapeFrame tf;
				tf.pitch = va[0];
				tf.yaw = va[1];
				tf.fmove = fmove;
				tf.smove = smove;
				tf.umove = umove;
				tf.buttons = buttons;
				out.frames.push_back(tf);
			}
		}
		return true;
	}

	bool WriteTas(const std::string& path, const TapeAnchor& anchor,
	              const std::string& map, const std::vector<TapeFrame>& frames,
	              std::string* err) {
		std::ofstream out(path, std::ios::binary | std::ios::trunc);
		if (!out)
			return Fail(err, "cannot open output .tas for writing");
		out.write("STAS", 4);
		const uint32_t version = 3;
		out.write(reinterpret_cast<const char*>(&version), 4);
		const uint8_t valid = anchor.valid ? 1 : 0;
		const uint8_t ducked = anchor.ducked ? 1 : 0;
		const float origin[3] = { anchor.origin.X, anchor.origin.Y, anchor.origin.Z };
		const float velocity[3] = { anchor.velocity.X, anchor.velocity.Y, anchor.velocity.Z };
		out.write(reinterpret_cast<const char*>(&valid), 1);
		out.write(reinterpret_cast<const char*>(origin), 12);
		out.write(reinterpret_cast<const char*>(velocity), 12);
		out.write(reinterpret_cast<const char*>(&anchor.pitch), 4);
		out.write(reinterpret_cast<const char*>(&anchor.yaw), 4);
		out.write(reinterpret_cast<const char*>(&ducked), 1);
		out.write(reinterpret_cast<const char*>(&anchor.stamina), 4);
		const uint32_t map_len = static_cast<uint32_t>(map.size());
		out.write(reinterpret_cast<const char*>(&map_len), 4);
		if (map_len)
			out.write(map.data(), map_len);
		const uint32_t nseg = 1;
		out.write(reinterpret_cast<const char*>(&nseg), 4);
		const uint32_t nframes = static_cast<uint32_t>(frames.size());
		out.write(reinterpret_cast<const char*>(&nframes), 4);
		for (const TapeFrame& f : frames) {
			const float va[2] = { f.pitch, f.yaw };
			const int32_t buttons = f.buttons;
			const uint8_t impulse = 0;
			const int16_t mdx = 0, mdy = 0;
			out.write(reinterpret_cast<const char*>(va), 8);
			out.write(reinterpret_cast<const char*>(&f.fmove), 4);
			out.write(reinterpret_cast<const char*>(&f.smove), 4);
			out.write(reinterpret_cast<const char*>(&f.umove), 4);
			out.write(reinterpret_cast<const char*>(&buttons), 4);
			out.write(reinterpret_cast<const char*>(&impulse), 1);
			out.write(reinterpret_cast<const char*>(&mdx), 2);
			out.write(reinterpret_cast<const char*>(&mdy), 2);
		}
		return static_cast<bool>(out);
	}

} // namespace Solver
