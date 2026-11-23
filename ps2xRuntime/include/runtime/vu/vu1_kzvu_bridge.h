#pragma once

#include <cstdint>

class PS2Runtime;

// Optional VU1 core: PCSX2's microVU x86-64 recompiler through kzvu (GPL-3.0+,
// https://github.com/drakolordx7/KillzoneRecomp ext/kzvu, Linux port in the
// sfex3blazing project). Built when the runtime is configured with
// PS2X_KZVU_DIR and PS2X_PCSX2_DIR.
//
// PS2X_VU1_CORE selects the VU1 core at startup:
//   microvu (default)  microVU recompiler
//   interp             PCSX2's VU interpreter (debugging)
//   builtin            the runtime's cycle-accurate VU1Interpreter
//
// Installing takes over VU1 micro/data memory and the VIF1 MSCAL/MSCNT
// callbacks. Returns false (and keeps VU1Interpreter) when kzvu is not built
// in, not selected or fails to start.
bool installKzvuVu1(PS2Runtime &runtime);

// VU0 micro mode (VCALLMS/VCALLMSR) on microVU0, installed with VU1 unless
// PS2X_VU0_CORE=builtin. Runs the program at startPc with the EE's COP2
// register file; returns false when kzvu VU0 is not active (the caller then
// uses the runtime's interpreter).
struct R5900Context;
bool runKzvuVu0(R5900Context *ctx, uint32_t startPc);
