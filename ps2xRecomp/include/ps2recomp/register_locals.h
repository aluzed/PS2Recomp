#ifndef PS2RECOMP_REGISTER_LOCALS_H
#define PS2RECOMP_REGISTER_LOCALS_H

#include <string>

namespace ps2recomp
{
    // "Register locals" code generation (config: register_locals = true).
    //
    // The classic generated code keeps every guest GPR in ctx->r[], so each
    // guest memory write (through uint8_t*, which may alias anything) forces
    // the C++ compiler to reload the registers. This pass rewrites one
    // generated function so that the low 64 bits of the GPRs it uses live in
    // C++ locals (ps2_gN), which the compiler can keep in host registers:
    //   - the locals are loaded from ctx before the entry switch;
    //   - instructions that only compute use the locals;
    //   - instructions that hand ctx to other code (calls, dispatch, syscalls,
    //     stubs, breaks, direct ctx->r[] use) stay classic between a flush of
    //     the written locals to ctx and a reload of all locals; a goto leaving
    //     such an instruction reloads first;
    //   - every return writes the locals back first.
    // Registers used with 128-bit accessors, and $zero, stay in ctx.
    // Same idea as the "register locals" mode of the KillzoneRecomp and
    // gow-native-pc ports (independent implementation for this generator).
    //
    // Returns the rewritten function, or the input unchanged when it has
    // nothing to cache or a form the pass does not handle.
    std::string applyRegisterLocals(const std::string &functionText);
}

#endif
