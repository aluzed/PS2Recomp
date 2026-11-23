#include "ps2recomp/register_locals.h"

#include <regex>
#include <set>
#include <sstream>
#include <string>
#include <vector>

namespace ps2recomp
{
    namespace
    {
        const std::regex kScalarUse(R"(\b(?:SET_)?GPR_(?:U32|S32|U64|S64|ZE32)\(ctx, (\d+)[,)])");
        const std::regex kVectorUse(R"(\b(?:SET_)?GPR_VEC\(ctx, (\d+)[,)])");
        const std::regex kSetter(R"(\bSET_GPR_(U32|S32|ZE32|U64|S64)\(ctx, (\d+), )");
        const std::regex kGetter(R"(\bGPR_(U32|S32|U64|S64)\(ctx, (\d+)\))");
        // A call that receives ctx: anything but the memory accessors.
        const std::regex kCtxCall(R"(([A-Za-z_][A-Za-z_0-9:]*(?:->[A-Za-z_][A-Za-z_0-9]*)?)\(rdram, ctx\b)");
        const std::regex kRuntimeCall(R"(runtime->([A-Za-z_][A-Za-z_0-9]*))");
        const std::regex kGoto(R"(goto (label_[0-9a-fA-F]+);)");
        const std::regex kReturn(R"(\breturn;)");

        bool isMemoryAccessor(const std::string &name)
        {
            static const std::set<std::string> accessors = {
                "runtime->Load8", "runtime->Load16", "runtime->Load32", "runtime->Load64", "runtime->Load128",
                "runtime->Store8", "runtime->Store16", "runtime->Store32", "runtime->Store64", "runtime->Store128"};
            return accessors.count(name) != 0;
        }

        bool escapes(const std::string &code)
        {
            if (code.find("ctx->r[") != std::string::npos || code.find("ps2_syscalls::") != std::string::npos ||
                code.find("ps2_stubs::") != std::string::npos)
                return true;
            for (std::sregex_iterator it(code.begin(), code.end(), kCtxCall), end; it != end; ++it)
                if (!isMemoryAccessor((*it)[1].str()))
                    return true;
            for (std::sregex_iterator it(code.begin(), code.end(), kRuntimeCall), end; it != end; ++it)
            {
                const std::string name = (*it)[1].str();
                if (name.rfind("Load", 0) != 0 && name.rfind("Store", 0) != 0 && name != "eeCheckpointDue" &&
                    name != "memory")
                    return true;
            }
            return false;
        }

        struct Chunk
        {
            std::vector<std::string> lines;
        };

        bool startsChunk(const std::string &line)
        {
            if (line.rfind("label_", 0) == 0)
                return true;
            return line.rfind("    // 0x", 0) == 0;
        }

        std::string local(int reg)
        {
            return "ps2_g" + std::to_string(reg);
        }

        std::string rewriteAccessors(const std::string &code, const std::set<int> &cached)
        {
            std::string out;
            std::string rest = code;
            // Setters: SET_GPR_S32(ctx, N, value) -> PS2_LSET_S32(ps2_gN, value)
            {
                std::string result;
                auto begin = rest.cbegin();
                std::smatch m;
                while (std::regex_search(begin, rest.cend(), m, kSetter))
                {
                    const int reg = std::stoi(m[2].str());
                    result.append(begin, m[0].first);
                    if (cached.count(reg))
                        result += "PS2_LSET_" + m[1].str() + "(" + local(reg) + ", ";
                    else
                        result += m[0].str();
                    begin = m[0].second;
                }
                result.append(begin, rest.cend());
                rest.swap(result);
            }
            // Getters: GPR_U32(ctx, N) -> ((uint32_t)ps2_gN)
            {
                std::string result;
                auto begin = rest.cbegin();
                std::smatch m;
                while (std::regex_search(begin, rest.cend(), m, kGetter))
                {
                    const int reg = std::stoi(m[2].str());
                    result.append(begin, m[0].first);
                    if (cached.count(reg))
                    {
                        const std::string kind = m[1].str();
                        const char *type = kind == "U32" ? "uint32_t" : kind == "S32" ? "int32_t" : kind == "U64" ? "uint64_t" : "int64_t";
                        result += std::string("((") + type + ")" + local(reg) + ")";
                    }
                    else
                        result += m[0].str();
                    begin = m[0].second;
                }
                result.append(begin, rest.cend());
                out.swap(result);
            }
            return out;
        }
    }

    std::string applyRegisterLocals(const std::string &functionText)
    {
        static const std::string kSignatureEnd = "PS2Runtime *runtime) {\n";
        const size_t sig = functionText.find(kSignatureEnd);
        const size_t close = functionText.rfind("}\n");
        if (sig == std::string::npos || close == std::string::npos || close < sig)
            return functionText;
        const size_t bodyStart = sig + kSignatureEnd.size();
        const std::string head = functionText.substr(0, bodyStart);
        const std::string body = functionText.substr(bodyStart, close - bodyStart);
        const std::string tail = functionText.substr(close);

        // Registers: scalar accessors only; any 128-bit use keeps a register in ctx.
        std::set<int> scalar, vector, written;
        for (std::sregex_iterator it(body.begin(), body.end(), kScalarUse), end; it != end; ++it)
            scalar.insert(std::stoi((*it)[1].str()));
        for (std::sregex_iterator it(body.begin(), body.end(), kVectorUse), end; it != end; ++it)
            vector.insert(std::stoi((*it)[1].str()));
        std::set<int> cached;
        for (int reg : scalar)
            if (reg != 0 && reg < 32 && !vector.count(reg))
                cached.insert(reg);
        if (cached.empty())
            return functionText;
        for (std::sregex_iterator it(body.begin(), body.end(), kSetter), end; it != end; ++it)
        {
            const int reg = std::stoi((*it)[2].str());
            if (cached.count(reg))
                written.insert(reg);
        }

        // Chunks: the prologue (entry switch), then one per label / instruction.
        std::vector<Chunk> chunks(1);
        {
            std::istringstream in(body);
            std::string line;
            while (std::getline(in, line))
            {
                if (startsChunk(line) && !chunks.back().lines.empty())
                    chunks.emplace_back();
                chunks.back().lines.push_back(line);
            }
        }

        std::ostringstream flush, reload;
        flush << "ps2_flush_locals();";
        reload << "ps2_reload_locals();";

        std::ostringstream out;
        out << head;
        // Locals and their sync helpers, before the entry switch so that every
        // entry (start or resume) sees up-to-date values.
        for (int reg : cached)
            out << "    uint64_t " << local(reg) << " = static_cast<uint64_t>(PS2_EXTRACT_EPI64_0(ctx->r[" << reg << "]));\n";
        out << "    auto ps2_flush_locals = [&]() {";
        for (int reg : written)
            out << " Ps2SetGprLow64(ctx, " << reg << ", _mm_cvtsi64_si128(static_cast<int64_t>(" << local(reg) << ")));";
        out << " };\n";
        out << "    auto ps2_reload_locals = [&]() {";
        for (int reg : cached)
            out << " " << local(reg) << " = static_cast<uint64_t>(PS2_EXTRACT_EPI64_0(ctx->r[" << reg << "]));";
        out << " };\n";
        out << "    (void)ps2_reload_locals;\n";

        for (size_t c = 0; c < chunks.size(); ++c)
        {
            std::string text;
            for (const std::string &line : chunks[c].lines)
                text += line + "\n";
            if (c == 0)
            {
                // Prologue: entry switch only (gotos keep the fresh locals).
                out << std::regex_replace(text, kReturn, "{ ps2_flush_locals(); return; }");
                continue;
            }
            if (!escapes(text))
            {
                std::string code = rewriteAccessors(text, cached);
                out << std::regex_replace(code, kReturn, "{ ps2_flush_locals(); return; }");
                continue;
            }
            // Classic between flush and reload. Leading label / comment lines stay first.
            size_t firstCode = 0;
            while (firstCode < chunks[c].lines.size() &&
                   (chunks[c].lines[firstCode].rfind("label_", 0) == 0 ||
                    chunks[c].lines[firstCode].rfind("    //", 0) == 0))
                ++firstCode;
            std::string lead, code;
            for (size_t i = 0; i < chunks[c].lines.size(); ++i)
                (i < firstCode ? lead : code) += chunks[c].lines[i] + "\n";
            out << lead << "    " << flush.str() << "\n"
                << std::regex_replace(code, kGoto, "{ ps2_reload_locals(); goto $1; }")
                << "    " << reload.str() << "\n";
        }
        out << tail;
        return out.str();
    }
}
