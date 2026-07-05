//===- wpa.cpp -- Whole program analysis -------------------------------------//
//
//                     SVF: Static Value-Flow Analysis
//
// Copyright (C) <2013-2017>  <Yulei Sui>
//

// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Affero General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Affero General Public License for more details.

// You should have received a copy of the GNU Affero General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
//
//===-----------------------------------------------------------------------===//

/*
 // Whole Program Pointer Analysis
 //
 // Author: Yulei Sui,
 */

#include "SVF-LLVM/LLVMUtil.h"
#include "SVF-LLVM/SVFIRBuilder.h"
#include "SVF-LLVM/SVFIRGetter.h"
#include "WPA/WPAPass.h"
#include "WPA/Andersen.h"
#include "WPA/AndersenInc.h"
#include "Util/CommandLine.h"
#include "Util/Options.h"
#include "Util/NodeIDAllocator.h"
#include "SVFIR/SVFFileSystem.h"
#include "SVFIR/SymbolTableInfo.h"
#include "SVFIR/SVFModule.h"
#include "Diff/SourceDiff.h"
#include "Diff/IRDiff.h"
#include <cstdlib>
#include <cstdio>
#include <iostream>
#include <sstream>
#include <vector>
#include <string>

using namespace llvm;
using namespace std;
using namespace SVF;

void diff()
{
    // double starttime = stat->getClk();   
    SourceDiffHandler *sourceDiff = SourceDiffHandler::getSourceDiffHandler();

    //std::cout << "sourceDiff.display:" << std::endl;
    sourceDiff->handle();
    //sourceDiff.display();
    //sourceDiff.dump();
    // double endtime = stat->getClk();
    // stat->StatTimeOfSourceDiff(starttime, endtime);

    // starttime = stat->getClk();
    IRDiffHandler* irDiff = IRDiffHandler::getIRDiffHandler();
    irDiff->parse();
    
    auto add = irDiff->getInstAddSet();
    auto del = irDiff->getInstDeleteSet();
    if (add.empty() && del.empty())
    {
        SVFUtil::outs() << "No inst changed.\n";
        return;
    }
    SVFUtil::outs() << "Add insts: " << add.size() << "\n";
    SVFUtil::outs() << "Del insts: " << del.size() << "\n";

    // endtime = stat->getClk();
    // stat->StatTimeOfIrDiff(starttime, endtime);

    // irDiff->dump("irdiffresult.txt",true);
}

/// Tokenize one line of continuous-mode input into whitespace-separated tokens.
static std::vector<std::string> tokenizeLine(const std::string& line)
{
    std::vector<std::string> tokens;
    std::istringstream iss(line);
    std::string token;
    while (iss >> token)
    {
        tokens.push_back(token);
    }
    return tokens;
}

/// Release all singletons that accumulate state across analysis rounds.
/// Must be called before loading a new bitcode module in continuous mode.
static void resetAnalysisState()
{
    // Diff handlers may hold raw pointers into the previous LLVM module,
    // so release them before the module itself is destroyed.
    SVFIRGetter::releaseSVFIRGetter();
    IRDiffHandler::releaseIRDiffHandler();
    SourceDiffHandler::releaseSourceDiffHandler();

    // Release PAG (also destroys ICFG/CHGraph and releases SVFModule).
    SVFIR::releaseSVFIR();

    // Release LLVM module set and reset the pre-processing flag.
    LLVMModuleSet::releaseLLVMModuleSet();

    // Release symbol table and node ID allocator.
    SymbolTableInfo::releaseSymbolInfo();
    NodeIDAllocator::unset();
}

/// Parse a continuous-mode input line and update the mutable Options that
/// control each round. Returns the input bitcode file path (positional arg).
/// Expected token order (flags can be reordered, but bitcode must be last):
///   -beforecpp <dir> -aftercpp <dir> -sourcediff <file> <bitcode>
static std::string parseRoundOptions(const std::vector<std::string>& tokens)
{
    std::string bcPath;
    // Reset per-round path options so values from previous rounds do not leak.
    Options::sourcediff.setValue("");
    for (size_t i = 0; i < tokens.size(); ++i)
    {
        const std::string& tok = tokens[i];
        if (tok == "-beforecpp" && i + 1 < tokens.size())
        {
            Options::beforecpp.setValue(tokens[++i]);
        }
        else if (tok == "-aftercpp" && i + 1 < tokens.size())
        {
            Options::aftercpp.setValue(tokens[++i]);
        }
        else if (tok == "-sourcediff" && i + 1 < tokens.size())
        {
            Options::sourcediff.setValue(tokens[++i]);
        }
        else if (tok == "-is-new" && i + 1 < tokens.size())
        {
            // Allow the user to override, but continuous mode normally sets it internally.
            std::string v = tokens[++i];
            Options::IsNew.setValue(v == "true");
        }
        else if (tok == "-irdiff")
        {
            // Continuous mode always enables diff when needed; ignore explicit flag here.
            // If a value follows ("true"/"false"), skip it.
            if (i + 1 < tokens.size() && (tokens[i + 1] == "true" || tokens[i + 1] == "false"))
                ++i;
        }
        else if (!tok.empty() && tok[0] != '-')
        {
            // Positional argument: bitcode path. Last positional wins.
            bcPath = tok;
        }
    }
    return bcPath;
}

/// Build SVFIR from a bitcode file path. Returns nullptr on failure.
static SVFIR* buildPAG(const std::string& bcPath)
{
    std::vector<std::string> moduleNameVec = {bcPath};

    if (Options::WriteAnder() == "ir_annotator")
    {
        LLVMModuleSet::preProcessBCs(moduleNameVec);
    }

    SVFModule* svfModule = LLVMModuleSet::buildSVFModule(moduleNameVec);
    if (svfModule == nullptr)
    {
        SVFUtil::errs() << "Failed to load input bitcode module(s).\n";
        return nullptr;
    }

    SVFIRBuilder builder(svfModule);
    SVFIR* pag = builder.build();
    if (pag == nullptr)
    {
        SVFUtil::errs() << "Failed to build SVFIR from input module(s).\n";
        return nullptr;
    }
    return pag;
}

/// Run one analysis round in continuous mode.
/// Returns true on success, false on error.
static bool runAnalysisRound(const std::vector<std::string>& tokens, int roundNum)
{
    SVFUtil::outs() << "\n========== Continuous round " << roundNum << " ==========\n";

    std::string bcPath = parseRoundOptions(tokens);
    if (bcPath.empty())
    {
        SVFUtil::errs() << "No input bitcode provided for round " << roundNum << ".\n";
        return false;
    }

    // If the user did not provide -sourcediff, generate a per-round diff file automatically.
    if (Options::sourcediff().empty())
    {
        std::string autoDiff = "./.silva_continuous_diff_round_" + std::to_string(roundNum) + ".txt";
        Options::sourcediff.setValue(autoDiff);
    }

    SVFUtil::outs() << "Input bitcode: " << bcPath << "\n";
    SVFUtil::outs() << "beforecpp: " << Options::beforecpp() << "\n";
    SVFUtil::outs() << "aftercpp:  " << Options::aftercpp() << "\n";
    SVFUtil::outs() << "sourcediff: " << Options::sourcediff() << "\n";

    // Reset all singletons from the previous round.
    resetAnalysisState();

    // Load new bitcode and build PAG.
    SVFIR* pag = buildPAG(bcPath);
    if (pag == nullptr)
        return false;

    // Run source + IR diff.
    diff();
    SVFIRGetter* irGetter = SVFIRGetter::getSVFIRGetter();
    (void)irGetter;

    IRDiffHandler* irDiff = IRDiffHandler::getIRDiffHandler();
    bool pureDeletion = irDiff->getInstAddSet().empty();

    // Choose analysis strategy based on diff type.
    PointerAnalysis::PTATY ptaTy;
    if (pureDeletion)
    {
        SVFUtil::outs() << "Diff is pure deletion -> running incremental Andersen (Andersen_INC).\n";
        Options::irdiff.setValue(true);
        Options::IsNew.setValue(false);
        ptaTy = PointerAnalysis::Andersen_INC;
    }
    else
    {
        SVFUtil::outs() << "Diff contains additions -> running full Andersen (Andersen_WPA).\n";
        Options::irdiff.setValue(false);
        Options::IsNew.setValue(false);
        ptaTy = PointerAnalysis::Andersen_WPA;
    }

    WPAPass wpa;
    wpa.runPointerAnalysis(pag, ptaTy);

    SVFUtil::outs() << "========== Round " << roundNum << " finished ==========\n";
    return true;
}

int main(int argc, char** argv)
{
    auto moduleNameVec =
        OptionBase::parseOptions(argc, argv, "Whole Program Points-to Analysis",
                                 "[options] <input-bitcode...>");

    if (Options::Continuous())
    {
        SVFUtil::outs() << "Entering continuous mode. Provide one round per line.\n";
        SVFUtil::outs() << "Format: -beforecpp <dir> -aftercpp <dir> -sourcediff <file> <bitcode>\n";
        SVFUtil::outs() << "Type 'quit' or send EOF to exit.\n";

        std::string line;
        int roundNum = 1;
        while (std::getline(std::cin, line))
        {
            // Trim leading/trailing whitespace.
            size_t start = line.find_first_not_of(" \t\r\n");
            if (start == std::string::npos)
                continue;
            size_t end = line.find_last_not_of(" \t\r\n");
            std::string trimmed = line.substr(start, end - start + 1);

            if (trimmed == "quit" || trimmed == "exit")
                break;

            std::vector<std::string> tokens = tokenizeLine(trimmed);
            if (tokens.empty())
                continue;

            bool ok = runAnalysisRound(tokens, roundNum);
            if (!ok)
            {
                SVFUtil::errs() << "Round " << roundNum << " failed. Continuing to next round.\n";
            }
            ++roundNum;
        }

        SVFUtil::outs() << "Exiting continuous mode.\n";
    }
    else
    {
        // Refers to content of a singleton unique_ptr<SVFIR> in SVFIR.
        SVFIR* pag;

        if (Options::ReadJson())
        {
            pag = SVFIRReader::read(moduleNameVec.front());
        }
        else
        {
            if (Options::WriteAnder() == "ir_annotator")
            {
                LLVMModuleSet::preProcessBCs(moduleNameVec);
            }

            SVFModule* svfModule = LLVMModuleSet::buildSVFModule(moduleNameVec);
            if (svfModule == nullptr)
            {
                SVFUtil::errs() << "Failed to load input bitcode module(s).\n";
                return 1;
            }

            /// Build SVFIR
            SVFIRBuilder builder(svfModule);
            pag = builder.build();
            if (pag == nullptr)
            {
                SVFUtil::errs() << "Failed to build SVFIR from input module(s).\n";
                return 1;
            }
        }
        if (Options::irdiff()) {
            diff();
            SVFIRGetter* irGetter = SVFIRGetter::getSVFIRGetter();
        }
        WPAPass wpa;
        wpa.runOnModule(pag);
    }

    // Temporary workaround for opaque-pointer migration: avoid teardown crash
    // in static SVFIR/ICFG destruction after successful analysis.
    std::fflush(nullptr);
    std::_Exit(0);
}
