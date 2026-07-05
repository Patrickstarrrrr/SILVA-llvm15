#include "Diff/IRDiff.h"
#include "Util/SVFUtil.h"
#include "Util/Options.h"
#include "SVF-LLVM/LLVMModule.h"

using namespace SVF;
using namespace SVFUtil;

IRDiffHandler *IRDiffHandler::irDiffHandler = nullptr;

void IRDiffHandler::reset()
{
    addVec.clear();
    deleteVec.clear();
    FunAddSet.clear();
    FunDeleteSet.clear();
    FunBodyAddSet.clear();
    FunBodyDeleteSet.clear();
    VarAddSet.clear();
    VarDeleteSet.clear();
    InstAddSet.clear();
    InstDeleteSet.clear();
}

void ModuleParser::parse(FileToDiffMapTy& sourceInfos,
                         GlobalVariableSetTy& vars, 
                         FunctionSetTy& functions, 
                         InstructionSetTy& insts,
                         bool before)
{
    // auto& modset=LLVMModuleSet::getLLVMModuleSet()->getLLVMModules();
    // if (svfm == nullptr)
    //     return;

    // int type = (t == DiffType::ADD ? 1 : 2);
    string DIR = before ? SourceDiffHandler::getBeforeRootDir() : SourceDiffHandler::getAfterRootDir();

    auto parseModule = [&](Module& mod) {
        // GlobalVar Diff
        for (auto F = mod.global_begin(), E = mod.global_end();
             F != E; ++F)
        {
            GlobalVariable& var = *F;

            if (MDNode* N = var.getMetadata("dbg"))
            {
                DIGlobalVariableExpression* LocExpr =
                    SVFUtil::cast<DIGlobalVariableExpression>(N);
                DIGlobalVariable* Loc = LocExpr->getVariable();
                unsigned Line = Loc->getLine();
                string dir = Loc->getFile()->getDirectory().str();
                string file = Loc->getFile()->getFilename().str();
                string path = getAbsPath(dir, file);
                if (SVF::Options::relapath())
                    path = getRelaPath(DIR, path);
                auto iter = sourceInfos.find(path);
                if (iter != sourceInfos.end())
                {
                    for (sourceDiffInfo E : iter->second)
                    {
                        if (Line >= E.getStart() && Line <= E.getEnd())
                        {
                            vars.insert(&var);
                            break;
                        }
                    }
                }
            }
        }

        for (auto F = mod.begin(), E = mod.end();
             F != E; ++F)
        {
            const Function& fun = *F;

            if (MDNode* N = fun.getMetadata("dbg"))
            {
                DISubprogram* Loc = SVFUtil::cast<DISubprogram>(N);
                unsigned Line = Loc->getLine();
                string dir = Loc->getFile()->getDirectory().str();
                string file = Loc->getFile()->getFilename().str();
                string path = getAbsPath(dir, file);
                if (SVF::Options::relapath())
                    path = getRelaPath(DIR, path);
                auto iter = sourceInfos.find(path);
                if (iter != sourceInfos.end())
                {
                    for (sourceDiffInfo E : iter->second)
                    {
                        if (Line >= E.getStart() && Line <= E.getEnd())
                        {
                            functions.insert(&fun);
                            break;
                        }
                    }
                }
            }

            for (Function::const_iterator iter = fun.begin(), eiter = fun.end();
                 iter != eiter; ++iter)
            {
                const BasicBlock& bb = *iter;
                for (BasicBlock::const_iterator bit = bb.begin(),
                                                ebit = bb.end();
                     bit != ebit; ++bit)
                {
                    const Instruction& inst = *bit;
                    if (MDNode* N = inst.getMetadata("dbg"))
                    {
                        DILocation* Loc = SVFUtil::cast<DILocation>(N);
                        unsigned Line = Loc->getLine();
                        string dir = Loc->getFile()->getDirectory().str();
                        string file = Loc->getFile()->getFilename().str();
                        string path = getAbsPath(dir, file);
                        if (SVF::Options::relapath())
                            path = getRelaPath(DIR, path);
                        auto iter = sourceInfos.find(path);
                        if (iter != sourceInfos.end())
                        {
                            for (sourceDiffInfo E : iter->second)
                            {
                                if (Line >= E.getStart() && Line <= E.getEnd())
                                {
                                    insts.insert(&inst);
                                    break;
                                }
                            }
                        }
                    }
                }
            }
        }
    };

    for(Module& mod : LLVMModuleSet::getLLVMModuleSet()->getLLVMModules()) {
        parseModule(mod);
    }
}

// void IRDiffHandler::parse(SVF::SVFModule *oldm, SVF::SVFModule *newm)
void IRDiffHandler::parse()
{
    // _new = newm;
    // _old = oldm;
    ModuleParser mod;
    SourceDiffHandler* h = SourceDiffHandler::getSourceDiffHandler();

    // Parse addDiff: try after-dir first, then before-dir
    mod.parse(h->getAddDiff(), VarAddSet, FunAddSet, InstAddSet, false);
    if (InstAddSet.empty()) {
        mod.parse(h->getAddDiff(), VarAddSet, FunAddSet, InstAddSet, true);
    }
    for (auto I : InstAddSet) {
        const Function* F = I->getParent()->getParent();
        FunBodyAddSet.insert(F);
    }

    // Parse deleteDiff: try before-dir first, then after-dir
    mod.parse(h->getDeleteDiff(), VarDeleteSet, FunDeleteSet, InstDeleteSet, true);
    if (InstDeleteSet.empty()) {
        mod.parse(h->getDeleteDiff(), VarDeleteSet, FunDeleteSet, InstDeleteSet, false);
    }
    for (auto I : InstDeleteSet) {
        const Function* F = I->getParent()->getParent();
        FunBodyDeleteSet.insert(F);
    }
}

void IRDiffHandler::dump(string resultFile, bool display)
{
    ofstream out(resultFile, ios::out);
    if (!out)
    {
        std::cerr << "Cannot open result file!" << std::endl;
    }

    out << "IR add begin:\n";
    for (auto E : InstAddSet)
    {
        // out << "\tType: " << E->getType() << ", Line: " << E->getLoc() << ", File: " << E->getFile() << "\n";
        out << "\t\tInst: " << E->getName().str() << ", Func: " << E->getFunction()->getName().str() << "\n";
    }
    out << "IR add end.\n\nIR delete begin:\n";
    for (auto E : InstDeleteSet)
    {
        // out << "\tType: " << E->getType() << ", Line: " << E->getRow() << ", File: " << E->getFile() << "\n";
        out << "\t\tInst: " << E->getName().str() << ", Func: " << E->getFunction()->getName().str() << "\n";
    }
    out << "IR delete end.\n\n";

    out << "Fun add begin:\n";
    for (auto E : FunBodyAddSet)
    {
        // out << "\tType: " << E->getType() << ", Line: " << E->getRow() << ", File: " << E->getFile() << "\n";
        //out << "\t\tInst: " << E->getInst()->getName().str() << ", Func: " << E->getFun()->getName().str() << "\n";
    }
    out << "Fun add end.\n\nFun delete begin:\n";
    for (auto E : FunBodyDeleteSet)
    {
        // out << "\tType: " << E->getType() << ", Line: " << E->getRow() << ", File: " << E->getFile() << "\n";
        //out << "\t\tInst: " << E->getInst()->getName().str() << ", Func: " << E->getFun()->getName().str() << "\n";
    }
    out << "Fun delete end.\n\n";

    // out << "Var add begin:\n";
    // for (auto E : varAddSet)
    // {
    //     out << "\tType: " << E.getType() << ", Line: " << E.getRow() << ", File: " << E.getFile() << "\n";
    //     //out << "\t\tInst: " << E.getInst()->getName().str() << ", Func: " << E.getFun()->getName().str() << "\n";
    // }
    // out << "Var add end.\n\nVar delete begin:\n";
    // for (auto E : varDeleteSet)
    // {
    //     out << "\tType: " << E.getType() << ", Line: " << E.getRow() << ", File: " << E.getFile() << "\n";
    //     //out << "\t\tInst: " << E.getInst()->getName().str() << ", Func: " << E.getFun()->getName().str() << "\n";
    // }
    // out << "Var delete end.\n";
}

























