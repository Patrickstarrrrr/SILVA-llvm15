#ifndef IR_DIFF_H
#define IR_DIFF_H

#include <queue>
#include <llvm/IR/Module.h>
#include <llvm/IR/InstIterator.h>
#include "Diff/SourceDiff.h"

using namespace llvm;
using namespace std;

enum DiffType
{
    ADD,
    DELETE
};

class DiffInfo
{
public:
    DiffInfo(int type, string file, int row) : _type(type), _file(file), _row(row)
    {
    }

    inline int getType()
    {
        return _type;
    }

    inline string getFile()
    {
        return _file;
    }

    inline int getRow()
    {
        return _row;
    }

protected:
    int _type;
    string _file;
    int _row;
};

class irDiffInfo : public DiffInfo
{
public:
    irDiffInfo(int type, string file, int row, Instruction *inst) : DiffInfo(type, file, row), _inst(inst)
    {
    }

    inline Instruction *getInst()
    {
        return _inst;
    }

private:
    Instruction *_inst;
};

class funDiffInfo : public DiffInfo
{
public:
    funDiffInfo(int type, string file, int row, Function *fun) : DiffInfo(type, file, row), _fun(fun)
    {
    }

    inline Function *getFun()
    {
        return _fun;
    }

private:
    Function *_fun;
};

class globalVarDiffInfo : public DiffInfo
{
public:
    globalVarDiffInfo(int type, string file, int row, GlobalVariable *var) : DiffInfo(type, file, row), _var(var)
    {
    }

    inline GlobalVariable *getVar()
    {
        return _var;
    }

private:
    GlobalVariable *_var;
};

class globalAliasDiffInfo : public DiffInfo
{
public:
    globalAliasDiffInfo(int type, string file, int row, GlobalAlias *alias) : DiffInfo(type, file, row), _alias(alias)
    {
    }

    inline GlobalAlias *getAlias()
    {
        return _alias;
    }

private:
    GlobalAlias *_alias;
};

class ModuleParser
{
public:
    typedef set<GlobalVariable*> GlobalVariableSetTy;
    typedef set<const Function*> FunctionSetTy;
    typedef set<const Instruction*> InstructionSetTy;
    typedef SourceDiffHandler::FileToDiffMapTy FileToDiffMapTy;

    ModuleParser() {}
    void parse(FileToDiffMapTy&, GlobalVariableSetTy&, FunctionSetTy&, InstructionSetTy&, bool);

private:
    string bc;
    string source;
    vector<sourceDiffInfo> sourceInfos;
    DiffType t;
    std::unique_ptr<llvm::Module> modPointer;

};

class IRDiffHandler
{
private:
    IRDiffHandler()
    {
    }

public:
    typedef set<const Function*> FunctionSetTy;
    typedef set<GlobalVariable*> GlobalVariableSetTy;
    typedef set<const Instruction*> InstructionSetTy;

    static IRDiffHandler *getIRDiffHandler()
    {
        if (irDiffHandler == nullptr)
        {
            irDiffHandler = new IRDiffHandler();
        }
        return irDiffHandler;
    }

    static void releaseIRDiffHandler()
    {
        delete irDiffHandler;
        irDiffHandler = nullptr;
    }

    void reset();

    void initSourceDiffInfo(vector<sourceDiffInfo>, vector<sourceDiffInfo>);

    // void parse(SVF::SVFModule *oldm, SVF::SVFModule *newm);
    void parse();

    void dump(string, bool);

    inline FunctionSetTy& getFunAddSet()
    {
        return FunAddSet;
    }

    inline FunctionSetTy& getFunDeleteSet()
    {
        return FunDeleteSet;
    }

    inline GlobalVariableSetTy& getVarAddSet()
    {
        return VarAddSet;
    }

    inline GlobalVariableSetTy& getVarDeleteSet()
    {
        return VarDeleteSet;
    }

    inline InstructionSetTy& getInstAddSet()
    {
        return InstAddSet;
    }

    inline InstructionSetTy& getInstDeleteSet()
    {
        return InstDeleteSet;
    }

    inline bool deleteFunBody(const Function* F)
    {
        return FunBodyDeleteSet.find(F) != FunBodyDeleteSet.end();
    }

    inline bool addFunBody(const Function* F)
    {
        return FunBodyAddSet.find(F) != FunBodyAddSet.end();
    }

    // SVF::SVFModule* getOldModule()
    // {
    //     return _old;
    // }

    // SVF::SVFModule* getNewModule()
    // {
    //     return _new;
    // }

private:
    static IRDiffHandler *irDiffHandler;

    // SVF::SVFModule* _old;
    // SVF::SVFModule* _new;

    vector<sourceDiffInfo> addVec;
    vector<sourceDiffInfo> deleteVec;

    FunctionSetTy FunAddSet;
    FunctionSetTy FunDeleteSet;
    FunctionSetTy FunBodyAddSet;
    FunctionSetTy FunBodyDeleteSet;
    GlobalVariableSetTy VarAddSet;
    GlobalVariableSetTy VarDeleteSet;
    InstructionSetTy InstAddSet;
    InstructionSetTy InstDeleteSet;
};

/*

class ProcessInstVisitor : public InstVisitor<ProcessInstVisitor>
{
public:
    ProcessInstVisitor(BasicBlock &bb, Function *function) : fun(function)
    {
        visit(bb);
    }

    void visitInst(Instruction &i)
    {
        DebugLoc loc = i.getDebugLoc();
        int row = loc.getLine();
        auto choose = [](sourceDiffInfo i) -> int
        {
            return i.getRow();
        };
        int pos = binarySearch(vec, vec.size(), row, choose);
        if (-1 != pos)
        {
            irDiffInfo info(vec[pos].getType(), vec[pos].getFile(), vec[pos].getRow(), &i, fun);
            res.push_back(info);
        }
    }

    vector<irDiffInfo> getRes()
    {
        return res;
    }

private:
    vector<sourceDiffInfo> vec;
    vector<irDiffInfo> res;
    Function *fun;
};

*/

#endif