#ifndef PAGGETTER_H_
#define PAGGETTER_H_

#include "SVFIR/SVFIR.h"
#include "SVF-LLVM/BasicTypes.h"
#include "SVF-LLVM/ICFGBuilder.h"
#include "SVF-LLVM/LLVMModule.h"
#include "SVF-LLVM/LLVMUtil.h"
#include "Diff/IRDiff.h"
#include "Util/Options.h"

namespace SVF
{

class SVFModule;

class SVFIRGetter: public llvm::InstVisitor<SVFIRGetter>
{
public:
    typedef std::vector<const SVFStmt*> StmtVec;
    typedef std::vector<const CallICFGNode*> CallsiteVec;
    typedef std::set<const Instruction*> InstructionSet;
private:
    SVFIR* pag;
    SVFModule* svfModule;
    const SVFBasicBlock* curBB;	///< Current basic block during SVFIR construction when visiting the module
    const SVFValue* curVal;	///< Current Value during SVFIR construction when visiting the module
    StmtVec stmts;
    CallsiteVec direCallsites;
    CallsiteVec indireCallsites;
    static std::unique_ptr<SVFIRGetter> irGetter;
    // InstructionSet diffInst;
public:
    /// Set current basic block in order to keep track of control flow information
    static inline SVFIRGetter* getSVFIRGetter()
    {
        if (irGetter == nullptr)
        {
            irGetter = std::unique_ptr<SVFIRGetter>(new SVFIRGetter(SVFIR::getPAG()->getModule()));
            IRDiffHandler* irDiff = IRDiffHandler::getIRDiffHandler();

            // Only process added instructions in is-new mode (insertion round)
            if (Options::IsNew()) {
                InstructionSet& insts = irDiff->getInstAddSet();
                for (auto inst: insts) {
                    irGetter->setCurrentLocation(inst,inst->getParent());
                    irGetter->visit(*const_cast<Instruction*>(inst));
                }
                for (auto stmt: irGetter->stmts) {
                    const_cast<SVFStmt*>(stmt)->setInserted();
                    irGetter->pag->getDiffStmts().insert(stmt);
                }
                for (auto cs: irGetter->direCallsites) {
                    irGetter->pag->getDiffDireCallSites().insert(cs);
                }
                for (auto cs: irGetter->indireCallsites) {
                    irGetter->pag->getDiffIndireCallSites().insert(cs);
                }
                irGetter->stmts.clear();
                irGetter->direCallsites.clear();
                irGetter->indireCallsites.clear();
            }

            // Only process deleted instructions in non-is-new mode (deletion round)
            if (!Options::IsNew()) {
                InstructionSet& insts = irDiff->getInstDeleteSet();
                for (auto inst: insts) {
                    irGetter->setCurrentLocation(inst,inst->getParent());
                    irGetter->visit(*const_cast<Instruction*>(inst));
                }
                for (auto stmt: irGetter->stmts) {
                    const_cast<SVFStmt*>(stmt)->setDeleted();
                    irGetter->pag->getDiffStmts().insert(stmt);
                }
                for (auto cs: irGetter->direCallsites) {
                    irGetter->pag->getDiffDireCallSites().insert(cs);
                }
                for (auto cs: irGetter->indireCallsites) {
                    irGetter->pag->getDiffIndireCallSites().insert(cs);
                }
            }
        }
        return irGetter.get();
    }

    static inline void releaseSVFIRGetter()
    {
        irGetter.reset();
    }

    /// Constructor
    SVFIRGetter(SVFModule* mod): pag(SVFIR::getPAG()), svfModule(mod), curBB(nullptr),curVal(nullptr)
    {
    }
    /// Destructor
    virtual ~SVFIRGetter()
    {
    }
    StmtVec& getDiffStmts()
    {
        return stmts;
    }
    CallsiteVec& getDirectCallsites()
    {
        return direCallsites;
    }
    CallsiteVec& getIndirectCallsites()
    {
        return indireCallsites;
    }
    /// Return SVFIR
    SVFIR* getPAG() const
    {
        return pag;
    }

    /// Compute offset of a gep instruction or gep constant expression
    bool computeGepOffset(const User *V, AccessPath& ap);
    
    /// Get different kinds of node
    //@{
    // GetValNode - Return the value node according to a LLVM Value.
    NodeID getValueNode(const Value* V)
    {
        // first handle gep edge if val if a constant expression
        // processCE(V);

        // strip off the constant cast and return the value node
        SVFValue* svfVal = LLVMModuleSet::getLLVMModuleSet()->getSVFValue(V);
        return pag->getValueNode(svfVal);
    }

    /// GetObject - Return the object node (stack/global/heap/function) according to a LLVM Value
    inline NodeID getObjectNode(const Value* V)
    {
        SVFValue* svfVal = LLVMModuleSet::getLLVMModuleSet()->getSVFValue(V);
        return pag->getObjectNode(svfVal);
    }

    /// getReturnNode - Return the node representing the unique return value of a function.
    inline NodeID getReturnNode(const SVFFunction *func)
    {
        return pag->getReturnNode(func);
    }

    /// getVarargNode - Return the node representing the unique variadic argument of a function.
    inline NodeID getVarargNode(const SVFFunction *func)
    {
        return pag->getVarargNode(func);
    }
    //@}
    
    /// Our visit overrides.
    //@{
    // Instructions that cannot be folded away.
    virtual void visitAllocaInst(AllocaInst &AI);
    void visitPHINode(PHINode &I);
    void visitStoreInst(StoreInst &I);
    void visitLoadInst(LoadInst &I);
    void visitGetElementPtrInst(GetElementPtrInst &I);
    void visitCallInst(CallInst &I);
    void visitInvokeInst(InvokeInst &II);
    void visitCallBrInst(CallBrInst &I);
    void visitCallSite(CallBase* cs);
    void visitReturnInst(ReturnInst &I);
    void visitCastInst(CastInst &I);
    void visitSelectInst(SelectInst &I);
    void visitExtractValueInst(ExtractValueInst  &EVI);
    void visitBranchInst(BranchInst &I);
    void visitSwitchInst(SwitchInst &I);
    void visitInsertValueInst(InsertValueInst &I)
    {
        addBlackHoleAddrEdge(getValueNode(&I));
    }
    // TerminatorInst and UnwindInst have been removed since llvm-8.0.0
    // void visitTerminatorInst(TerminatorInst &TI) {}
    // void visitUnwindInst(UnwindInst &I) { /*returns void*/}

    void visitBinaryOperator(BinaryOperator &I);
    void visitUnaryOperator(UnaryOperator &I);
    void visitCmpInst(CmpInst &I);

    /// TODO: var arguments need to be handled.
    /// https://llvm.org/docs/LangRef.html#id1911
    void visitVAArgInst(VAArgInst&);
    void visitVACopyInst(VACopyInst&) {}
    void visitVAEndInst(VAEndInst&) {}
    void visitVAStartInst(VAStartInst&) {}

    /// <result> = freeze ty <val>
    /// If <val> is undef or poison, ‘freeze’ returns an arbitrary, but fixed value of type `ty`
    /// Otherwise, this instruction is a no-op and returns the input <val>
    void visitFreezeInst(FreezeInst& I);

    void visitExtractElementInst(ExtractElementInst &I);

    void visitInsertElementInst(InsertElementInst &I)
    {
        addBlackHoleAddrEdge(getValueNode(&I));
    }
    void visitShuffleVectorInst(ShuffleVectorInst &I)
    {
        addBlackHoleAddrEdge(getValueNode(&I));
    }
    void visitLandingPadInst(LandingPadInst &I)
    {
        addBlackHoleAddrEdge(getValueNode(&I));
    }

    /// Instruction not that often
    void visitResumeInst(ResumeInst&)   /*returns void*/
    {
    }
    void visitUnreachableInst(UnreachableInst&)   /*returns void*/
    {
    }
    void visitFenceInst(FenceInst &I)   /*returns void*/
    {
        addBlackHoleAddrEdge(getValueNode(&I));
    }
    void visitAtomicCmpXchgInst(AtomicCmpXchgInst &I)
    {
        addBlackHoleAddrEdge(getValueNode(&I));
    }
    void visitAtomicRMWInst(AtomicRMWInst &I)
    {
        addBlackHoleAddrEdge(getValueNode(&I));
    }

    /// Provide base case for our instruction visit.
    inline void visitInstruction(Instruction&)
    {
        // If a new instruction is added to LLVM that we don't handle.
        // TODO: ignore here:
    }
    //}@


    /// Handle direct call
    void handleDirectCall(CallBase* cs, const Function *F);

    /// Handle indirect call
    void handleIndCall(CallBase* cs);

    /// Handle external call
    //@{
    virtual const Type *getBaseTypeAndFlattenedFields(const Value *V, std::vector<AccessPath> &fields, const Value* szValue);
    virtual void addComplexConsForExt(Value *D, Value *S, const Value* sz);
    virtual void handleExtCall(const CallBase* cs, const SVFFunction* svfCallee);
    //@}

    /// Set current basic block in order to keep track of control flow information
    inline void setCurrentLocation(const Value* val, const BasicBlock* bb)
    {
        curBB = (bb == nullptr? nullptr : LLVMModuleSet::getLLVMModuleSet()->getSVFBasicBlock(bb));
        curVal = (val == nullptr ? nullptr: LLVMModuleSet::getLLVMModuleSet()->getSVFValue(val));
    }
    inline void setCurrentLocation(const SVFValue* val, const SVFBasicBlock* bb)
    {
        curBB = bb;
        curVal = val;
    }
    inline const SVFValue* getCurrentValue() const
    {
        return curVal;
    }
    inline const SVFBasicBlock* getCurrentBB() const
    {
        return curBB;
    }

    NodeID getGepValVar(const Value* val, const AccessPath& ap, const SVFType* elementType);
    // void setCurrentBBAndValueForPAGEdge(PAGEdge* edge);

    inline void addBlackHoleAddrEdge(NodeID node)
    {
        if(SVFStmt *edge = pag->getBlackHoleAddrStmt(node))
            stmts.push_back(edge);
    }

    /// Add Address edge
    inline void addAddrEdge(NodeID src, NodeID dst)
    {
        if(SVFStmt *edge = pag->getAddrStmt(src, dst))
        {
            stmts.push_back(edge);
        }
    }
    /// Add Copy edge
    inline void addCopyEdge(NodeID src, NodeID dst)
    {
        if(SVFStmt *edge = pag->getCopyStmt(src, dst))
        {
            stmts.push_back(edge);
        }
    }
    /// Add Copy edge
    inline void addPhiStmt(NodeID res, NodeID opnd, const ICFGNode* pred)
    {
        /// If we already added this phi node, then skip this adding
        if(SVFStmt *edge = pag->getPhiStmt(res,opnd,pred)) {
            stmts.push_back(edge);
        }
    }
    /// Add SelectStmt
    inline void addSelectStmt(NodeID res, NodeID op1, NodeID op2, NodeID cond)
    {
        if(SVFStmt *edge = pag->getSelectStmt(res,op1,op2,cond)) {
            stmts.push_back(edge);
        }
    }
    /// Add Copy edge
    inline void addCmpEdge(NodeID op1, NodeID op2, NodeID dst, u32_t predict)
    {
        if(SVFStmt *edge = pag->getCmpStmt(op1, op2, dst, predict))
            stmts.push_back(edge);
    }
    /// Add Copy edge
    inline void addBinaryOPEdge(NodeID op1, NodeID op2, NodeID dst, u32_t opcode)
    {
        if(SVFStmt *edge = pag->getBinaryOPStmt(op1, op2, dst, opcode))
            stmts.push_back(edge);
    }
    /// Add Unary edge
    inline void addUnaryOPEdge(NodeID src, NodeID dst, u32_t opcode)
    {
        if(SVFStmt *edge = pag->getUnaryOPStmt(src, dst, opcode))
            stmts.push_back(edge);
    }
    /// Add Branch statement
    inline void addBranchStmt(NodeID br, NodeID cond, const BranchStmt::SuccAndCondPairVec& succs)
    {
        if(SVFStmt *edge = pag->getBranchStmt(br, cond, succs))
            stmts.push_back(edge);
    }
    /// Add Load edge
    inline void addLoadEdge(NodeID src, NodeID dst)
    {
        if(SVFStmt *edge = pag->getLoadStmt(src, dst))
            stmts.push_back(edge);
    }
    /// Add Store edge
    inline void addStoreEdge(NodeID src, NodeID dst)
    {
        IntraICFGNode* node;
        if (const SVFInstruction* inst = SVFUtil::dyn_cast<SVFInstruction>(curVal))
            node = pag->getICFG()->getIntraICFGNode(inst);
        else
            node = nullptr;
        if (SVFStmt* edge = pag->getStoreStmt(src, dst, node))
            stmts.push_back(edge);
    }
    /// Add Call edge
    inline void addCallEdge(NodeID src, NodeID dst, const CallICFGNode* cs, const FunEntryICFGNode* entry)
    {
        if (SVFStmt* edge = pag->getCallPE(src, dst, cs, entry))
            stmts.push_back(edge);
    }
    /// Add Return edge
    inline void addRetEdge(NodeID src, NodeID dst, const CallICFGNode* cs, const FunExitICFGNode* exit)
    {
        if (SVFStmt* edge = pag->getRetPE(src, dst, cs, exit))
            stmts.push_back(edge);
    }
    /// Add Gep edge
    inline void addGepEdge(NodeID src, NodeID dst, const AccessPath& ap, bool constGep)
    {
        if (SVFStmt* edge = pag->getGepStmt(src, dst, ap, constGep))
            stmts.push_back(edge);
    }
    /// Add Offset(Gep) edge
    inline void addNormalGepEdge(NodeID src, NodeID dst, const AccessPath& ap)
    {
        if (SVFStmt* edge = pag->getNormalGepStmt(src, dst, ap))
            stmts.push_back(edge);
    }
    /// Add Variant(Gep) edge
    inline void addVariantGepEdge(NodeID src, NodeID dst, const AccessPath& ap)
    {
        if (SVFStmt* edge = pag->getVariantGepStmt(src, dst, ap))
            stmts.push_back(edge);
    }
    /// Add Thread fork edge for parameter passing
    inline void addThreadForkEdge(NodeID src, NodeID dst, const CallICFGNode* cs, const FunEntryICFGNode* entry)
    {
        if (SVFStmt* edge = pag->getThreadForkPE(src, dst, cs, entry))
            stmts.push_back(edge);
    }
    /// Add Thread join edge for parameter passing
    inline void addThreadJoinEdge(NodeID src, NodeID dst, const CallICFGNode* cs, const FunExitICFGNode* exit)
    {
        if (SVFStmt* edge = pag->getThreadJoinPE(src, dst, cs, exit))
            stmts.push_back(edge);
    }
    //@}

    AccessPath getAccessPathFromBaseNode(NodeID nodeId);
    /// Get the base value of (i8* src and i8* dst) for external argument (e.g. memcpy(i8* dst, i8* src, int size))
    const Value* getBaseValueForExtArg(const Value* V);

};


} // end namespace SVF
#endif /* PAGGETTER_H_ */