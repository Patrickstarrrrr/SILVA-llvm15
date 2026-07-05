#ifndef INCLUDE_WPA_ANDERSENINC_H_
#define INCLUDE_WPA_ANDERSENINC_H_

#include <unordered_map>
#define STAT_TIME(timeOfFunc, ...) \
    do { \
        double dStartTime = stat->getClk(); \
        __VA_ARGS__; \
        double dEndTime = stat->getClk(); \
        timeOfFunc += (double)(dEndTime - dStartTime) / TIMEINTERVAL; \
    } while (0)


#include "WPA/Andersen.h"
#include "Graphs/SuperConsG.h"
#include "Graphs/FlatConsG.h"
namespace SVF{
class SDK 
{
public:

    NodeID src;
    NodeID dst;
    FConstraintEdge::FConstraintEdgeK kind;
    AccessPath ap;
    FConstraintEdge* compEdge;
    bool byComplex;
    SDK(NodeID s, NodeID d, FConstraintEdge::FConstraintEdgeK k, FConstraintEdge* ce = nullptr, bool bc = false)
    {
        src = s;
        dst = d;
        kind = k;
        compEdge = ce;
        byComplex = bc;
    }
    SDK(NodeID s, NodeID d, FConstraintEdge::FConstraintEdgeK k, const AccessPath& ap_)
    {
        src = s;
        dst = d;
        kind = k;
        ap = ap_;
        compEdge = nullptr;
        byComplex = false;
    }
    ~SDK(){}
};

// class AndersenInc : public AndersenWaveDiff
typedef WPASolver<SConstraintGraph*> WPASConstraintSolver;
class AndersenInc: public WPASConstraintSolver, public BVDataPTAImpl
{
    
private:
    static AndersenInc* incpta;
    // static AndersenWaveDiff* wdpta;
    

protected:
    /// Flat Constraint Graph
    
    FConstraintGraph* fCG;
    /// Super Constraint Graph
    SConstraintGraph* sCG;

public:
    
    AndersenInc(SVFIR* _pag, PTATY type = Andersen_INC, bool alias_check = true)
    // AndersenWaveDiff(_pag, type, alias_check), fCG(nullptr), sCG(nullptr)
        : BVDataPTAImpl(_pag, type, alias_check)
    {
        // wdpta = new AndersenWaveDiff(pag);
        iterationForPrintStat = OnTheFlyIterBudgetForStat;
    }

    ~AndersenInc();

    /// Andersen analysis
    virtual void analyze() override;
    void analyze_inc_reset();
    void analyze_inc();
    void getDiffSDK(bool additionReset = false);
    /// Initialize analysis
    virtual void initialize() override;

    /// Finalize analysis
    virtual void finalize() override;

    /// Methods for support type inquiry through isa, cast, and dyn_cast:
    //@{
    static inline bool classof(const AndersenInc *)
    {
        return true;
    }
    static inline bool classof(const PointerAnalysis *pta)
    {
        return (pta->getAnalysisTy() == Andersen_INC);
    }
    //@}

    inline FConstraintGraph* getFConstraintGraph() 
    {
        return fCG;
    }
    inline SConstraintGraph* getSConstraintGraph()
    {
        return sCG;
    }

    /// dump statistics
    inline void printStat()
    {
        PointerAnalysis::dumpStat();
    }

    virtual void normalizePointsTo() override;

    /// remove redundant gepnodes in constraint graph
    void cleanConsCG(NodeID id);

    NodeBS redundantGepNodes;

    /// Statistics
    //@{
    static u32_t numOfProcessedAddr;   /// Number of processed Addr edge
    static u32_t numOfProcessedCopy;   /// Number of processed Copy edge
    static u32_t numOfProcessedGep;    /// Number of processed Gep edge
    static u32_t numOfProcessedLoad;   /// Number of processed Load edge
    static u32_t numOfProcessedStore;  /// Number of processed Store edge
    static u32_t numOfSfrs;
    static u32_t numOfFieldExpand;

    static u32_t numOfSCCDetection;
    static u32_t numOfSCCDetectionIPAIns;
    static u32_t numOfSCCDetectionIPADel;
    static u32_t numOfSCCDetectionIns;
    static u32_t numOfSCCDetectionDel;

    static double timeOfSCCDetection;
    static double timeOfSCCDetectionIns;
    static double timeOfSCCDetectionDel;
    static double timeOfSCCDetectionIPAIns;
    static double timeOfSCCDetectionIPADel;

    static double timeOfSCCMerges;
    static double timeOfCollapse;
    static double timeOfCollapsePWC;
    static u32_t AveragePointsToSetSize;
    static u32_t MaxPointsToSetSize;
    static double timeOfProcessCopyGep;
    static double timeOfProcessLoadStore;
    static double timeOfUpdateCallGraph;
    
    static double timeOfExhaustivePTA;
    static double timeOfIncrementalPTA;
    static double timeOfDeletionPTA;
    static double timeOfInsertionPTA;

    static double timeOfDeletionSCC;
    static double timeOfInsertionSCC;
    static double timeOfDeletionProp;
    static double timeOfInsertionProp;

    static double timeOfDelComputeFP;
    static double timeOfInsComputeFP;

    static double timeOfSCGRebuild;
    static double timeOfSCGDelete;
    static unsigned numOfRedetectSCC;

    //@}
    typedef SCCDetection<SConstraintGraph*> CGSCC;
    typedef OrderedMap<CallSite, NodeID> CallSite2DummyValPN;
    typedef Map<NodeID, NodeID>  ID2IDMap;
    
    ID2IDMap field2Base;
    /// Reset data
    inline void resetData()
    {
        AveragePointsToSetSize = 0;
        MaxPointsToSetSize = 0;
        timeOfProcessCopyGep = 0;
        timeOfProcessLoadStore = 0;

        timeOfExhaustivePTA = 0;
        timeOfIncrementalPTA = 0;
        timeOfDeletionPTA = 0;
        timeOfInsertionPTA = 0;
    }

    /// SCC methods
    //@{
    inline NodeID sccRepNode(NodeID id) const
    {
        return sCG->sccRepNode(id);
    }
    inline NodeBS& sccSubNodes(NodeID repId)
    {
        return sCG->sccSubNodes(repId);
    }
    //@}

    /// Operation of points-to set
    virtual inline const PointsTo& getPts(NodeID id)
    {
        return getPTDataTy()->getPts(sccRepNode(id));
    }
    virtual inline bool unionPts(NodeID id, const PointsTo& target)
    {
        id = sccRepNode(id);
        return getPTDataTy()->unionPts(id, target);
    }
    virtual inline bool unionPts(NodeID id, NodeID ptd)
    {
        id = sccRepNode(id);
        ptd = sccRepNode(ptd);
        return getPTDataTy()->unionPts(id,ptd);
    }


    void dumpTopLevelPtsTo();

    inline void setDetectPWC(bool flag)
    {
        Options::DetectPWC.setValue(flag);
    }

protected:
    CallSite2DummyValPN callsite2DummyValPN;        ///< Map an instruction to a dummy obj which created at an indirect callsite, which invokes a heap allocator
    void heapAllocatorViaIndCall(CallSite cs,NodePairSet &cpySrcNodes);

    /// Handle diff points-to set.
    virtual inline void computeDiffPts(NodeID id)
    {
        if (Options::DiffPts())
        {
            NodeID rep = sccRepNode(id);
            getDiffPTDataTy()->computeDiffPts(rep, getDiffPTDataTy()->getPts(rep));
        }
    }
    virtual inline const PointsTo& getDiffPts(NodeID id)
    {
        NodeID rep = sccRepNode(id);
        if (Options::DiffPts())
            return getDiffPTDataTy()->getDiffPts(rep);
        else
            return getPTDataTy()->getPts(rep);
    }

    /// Handle propagated points-to set.
    inline void updatePropaPts(NodeID dstId, NodeID srcId)
    {
        if (!Options::DiffPts())
            return;
        NodeID srcRep = sccRepNode(srcId);
        NodeID dstRep = sccRepNode(dstId);
        getDiffPTDataTy()->updatePropaPtsMap(srcRep, dstRep);
    }
    inline void clearPropaPts(NodeID src)
    {
        if (Options::DiffPts())
        {
            NodeID rep = sccRepNode(src);
            getDiffPTDataTy()->clearPropaPts(rep);
        }
    }

    virtual void initWorklist() {}

    /// Override WPASolver function in order to use the default solver
    // virtual void processNode(NodeID nodeId);

public:
    static AndersenInc* createAndersenInc(SVFIR* _pag)
    {
        if (incpta == nullptr)
        {
            incpta = new AndersenInc(_pag, Andersen_INC, false);
            // wdpta = new AndersenWaveDiff(_pag); //
            // wdpta->analyze(); //
            incpta->analyze();
            return incpta;
        }
        return incpta;
    }
    static void releaseAndersenInc()
    {
        if (incpta) {
            // delete wdpta;
            delete incpta;
        }
        incpta = nullptr;
    }

    
    /// handling various constraints
    //@{
    virtual void processAllAddr();

    virtual bool processLoad(NodeID, const SConstraintEdge* load);
    virtual bool processStore(NodeID, const SConstraintEdge* store);
    virtual bool processCopy(NodeID, const SConstraintEdge* edge);
    virtual bool processGep(NodeID, const GepSCGEdge* edge);
    virtual void handleCopyGep(SConstraintNode* node);
    virtual void handleLoadStore(SConstraintNode* node);
    virtual void processAddr(const AddrSCGEdge* addr);    
    virtual bool processGepPts(const PointsTo& pts, const GepSCGEdge* edge);
    //@}

    /// Add copy edge on constraint graph
    virtual inline bool addCopyEdge(NodeID src, NodeID dst)
    {
        fCG->addCopyFCGEdge(src, dst);
        if (sCG->addCopySCGEdge(src, dst, true))
        {
            updatePropaPts(src, dst);
            return true;
        }
        return false;
    }

    /// Update call graph for the input indirect callsites
    virtual bool updateCallGraph(const CallSiteToFunPtrMap& callsites);

    /// Connect formal and actual parameters for indirect callsites
    void connectCaller2CalleeParams(CallSite cs, const SVFFunction* F, NodePairSet& cpySrcNodes);


    /// Merge sub node to its rep
    virtual void mergeNodeToRep(NodeID nodeId, NodeID newRepId);
    virtual bool mergeSrcToTgt(NodeID srcId, NodeID tgtId);
    /// Merge sub node in a SCC cycle to their rep node
    //@{
    virtual void mergeSccNodes(NodeID repNodeId, const NodeBS& subNodes);
    virtual void mergeSccCycle();
    //@}
    /// Collapse a field object into its base for field insensitive anlaysis
    //@{
    virtual void collapsePWCNode(NodeID nodeId);
    virtual void collapseFields();
    virtual bool collapseNodePts(NodeID nodeId);
    virtual bool collapseField(NodeID nodeId);
    //@}

    /// Updates subnodes of its rep, and rep node of its subs
    void updateNodeRepAndSubs(NodeID nodeId,NodeID newRepId);
    
    /// SCC detection
    virtual NodeStack& SCCDetect();
    // void NodeStack& SCCDetectIns();
    
    /// Get PTA name
    virtual const std::string PTAName() const
    {
        return "AndersenInc";
    }
    

   

    virtual void solveWorklist();
    virtual void processNode(NodeID nodeId);
    virtual void postProcessNode(NodeID nodeId);
    virtual bool handleLoad(NodeID id, const SConstraintEdge* load); /// TODO: --wjy
    virtual bool handleStore(NodeID id, const SConstraintEdge* store);/// TODO: --wjy
    virtual bool addCopyEdgeByComplexEdge(NodeID fsrc, NodeID fdst, FConstraintEdge* complexEdge);

// Incremental API
public:
    // typedef std::set<const llvm::Instruction*> InstructionSetTy;
    // typedef std::set<const llvm::Function*> FunctionSetTy;

    struct PtsDiff 
    {
        NodeID nodeId;
        PointsTo prePts;
        PointsTo nowPts;
        PointsTo insPts;
        PointsTo delPts;
    };
    typedef std::unordered_map<NodeID, PtsDiff*> PtsDiffMap;
    typedef std::unordered_map<NodeID, std::set<SDK*>> RepEdgeSetMap;
    typedef std::unordered_set<PAGEdge*> StmtSet;

private:
    bool SCCBreak;
    // std::vector<SrcDstKind *> delEdgesVec;  // deleted constraintEdges:(src, dst, edgeKind)
    // std::vector<SrcDstKind *> insEdgesVec;  // inserted constraintEdges
    FIFOWorkList<FConstraintEdge *> delEdgesWL;
    FIFOWorkList<FConstraintEdge *> insEdgesWL;
    FIFOWorkList<FConstraintEdge *> insPropWL;

    std::unordered_map<NodeID, PointsTo> insPropMap;
    std::unordered_map<NodeID, PointsTo> delPropMap;

    std::unordered_set<NodeID> unFilterSet;
    std::vector<SDK*> insDirectConsVec;
    std::vector<SDK*> insEdgeVec;
    std::vector<SDK*> insDirectEdgeVec;
    std::vector<SDK*> delEdgeVec;
    std::vector<SDK*> delDirectEdgeVec;
    FIFOWorkList<NodeID> redetectReps;
    RepEdgeSetMap rep2EdgeSet;
    PtsDiffMap fpPtsDiffMap; // fun ptr pts diff
    PtsDiffMap allPtsDiffMap; // all ptr pts diff
    u32_t incRound;
    NodeBS ptsChangeNodes;
    StmtSet delStmts; // TODO: -- wjy
    StmtSet insStmts; // TODO: -- wjy
    unsigned delNodeNum = 0;
    unsigned insNodeNum = 0;
    NodeBS deadObjectCandidates;    ///< objects whose addr edge was deleted; may need final cleanup
    // NodeBS ptsChainChangeNodes;
public:
    inline StmtSet& getDelStmts() { return delStmts; }
    inline StmtSet& getInsStmts() { return insStmts; }
    // void computePtsChainChangeNodes();
    // NodeBS& addRevChainNode(NodeID src);
    inline const NodeBS& getPtsChangeNodes()
    {
        return ptsChangeNodes;
    }
    inline bool isPtsChange(NodeID id)
    {
        return ptsChangeNodes.test(id);
    }
    inline const PointsTo& getInsPts(NodeID id)
    {
        return allPtsDiffMap[id]->insPts;
    }
    inline const PointsTo& getDelPts(NodeID id)
    {
        return allPtsDiffMap[id]->delPts;
    }
    inline const PointsTo& getPrePts(NodeID id)
    {
        return allPtsDiffMap[id]->prePts;
    }
public:
    bool pushIntoDelEdgesWL(NodeID src, NodeID dst, FConstraintEdge::FConstraintEdgeK kind);
    bool pushIntoInsEdgesWL(NodeID src, NodeID dst, FConstraintEdge::FConstraintEdgeK kind);

private:
    void singleIncremental(NodeID srcid, NodeID dstid, FConstraintEdge::FConstraintEdgeK kind, int count);
    void stepIncremental();
    void changeProportionInc();
    /// handling deletion
    //@{
    void processDeletion();
    void processDeletion_EdgeConstraint();
    void processDeletion_EdgeConstraint_IPA();
    void processDeletion_EdgeConstraint_Lazy();
    // void processSCCRemoveEdge(NodeID srcid, NodeID dstid, FConstraintEdge::FConstraintEdgeK kind);
    void processLoadRemoval(NodeID srcid, NodeID dstid);
    void processStoreRemoval(NodeID srcid, NodeID dstid);

    void processAddrRemoval(NodeID srcid, NodeID dstid);
    void processAddrRemoval_IPA(NodeID srcid, NodeID dstid);
    void processAddrRemoval_Lazy(NodeID srcid, NodeID dstid);
    
    void processCopyRemoval(NodeID srcid, NodeID dstid);
    void processCopyEdgeRemoval(NodeID srcid, NodeID dstid);
    void processCopyEdgeRemoval_IPA(NodeID srcid, NodeID dstid);
    void processCopyEdgeRemoval_Lazy(NodeID srcid, NodeID dstid);
    void processCopyConstraintRemoval(NodeID srcid, NodeID dstid);
    void processCopyConstraintRemoval_Lazy(NodeID srcid, NodeID dstid);

    void processVariantGepRemoval(NodeID srcid, NodeID dstid);
    void processVariantGepEdgeRemoval(NodeID srcid, NodeID dstid);
    void processVariantGepEdgeRemoval_IPA(NodeID srcid, NodeID dstid);
    void processVariantGepEdgeRemoval_Lazy(NodeID srcid, NodeID dstid);
    void processVariantGepConstraintRemoval(NodeID srcid, NodeID dstid);
    void processVariantGepConstraintRemoval_Lazy(NodeID srcid, NodeID dstid);

    void processNormalGepRemoval(NodeID srcid, NodeID dstid, const AccessPath& ap);
    void processNormalGepEdgeRemoval(NodeID srcid, NodeID dstid, const AccessPath& ap);
    void processNormalGepEdgeRemoval_IPA(NodeID srcid, NodeID dstid, const AccessPath& ap);
    void processNormalGepEdgeRemoval_Lazy(NodeID srcid, NodeID dstid, const AccessPath& ap);
    void processNormalGepConstraintRemoval(NodeID srcid, NodeID dstid, const AccessPath& ap);
    void processNormalGepConstraintRemoval_Lazy(NodeID srcid, NodeID dstid, const AccessPath& ap);

    /// Remove objects that are no longer reachable after deletion from all
    /// points-to sets, so they do not leak into memory regions/SVFG.
    void cleanupDeadObjects();
    void removeObjFromAllPts(NodeID obj);

    void processSCCRedetection();
    bool processSCCRedetection_IPA(NodeID rep);
    void propagateDelPts(const PointsTo& pts, NodeID nodeId);
    void propagateDelPts_IPA(const PointsTo& pts, NodeID nodeId, NodeBS& proping);

    void initFpPDM();
    void computeFpPDM();
    void initAllPDM();
    void computeAllPDM();

    void dumpPts(const PointsTo & pts);
    void dumpSDK(NodeID src, NodeID dst, FConstraintEdge::FConstraintEdgeK kind);

    bool updateCallGraphDel(const CallSiteToFunPtrMap& callsites);
    void onTheFlyCallGraphSolveDel(const CallSiteToFunPtrMap& callsites, CallEdgeMap& newEdges);
    void resolveCPPIndCallsDel(const CallICFGNode* cs, const PointsTo& target, CallEdgeMap& newEdges);
    void resolveIndCallsDel(const CallICFGNode* cs, const PointsTo& target, CallEdgeMap& newEdges);
    void connectVCallToVFnsDel(const CallICFGNode* cs, const VFunSet &vfns, CallEdgeMap& newEdges);
    void connectCaller2CalleeParamsDel(CallSite cs, const SVFFunction* F, NodePairSet &cpySrcNodes);
    // void heapAllocatorViaIndCallDel(CallSite cs, NodePairSet &cpySrcNodes);
    //@}

    /// pseudo wpa
    void processInsertion_pseudo();
    void processLoadAddition_pseudo(NodeID srcid, NodeID dstid);
    void processStoreAddition_pseudo(NodeID srcid, NodeID dstid);
    void processAddrAddition_pseudo(NodeID srcid, NodeID dstid); 
    void processCopyAddition_pseudo(NodeID srcid, NodeID dstid);
    void processVariantGepAddition_pseudo(NodeID srcid, NodeID dstid);
    void processNormalGepAddition_pseudo(NodeID srcid, NodeID dstid, const AccessPath& ap);

    /// handling insertion
    //@{
    void processInsertion();
    void processInsertion_IPA();
    // void processInsertion_Lazy(); // 2024.4 TODO
    bool processLoadAddition(NodeID srcid, NodeID dstid);
    bool processStoreAddition(NodeID srcid, NodeID dstid);
    void processAddrAddition(NodeID srcid, NodeID dstid); 
    void processAddrAddition_IPA(NodeID srcid, NodeID dstid); 
    bool processCopyAddition(NodeID srcid, NodeID dstid, FConstraintEdge* complexEdge = nullptr);
    bool processCopyAddition_IPA(NodeID srcid, NodeID dstid, FConstraintEdge* complexEdge = nullptr);
    
    bool processVariantGepAddition(NodeID srcid, NodeID dstid);
    bool processVariantGepAddition_IPA(NodeID srcid, NodeID dstid);
    bool processNormalGepAddition(NodeID srcid, NodeID dstid, const AccessPath& ap);
    bool processNormalGepAddition_IPA(NodeID srcid, NodeID dstid, const AccessPath& ap);
    void processCopyConstraintAddition(NodeID srcid, NodeID dstid);
    void processCopyConstraintAddition_IPA(NodeID srcid, NodeID dstid);
    // void processCopyConstraintAddition_IPA(NodeID srcid, NodeID dstid);
    void processVariantGepConstraintAddition(NodeID srcid, NodeID dstid);
    void processVariantGepConstraintAddition_IPA(NodeID srcid, NodeID dstid);
    void processNormalGepConstraintAddition(NodeID srcid, NodeID dstid, const AccessPath& ap);
    void processNormalGepConstraintAddition_IPA(NodeID srcid, NodeID dstid, const AccessPath& ap);
    void propagateInsPts(const PointsTo& pts, NodeID nodeId, bool sameSCC = false);
    void propagateInsPts_IPA(const PointsTo& pts, NodeID nodeId, bool sameSCC = false);
    // void propagateInsPts_Lazy(const PointsTo& pts, NodeID nodeId, bool sameSCC = false); // 2024.4 TODO

    bool updateCallGraphIns(const CallSiteToFunPtrMap& callsites);
    void onTheFlyCallGraphSolveIns(const CallSiteToFunPtrMap& callsites, CallEdgeMap& newEdges);
    void resolveCPPIndCallsIns(const CallICFGNode* cs, const PointsTo& target, CallEdgeMap& newEdges);
    void resolveIndCallsIns(const CallICFGNode* cs, const PointsTo& target, CallEdgeMap& newEdges);
    void connectVCallToVFnsIns(const CallICFGNode* cs, const VFunSet &vfns, CallEdgeMap& newEdges);
    void connectCaller2CalleeParamsIns(CallSite cs, const SVFFunction* F, NodePairSet &cpySrcNodes);
    // @}
    

}; // End class AndersenInc

} // End namespace SVF

#endif /*INCLUDE_WPA_ANDERSENINC_H_*/


// TODOLIST 2023.9.11 --wjy
// 0. Set type WL for edges ------------------------- Done 9.12
// 1. SCC ReDetect When Restore, note: Target fEdge and sEdge should be removed ---------- Done 9.14
// 2.1 Copy Edge Removal ---------------------------- Done 9.13
// 2.2 Gep Edge Removal ----------------------------- Done 9.13
// 3. Load/Store Edge Removal ----------------------- Done 9.12
// 4. Addr Edge Removal ----------------------------- Done 9.12
// 5. Propagate Deletion Pts Change ----------------- Done 9.14
// 6. Process Function Pointer ---------------------- Done 9.15
// 7. Set Pts for subnodes when SCCRestore. Should it be implemented in AndersenInc? ----- Done 9.17
// 8. Set field sensitivity for node when gep edge is removed?
// 9. Original Copy constraint removal  ------------- Done 9.14

// TODOLIST 2023.9.12 --wjy
// 1. Copy Edge Addition
// 2. Gep Edge Addition
// 3. Load/Store Edge Addition
// 4. Addr Edge Addition -------------------- Done 9.18
// 5. Propagate Insertion Pts Change
// 6. Process Function Pointer
//