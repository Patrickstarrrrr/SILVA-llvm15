#include "WPA/AndersenInc.h"
#include "MemoryModel/PointsTo.h"
#include "Graphs/SuperConsGEdge.h"
#include "Graphs/SuperConsGNode.h"
#include "SVFIR/SVFType.h"
#include "Util/Options.h"
#include "Graphs/CHG.h"
#include "Util/SVFUtil.h"
#include "MemoryModel/PointsTo.h"
#include "WPA/Steensgaard.h"

// #include "AndersenInc.h"

using namespace SVF;
using namespace SVFUtil;
using namespace std;

AndersenInc* AndersenInc::incpta = nullptr;
// AndersenWaveDiff* AndersenInc::wdpta = nullptr;

u32_t AndersenInc::numOfProcessedAddr = 0;
u32_t AndersenInc::numOfProcessedCopy = 0;
u32_t AndersenInc::numOfProcessedGep = 0;
u32_t AndersenInc::numOfProcessedLoad = 0;
u32_t AndersenInc::numOfProcessedStore = 0;
u32_t AndersenInc::numOfSfrs = 0;
u32_t AndersenInc::numOfFieldExpand = 0;

u32_t AndersenInc::numOfSCCDetection = 0;
u32_t AndersenInc::numOfSCCDetectionIns = 0;
u32_t AndersenInc::numOfSCCDetectionDel = 0;
u32_t AndersenInc::numOfSCCDetectionIPAIns = 0;
u32_t AndersenInc::numOfSCCDetectionIPADel = 0;


double AndersenInc::timeOfSCCDetection = 0;
double AndersenInc::timeOfSCCDetectionIns = 0;
double AndersenInc::timeOfSCCDetectionDel = 0;
double AndersenInc::timeOfSCCDetectionIPADel = 0;
double AndersenInc::timeOfSCCDetectionIPAIns = 0;

double AndersenInc::timeOfSCCMerges = 0;
double AndersenInc::timeOfCollapse = 0;
double AndersenInc::timeOfCollapsePWC = 0;

u32_t AndersenInc::AveragePointsToSetSize = 0;
u32_t AndersenInc::MaxPointsToSetSize = 0;
double AndersenInc::timeOfProcessCopyGep = 0;
double AndersenInc::timeOfProcessLoadStore = 0;
double AndersenInc::timeOfUpdateCallGraph = 0;

double AndersenInc::timeOfExhaustivePTA = 0;
double AndersenInc::timeOfIncrementalPTA = 0;
double AndersenInc::timeOfDeletionPTA = 0;
double AndersenInc::timeOfInsertionPTA = 0;

double AndersenInc::timeOfDeletionSCC = 0;
double AndersenInc::timeOfInsertionSCC = 0;
double AndersenInc::timeOfDeletionProp = 0;
double AndersenInc::timeOfInsertionProp = 0;

double AndersenInc::timeOfDelComputeFP = 0;
double AndersenInc::timeOfInsComputeFP = 0;
double AndersenInc::timeOfSCGRebuild = 0;
double AndersenInc::timeOfSCGDelete = 0;

unsigned AndersenInc::numOfRedetectSCC = 0;

/*!
 * Initialize
 */
void AndersenInc::initialize()
{
    ///@ Andersen::initialize() {
    resetData();
    ///@@ AndersenBase::initialize() {

    PointerAnalysis::initialize();
    // ///@@@ PointerAnalysis::initialize() {
    // // assert(pag && "SVFIR has not been built!");
    // svfMod = pag->getModule();
    // chgraph = pag->getCHG();

    // PTACallGraph* cg = new PTACallGraph();
    // CallGraphBuilder bd(cg,pag->getICFG());
    // ptaCallGraph = bd.buildCallGraph(pag->getModule());

    // callGraphSCCDetection();
    // if (Options::CallGraphDotGraph())
    //     getPTACallGraph()->dump("callgraph_initial");
    // ///@@@ } PointerAnalysis::initialize()

    // consCG = new ConstraintGraph(pag);
    fCG = new FConstraintGraph(pag);
    sCG = new SConstraintGraph(pag, fCG);
    setGraph(sCG);
    stat = new AndersenIncStat(this);
    if (Options::ConsCGDotGraph())
        sCG->dump("SConsCG_initial");
    ///@@ } AndersenBase::initialize()

    processAllAddr();
    ///@ } Andersen::intialize()



    setDetectPWC(true);   // Standard wave propagation always collapses PWCs
}

SVF::AndersenInc::~AndersenInc()
{
    // delete wdpta;

    delete sCG;
    sCG = nullptr;
    delete fCG;
    fCG = nullptr;
}

void AndersenInc::finalize()
{
    // AndersenBase::finalize()
    if (Options::ConsCGDotGraph())
        sCG->dump("SconsCG_final");

    if (Options::PrintCGGraph())
        sCG->print();

    // post process field 2 base
    for (auto it = field2Base.begin(), eit = field2Base.end(); it != eit; ++it)
    {
        NodeID field = it->first, base = sccRepNode(it->second);
        if (sCG->hasGNode(field))
            mergeNodeToRep(field, base);
    }

    BVDataPTAImpl::finalize();
}
void AndersenInc::analyze_inc_reset()
{
    SVFUtil::outs() << "Initialze incremental edgeVec(Reset):\n";
    getDiffSDK(true);
    initAllPDM();

    // Reset round: process deletion edges (isInserted -> del)
    if (!delEdgeVec.empty() || !delDirectEdgeVec.empty()) {
        SVFUtil::outs() << "Process deletion analysis(Reset):\n";
        double delStart = stat->getClk();
        if (Options::IPA())
            processDeletion_EdgeConstraint_IPA();
        else
            processDeletion_EdgeConstraint_Lazy();
        double delEnd = stat->getClk();
        timeOfDeletionPTA +=  (delEnd - delStart) / TIMEINTERVAL;
        SVFUtil::outs() << "Time of deletion PTA(Reset): " << timeOfDeletionPTA << "\n";
        SVFUtil::outs() << "  - Time of SCC Deletion(Reset): " << timeOfDeletionSCC << "\n";
        SVFUtil::outs() << "    -- Time of SCC Build TempG(Reset): " << sCG->timeOfBuildTempG << "\n";
        SVFUtil::outs() << "    -- Num of Save TempG(Reset): " << sCG->numOfSaveTempG << "\n";
        SVFUtil::outs() << "    -- Time of SCC Find(Reset): " << sCG->timeOfSCCFind << "\n";
        SVFUtil::outs() << "    -- Time of SCC Edge Restore(Reset): " << sCG->timeOfSCCEdgeRestore << "\n";
        SVFUtil::outs() << "      --- Time of Restore collect edge(Reset): " << sCG->timeOfCollectEdge << "\n";
        SVFUtil::outs() << "      --- Time of Restore remove edge(Reset): " << sCG->timeOfRemoveEdge << "\n";
        SVFUtil::outs() << "      --- Time of Restore add edge(Reset): " << sCG->timeOfAddEdge<< "\n";
        SVFUtil::outs() << "  - Time of Del Pts Prop(Reset): " << timeOfDeletionProp << "\n";
        SVFUtil::outs() << "  - Num of Del Nodes: " << delNodeNum << "\n";
        SVFUtil::outs() << "------------------------------------------------------------------\n";
    }

    // Reset round also handles insertion edges (isDeleted -> ins)
    if (!insEdgeVec.empty() || !insDirectEdgeVec.empty()) {
        SVFUtil::outs() << "Process insertion analysis(Reset):\n";
        double insStart = stat->getClk();
        if (Options::IPA())
            processInsertion_IPA();
        else
            processInsertion();
        double insEnd = stat->getClk();
        timeOfInsertionPTA += (insEnd - insStart) / TIMEINTERVAL;
        SVFUtil::outs() << "Time of insertion PTA(Reset): " << timeOfInsertionPTA << "\n";
        SVFUtil::outs() << "  - Time of SCC Insertion (so far): " << timeOfInsertionSCC << "\n";
        SVFUtil::outs() << "  - Time of Ins Pts Prop: " << timeOfInsertionProp << "\n";
        SVFUtil::outs() << "  - Num of Ins Nodes: " << insNodeNum << "\n";
        SVFUtil::outs() << "------------------------------------------------------------------\n";
    }

    computeAllPDM();
}
void AndersenInc::analyze_inc()
{
    SVFUtil::outs() << "Initialze incremental edgeVec:\n";
    deadObjectCandidates.clear();
    getDiffSDK();
    initAllPDM();

    // Process deletion edges (isDeleted -> del)
    if (!delEdgeVec.empty() || !delDirectEdgeVec.empty()) {
        SVFUtil::outs() << "Process deletion analysis:\n";
        double delStart = stat->getClk();
        if (Options::IPA())
            processDeletion_EdgeConstraint_IPA();
        else
            processDeletion_EdgeConstraint_Lazy();
        double delEnd = stat->getClk();
        timeOfDeletionPTA +=  (delEnd - delStart) / TIMEINTERVAL;
        SVFUtil::outs() << "Time of deletion PTA: " << timeOfDeletionPTA << "\n";
        SVFUtil::outs() << "  - Time of SCC Deletion: " << timeOfDeletionSCC << "\n";
        SVFUtil::outs() << "    -- Time of SCC Build TempG: " << sCG->timeOfBuildTempG << "\n";
        SVFUtil::outs() << "    -- Num of Save TempG: " << sCG->numOfSaveTempG << "\n";
        SVFUtil::outs() << "    -- Time of SCC Find: " << sCG->timeOfSCCFind << "\n";
        SVFUtil::outs() << "    -- Time of SCC Edge Restore: " << sCG->timeOfSCCEdgeRestore << "\n";
        SVFUtil::outs() << "      --- Time of Restore collect edge: " << sCG->timeOfCollectEdge << "\n";
        SVFUtil::outs() << "      --- Time of Restore remove edge: " << sCG->timeOfRemoveEdge << "\n";
        SVFUtil::outs() << "      --- Time of Restore add edge: " << sCG->timeOfAddEdge<< "\n";
        SVFUtil::outs() << "  - Time of Del Pts Prop: " << timeOfDeletionProp << "\n";
        SVFUtil::outs() << "  - Num of Del Nodes: " << delNodeNum << "\n";
        SVFUtil::outs() << "------------------------------------------------------------------\n";

        // Stack objects whose addr edge was deleted may still be referenced by
        // stale points-to facts.  Force-remove them so they do not leak into
        // memory regions / SVFG.
        cleanupDeadObjects();
    }

    // Process insertion edges (isInserted -> ins)
    if (!insEdgeVec.empty() || !insDirectEdgeVec.empty()) {
        SVFUtil::outs() << "Process insertion analysis:\n";
        double insStart = stat->getClk();
        if (Options::IPA())
            processInsertion_IPA();
        else
            processInsertion();
        double insEnd = stat->getClk();
        timeOfInsertionPTA += (insEnd - insStart) / TIMEINTERVAL;
        SVFUtil::outs() << "Time of insertion PTA: " << timeOfInsertionPTA << "\n";
        SVFUtil::outs() << "  - Time of SCC Insertion (so far): " << timeOfInsertionSCC << "\n";
        SVFUtil::outs() << "  - Time of Ins Pts Prop: " << timeOfInsertionProp << "\n";
        SVFUtil::outs() << "  - Num of Ins Nodes: " << insNodeNum << "\n";
        SVFUtil::outs() << "------------------------------------------------------------------\n";
    }

    timeOfIncrementalPTA = timeOfDeletionPTA + timeOfInsertionPTA;
    SVFUtil::outs() << "Time of incremental PTA: " << timeOfIncrementalPTA << "\n";

    double diffstart = stat->getClk();
    computeAllPDM();
    double diffend = stat->getClk();
    SVFUtil::outs() << "Time of Diff PTA: " << (diffend-diffstart)/TIMEINTERVAL << "\n";
    stat->performStat();
}

void AndersenInc::getDiffSDK(bool additionReset)
{
    unsigned addrcount = 0, copycount = 0, vgepcount = 0,
        ngepcount = 0, loadcount = 0, storecount = 0;
    IRGraph::SVFStmtSet& diffStmts = pag->getDiffStmts();
    for (auto iter = diffStmts.begin(), eiter =
                diffStmts.end(); iter != eiter; ++iter)
    {
        const SVFStmt* stmt = *iter;
        if ( !((*iter)->isPTAEdge()) )
            continue;

        // Use isInserted/isDeleted flags to determine edge direction,
        // independent of Options::IsNew().
        bool isInserted = stmt->isInserted;
        bool isDeleted = stmt->isDeleted;

        // Determine target vector based on stmt flag and round:
        //   - isInserted: reset->del, incremental->ins
        //   - isDeleted:  reset->ins, incremental->del
        auto pushSDK = [&](NodeID src, NodeID dst, FConstraintEdge::FConstraintEdgeK kind, const AccessPath& ap = AccessPath()) {
            if (isInserted) {
                if (!additionReset) {
                    insEdgeVec.push_back(new SDK(src, dst, kind, ap));
                } else {
                    delEdgeVec.push_back(new SDK(src, dst, kind, ap));
                }
            } else if (isDeleted) {
                if (!additionReset) {
                    delEdgeVec.push_back(new SDK(src, dst, kind, ap));
                } else {
                    insEdgeVec.push_back(new SDK(src, dst, kind, ap));
                }
            }
        };
        auto pushDirectSDK = [&](NodeID src, NodeID dst, FConstraintEdge::FConstraintEdgeK kind, const AccessPath& ap = AccessPath()) {
            if (isInserted) {
                if (!additionReset) {
                    insDirectEdgeVec.push_back(new SDK(src, dst, kind, ap));
                } else {
                    delDirectEdgeVec.push_back(new SDK(src, dst, kind, ap));
                }
            } else if (isDeleted) {
                if (!additionReset) {
                    delDirectEdgeVec.push_back(new SDK(src, dst, kind, ap));
                } else {
                    insDirectEdgeVec.push_back(new SDK(src, dst, kind, ap));
                }
            }
        };

        if (SVFUtil::isa<AddrStmt>(stmt)) {
            addrcount ++;
            const AddrStmt* edge = SVFUtil::cast<AddrStmt>(stmt);
            NodeID src = edge->getRHSVarID();
            NodeID dst = edge->getLHSVarID();
            pushSDK(src, dst, FConstraintEdge::FAddr);
        }
        else if (SVFUtil::isa<CopyStmt>(stmt)) {
            copycount ++;
            const CopyStmt* edge = SVFUtil::cast<CopyStmt>(*iter);
            NodeID src = edge->getRHSVarID();
            NodeID dst = edge->getLHSVarID();
            pushDirectSDK(src, dst, FConstraintEdge::FCopy);
        }
        else if (SVFUtil::isa<PhiStmt>(stmt)) {
            const PhiStmt* edge = SVFUtil::cast<PhiStmt>(*iter);
            for (const auto opVar: edge->getOpndVars()) {
                copycount++;
                NodeID src = opVar->getId();
                NodeID dst = edge->getResID();
                pushDirectSDK(src, dst, FConstraintEdge::FCopy);
            }
        }
        else if (SVFUtil::isa<SelectStmt>(stmt)) {
            const SelectStmt* edge = SVFUtil::cast<SelectStmt>(*iter);
            for (const auto opVar: edge->getOpndVars()) {
                copycount++;
                NodeID src = opVar->getId();
                NodeID dst = edge->getResID();
                pushDirectSDK(src, dst, FConstraintEdge::FCopy);
            }
        }
        else if (SVFUtil::isa<CallPE>(stmt)) {
            copycount ++;
            const CallPE* edge = SVFUtil::cast<CallPE>(*iter);
            NodeID src = edge->getRHSVarID();
            NodeID dst = edge->getLHSVarID();
            pushDirectSDK(src, dst, FConstraintEdge::FCopy);
        }
        else if (SVFUtil::isa<RetPE>(stmt)) {
            copycount ++;
            const RetPE* edge = SVFUtil::cast<RetPE>(*iter);
            NodeID src = edge->getRHSVarID();
            NodeID dst = edge->getLHSVarID();
            pushDirectSDK(src, dst, FConstraintEdge::FCopy);
        }
        else if (SVFUtil::isa<TDForkPE>(stmt)) {
            copycount ++;
            const TDForkPE* edge = SVFUtil::cast<TDForkPE>(*iter);
            NodeID src = edge->getRHSVarID();
            NodeID dst = edge->getLHSVarID();
            pushDirectSDK(src, dst, FConstraintEdge::FCopy);
        }
        else if (SVFUtil::isa<TDJoinPE>(stmt)) {
            copycount++;
            const TDJoinPE* edge = SVFUtil::cast<TDJoinPE>(*iter);
            NodeID src = edge->getRHSVarID();
            NodeID dst = edge->getLHSVarID();
            pushDirectSDK(src, dst, FConstraintEdge::FCopy);
        }
        else if (SVFUtil::isa<GepStmt>(stmt)) {
            const GepStmt* edge = SVFUtil::cast<GepStmt>(*iter);
            NodeID src = edge->getRHSVarID();
            NodeID dst = edge->getLHSVarID();
            if (edge->isVariantFieldGep()) {
                vgepcount++;
                pushDirectSDK(src, dst, FConstraintEdge::FVariantGep);
            }
            else {
                ngepcount++;
                pushDirectSDK(src, dst, FConstraintEdge::FNormalGep, edge->getAccessPath());
            }
        }
        else if (SVFUtil::isa<StoreStmt>(stmt)) {
            storecount ++;
            const StoreStmt* edge = SVFUtil::cast<StoreStmt>(*iter);
            NodeID src = edge->getRHSVarID();
            NodeID dst = edge->getLHSVarID();
            pushSDK(src, dst, FConstraintEdge::FStore);
        }
        else if (SVFUtil::isa<LoadStmt>(stmt)) {
            loadcount ++;
            const LoadStmt* edge = SVFUtil::cast<LoadStmt>(*iter);
            NodeID src = edge->getRHSVarID();
            NodeID dst = edge->getLHSVarID();
            pushSDK(src, dst, FConstraintEdge::FLoad);
        }

    }

    SVFUtil::outs() << "Total Stmts: " << diffStmts.size() << "\n";
    SVFUtil::outs() << "Total PTAStmts: " << addrcount + copycount + 
        vgepcount + ngepcount + loadcount + storecount << "\n";
    SVFUtil::outs() << "Addr Stmts: " << addrcount << "\n";
    SVFUtil::outs() << "Copy Stmts: " << copycount << "\n";
    SVFUtil::outs() << "VGep Stmts: " << vgepcount << "\n";
    SVFUtil::outs() << "NGep Stmts: " << ngepcount << "\n";
    SVFUtil::outs() << "Load Stmts: " << loadcount << "\n";
    SVFUtil::outs() << "Store Stmts: " << storecount << "\n";
}

void AndersenInc::analyze()
{

    // wdpta->analyze();
    initialize();

    // if(!readResultsFromFile)
    // {
    // Start solving constraints
    DBOUT(DGENERAL, outs() << SVFUtil::pasMsg("Start Solving Constraints\n"));

    bool limitTimerSet = SVFUtil::startAnalysisLimitTimer(Options::AnderTimeLimit());

    double eptaStart = stat->getClk();
    initWorklist();
    do
    {
        numOfIteration++;
        if (0 == numOfIteration % iterationForPrintStat)
            printStat();

        reanalyze = false;

        solveWorklist();

        if (updateCallGraph(getIndirectCallsites()))
            reanalyze = true;

    }
    while (reanalyze);

    // Analysis is finished, reset the alarm if we set it.
    SVFUtil::stopAnalysisLimitTimer(limitTimerSet);

    // normalizePointsTo();
    
    double eptaEnd = stat->getClk();
    timeOfExhaustivePTA += (eptaEnd - eptaStart) / TIMEINTERVAL;
    SVFUtil::outs() << "Time of Exhaustive PTA: " << timeOfExhaustivePTA << "\n";
    DBOUT(DGENERAL, outs() << SVFUtil::pasMsg("Finish Solving Constraints\n"));

    if (!Options::irdiff()) {
        initAllPDM();
        u32_t sr, spr, step;
        // }
        double changep = Options::ChangeProportion();
        if (changep != 0)
            changeProportionInc();
        else {
            step = Options::IncStep();
            u32_t stepCount = 0;
            incRound = 0;
            if (Options::IncAddr()) {
                sr = Options::AddrStartingRound();
                spr = Options::AddrSpecificRound();
                int addrcount = 0;
                SVFStmt::SVFStmtSetTy& addrs = pag->getPTASVFStmtSet(SVFStmt::Addr);
                for (SVFStmt::SVFStmtSetTy::iterator iter = addrs.begin(), eiter =
                            addrs.end(); iter != eiter; ++iter)
                {
                    addrcount++;
                    if (addrcount < sr)
                        continue; 
                    if (spr != 0 && spr != addrcount)
                        continue;
                    stepCount++;
                    const AddrStmt* edge = SVFUtil::cast<AddrStmt>(*iter);
                    // addAddrFCGEdge(edge->getRHSVarID(),edge->getLHSVarID());
                    NodeID src = edge->getRHSVarID();
                    NodeID dst = edge->getLHSVarID();
                    delEdgeVec.push_back(new SDK(src, dst, FConstraintEdge::FAddr));
                    insEdgeVec.push_back(new SDK(src, dst, FConstraintEdge::FAddr));
                    if (stepCount == step) {
                        stepCount = 0;
                        incRound++;
                        stepIncremental();
                    }
                    // singleIncremental(src, dst, FConstraintEdge::FAddr, addrcount);
                }
            }

            if (Options::IncCopy()) {
                sr = Options::CopyStartingRound();
                spr = Options::CopySpecificRound();
                int copycount = 0;
                SVFStmt::SVFStmtSetTy& copys = pag->getPTASVFStmtSet(SVFStmt::Copy);
                for (SVFStmt::SVFStmtSetTy::iterator iter = copys.begin(), eiter =
                            copys.end(); iter != eiter; ++iter)
                {
                    copycount++;
                    if (copycount < sr)
                        continue; 
                    if (spr != 0 && spr != copycount)
                        continue;
                    stepCount++;
                    const CopyStmt* edge = SVFUtil::cast<CopyStmt>(*iter);
                    // addCopyFCGEdge(edge->getRHSVarID(),edge->getLHSVarID());
                    NodeID src = edge->getRHSVarID();
                    NodeID dst = edge->getLHSVarID();
                    delDirectEdgeVec.push_back(new SDK(src, dst, FConstraintEdge::FCopy));
                    insDirectEdgeVec.push_back(new SDK(src, dst, FConstraintEdge::FCopy));
                    if (stepCount == step) {
                        stepCount = 0;
                        incRound++;
                        stepIncremental();
                    }
                    // singleIncremental(src, dst, FConstraintEdge::FCopy, copycount);
                }

                SVFStmt::SVFStmtSetTy& phis = pag->getPTASVFStmtSet(SVFStmt::Phi);
                for (SVFStmt::SVFStmtSetTy::iterator iter = phis.begin(), eiter =
                            phis.end(); iter != eiter; ++iter)
                {
                    const PhiStmt* edge = SVFUtil::cast<PhiStmt>(*iter);
                    for(const auto opVar : edge->getOpndVars()) {
                        // addCopyFCGEdge(opVar->getId(),edge->getResID());
                        copycount++;
                        if (copycount < sr)
                            continue; 
                        if (spr != 0 && spr != copycount)
                            continue;
                        stepCount++;
                        NodeID src = opVar->getId();
                        NodeID dst = edge->getResID();
                        delDirectEdgeVec.push_back(new SDK(src, dst, FConstraintEdge::FCopy));
                        insDirectEdgeVec.push_back(new SDK(src, dst, FConstraintEdge::FCopy));
                        if (stepCount == step) {
                            stepCount = 0;
                            incRound++;
                            stepIncremental();
                        }
                        // singleIncremental(src, dst, FConstraintEdge::FCopy, copycount);
                    }
                }

                SVFStmt::SVFStmtSetTy& selects = pag->getPTASVFStmtSet(SVFStmt::Select);
                for (SVFStmt::SVFStmtSetTy::iterator iter = selects.begin(), eiter =
                            selects.end(); iter != eiter; ++iter)
                {
                    const SelectStmt* edge = SVFUtil::cast<SelectStmt>(*iter);
                    for(const auto opVar : edge->getOpndVars()) {
                        // addCopyFCGEdge(opVar->getId(),edge->getResID());
                        copycount++;
                        if (copycount < sr)
                            continue; 
                        if (spr != 0 && spr != copycount)
                            continue;
                        stepCount++;
                        NodeID src = opVar->getId();
                        NodeID dst = edge->getResID();
                        delDirectEdgeVec.push_back(new SDK(src, dst, FConstraintEdge::FCopy));
                        insDirectEdgeVec.push_back(new SDK(src, dst, FConstraintEdge::FCopy));
                        if (stepCount == step) {
                            stepCount = 0;
                            incRound++;
                            stepIncremental();
                        }
                        // singleIncremental(src, dst, FConstraintEdge::FCopy, copycount);
                    }
                }

                SVFStmt::SVFStmtSetTy& calls = pag->getPTASVFStmtSet(SVFStmt::Call);
                for (SVFStmt::SVFStmtSetTy::iterator iter = calls.begin(), eiter =
                            calls.end(); iter != eiter; ++iter)
                {
                    copycount++;
                    if (copycount < sr)
                        continue; 
                    if (spr != 0 && spr != copycount)
                        continue;
                    stepCount++;
                    const CallPE* edge = SVFUtil::cast<CallPE>(*iter);
                    // addCopyFCGEdge(edge->getRHSVarID(),edge->getLHSVarID());
                    NodeID src = edge->getRHSVarID();
                    NodeID dst = edge->getLHSVarID();
                    delDirectEdgeVec.push_back(new SDK(src, dst, FConstraintEdge::FCopy));
                    insDirectEdgeVec.push_back(new SDK(src, dst, FConstraintEdge::FCopy));
                    if (stepCount == step) {
                        stepCount = 0;
                        incRound++;
                        stepIncremental();
                    }
                    // singleIncremental(src, dst, FConstraintEdge::FCopy, copycount);
                }

                SVFStmt::SVFStmtSetTy& rets = pag->getPTASVFStmtSet(SVFStmt::Ret);
                for (SVFStmt::SVFStmtSetTy::iterator iter = rets.begin(), eiter =
                            rets.end(); iter != eiter; ++iter)
                {
                    copycount++;
                    if (copycount < sr)
                        continue; 
                    if (spr != 0 && spr != copycount)
                        continue;
                    stepCount++;
                    const RetPE* edge = SVFUtil::cast<RetPE>(*iter);
                    // addCopyFCGEdge(edge->getRHSVarID(),edge->getLHSVarID());
                    NodeID src = edge->getRHSVarID();
                    NodeID dst = edge->getLHSVarID();
                    delDirectEdgeVec.push_back(new SDK(src, dst, FConstraintEdge::FCopy));
                    insDirectEdgeVec.push_back(new SDK(src, dst, FConstraintEdge::FCopy));
                    if (stepCount == step) {
                        stepCount = 0;
                        incRound++;
                        stepIncremental();
                    }
                    // singleIncremental(src, dst, FConstraintEdge::FCopy, copycount);
                }

                SVFStmt::SVFStmtSetTy& tdfks = pag->getPTASVFStmtSet(SVFStmt::ThreadFork);
                for (SVFStmt::SVFStmtSetTy::iterator iter = tdfks.begin(), eiter =
                            tdfks.end(); iter != eiter; ++iter)
                {
                    copycount++;
                    if (copycount < sr)
                        continue; 
                    if (spr != 0 && spr != copycount)
                        continue;
                    stepCount++;
                    const TDForkPE* edge = SVFUtil::cast<TDForkPE>(*iter);
                    // addCopyFCGEdge(edge->getRHSVarID(),edge->getLHSVarID());
                    NodeID src = edge->getRHSVarID();
                    NodeID dst = edge->getLHSVarID();
                    delDirectEdgeVec.push_back(new SDK(src, dst, FConstraintEdge::FCopy));
                    insDirectEdgeVec.push_back(new SDK(src, dst, FConstraintEdge::FCopy));
                    if (stepCount == step) {
                        stepCount = 0;
                        incRound++;
                        stepIncremental();
                    }
                    // singleIncremental(src, dst, FConstraintEdge::FCopy, copycount);
                }

                SVFStmt::SVFStmtSetTy& tdjns = pag->getPTASVFStmtSet(SVFStmt::ThreadJoin);
                for (SVFStmt::SVFStmtSetTy::iterator iter = tdjns.begin(), eiter =
                            tdjns.end(); iter != eiter; ++iter)
                {
                    copycount++;
                    if (copycount < sr)
                        continue; 
                    if (spr != 0 && spr != copycount)
                        continue;
                    stepCount++;
                    const TDJoinPE* edge = SVFUtil::cast<TDJoinPE>(*iter);
                    // addCopyFCGEdge(edge->getRHSVarID(),edge->getLHSVarID());
                    NodeID src = edge->getRHSVarID();
                    NodeID dst = edge->getLHSVarID();
                    delDirectEdgeVec.push_back(new SDK(src, dst, FConstraintEdge::FCopy));
                    insDirectEdgeVec.push_back(new SDK(src, dst, FConstraintEdge::FCopy));
                    if (stepCount == step) {
                        stepCount = 0;
                        incRound++;
                        stepIncremental();
                    }
                    // singleIncremental(src, dst, FConstraintEdge::FCopy, copycount);
                }
            }

            if (Options::IncVGep()) { // TODO: NGep
                u32_t vsr = Options::VGepStartingRound();
                u32_t vspr = Options::VGepSpecificRound();
                u32_t nsr = Options::NGepStartingRound();
                u32_t nspr = Options::NGepSpecificRound();
                int gepcount = 0;
                int vgepcount = 0;
                int ngepcount = 0;
                SVFStmt::SVFStmtSetTy& ngeps = pag->getPTASVFStmtSet(SVFStmt::Gep);
                for (SVFStmt::SVFStmtSetTy::iterator iter = ngeps.begin(), eiter =
                            ngeps.end(); iter != eiter; ++iter)
                {
                    gepcount++;
                    GepStmt* edge = SVFUtil::cast<GepStmt>(*iter);
                    if(edge->isVariantFieldGep()) {
                        stepCount++;
                        vgepcount++;
                        if (vgepcount < vsr)
                            continue; 
                        if (vspr != 0 && vspr != vgepcount)
                            continue;
                        // addVariantGepFCGEdge(edge->getRHSVarID(),edge->getLHSVarID());
                        NodeID src = edge->getRHSVarID();
                        NodeID dst = edge->getLHSVarID();
                        delDirectEdgeVec.push_back(new SDK(src, dst, FConstraintEdge::FVariantGep));
                        insDirectEdgeVec.push_back(new SDK(src, dst, FConstraintEdge::FVariantGep));
                        if (stepCount == step) {
                            stepCount = 0;
                            incRound++;
                            stepIncremental();
                        }
                        // singleIncremental(src, dst, FConstraintEdge::FVariantGep, gepcount);
                    }
                    else {
                        ngepcount++;
                        // if (ngepcount < nsr)
                        //     continue; 
                        // if (nspr != 0 && nspr != ngepcount)
                        //     continue;
                        // addNormalGepFCGEdge(edge->getRHSVarID(),edge->getLHSVarID(),edge->getAccessPath());
                    }
                }
            }

            if (Options::IncStore()) {
                sr = Options::StoreStartingRound();
                spr = Options::StoreSpecificRound();
                int storecount = 0;
                SVFStmt::SVFStmtSetTy& stores = pag->getPTASVFStmtSet(SVFStmt::Store);
                for (SVFStmt::SVFStmtSetTy::iterator iter = stores.begin(), eiter =
                            stores.end(); iter != eiter; ++iter)
                {
                    storecount ++;
                    if (storecount < sr)
                        continue; 
                    if (spr != 0 && spr != storecount)
                        continue;
                    stepCount++;
                    StoreStmt* edge = SVFUtil::cast<StoreStmt>(*iter);
                    NodeID src = edge->getRHSVarID();
                    NodeID dst = edge->getLHSVarID();
                    delEdgeVec.push_back(new SDK(src, dst, FConstraintEdge::FStore));
                    insEdgeVec.push_back(new SDK(src, dst, FConstraintEdge::FStore));
                    if (stepCount == step) {
                        stepCount = 0;
                        incRound++;
                        stepIncremental();
                    }
                    // singleIncremental(src, dst, FConstraintEdge::FStore, storecount);
                }
            }

            if (Options::IncLoad()) {
                sr = Options::LoadStartingRound();
                spr = Options::LoadSpecificRound();
                int loadcount = 0;
                SVFStmt::SVFStmtSetTy& loads = pag->getPTASVFStmtSet(SVFStmt::Load);
                for (SVFStmt::SVFStmtSetTy::iterator iter = loads.begin(), eiter =
                            loads.end(); iter != eiter; ++iter)
                {
                    loadcount ++;
                    if (loadcount < sr)
                        continue; 
                    if (spr != 0 && spr != loadcount)
                        continue;
                    stepCount++;
                    LoadStmt* edge = SVFUtil::cast<LoadStmt>(*iter);
                    NodeID src = edge->getRHSVarID();
                    NodeID dst = edge->getLHSVarID();
                    delEdgeVec.push_back(new SDK(src, dst, FConstraintEdge::FLoad));
                    insEdgeVec.push_back(new SDK(src, dst, FConstraintEdge::FLoad));
                    if (stepCount == step) {
                        stepCount = 0;
                        incRound++;
                        stepIncremental();
                    }
                    // singleIncremental(src, dst, FConstraintEdge::FLoad, loadcount);
                }
            }
            if (stepCount != 0) {
            SVFUtil::outs() << "Last Round with " << stepCount << " stmts\n"; 
            incRound++;
            stepIncremental();
        }
        }
    }
    finalize();
}

void AndersenInc::changeProportionInc()
{
    double changep = Options::ChangeProportion();
    u32_t incRound = Options::IncRound();
    u32_t stmtcount = 0;
    u32_t roundcount = 0;
    u32_t step = 0;
    while (roundcount < incRound) {
        stmtcount = 0;
        roundcount++;
        incRound++;
        SVFUtil::outs() << "****** stmt info ****** "<< "\n";
        SVFStmt::SVFStmtSetTy& addrs = pag->getPTASVFStmtSet(SVFStmt::Addr);
        step = addrs.size() * changep <= 1 ? 1 : (addrs.size() * changep);
        SVFUtil::outs() << "addr step: "<< step << "\n";
        u32_t addrcount = 0;
        u32_t addrbegin = (roundcount - 1) * step + 1;
        u32_t addrend = (roundcount) * step;
        for (SVFStmt::SVFStmtSetTy::iterator iter = addrs.begin(), eiter =
                addrs.end(); iter != eiter; ++iter)
        {
            addrcount++;
            if (addrcount < addrbegin)
                continue;
            if (addrcount >= addrbegin && addrcount <= addrend) {
                stmtcount++;
                const AddrStmt* edge = SVFUtil::cast<AddrStmt>(*iter);
                NodeID src = edge->getRHSVarID();
                NodeID dst = edge->getLHSVarID();
                delEdgeVec.push_back(new SDK(src, dst, FConstraintEdge::FAddr));
                insEdgeVec.push_back(new SDK(src, dst, FConstraintEdge::FAddr));
            }
            if (addrcount > addrend)
                break;
        }

        SVFStmt::SVFStmtSetTy& copys = pag->getPTASVFStmtSet(SVFStmt::Copy);
        step = copys.size() * changep <= 1 ? 1 : (copys.size() * changep);
        SVFUtil::outs() << "copy step: "<< step << "\n";
        u32_t copycount = 0;
        u32_t copybegin = (roundcount - 1) * step + 1;
        u32_t copyend = roundcount * step;
        for (SVFStmt::SVFStmtSetTy::iterator iter = copys.begin(), eiter =
            copys.end(); iter != eiter; ++iter)
        {
            copycount++;
            if (copycount < copybegin)
                continue;
            if (copycount >= copybegin && copycount <= copyend) {
                stmtcount++;
                const CopyStmt* edge = SVFUtil::cast<CopyStmt>(*iter);
                NodeID src = edge->getRHSVarID();
                NodeID dst = edge->getLHSVarID();
                delDirectEdgeVec.push_back(new SDK(src, dst, FConstraintEdge::FCopy));
                insDirectEdgeVec.push_back(new SDK(src, dst, FConstraintEdge::FCopy));
            }
            if (copycount > copyend)
                break;
        }

        SVFStmt::SVFStmtSetTy& phis = pag->getPTASVFStmtSet(SVFStmt::Phi);
        step = phis.size() * changep <= 1 ? 1 : (phis.size() * changep);
        SVFUtil::outs() << "phi step: "<< step << "\n";
        copycount = 0;
        copybegin = (roundcount - 1) * step + 1;
        copyend = roundcount * step;
        for (SVFStmt::SVFStmtSetTy::iterator iter = phis.begin(), eiter =
            phis.end(); iter != eiter; ++iter)
        {
            copycount++;
            if (copycount < copybegin)
                continue;
            if (copycount >= copybegin && copycount <= copyend) {
                const PhiStmt* edge = SVFUtil::cast<PhiStmt>(*iter);
                for(const auto opVar : edge->getOpndVars()) {
                    stmtcount++;
                    NodeID src = opVar->getId();
                    NodeID dst = edge->getResID();
                    delDirectEdgeVec.push_back(new SDK(src, dst, FConstraintEdge::FCopy));
                    insDirectEdgeVec.push_back(new SDK(src, dst, FConstraintEdge::FCopy));
                }
            }
            if (copycount > copyend)
                break;
        }

        SVFStmt::SVFStmtSetTy& selects = pag->getPTASVFStmtSet(SVFStmt::Select);
        step = selects.size() * changep <= 1 ? 1 : (selects.size() * changep);
        SVFUtil::outs() << "select step: "<< step << "\n";
        copycount = 0;
        copybegin = (roundcount - 1) * step + 1;
        copyend = roundcount * step;
        for (SVFStmt::SVFStmtSetTy::iterator iter = selects.begin(), eiter =
            selects.end(); iter != eiter; ++iter)
        {
            copycount++;
            if (copycount < copybegin)
                continue;
            if (copycount >= copybegin && copycount <= copyend) {
                const SelectStmt* edge = SVFUtil::cast<SelectStmt>(*iter);
                for(const auto opVar : edge->getOpndVars()) {
                    stmtcount++;
                    NodeID src = opVar->getId();
                    NodeID dst = edge->getResID();
                    delDirectEdgeVec.push_back(new SDK(src, dst, FConstraintEdge::FCopy));
                    insDirectEdgeVec.push_back(new SDK(src, dst, FConstraintEdge::FCopy));
                }
            }
            if (copycount > copyend)
                break;
        }

        SVFStmt::SVFStmtSetTy& calls = pag->getPTASVFStmtSet(SVFStmt::Call);
        step = calls.size() * changep <= 1 ? 1 : (calls.size() * changep);
        SVFUtil::outs() << "call step: "<< step << "\n";
        copycount = 0;
        copybegin = (roundcount - 1) * step + 1;
        copyend = roundcount * step;
        for (SVFStmt::SVFStmtSetTy::iterator iter = calls.begin(), eiter =
            calls.end(); iter != eiter; ++iter)
        {
            copycount++;
            if (copycount < copybegin)
                continue;
            if (copycount >= copybegin && copycount <= copyend) {
                stmtcount++;
                const CallPE* edge = SVFUtil::cast<CallPE>(*iter);
                NodeID src = edge->getRHSVarID();
                NodeID dst = edge->getLHSVarID();
                delDirectEdgeVec.push_back(new SDK(src, dst, FConstraintEdge::FCopy));
                insDirectEdgeVec.push_back(new SDK(src, dst, FConstraintEdge::FCopy));
            }
            if (copycount > copyend)
                break;
        }

        SVFStmt::SVFStmtSetTy& rets = pag->getPTASVFStmtSet(SVFStmt::Ret);
        step = rets.size() * changep <= 1 ? 1 : (rets.size() * changep);
        SVFUtil::outs() << "ret step: "<< step << "\n";
        copycount = 0;
        copybegin = (roundcount - 1) * step + 1;
        copyend = roundcount * step;
        for (SVFStmt::SVFStmtSetTy::iterator iter = rets.begin(), eiter =
            rets.end(); iter != eiter; ++iter)
        {
            copycount++;
            if (copycount < copybegin)
                continue;
            if (copycount >= copybegin && copycount <= copyend) {
                stmtcount++;
                const RetPE* edge = SVFUtil::cast<RetPE>(*iter);
                NodeID src = edge->getRHSVarID();
                NodeID dst = edge->getLHSVarID();
                delDirectEdgeVec.push_back(new SDK(src, dst, FConstraintEdge::FCopy));
                insDirectEdgeVec.push_back(new SDK(src, dst, FConstraintEdge::FCopy));
            }
            if (copycount > copyend)
                break;
        }

        SVFStmt::SVFStmtSetTy& geps = pag->getPTASVFStmtSet(SVFStmt::Gep);
        step = geps.size() * changep <= 1 ? 1 : (geps.size() * changep);
        SVFUtil::outs() << "gep step: "<< step << "\n";
        u32_t gepcount = 0;
        u32_t gepbegin = (roundcount - 1) * step + 1;
        u32_t gepend = roundcount * step;
        for (SVFStmt::SVFStmtSetTy::iterator iter = geps.begin(), eiter =
            geps.end(); iter != eiter; ++iter)
        {
            gepcount++;
            if (gepcount < gepbegin)
                continue;
            if (gepcount >= gepbegin && gepcount <= gepend) {
                GepStmt* edge = SVFUtil::cast<GepStmt>(*iter);
                if(edge->isVariantFieldGep()) {
                    stmtcount++;
                    NodeID src = edge->getRHSVarID();
                    NodeID dst = edge->getLHSVarID();
                    delDirectEdgeVec.push_back(new SDK(src, dst, FConstraintEdge::FVariantGep));
                    insDirectEdgeVec.push_back(new SDK(src, dst, FConstraintEdge::FVariantGep));
                }
            }
            if (gepcount > gepend)
                break;
        }

        SVFStmt::SVFStmtSetTy& stores = pag->getPTASVFStmtSet(SVFStmt::Store);
        step = stores.size() * changep <= 1 ? 1 : (stores.size() * changep);
        SVFUtil::outs() << "store step: "<< step << "\n";
        u32_t storecount = 0;
        u32_t storebegin = (roundcount - 1) * step + 1;
        u32_t storeend = (roundcount) * step;
        for (SVFStmt::SVFStmtSetTy::iterator iter = stores.begin(), eiter =
                stores.end(); iter != eiter; ++iter)
        {
            storecount++;
            if (storecount < storebegin)
                continue;
            if (storecount >= storebegin && storecount <= storeend) {
                stmtcount++;
                StoreStmt* edge = SVFUtil::cast<StoreStmt>(*iter);
                NodeID src = edge->getRHSVarID();
                NodeID dst = edge->getLHSVarID();
                delEdgeVec.push_back(new SDK(src, dst, FConstraintEdge::FStore));
                insEdgeVec.push_back(new SDK(src, dst, FConstraintEdge::FStore));
            }
            if (storecount > storeend)
                break;
        }

        SVFStmt::SVFStmtSetTy& loads = pag->getPTASVFStmtSet(SVFStmt::Load);
        step = loads.size() * changep <= 1 ? 1 : (loads.size() * changep);
        SVFUtil::outs() << "load step: "<< step << "\n";
        u32_t loadcount = 0;
        u32_t loadbegin = (roundcount - 1) * step + 1;
        u32_t loadend = (roundcount) * step;
        for (SVFStmt::SVFStmtSetTy::iterator iter = loads.begin(), eiter =
                loads.end(); iter != eiter; ++iter)
        {
            loadcount++;
            if (loadcount < loadbegin)
                continue;
            if (loadcount >= loadbegin && loadcount <= loadend) {
                stmtcount++;
                LoadStmt* edge = SVFUtil::cast<LoadStmt>(*iter);
                NodeID src = edge->getRHSVarID();
                NodeID dst = edge->getLHSVarID();
                delEdgeVec.push_back(new SDK(src, dst, FConstraintEdge::FLoad));
                insEdgeVec.push_back(new SDK(src, dst, FConstraintEdge::FLoad));
            }
            if (loadcount > loadend)
                break;
        }
        SVFUtil::outs() << "total stmt: "<< stmtcount << "\n";
        stepIncremental();
        
    }

    

}

void AndersenInc::stepIncremental()
{
    SVFUtil::outs() << "========== Incremental Analysis Round [" << incRound << "] ==========\n";
    
    double delStart = stat->getClk();
    // processDeletion(); -- newwl
    processDeletion_EdgeConstraint();
    double delEnd = stat->getClk();
    double delThisRound = (delEnd - delStart) / TIMEINTERVAL;
    timeOfDeletionPTA += delThisRound;
    // computeAllPDM();//
    SVFUtil::outs() << "Time of deletion PTA: " << delThisRound << "\n";
    SVFUtil::outs() << "  - Time of SCC Deletion (so far): " << timeOfDeletionSCC << "\n";
    SVFUtil::outs() << "    -- Time of SCC Build TempG (so far): " << sCG->timeOfBuildTempG << "\n";
    SVFUtil::outs() << "    -- Num of Save TempG (so far): " << sCG->numOfSaveTempG << "\n";
    SVFUtil::outs() << "    -- Time of SCC Find (so far): " << sCG->timeOfSCCFind << "\n";
    SVFUtil::outs() << "    -- Time of SCC Edge Restore (so far): " << sCG->timeOfSCCEdgeRestore << "\n";
    SVFUtil::outs() << "      --- Time of Restore collect edge: " << sCG->timeOfCollectEdge << "\n";
    SVFUtil::outs() << "      --- Time of Restore remove edge: " << sCG->timeOfRemoveEdge << "\n";
    SVFUtil::outs() << "      --- Time of Restore add edge: " << sCG->timeOfAddEdge<< "\n";
    SVFUtil::outs() << "  - Time of Del Pts Prop " << timeOfDeletionProp << "\n";
    SVFUtil::outs() << "------------------------------------------------------------------\n";
    // 
    double insStart = stat->getClk();
    if (Options::Pseudo())
        processInsertion_pseudo();
    else
        processInsertion();
    double insEnd = stat->getClk();

    double insThisRound = (insEnd - insStart) / TIMEINTERVAL;
    timeOfInsertionPTA += insThisRound;
    SVFUtil::outs() << "Time of insertion PTA: " << insThisRound << "\n";
    SVFUtil::outs() << "  - Time of SCC Insertion (so far): " << timeOfInsertionSCC << "\n";
    SVFUtil::outs() << "  - Time of Ins Pts Prop: " << timeOfInsertionProp << "\n";
    SVFUtil::outs() << "------------------------------------------------------------------\n";
    timeOfIncrementalPTA += delThisRound + insThisRound;
    SVFUtil::outs() << "Time of incremental PTA: " << delThisRound + insThisRound << "\n";
    
    
    // normalizePointsTo();
    computeAllPDM();
}

void AndersenInc::cleanConsCG(NodeID id)
{
    sCG->resetSubs(sCG->getRep(id));
    for (NodeID sub: sCG->getSubs(id))
        sCG->resetRep(sub);
    sCG->resetSubs(id);
    sCG->resetRep(id);
    assert(!sCG->hasGNode(id) && "this is either a rep nodeid or a sub nodeid should have already been merged to its field-insensitive base! ");
}

void AndersenInc::normalizePointsTo()
{
    SVFIR::MemObjToFieldsMap &memToFieldsMap = pag->getMemToFieldsMap();
    SVFIR::NodeOffsetMap &GepObjVarMap = pag->getGepObjNodeMap();

    // clear GepObjVarMap/memToFieldsMap/nodeToSubsMap/nodeToRepMap
    // for redundant gepnodes and remove those nodes from pag
    for (NodeID n: redundantGepNodes)
    {
        NodeID base = pag->getBaseObjVar(n);
        GepObjVar *gepNode = SVFUtil::dyn_cast<GepObjVar>(pag->getGNode(n));
        assert(gepNode && "Not a gep node in redundantGepNodes set");
        const APOffset apOffset = gepNode->getConstantFieldIdx();
        GepObjVarMap.erase(std::make_pair(base, apOffset));
        memToFieldsMap[base].reset(n);
        // cleanConsCG(n); // wjy: incremental analysis should keep old consCG

        // pag->removeGNode(gepNode); // wjy: incremental analysis should keep old consCG
    }
}

/*!
 * Process copy and gep edges
 */
void AndersenInc::handleCopyGep(SConstraintNode* node)
{
    NodeID nodeId = node->getId();
    computeDiffPts(nodeId);

    if (!getDiffPts(nodeId).empty())
    {
        for (SConstraintEdge* edge : node->getCopyOutEdges()) 
        {
            NodeID dstId = edge->getDstID();
            if (nodeId != dstId)
                processCopy(nodeId, edge);
        }
        for (SConstraintEdge* edge : node->getGepOutEdges())
        {
            if (GepSCGEdge* gepEdge = SVFUtil::dyn_cast<GepSCGEdge>(edge)) {
                NodeID dstId = gepEdge->getDstID();
                if (nodeId != dstId)
                    processGep(nodeId, gepEdge);
                // TOOD: gep inside scc --wjy
            }
        }
    }
}

/*!
 * Process load and store edges
 */
void AndersenInc::handleLoadStore(SConstraintNode *node)
{
    NodeID nodeId = node->getId();
    for (PointsTo::iterator piter = getPts(nodeId).begin(), epiter =
                getPts(nodeId).end(); piter != epiter; ++piter)
    {
        NodeID ptd = *piter;
        // handle load
        for (SConstraintNode::const_iterator it = node->outgoingLoadsBegin(),
                eit = node->outgoingLoadsEnd(); it != eit; ++it)
        {
            if (processLoad(ptd, *it))
                pushIntoWorklist(ptd);
        }

        // handle store
        for (SConstraintNode::const_iterator it = node->incomingStoresBegin(),
                eit = node->incomingStoresEnd(); it != eit; ++it)
        {
            if (processStore(ptd, *it))
                pushIntoWorklist((*it)->getSrcID());
        }
    }
}

/*!
 * Process address edges
 */
void AndersenInc::processAllAddr()
{
    for (SConstraintGraph::const_iterator nodeIt = sCG->begin(), nodeEit = sCG->end(); nodeIt != nodeEit; nodeIt++)
    {
        SConstraintNode*  cgNode = nodeIt->second;
        for (SConstraintNode::const_iterator it = cgNode->incomingAddrsBegin(), eit = cgNode->incomingAddrsEnd();
                it != eit; ++it)
            processAddr(SVFUtil::cast<AddrSCGEdge>(*it));
    }
}

/*!
 * Process address edges
 */
void AndersenInc::processAddr(const AddrSCGEdge* addr)
{
    numOfProcessedAddr++;

    NodeID dst = addr->getDstID();
    NodeID src = addr->getSrcID();
    if(addPts(dst,src))
        pushIntoWorklist(dst);
}



/*!
 * Process load edges
 *	src --load--> dst,
 *	node \in pts(src) ==>  node--copy-->dst
 */
bool AndersenInc::processLoad(NodeID node, const SConstraintEdge* load)
{
    /// TODO: New copy edges are also added for black hole obj node to
    ///       make gcc in spec 2000 pass the flow-sensitive analysis.
    ///       Try to handle black hole obj in an appropiate way.
//	if (pag->isBlkObjOrConstantObj(node) || isNonPointerObj(node))
    if (pag->isConstantObj(node) || isNonPointerObj(node))
        return false;

    numOfProcessedLoad++;

    // process all flat edges
    bool addnew = false;
    for (auto it = load->getFEdgeSet().begin(), eit = load->getFEdgeSet().end(); it != eit; ++it)
    {
        FConstraintEdge* fLoad = *it;
        NodeID fsrc = node;
        if ( !(fLoad->getCompCache().test(fsrc)) ) {
            NodeID fdst = fLoad->getDstID();
            addnew |= addCopyEdgeByComplexEdge(fsrc, fdst, fLoad);
        }
    }
    
    return addnew;
    // NodeID dst = load->getDstID();
    // return addCopyEdge(node, dst);
}

/*!
 * Process store edges
 *	src --store--> dst,
 *	node \in pts(dst) ==>  src--copy-->node
 */
bool AndersenInc::processStore(NodeID node, const SConstraintEdge* store)
{
    /// TODO: New copy edges are also added for black hole obj node to
    ///       make gcc in spec 2000 pass the flow-sensitive analysis.
    ///       Try to handle black hole obj in an appropiate way
//	if (pag->isBlkObjOrConstantObj(node) || isNonPointerObj(node))
    if (pag->isConstantObj(node) || isNonPointerObj(node))
        return false;

    numOfProcessedStore++;

    bool addnew = false;
    for (auto it = store->getFEdgeSet().begin(), eit = store->getFEdgeSet().end(); it != eit; ++it)
    {
        FConstraintEdge* fStore = *it;
        NodeID fdst = node;
        if ( !(fStore->getCompCache().test(fdst)) ) {
            NodeID fsrc = fStore->getSrcID();
            addnew |= addCopyEdgeByComplexEdge(fsrc, fdst, fStore);
        }
    }
    return addnew;
    // NodeID src = store->getSrcID();
    // return addCopyEdge(src, node);
}

bool AndersenInc::addCopyEdgeByComplexEdge(NodeID fsrc, NodeID fdst, FConstraintEdge* complexEdge)
{
    fCG->addCopyFCGEdge(fsrc, fdst, complexEdge);
    if (sCG->addCopySCGEdge(fsrc, fdst, true))
    {
        updatePropaPts(fsrc, fdst);
        return true;
    }
    return false;
}

/*!
 * Process copy edges
 *	src --copy--> dst,
 *	union pts(dst) with pts(src)
 */
bool AndersenInc::processCopy(NodeID node, const SConstraintEdge* edge)
{
    numOfProcessedCopy++;

    assert((SVFUtil::isa<CopySCGEdge>(edge)) && "not copy/call/ret ??");
    NodeID dst = edge->getDstID();
    const PointsTo& srcPts = getDiffPts(node);

    bool changed = unionPts(dst, srcPts);
    if (changed)
        pushIntoWorklist(dst);
    return changed;
}

/*!
 * Process gep edges
 *	src --gep--> dst,
 *	for each srcPtdNode \in pts(src) ==> add fieldSrcPtdNode into tmpDstPts
 *		union pts(dst) with tmpDstPts
 */
bool AndersenInc::processGep(NodeID, const GepSCGEdge* edge)
{
    const PointsTo& srcPts = getDiffPts(edge->getSrcID());
    return processGepPts(srcPts, edge);
}

/*!
 * Compute points-to for gep edges
 */
bool AndersenInc::processGepPts(const PointsTo& pts, const GepSCGEdge* edge)
{
    numOfProcessedGep++;

    PointsTo tmpDstPts;
    if (SVFUtil::isa<VariantGepSCGEdge>(edge))
    {
        // If a pointer is connected by a variant gep edge,
        // then set this memory object to be field insensitive,
        // unless the object is a black hole/constant.
        for (NodeID o : pts)
        {
            if (sCG->isBlkObjOrConstantObj(o))
            {
                tmpDstPts.set(o);
                continue;
            }

            if (!isFieldInsensitive(o))
            {
                setObjFieldInsensitive(o);
                sCG->addNodeToBeCollapsed(sCG->getBaseObjVar(o));
            }

            // Add the field-insensitive node into pts.
            NodeID baseId = sCG->getFIObjVar(o);
            tmpDstPts.set(baseId);
        }
    }
    else if (const NormalGepSCGEdge* normalGepEdge = SVFUtil::dyn_cast<NormalGepSCGEdge>(edge))
    {
        // TODO: after the node is set to field insensitive, handling invariant
        // gep edge may lose precision because offsets here are ignored, and the
        // base object is always returned.
        for (NodeID o : pts)
        {
            if (sCG->isBlkObjOrConstantObj(o) || isFieldInsensitive(o))
            {
                tmpDstPts.set(o);
                continue;
            }
            // addGepObjVar for fCG firstly, then addGepObjVar for sCG
            fCG->getGepObjVar(o,normalGepEdge->getAccessPath().getConstantFieldIdx());
            NodeID fieldSrcPtdNode = sCG->getGepObjVar(o, normalGepEdge->getAccessPath().getConstantFieldIdx());
            tmpDstPts.set(fieldSrcPtdNode);
        }
    }
    else
    {
        assert(false && "Andersen::processGepPts: New type GEP edge type?");
    }

    NodeID dstId = edge->getDstID();
    if (unionPts(dstId, tmpDstPts))
    {
        pushIntoWorklist(dstId);
        return true;
    }

    return false;
}

/**
 * Detect and collapse PWC nodes produced by processing gep edges, under the constraint of field limit.
 */
inline void AndersenInc::collapsePWCNode(NodeID nodeId)
{
    // If a node is a PWC node, collapse all its points-to tarsget.
    // collapseNodePts() may change the points-to set of the nodes which have been processed
    // before, in this case, we may need to re-do the analysis.
    if (sCG->isPWCNode(nodeId) && collapseNodePts(nodeId))
        reanalyze = true;
}

inline void AndersenInc::collapseFields()
{
    while (sCG->hasNodesToBeCollapsed())
    {
        NodeID node = sCG->getNextCollapseNode();
        // collapseField() may change the points-to set of the nodes which have been processed
        // before, in this case, we may need to re-do the analysis.
        if (collapseField(node))
            reanalyze = true;
    }
}

/*
 * Merge constraint graph nodes based on SCC cycle detected.
 */
void AndersenInc::mergeSccCycle()
{
    NodeStack revTopoOrder;
    NodeStack & topoOrder = getSCCDetector()->topoNodeStack();
    while (!topoOrder.empty())
    {
        NodeID repNodeId = topoOrder.top();
        topoOrder.pop();
        revTopoOrder.push(repNodeId);
        const NodeBS& subNodes = getSCCDetector()->subNodes(repNodeId);
        // merge sub nodes to rep node
        mergeSccNodes(repNodeId, subNodes);
    }

    // restore the topological order for later solving.
    while (!revTopoOrder.empty())
    {
        NodeID nodeId = revTopoOrder.top();
        revTopoOrder.pop();
        topoOrder.push(nodeId);
    }
}

/**
 * Union points-to of subscc nodes into its rep nodes
 * Move incoming/outgoing direct edges of sub node to rep node
 */
void AndersenInc::mergeSccNodes(NodeID repNodeId, const NodeBS& subNodes)
{
    for (NodeBS::iterator nodeIt = subNodes.begin(); nodeIt != subNodes.end(); nodeIt++)
    {
        NodeID subNodeId = *nodeIt;
        if (subNodeId != repNodeId)
        {
            mergeNodeToRep(subNodeId, repNodeId);
        }
    }
}

/**
 * Collapse node's points-to set. Change all points-to elements into field-insensitive.
 */
bool AndersenInc::collapseNodePts(NodeID nodeId)
{
    bool changed = false;
    const PointsTo& nodePts = getPts(nodeId);
    /// Points to set may be changed during collapse, so use a clone instead.
    PointsTo ptsClone = nodePts;
    for (PointsTo::iterator ptsIt = ptsClone.begin(), ptsEit = ptsClone.end(); ptsIt != ptsEit; ptsIt++)
    {
        if (isFieldInsensitive(*ptsIt))
            continue;

        if (collapseField(*ptsIt))
            changed = true;
    }
    return changed;
}

/**
 * Collapse field. make struct with the same base as nodeId become field-insensitive.
 */
bool AndersenInc::collapseField(NodeID nodeId)
{
    /// Black hole doesn't have structures, no collapse is needed.
    /// In later versions, instead of using base node to represent the struct,
    /// we'll create new field-insensitive node. To avoid creating a new "black hole"
    /// node, do not collapse field for black hole node.
    if (sCG->isBlkObjOrConstantObj(nodeId))
        return false;

    bool changed = false;

    // double start = stat->getClk();

    // set base node field-insensitive.
    setObjFieldInsensitive(nodeId);

    // replace all occurrences of each field with the field-insensitive node
    NodeID baseId = sCG->getFIObjVar(nodeId);
    NodeID baseRepNodeId = sCG->sccRepNode(baseId);
    NodeBS & allFields = sCG->getAllFieldsObjVars(baseId);
    for (NodeBS::iterator fieldIt = allFields.begin(), fieldEit = allFields.end(); fieldIt != fieldEit; fieldIt++)
    {
        NodeID fieldId = *fieldIt;
        if (fieldId != baseId)
        {
            // use the reverse pts of this field node to find all pointers point to it
            const NodeSet revPts = getRevPts(fieldId);
            for (const NodeID o : revPts)
            {
                // change the points-to target from field to base node
                clearPts(o, fieldId);
                addPts(o, baseId);
                pushIntoWorklist(o);

                changed = true;
            }
            // merge field node into base node, including edges and pts.
            NodeID fieldRepNodeId = sCG->sccRepNode(fieldId);
            mergeNodeToRep(fieldRepNodeId, baseRepNodeId);
            
            if (fieldId != baseRepNodeId)
            {
                // gep node fieldId becomes redundant if it is merged to its base node who is set as field-insensitive
                // two node IDs should be different otherwise this field is actually the base and should not be removed.
                field2Base[fieldId] = baseRepNodeId;
                redundantGepNodes.set(fieldId);
            }
        }
    }

    if (sCG->isPWCNode(baseRepNodeId))
        if (collapseNodePts(baseRepNodeId))
            changed = true;

    // double end = stat->getClk();
    // timeOfCollapse += (end - start) / TIMEINTERVAL;

    return changed;
}

/*!
 * merge nodeId to newRepId. Return true if the newRepId is a PWC node
 */
bool AndersenInc::mergeSrcToTgt(NodeID nodeId, NodeID newRepId)
{

    if(nodeId==newRepId)
        return false;

    /// union pts of node to rep
    updatePropaPts(newRepId, nodeId);
    unionPts(newRepId,nodeId);

    /// move the edges from node to rep, and remove the node
    SConstraintNode* node = sCG->getSConstraintNode(nodeId);
    bool pwc = sCG->moveEdgesToRepNode(node, sCG->getSConstraintNode(newRepId));

    /// 1. if find gep edges inside SCC cycle, the rep node will become a PWC node and
    /// its pts should be collapsed later.
    /// 2. if the node to be merged is already a PWC node, the rep node will also become
    /// a PWC node as it will have a self-cycle gep edge.
    if(node->isPWCNode())
        pwc = true;

    /// set rep and sub relations
    updateNodeRepAndSubs(node->getId(),newRepId);

    sCG->removeSConstraintNode(node);

    return pwc;
}

/*
 * Merge a node to its rep node based on SCC detection
 */
void AndersenInc::mergeNodeToRep(NodeID nodeId,NodeID newRepId)
{

    if (mergeSrcToTgt(nodeId,newRepId))
        sCG->setPWCNode(newRepId);
}

/*
 * Updates subnodes of its rep, and rep node of its subs
 */
void AndersenInc::updateNodeRepAndSubs(NodeID nodeId, NodeID newRepId)
{
    sCG->setRep(nodeId,newRepId);
    NodeBS repSubs;
    repSubs.set(nodeId);
    /// update nodeToRepMap, for each subs of current node updates its rep to newRepId
    //  update nodeToSubsMap, union its subs with its rep Subs
    NodeBS& nodeSubs = sCG->sccSubNodes(nodeId);
    for(NodeBS::iterator sit = nodeSubs.begin(), esit = nodeSubs.end(); sit!=esit; ++sit)
    {
        NodeID subId = *sit;
        sCG->setRep(subId,newRepId);
    }
    repSubs |= nodeSubs;
    sCG->setSubs(newRepId,repSubs);
    sCG->resetSubs(nodeId);
}

/*!
 * SCC detection on constraint graph
 */
NodeStack& AndersenInc::SCCDetect()
{
    numOfSCCDetection++;

    double sccStart = stat->getClk();
    WPASConstraintSolver::SCCDetect();
    double sccEnd = stat->getClk();

    timeOfSCCDetection +=  (sccEnd - sccStart)/TIMEINTERVAL;

    double mergeStart = stat->getClk();

    mergeSccCycle();

    double mergeEnd = stat->getClk();

    timeOfSCCMerges +=  (mergeEnd - mergeStart)/TIMEINTERVAL;

    return getSCCDetector()->topoNodeStack();
}

/*!
 * Update call graph for the input indirect callsites
 */
bool AndersenInc::updateCallGraph(const CallSiteToFunPtrMap& callsites)
{

    double cgUpdateStart = stat->getClk();

    CallEdgeMap newEdges;
    onTheFlyCallGraphSolve(callsites,newEdges);
    NodePairSet cpySrcNodes;	/// nodes as a src of a generated new copy edge
    for(CallEdgeMap::iterator it = newEdges.begin(), eit = newEdges.end(); it!=eit; ++it )
    {
        CallSite cs = SVFUtil::getSVFCallSite(it->first->getCallSite());
        for(FunctionSet::iterator cit = it->second.begin(), ecit = it->second.end(); cit!=ecit; ++cit)
        {
            connectCaller2CalleeParams(cs,*cit,cpySrcNodes);
        }
    }
    for(NodePairSet::iterator it = cpySrcNodes.begin(), eit = cpySrcNodes.end(); it!=eit; ++it)
    {
        pushIntoWorklist(it->first);
    }

    double cgUpdateEnd = stat->getClk();
    timeOfUpdateCallGraph += (cgUpdateEnd - cgUpdateStart) / TIMEINTERVAL;

    return (!newEdges.empty());
}

void AndersenInc::heapAllocatorViaIndCall(CallSite cs, NodePairSet &cpySrcNodes)
{
    assert(SVFUtil::getCallee(cs) == nullptr && "not an indirect callsite?");
    RetICFGNode* retBlockNode = pag->getICFG()->getRetICFGNode(cs.getInstruction());
    const PAGNode* cs_return = pag->getCallSiteRet(retBlockNode);
    NodeID srcret;
    CallSite2DummyValPN::const_iterator it = callsite2DummyValPN.find(cs);
    if(it != callsite2DummyValPN.end())
    {
        // srcret = sccRepNode(it->second);
        srcret = it->second;
    }
    else
    {
        NodeID valNode = pag->addDummyValNode();
        NodeID objNode = pag->addDummyObjNode(cs.getType());
        addPts(valNode,objNode);
        callsite2DummyValPN.insert(std::make_pair(cs,valNode));
        fCG->addFConstraintNode(new FConstraintNode(valNode),valNode);
        fCG->addFConstraintNode(new FConstraintNode(objNode),objNode);
        sCG->addSConstraintNode(new SConstraintNode(valNode),valNode);
        sCG->addSConstraintNode(new SConstraintNode(objNode),objNode);
        srcret = valNode;
    }

    // NodeID dstrec = sccRepNode(cs_return->getId());
    NodeID dstrec = cs_return->getId();

    if(addCopyEdge(srcret, dstrec)) /// add copy sEdge with the original id. --wjy
        cpySrcNodes.insert(std::make_pair(srcret,dstrec));
}

/*!
 * Connect formal and actual parameters for indirect callsites
 */
void AndersenInc::connectCaller2CalleeParams(CallSite cs, const SVFFunction* F, NodePairSet &cpySrcNodes)
{
    assert(F);

    DBOUT(DAndersen, outs() << "connect parameters from indirect callsite " << cs.getInstruction()->toString() << " to callee " << *F << "\n");

    CallICFGNode* callBlockNode = pag->getICFG()->getCallICFGNode(cs.getInstruction());
    RetICFGNode* retBlockNode = pag->getICFG()->getRetICFGNode(cs.getInstruction());

    if(SVFUtil::isHeapAllocExtFunViaRet(F) && pag->callsiteHasRet(retBlockNode))
    {
        heapAllocatorViaIndCall(cs,cpySrcNodes);
    }

    if (pag->funHasRet(F) && pag->callsiteHasRet(retBlockNode))
    {
        const PAGNode* cs_return = pag->getCallSiteRet(retBlockNode);
        const PAGNode* fun_return = pag->getFunRet(F);
        if (cs_return->isPointer() && fun_return->isPointer())
        {
            // NodeID dstrec = sccRepNode(cs_return->getId());
            // NodeID srcret = sccRepNode(fun_return->getId());
            NodeID dstrec = cs_return->getId();
            NodeID srcret = fun_return->getId();
            if(addCopyEdge(srcret, dstrec)) /// add copy sEdge with the original id. --wjy
            {
                cpySrcNodes.insert(std::make_pair(srcret,dstrec));
            }
        }
        else
        {
            DBOUT(DAndersen, outs() << "not a pointer ignored\n");
        }
    }

    if (pag->hasCallSiteArgsMap(callBlockNode) && pag->hasFunArgsList(F))
    {

        // connect actual and formal param
        const SVFIR::SVFVarList& csArgList = pag->getCallSiteArgsList(callBlockNode);
        const SVFIR::SVFVarList& funArgList = pag->getFunArgsList(F);
        //Go through the fixed parameters.
        DBOUT(DPAGBuild, outs() << "      args:");
        SVFIR::SVFVarList::const_iterator funArgIt = funArgList.begin(), funArgEit = funArgList.end();
        SVFIR::SVFVarList::const_iterator csArgIt  = csArgList.begin(), csArgEit = csArgList.end();
        for (; funArgIt != funArgEit; ++csArgIt, ++funArgIt)
        {
            //Some programs (e.g. Linux kernel) leave unneeded parameters empty.
            if (csArgIt  == csArgEit)
            {
                DBOUT(DAndersen, outs() << " !! not enough args\n");
                break;
            }
            const PAGNode *cs_arg = *csArgIt ;
            const PAGNode *fun_arg = *funArgIt;

            if (cs_arg->isPointer() && fun_arg->isPointer())
            {
                DBOUT(DAndersen, outs() << "process actual parm  " << cs_arg->toString() << " \n");
                // NodeID srcAA = sccRepNode(cs_arg->getId());
                // NodeID dstFA = sccRepNode(fun_arg->getId());
                NodeID srcAA = cs_arg->getId();
                NodeID dstFA = fun_arg->getId();
                if(addCopyEdge(srcAA, dstFA)) /// add copy sEdge with the original id. --wjy
                {
                    cpySrcNodes.insert(std::make_pair(srcAA,dstFA));
                }
            }
        }

        //Any remaining actual args must be varargs.
        if (F->isVarArg())
        {
            // NodeID vaF = sccRepNode(pag->getVarargNode(F));
            NodeID vaF = pag->getVarargNode(F);
            DBOUT(DPAGBuild, outs() << "\n      varargs:");
            for (; csArgIt != csArgEit; ++csArgIt)
            {
                const PAGNode *cs_arg = *csArgIt;
                if (cs_arg->isPointer())
                {
                    // NodeID vnAA = sccRepNode(cs_arg->getId());
                    NodeID vnAA = cs_arg->getId();
                    if (addCopyEdge(vnAA,vaF)) /// add copy sEdge with the original id. --wjy
                    {
                        cpySrcNodes.insert(std::make_pair(vnAA,vaF));
                    }
                }
            }
        }
        if(csArgIt != csArgEit)
        {
            writeWrnMsg("too many args to non-vararg func.");
            writeWrnMsg("(" + cs.getInstruction()->getSourceLoc() + ")");
        }
    }
}


/*!
 * Print pag nodes' pts by an ascending order
 */
void AndersenInc::dumpTopLevelPtsTo()
{
    for (OrderedNodeSet::iterator nIter = this->getAllValidPtrs().begin();
            nIter != this->getAllValidPtrs().end(); ++nIter)
    {
        const PAGNode* node = getPAG()->getGNode(*nIter);
        if (getPAG()->isValidTopLevelPtr(node))
        {
            const PointsTo& pts = this->getPts(node->getId());
            outs() << "\nNodeID " << node->getId() << " ";

            if (pts.empty())
            {
                outs() << "\t\tPointsTo: {empty}\n\n";
            }
            else
            {
                outs() << "\t\tPointsTo: { ";

                multiset<u32_t> line;
                for (PointsTo::iterator it = pts.begin(), eit = pts.end();
                        it != eit; ++it)
                {
                    line.insert(*it);
                }
                for (multiset<u32_t>::const_iterator it = line.begin(); it != line.end(); ++it)
                    outs() << *it << " ";
                outs() << "}\n\n";
            }
        }
    }

    outs().flush();
}



/*!
 * solve worklist
 */
void AndersenInc::solveWorklist()
{
    // Initialize the nodeStack via a whole SCC detection
    // Nodes in nodeStack are in topological order by default.
    NodeStack& nodeStack = SCCDetect();

    // Process nodeStack and put the changed nodes into workList.
    while (!nodeStack.empty())
    {
        NodeID nodeId = nodeStack.top();
        nodeStack.pop();
        double collapsePWCStart = stat->getClk();
        collapsePWCNode(nodeId);
        double collapsePWCEnd = stat->getClk();
        timeOfCollapsePWC += (collapsePWCEnd - collapsePWCStart) / TIMEINTERVAL;
        // process nodes in nodeStack
        processNode(nodeId);

        double collapseFieldsStart = stat->getClk();
        collapseFields();
        double collapseFieldsEnd = stat->getClk();
        timeOfCollapse += (collapseFieldsEnd - collapseFieldsStart) / TIMEINTERVAL;
    }

    // New nodes will be inserted into workList during processing.
    while (!isWorklistEmpty())
    {
        NodeID nodeId = popFromWorklist();
        // process nodes in worklist
        postProcessNode(nodeId);
    }
}

/*!
 * Process edge PAGNode
 */
void AndersenInc::processNode(NodeID nodeId)
{
    // This node may be merged during collapseNodePts() which means it is no longer a rep node
    // in the graph. Only rep node needs to be handled.
    if (sccRepNode(nodeId) != nodeId)
        return;

    double propStart = stat->getClk();
    SConstraintNode* node = sCG->getSConstraintNode(nodeId);
    handleCopyGep(node);
    double propEnd = stat->getClk();
    timeOfProcessCopyGep += (propEnd - propStart) / TIMEINTERVAL;
}

/*!
 * Post process node
 */
void AndersenInc::postProcessNode(NodeID nodeId)
{
    double insertStart = stat->getClk();

    SConstraintNode* node = sCG->getSConstraintNode(nodeId);

    // handle load
    for (SConstraintNode::const_iterator it = node->outgoingLoadsBegin(), eit = node->outgoingLoadsEnd();
            it != eit; ++it)
    {
        if (handleLoad(nodeId, *it))
            reanalyze = true;
    }
    // handle store
    for (SConstraintNode::const_iterator it = node->incomingStoresBegin(), eit =  node->incomingStoresEnd();
            it != eit; ++it)
    {
        if (handleStore(nodeId, *it))
            reanalyze = true;
    }

    double insertEnd = stat->getClk();
    timeOfProcessLoadStore += (insertEnd - insertStart) / TIMEINTERVAL;
}

/*!
 * Handle load
 */
bool AndersenInc::handleLoad(NodeID nodeId, const SConstraintEdge* edge)
{
    bool changed = false;
    for (PointsTo::iterator piter = getPts(nodeId).begin(), epiter = getPts(nodeId).end();
            piter != epiter; ++piter)
    {
        if (processLoad(*piter, edge))
        {
            changed = true;
        }
    }
    return changed;
}

/*!
 * Handle store
 */
bool AndersenInc::handleStore(NodeID nodeId, const SConstraintEdge* edge)
{
    bool changed = false;
    for (PointsTo::iterator piter = getPts(nodeId).begin(), epiter = getPts(nodeId).end();
            piter != epiter; ++piter)
    {
        if (processStore(*piter, edge))
        {
            changed = true;
        }
    }
    return changed;
}


bool AndersenInc::pushIntoDelEdgesWL(NodeID src, NodeID dst, FConstraintEdge::FConstraintEdgeK kind)
{
    FConstraintNode* srcNode = fCG->getFConstraintNode(src);
    FConstraintNode* dstNode = fCG->getFConstraintNode(dst);
    if (fCG->hasEdge(srcNode, dstNode, kind)) {
        FConstraintEdge* edge = fCG->getEdge(srcNode, dstNode, kind);
        delEdgesWL.push(edge);
        return true;
    }
    return false;
}

bool AndersenInc::pushIntoInsEdgesWL(NodeID src, NodeID dst, FConstraintEdge::FConstraintEdgeK kind)
{
    FConstraintNode* srcNode = fCG->getFConstraintNode(src);
    FConstraintNode* dstNode = fCG->getFConstraintNode(dst);
    if (fCG->hasEdge(srcNode, dstNode, kind)) {
        FConstraintEdge* edge = fCG->getEdge(srcNode, dstNode, kind);
        insEdgesWL.push(edge);
        return true;
    }
    return false;
}

/*
 * srcid --Load--> dstid
 * for o in pts(srcid):
 *     o --Copy--> dstid
 */
void AndersenInc::processLoadRemoval(NodeID srcid, NodeID dstid)
{
    SConstraintNode* sSrcNode = sCG->getSConstraintNode(srcid);
    SConstraintNode* sDstNode = sCG->getSConstraintNode(dstid);
    SConstraintEdge* sLoad = sCG->getEdge(sSrcNode, sDstNode, SConstraintEdge::SLoad);

    FConstraintNode* fSrcNode = fCG->getFConstraintNode(srcid);
    FConstraintNode* fDstNode = fCG->getFConstraintNode(dstid);
    FConstraintEdge* fLoad = fCG->getEdge(fSrcNode, fDstNode, FConstraintEdge::FLoad);

    // 1. Process copy edge with this complex load constraint.
    const PointsTo& srcPts = getPts(srcid);
    for (NodeID o: srcPts) {
        if (pag->isConstantObj(o) || isNonPointerObj(o))
            continue;
        FConstraintNode* fONode = fCG->getFConstraintNode(o);
        if (!fCG->hasEdge(fONode, fDstNode, FConstraintEdge::FCopy))
            continue;
        FConstraintEdge* fEdge = fCG->getEdge(fONode, fDstNode, FConstraintEdge::FCopy);
        CopyFCGEdge* fCopy = SVFUtil::dyn_cast<CopyFCGEdge>(fEdge);
        fCopy->removeComplexEdge(fLoad);
        if (fCopy->getComplexEdgeSet().empty()) {
            // fCopy need to be removed
            // pushIntoDelEdgesWL(o, dstid, FConstraintEdge::FCopy); -- newwl
            delDirectEdgeVec.push_back(new SDK(o, dstid, FConstraintEdge::FCopy));
        }
    }

    // 2. Process fedge set of the sedge which this fedge retargeted to.
    sLoad->removeFEdge(fLoad);
    if (sLoad->getFEdgeSet().empty()) {
        sCG->removeLoadEdge(SVFUtil::dyn_cast<LoadSCGEdge>(sLoad));
    }

    // 3. process fedges removal
    fCG->removeLoadEdge(SVFUtil::dyn_cast<LoadFCGEdge>(fLoad));
}

/*
 * srcid --Store--> dstid
 * for o in pts(dstid):
 *     srcid --Copy--> o
 */
void AndersenInc::processStoreRemoval(NodeID srcid, NodeID dstid)
{
    SConstraintNode* sSrcNode = sCG->getSConstraintNode(srcid);
    SConstraintNode* sDstNode = sCG->getSConstraintNode(dstid);
    SConstraintEdge* sStore = sCG->getEdge(sSrcNode, sDstNode, SConstraintEdge::SStore);

    FConstraintNode* fSrcNode = fCG->getFConstraintNode(srcid);
    FConstraintNode* fDstNode = fCG->getFConstraintNode(dstid);
    FConstraintEdge* fStore = fCG->getFEdgeOrNullptr(fSrcNode, fDstNode, FConstraintEdge::FStore);
    if (fStore == nullptr)
        return;
    const PointsTo& dstPts = getPts(dstid);

    for (NodeID o: dstPts) {
        if (pag->isConstantObj(o) || isNonPointerObj(o))
            continue;
        FConstraintNode* fONode = fCG->getFConstraintNode(o);
        if (!fCG->hasEdge(fSrcNode, fONode, FConstraintEdge::FCopy))
            continue;
        FConstraintEdge* fEdge = fCG->getEdge(fSrcNode, fONode, FConstraintEdge::FCopy);
        CopyFCGEdge* fCopy = SVFUtil::dyn_cast<CopyFCGEdge>(fEdge);
        fCopy->removeComplexEdge(fStore);
        if (fCopy->getComplexEdgeSet().empty()) {
            // fCopy need to be removed
            // pushIntoDelEdgesWL(srcid, o, FConstraintEdge::FCopy); -- newwl
            delDirectEdgeVec.push_back(new SDK(srcid, o, FConstraintEdge::FCopy));
        }
    }

    sStore->removeFEdge(fStore);
    if (sStore->getFEdgeSet().empty()) {
        sCG->removeStoreEdge(SVFUtil::dyn_cast<StoreSCGEdge>(sStore));
    }

    fCG->removeStoreEdge(SVFUtil::dyn_cast<StoreFCGEdge>(fStore));
}

/*
 * s --Addr--> d
 * pts(d) = pts(d) \Union {s}
 */
void AndersenInc::processAddrRemoval(NodeID srcid, NodeID dstid)
{
    SConstraintNode* sSrcNode = sCG->getSConstraintNode(srcid);
    SConstraintNode* sDstNode = sCG->getSConstraintNode(dstid);
    SConstraintEdge* sAddr = sCG->getEdge(sSrcNode, sDstNode, SConstraintEdge::SAddr);

    FConstraintNode* fSrcNode = fCG->getFConstraintNode(srcid);
    FConstraintNode* fDstNode = fCG->getFConstraintNode(dstid);
    FConstraintEdge* fAddr = fCG->getEdge(fSrcNode, fDstNode, FConstraintEdge::FAddr);

    sAddr->removeFEdge(fAddr);
    if (sAddr->getFEdgeSet().empty()) {
        sCG->removeAddrEdge(SVFUtil::dyn_cast<AddrSCGEdge>(sAddr));
    }
    
    fCG->removeAddrEdge(SVFUtil::dyn_cast<AddrFCGEdge>(fAddr));

    // srcid is the object node; its address edge was removed, so it is a
    // candidate for dead-object cleanup at the end of deletion.
    deadObjectCandidates.set(srcid);

    PointsTo srcSet;
    srcSet.set(srcid);
    STAT_TIME(timeOfDeletionProp, propagateDelPts(srcSet, dstid));
}
/*
 * s --Addr--> d
 * pts(d) = pts(d) \Union {s}
 */
void AndersenInc::processAddrRemoval_IPA(NodeID srcid, NodeID dstid)
{
    SConstraintNode* sSrcNode = sCG->getSConstraintNode(srcid);
    SConstraintNode* sDstNode = sCG->getSConstraintNode(dstid);
    SConstraintEdge* sAddr = sCG->getEdge(sSrcNode, sDstNode, SConstraintEdge::SAddr);

    FConstraintNode* fSrcNode = fCG->getFConstraintNode(srcid);
    FConstraintNode* fDstNode = fCG->getFConstraintNode(dstid);
    FConstraintEdge* fAddr = fCG->getEdge(fSrcNode, fDstNode, FConstraintEdge::FAddr);

    sAddr->removeFEdge(fAddr);
    if (sAddr->getFEdgeSet().empty()) {
        sCG->removeAddrEdge(SVFUtil::dyn_cast<AddrSCGEdge>(sAddr));
    }
    
    fCG->removeAddrEdge(SVFUtil::dyn_cast<AddrFCGEdge>(fAddr));

    deadObjectCandidates.set(srcid);

    PointsTo srcSet;
    srcSet.set(srcid);
    NodeBS proping;
    STAT_TIME(timeOfDeletionProp, propagateDelPts_IPA(srcSet, dstid, proping));
}
/*
 * s --Addr--> d
 * pts(d) = pts(d) \Union {s}
 */
void AndersenInc::processAddrRemoval_Lazy(NodeID srcid, NodeID dstid)
{
    SConstraintNode* sSrcNode = sCG->getSConstraintNode(srcid);
    SConstraintNode* sDstNode = sCG->getSConstraintNode(dstid);
    SConstraintEdge* sAddr = sCG->getEdge(sSrcNode, sDstNode, SConstraintEdge::SAddr);

    FConstraintNode* fSrcNode = fCG->getFConstraintNode(srcid);
    FConstraintNode* fDstNode = fCG->getFConstraintNode(dstid);
    FConstraintEdge* fAddr = fCG->getEdge(fSrcNode, fDstNode, FConstraintEdge::FAddr);

    sAddr->removeFEdge(fAddr);
    if (sAddr->getFEdgeSet().empty()) {
        sCG->removeAddrEdge(SVFUtil::dyn_cast<AddrSCGEdge>(sAddr));
    }
    
    fCG->removeAddrEdge(SVFUtil::dyn_cast<AddrFCGEdge>(fAddr));

    deadObjectCandidates.set(srcid);

    PointsTo srcSet;
    srcSet.set(srcid);
    delPropMap[dstid] |= srcSet; // collect changes
}


void AndersenInc::processSCCRedetection()
{
//     double sccStart = stat->getClk();
//     if (!redetectReps.empty()) {
//         numOfRedetectSCC ++;
        
//         double deleteStart = stat->getClk();
//         delete sCG;
//         double deleteEnd = stat->getClk();
//         timeOfSCGDelete += (deleteEnd - deleteStart) / TIMEINTERVAL;

//         double rebuildStart = stat->getClk();
//         sCG = new SConstraintGraph(pag, fCG, true);
//         double rebuildEnd = stat->getClk();
//         timeOfSCGRebuild += (rebuildEnd - rebuildStart) / TIMEINTERVAL;

//         setGraph(sCG);
//         SCCDetect();

//         while(!redetectReps.empty()) {
//             NodeID oldRep = redetectReps.pop();
//             for (SDK* sdk: rep2EdgeSet[oldRep]) {
//                 if (sCG->getSConstraintNode(sdk->src) == sCG->getSConstraintNode(sdk->dst)) {
//                     delete sdk;
//                     continue;
//                 }
//                 delEdgeVec.push_back(sdk);
//             }
//             rep2EdgeSet[oldRep].clear();
//         }
//     }
//     double sccEnd = stat->getClk();
//     timeOfDeletionSCC += (sccEnd - sccStart) / TIMEINTERVAL;
    while (!redetectReps.empty())
    {
        NodeID oldRep = redetectReps.pop();
        NodeBS newReps;
        const PointsTo& oldRepPts = getPts(oldRep);
        double sccStart = stat->getClk();
        unsigned sccKeep = sCG->sccBreakDetect(oldRep, newReps, stat);
        double sccEnd = stat->getClk();
        timeOfDeletionSCC += (sccEnd - sccStart) / TIMEINTERVAL;
        timeOfSCCDetectionDel += (sccEnd - sccStart) / TIMEINTERVAL;
        numOfSCCDetectionDel ++;
        if (1 == sccKeep) {
            for (SDK* sdk: rep2EdgeSet[oldRep])
                delete sdk;
            rep2EdgeSet[oldRep].clear();
            continue;
        }
        else {
            // SCC Restore
            SCCBreak = true;
            for (NodeID id: newReps)
                unionPts(id, oldRepPts);
            for (SDK* sdk: rep2EdgeSet[oldRep]) {
                if (sCG->getSConstraintNode(sdk->src) == sCG->getSConstraintNode(sdk->dst)) {
                    delete sdk;
                    continue;
                }
                // delDirectEdgeVec.push_back(sdk);
                // deldiEdgeVec.push_back(sdk);
                NodeID src = sdk->src;
                NodeID dst = sdk->dst;
                FConstraintEdge::FConstraintEdgeK kind = sdk->kind;
                if (kind == FConstraintEdge::FCopy)
                    // needSCCDetect |= 
                    processCopyConstraintRemoval_Lazy(src, dst);
                else if (kind == FConstraintEdge::FVariantGep) {
                    processVariantGepConstraintRemoval_Lazy(src, dst);
                }
                else if (kind == FConstraintEdge::FNormalGep) {
                    const AccessPath& ap = sdk->ap;
                    processNormalGepConstraintRemoval_Lazy(src, dst, ap);
                }
                delete sdk;
            }
            rep2EdgeSet[oldRep].clear();
        }
    }
}

bool AndersenInc::processSCCRedetection_IPA(NodeID rep)
{
    NodeID oldRep = rep;
    NodeBS newReps;
    const PointsTo& oldRepPts = getPts(oldRep);
    double sccStart = stat->getClk();
    unsigned sccKeep = sCG->sccBreakDetect(oldRep, newReps, stat);
    double sccEnd = stat->getClk();
    timeOfSCCDetectionIPADel += (sccEnd - sccStart) / TIMEINTERVAL;
    numOfSCCDetectionIPADel++;
    if (1 == sccKeep) {
        return false;
    }
    else {
        // SCC Restore
        SCCBreak = true;
        for (NodeID id: newReps)
            unionPts(id, oldRepPts);
        return true;
        // for (SDK* sdk: rep2EdgeSet[oldRep]) {
        //     if (sCG->getSConstraintNode(sdk->src) == sCG->getSConstraintNode(sdk->dst)) {
        //         delete sdk;
        //         continue;
        //     }
        //     delEdgeVec.push_back(sdk);
        // }
        // rep2EdgeSet[oldRep].clear();
    }
    // }
}


void AndersenInc::processCopyEdgeRemoval(NodeID srcid, NodeID dstid)
{
    FConstraintNode* fSrcNode = fCG->getFConstraintNode(srcid);
    FConstraintNode* fDstNode = fCG->getFConstraintNode(dstid);
    FConstraintEdge* fCopy = fCG->getFEdgeOrNullptr(fSrcNode, fDstNode, FConstraintEdge::FCopy);
    if (fCopy == nullptr)
        return;
    CopyFCGEdge* copyFCGEdge = SVFUtil::dyn_cast<CopyFCGEdge>(fCopy);
    if (! copyFCGEdge->getComplexEdgeSet().empty()) {
        // Copy not by complex constraint
        // Process the complexEdgeSet here
        // Copy by complex constraint
        // The complexEdgeSet has already been processed when handle load/store

        // not empty, original copy removal (remove self complex constraint)
        copyFCGEdge->removeComplexEdge(fCopy);
        if (! copyFCGEdge->getComplexEdgeSet().empty())
            return; // do nothing if the complex set is still not empty
    }
    // The complex set is empty, then the copyEdge should be removed.
    SConstraintNode* sSrcNode = sCG->getSConstraintNode(srcid);
    SConstraintNode* sDstNode = sCG->getSConstraintNode(dstid);
    SConstraintEdge* sCopy = sCG->getEdge(sSrcNode, sDstNode, SConstraintEdge::SCopy);
    sCopy->removeFEdge(fCopy);
    fCG->removeDirectEdge(fCopy);
    if (sSrcNode == sDstNode) {
        // Copy Edge in SCC
        NodeID repId = sSrcNode->getId();
        redetectReps.push(repId);
        rep2EdgeSet[repId].insert(new SDK(srcid, dstid, FConstraintEdge::FCopy));
        sCG->removeTempGEdge(srcid , dstid, FConstraintEdge::FCopy);
        return;
    }
    else {
        // Copy Edge not in SCC
        if (sCopy->getFEdgeSet().empty()) {
            sCG->removeDirectEdge(sCopy);
            delEdgeVec.push_back(new SDK(srcid, dstid, FConstraintEdge::FCopy));
            return;
        }
    }
}
void AndersenInc::processCopyEdgeRemoval_Lazy(NodeID srcid, NodeID dstid)
{
    FConstraintNode* fSrcNode = fCG->getFConstraintNode(srcid);
    FConstraintNode* fDstNode = fCG->getFConstraintNode(dstid);
    FConstraintEdge* fCopy = fCG->getFEdgeOrNullptr(fSrcNode, fDstNode, FConstraintEdge::FCopy);
    if (fCopy == nullptr)
        return;
    CopyFCGEdge* copyFCGEdge = SVFUtil::dyn_cast<CopyFCGEdge>(fCopy);
    if (! copyFCGEdge->getComplexEdgeSet().empty()) {
        // Copy not by complex constraint
        // Process the complexEdgeSet here
        // Copy by complex constraint
        // The complexEdgeSet has already been processed when handle load/store

        // not empty, original copy removal (remove self complex constraint)
        copyFCGEdge->removeComplexEdge(fCopy);
        if (! copyFCGEdge->getComplexEdgeSet().empty())
            return; // do nothing if the complex set is still not empty
    }
    // The complex set is empty, then the copyEdge should be removed.
    SConstraintNode* sSrcNode = sCG->getSConstraintNode(srcid);
    SConstraintNode* sDstNode = sCG->getSConstraintNode(dstid);
    SConstraintEdge* sCopy = sCG->getEdge(sSrcNode, sDstNode, SConstraintEdge::SCopy);
    sCopy->removeFEdge(fCopy);
    fCG->removeDirectEdge(fCopy);
    if (sSrcNode == sDstNode) {
        // Copy Edge in SCC
        NodeID repId = sSrcNode->getId();
        redetectReps.push(repId);
        rep2EdgeSet[repId].insert(new SDK(srcid, dstid, FConstraintEdge::FCopy));
        sCG->removeTempGEdge(srcid , dstid, FConstraintEdge::FCopy);
        return;
    }
    else {
        // Copy Edge not in SCC
        if (sCopy->getFEdgeSet().empty()) {
            sCG->removeDirectEdge(sCopy);
            // delEdgeVec.push_back(new SDK(srcid, dstid, FConstraintEdge::FCopy));
            // return;
            processCopyConstraintRemoval_Lazy(srcid, dstid);
        }
    }
}
void AndersenInc::processCopyEdgeRemoval_IPA(NodeID srcid, NodeID dstid)
{
    FConstraintNode* fSrcNode = fCG->getFConstraintNode(srcid);
    FConstraintNode* fDstNode = fCG->getFConstraintNode(dstid);
    FConstraintEdge* fCopy = fCG->getFEdgeOrNullptr(fSrcNode, fDstNode, FConstraintEdge::FCopy);
    if (fCopy == nullptr)
        return;
    CopyFCGEdge* copyFCGEdge = SVFUtil::dyn_cast<CopyFCGEdge>(fCopy);
    if (! copyFCGEdge->getComplexEdgeSet().empty()) {
        // Copy not by complex constraint
        // Process the complexEdgeSet here
        // Copy by complex constraint
        // The complexEdgeSet has already been processed when handle load/store

        // not empty, original copy removal (remove self complex constraint)
        copyFCGEdge->removeComplexEdge(fCopy);
        if (! copyFCGEdge->getComplexEdgeSet().empty())
            return; // do nothing if the complex set is still not empty
    }
    // The complex set is empty, then the copyEdge should be removed.
    SConstraintNode* sSrcNode = sCG->getSConstraintNode(srcid);
    SConstraintNode* sDstNode = sCG->getSConstraintNode(dstid);
    SConstraintEdge* sCopy = sCG->getEdge(sSrcNode, sDstNode, SConstraintEdge::SCopy);
    sCopy->removeFEdge(fCopy);
    fCG->removeDirectEdge(fCopy);
    if (sSrcNode == sDstNode) {
        // Copy Edge in SCC
        NodeID repId = sSrcNode->getId();
        bool prop = processSCCRedetection_IPA(repId);
        if (prop) {
            processCopyConstraintRemoval(srcid, dstid);
        }
        return;
    }
    else {
        // Copy Edge not in SCC
        if (sCopy->getFEdgeSet().empty()) {
            sCG->removeDirectEdge(sCopy);
            // IPA propagate changes eagerly
            processCopyConstraintRemoval(srcid, dstid);
            return;
        }
    }
}

void AndersenInc::processCopyConstraintRemoval(NodeID srcid, NodeID dstid)
{
    NodeBS proping;
    STAT_TIME(timeOfDeletionProp, propagateDelPts_IPA(getPts(srcid), dstid, proping));
}
void AndersenInc::processCopyConstraintRemoval_Lazy(NodeID srcid, NodeID dstid)
{
    dstid = sccRepNode(dstid);
    delPropMap[dstid] |= getPts(srcid);
}
/*
 * s --Copy--> d
 * pts(d) = pts(d) \Union pts(s)
 */
void AndersenInc::processCopyRemoval(NodeID srcid, NodeID dstid)
{
    // process original copy first
    FConstraintNode* fSrcNode = fCG->getFConstraintNode(srcid);
    FConstraintNode* fDstNode = fCG->getFConstraintNode(dstid);
    FConstraintEdge* fCopy = fCG->getEdge(fSrcNode, fDstNode, FConstraintEdge::FCopy);
    CopyFCGEdge* copyFCGEdge = SVFUtil::dyn_cast<CopyFCGEdge>(fCopy);
    if (! copyFCGEdge->getComplexEdgeSet().empty()) {
        // not empty, original copy removal (remove self complex constraint)
        copyFCGEdge->removeComplexEdge(fCopy);
        if (! copyFCGEdge->getComplexEdgeSet().empty())
            return; // do nothing if the complex set is still not empty
    }

    SConstraintNode* sSrcNode = sCG->getSConstraintNode(srcid);
    SConstraintNode* sDstNode = sCG->getSConstraintNode(dstid);
    if (sSrcNode == sDstNode) {
        // Copy Edge in SCC
        NodeBS newReps;
        NodeID oldRep = sSrcNode->getId();
        const PointsTo& oldRepPts = getPts(oldRep); // save old rep pts before scc detect
        // dumpPts(oldRepPts);
        double sccStart = stat->getClk();
        unsigned sccKeep = sCG->sccBreakDetect(srcid, dstid, FConstraintEdge::FCopy, newReps, oldRep);
        double sccEnd = stat->getClk();
        timeOfDeletionSCC += (sccEnd - sccStart) / TIMEINTERVAL;
        if (1 == sccKeep) {
            // SCC KEEP
            // SCC KEEP should remove the fEdge
            return;
        }
        // SCC Restore
        // SCC Restore should remove the fEdge and sEdge
        // reset Pts for new rep nodes
        // dumpPts(getPts(oldRep));
        for (NodeID id: newReps) {
            unionPts(id, oldRepPts);
        }
        STAT_TIME(timeOfDeletionProp, propagateDelPts(getPts(srcid), dstid));
    }
    else {
        if (sCG->hasEdge(sSrcNode, sDstNode, SConstraintEdge::SCopy))
        {
            SConstraintEdge* sCopy = sCG->getEdge(sSrcNode, sDstNode, SConstraintEdge::SCopy);
            // 
            sCopy->removeFEdge(fCopy);
            if (sCopy->getFEdgeSet().empty()) {
                sCG->removeDirectEdge(sCopy);
                STAT_TIME(timeOfDeletionProp, propagateDelPts(getPts(sSrcNode->getId()), sDstNode->getId()));   
            }
        }
        fCG->removeDirectEdge(fCopy);
    }
}


void AndersenInc::processVariantGepEdgeRemoval(NodeID srcid, NodeID dstid)
{
    SConstraintNode* sSrcNode = sCG->getSConstraintNode(srcid);
    SConstraintNode* sDstNode = sCG->getSConstraintNode(dstid);
    SConstraintEdge* sVGep = sCG->getEdge(sSrcNode, sDstNode, SConstraintEdge::SVariantGep);
    FConstraintNode* fSrcNode = fCG->getFConstraintNode(srcid);
    FConstraintNode* fDstNode = fCG->getFConstraintNode(dstid);
    FConstraintEdge* fVGep = fCG->getEdge(fSrcNode, fDstNode, FConstraintEdge::FVariantGep);
    sVGep->removeFEdge(fVGep);
    fCG->removeDirectEdge(fVGep);
    if (sSrcNode == sDstNode) {
        // Edge in SCC
        NodeID repId = sSrcNode->getId();
        redetectReps.push(repId);
        rep2EdgeSet[repId].insert(new SDK(srcid, dstid, FConstraintEdge::FVariantGep));
        sCG->removeTempGEdge(srcid , dstid, FConstraintEdge::FVariantGep);
        return;
    }
    else {
        // Edge not in SCC
        if (sVGep->getFEdgeSet().empty()) {
            sCG->removeDirectEdge(sVGep);
            delEdgeVec.push_back(new SDK(srcid, dstid, FConstraintEdge::FVariantGep));
            return;
        }
    }
}
void AndersenInc::processVariantGepEdgeRemoval_Lazy(NodeID srcid, NodeID dstid)
{
    SConstraintNode* sSrcNode = sCG->getSConstraintNode(srcid);
    SConstraintNode* sDstNode = sCG->getSConstraintNode(dstid);
    SConstraintEdge* sVGep = sCG->getEdge(sSrcNode, sDstNode, SConstraintEdge::SVariantGep);
    FConstraintNode* fSrcNode = fCG->getFConstraintNode(srcid);
    FConstraintNode* fDstNode = fCG->getFConstraintNode(dstid);
    FConstraintEdge* fVGep = fCG->getEdge(fSrcNode, fDstNode, FConstraintEdge::FVariantGep);
    sVGep->removeFEdge(fVGep);
    fCG->removeDirectEdge(fVGep);
    if (sSrcNode == sDstNode) {
        // Edge in SCC
        NodeID repId = sSrcNode->getId();
        redetectReps.push(repId);
        rep2EdgeSet[repId].insert(new SDK(srcid, dstid, FConstraintEdge::FVariantGep));
        sCG->removeTempGEdge(srcid , dstid, FConstraintEdge::FVariantGep);
        return;
    }
    else {
        // Edge not in SCC
        if (sVGep->getFEdgeSet().empty()) {
            sCG->removeDirectEdge(sVGep);
            // delEdgeVec.push_back(new SDK(srcid, dstid, FConstraintEdge::FVariantGep));
            // return;
            processVariantGepConstraintRemoval_Lazy(srcid, dstid);
        }
    }
}
void AndersenInc::processVariantGepEdgeRemoval_IPA(NodeID srcid, NodeID dstid)
{
    SConstraintNode* sSrcNode = sCG->getSConstraintNode(srcid);
    SConstraintNode* sDstNode = sCG->getSConstraintNode(dstid);
    SConstraintEdge* sVGep = sCG->getEdge(sSrcNode, sDstNode, SConstraintEdge::SVariantGep);
    FConstraintNode* fSrcNode = fCG->getFConstraintNode(srcid);
    FConstraintNode* fDstNode = fCG->getFConstraintNode(dstid);
    FConstraintEdge* fVGep = fCG->getEdge(fSrcNode, fDstNode, FConstraintEdge::FVariantGep);
    sVGep->removeFEdge(fVGep);
    fCG->removeDirectEdge(fVGep);
    if (sSrcNode == sDstNode) {
        // Copy Edge in SCC
        NodeID repId = sSrcNode->getId();
        bool prop = processSCCRedetection_IPA(repId);
        if (prop) {
            processVariantGepConstraintRemoval(srcid, dstid);
        }
        return;
    }
    else {
        // Copy Edge not in SCC
        if (sVGep->getFEdgeSet().empty()) {
            sCG->removeDirectEdge(sVGep);
            // IPA propagate changes eagerly
            processVariantGepConstraintRemoval(srcid, dstid);
            return;
        }
    }
}


void AndersenInc::processVariantGepConstraintRemoval(NodeID srcid, NodeID dstid)
{
    PointsTo tmpPts;
    for (NodeID o: getPts(srcid)) {
        if (sCG->isBlkObjOrConstantObj(o))
        {
            tmpPts.set(o);
            continue;
        }

        NodeID baseId = sCG->getFIObjVar(o);
        tmpPts.set(baseId);
    }
    NodeBS proping;
    STAT_TIME(timeOfDeletionProp, propagateDelPts_IPA(tmpPts, dstid, proping));
}
void AndersenInc::processVariantGepConstraintRemoval_Lazy(NodeID srcid, NodeID dstid)
{
    PointsTo tmpPts;
    for (NodeID o: getPts(srcid)) {
        if (sCG->isBlkObjOrConstantObj(o))
        {
            tmpPts.set(o);
            continue;
        }

        NodeID baseId = sCG->getFIObjVar(o);
        tmpPts.set(baseId);
    }
    // STAT_TIME(timeOfDeletionProp, propagateDelPts(tmpPts, dstid));
    dstid = sccRepNode(dstid);
    delPropMap[dstid] |= tmpPts;
}
/*
 * s --VGep--> d
 * pts(d) = pts(d) \Union FI(pts(s))
 */
void AndersenInc::processVariantGepRemoval(NodeID srcid, NodeID dstid)
{
    SConstraintNode* sSrcNode = sCG->getSConstraintNode(srcid);
    SConstraintNode* sDstNode = sCG->getSConstraintNode(dstid);
    PointsTo tmpDstPts;
    if (sSrcNode == sDstNode) {
        // VGep Edge in SCC
        NodeBS newReps;
        NodeID oldRep;
        double sccStart = stat->getClk();
        unsigned sccKeep = sCG->sccBreakDetect(srcid, dstid, FConstraintEdge::FVariantGep, newReps, oldRep);
        double sccEnd = stat->getClk();
        timeOfDeletionSCC += (sccEnd - sccStart) / TIMEINTERVAL;
        if (1 == sccKeep) {
            // SCC KEEP
            // SCC KEEP should remove the fEdge
            return;
        }
        // SCC Restore
        // SCC Restore should remove the fEdge and sEdge
        // reset Pts for new rep nodes
        for (NodeID id: newReps) {
            unionPts(id, getPts(oldRep));
        }
        // SCC Restore
        // SCC Restore should remove the fEdge and sEdge

        // VGep Constraint
        for (NodeID o: getPts(srcid)) {
            if (sCG->isBlkObjOrConstantObj(o))
            {
                tmpDstPts.set(o);
                continue;
            }

            // if (!isFieldInsensitive(o))
            // {
            //     setObjFieldInsensitive(o);
            //     sCG->addNodeToBeCollapsed(sCG->getBaseObjVar(o));
            // }

            // Add the field-insensitive node into pts.
            NodeID baseId = sCG->getFIObjVar(o);
            tmpDstPts.set(baseId);
        }
        STAT_TIME(timeOfDeletionProp, propagateDelPts(tmpDstPts, dstid));
    }
    else {
        SConstraintEdge* sVGep = sCG->getEdge(sSrcNode, sDstNode, SConstraintEdge::SVariantGep);
        FConstraintNode* fSrcNode = fCG->getFConstraintNode(srcid);
        FConstraintNode* fDstNode = fCG->getFConstraintNode(dstid);
        FConstraintEdge* fVGep = fCG->getEdge(fSrcNode, fDstNode, FConstraintEdge::FVariantGep);

        // 
        sVGep->removeFEdge(fVGep);
        if (sVGep->getFEdgeSet().empty()) {
            sCG->removeDirectEdge(sVGep);
            // VGep Constraint
            for (NodeID o: getPts(sSrcNode->getId())) {
                if (sCG->isBlkObjOrConstantObj(o))
                {
                    tmpDstPts.set(o);
                    continue;
                }
                // About isFieldInsensitive?
                // Add the field-insensitive node into pts.
                NodeID baseId = sCG->getFIObjVar(o);
                tmpDstPts.set(baseId);
            }
            STAT_TIME(timeOfDeletionProp, propagateDelPts(tmpDstPts, sDstNode->getId()));
        }

        fCG->removeDirectEdge(fVGep);
    }
}

void AndersenInc::processNormalGepEdgeRemoval(NodeID srcid, NodeID dstid, const AccessPath& ap)
{
    SConstraintNode* sSrcNode = sCG->getSConstraintNode(srcid);
    SConstraintNode* sDstNode = sCG->getSConstraintNode(dstid);
    SConstraintEdge* sNGep = sCG->getEdge(sSrcNode, sDstNode, SConstraintEdge::SNormalGep);
    FConstraintNode* fSrcNode = fCG->getFConstraintNode(srcid);
    FConstraintNode* fDstNode = fCG->getFConstraintNode(dstid);
    FConstraintEdge* fNGep = fCG->getEdge(fSrcNode, fDstNode, FConstraintEdge::FNormalGep);
    sNGep->removeFEdge(fNGep);
    fCG->removeDirectEdge(fNGep);
    if (sSrcNode == sDstNode) {
        // Edge in SCC
        NodeID repId = sSrcNode->getId();
        redetectReps.push(repId);
        rep2EdgeSet[repId].insert(new SDK(srcid, dstid, FConstraintEdge::FNormalGep, ap));
        sCG->removeTempGEdge(srcid , dstid, FConstraintEdge::FNormalGep);
        return;
    }
    else {
        // Edge not in SCC
        if (sNGep->getFEdgeSet().empty()) {
            sCG->removeDirectEdge(sNGep);
            delEdgeVec.push_back(new SDK(srcid, dstid, FConstraintEdge::FNormalGep, ap));
            return;
        }
    }
}
void AndersenInc::processNormalGepEdgeRemoval_Lazy(NodeID srcid, NodeID dstid, const AccessPath& ap)
{
    SConstraintNode* sSrcNode = sCG->getSConstraintNode(srcid);
    SConstraintNode* sDstNode = sCG->getSConstraintNode(dstid);
    SConstraintEdge* sNGep = sCG->getEdge(sSrcNode, sDstNode, SConstraintEdge::SNormalGep);
    FConstraintNode* fSrcNode = fCG->getFConstraintNode(srcid);
    FConstraintNode* fDstNode = fCG->getFConstraintNode(dstid);
    FConstraintEdge* fNGep = fCG->getEdge(fSrcNode, fDstNode, FConstraintEdge::FNormalGep);
    sNGep->removeFEdge(fNGep);
    fCG->removeDirectEdge(fNGep);
    if (sSrcNode == sDstNode) {
        // Edge in SCC
        NodeID repId = sSrcNode->getId();
        redetectReps.push(repId);
        rep2EdgeSet[repId].insert(new SDK(srcid, dstid, FConstraintEdge::FNormalGep, ap));
        sCG->removeTempGEdge(srcid , dstid, FConstraintEdge::FNormalGep);
        return;
    }
    else {
        // Edge not in SCC
        if (sNGep->getFEdgeSet().empty()) {
            sCG->removeDirectEdge(sNGep);
            // delEdgeVec.push_back(new SDK(srcid, dstid, FConstraintEdge::FNormalGep, ap));
            // return;
            processNormalGepConstraintRemoval_Lazy(srcid, dstid, ap);
        }
    }
}
void AndersenInc::processNormalGepEdgeRemoval_IPA(NodeID srcid, NodeID dstid, const AccessPath& ap)
{
    SConstraintNode* sSrcNode = sCG->getSConstraintNode(srcid);
    SConstraintNode* sDstNode = sCG->getSConstraintNode(dstid);
    SConstraintEdge* sNGep = sCG->getEdge(sSrcNode, sDstNode, SConstraintEdge::SNormalGep);
    FConstraintNode* fSrcNode = fCG->getFConstraintNode(srcid);
    FConstraintNode* fDstNode = fCG->getFConstraintNode(dstid);
    FConstraintEdge* fNGep = fCG->getEdge(fSrcNode, fDstNode, FConstraintEdge::FNormalGep);
    sNGep->removeFEdge(fNGep);
    fCG->removeDirectEdge(fNGep);
    if (sSrcNode == sDstNode) {
        // Copy Edge in SCC
        NodeID repId = sSrcNode->getId();
        bool prop = processSCCRedetection_IPA(repId);
        if (prop) {
            processNormalGepConstraintRemoval(srcid, dstid, ap);
        }
        return;
    }
    else {
        // Copy Edge not in SCC
        if (sNGep->getFEdgeSet().empty()) {
            sCG->removeDirectEdge(sNGep);
            // IPA propagate changes eagerly
            processNormalGepConstraintRemoval(srcid, dstid, ap);
            return;
        }
    }
}

void AndersenInc::processNormalGepConstraintRemoval(NodeID srcid, NodeID dstid, const AccessPath& ap)
{
    PointsTo tmpPts;
    for (NodeID o : getPts(srcid))
    {
        if (sCG->isBlkObjOrConstantObj(o) || isFieldInsensitive(o))
        {
            tmpPts.set(o);
            continue;
        }
        NodeID fieldSrcPtdNode = sCG->getGepObjVar(o, ap.getConstantFieldIdx());
        tmpPts.set(fieldSrcPtdNode);
    }
    NodeBS proping;
    STAT_TIME(timeOfDeletionProp, propagateDelPts_IPA(tmpPts, dstid, proping));
}
void AndersenInc::processNormalGepConstraintRemoval_Lazy(NodeID srcid, NodeID dstid, const AccessPath& ap)
{
    PointsTo tmpPts;
    for (NodeID o : getPts(srcid))
    {
        if (sCG->isBlkObjOrConstantObj(o) || isFieldInsensitive(o))
        {
            tmpPts.set(o);
            continue;
        }
        NodeID fieldSrcPtdNode = sCG->getGepObjVar(o, ap.getConstantFieldIdx());
        tmpPts.set(fieldSrcPtdNode);
    }
    // STAT_TIME(timeOfDeletionProp, propagateDelPts(tmpPts, dstid));
    dstid = sccRepNode(dstid);
    delPropMap[dstid] |= tmpPts;
}

void AndersenInc::removeObjFromAllPts(NodeID obj)
{
    for (SVFIR::iterator it = pag->begin(), eit = pag->end(); it != eit; ++it)
    {
        NodeID id = it->first;
        if (getPts(id).test(obj))
            clearPts(id, obj);
    }
}

void AndersenInc::cleanupDeadObjects()
{
    if (deadObjectCandidates.empty())
        return;

    // Count live (non-deleted) addr edges per candidate object in one pass.
    Map<NodeID, unsigned> liveAddrCount;
    SVFStmt::SVFStmtSetTy& addrs = pag->getPTASVFStmtSet(SVFStmt::Addr);
    for (SVFStmt::SVFStmtSetTy::iterator it = addrs.begin(), eit = addrs.end();
         it != eit; ++it)
    {
        const AddrStmt* addr = SVFUtil::cast<AddrStmt>(*it);
        if (addr->isDeleted)
            continue;
        NodeID obj = addr->getRHSVarID();
        if (deadObjectCandidates.test(obj))
            ++liveAddrCount[obj];
    }

    unsigned removed = 0;
    for (NodeID obj : deadObjectCandidates)
    {
        if (liveAddrCount[obj] > 0)
            continue;

        const SVFVar* node = pag->getGNode(obj);
        const MemObj* mem = nullptr;
        if (const ObjVar* objNode = SVFUtil::dyn_cast<ObjVar>(node))
            mem = objNode->getMemObj();
        else if (const GepObjVar* gepObj = SVFUtil::dyn_cast<GepObjVar>(node))
            mem = gepObj->getMemObj();

        // Only remove stack objects whose addr edge was deleted.  Global/heap
        // objects may be reachable through other means, and field objects are
        // handled via their base object.
        if (mem && mem->isStack())
        {
            removeObjFromAllPts(obj);
            ++removed;
        }
    }

    if (removed > 0)
        SVFUtil::outs() << "Removed " << removed << " dead stack object(s) from points-to sets.\n";
}

/*
 * s --NGep: ap--> d
 * pts(d) = pts(d) \Union AP(pts(s))
 */
void AndersenInc::processNormalGepRemoval(NodeID srcid, NodeID dstid, const AccessPath& ap)
{
    SConstraintNode* sSrcNode = sCG->getSConstraintNode(srcid);
    SConstraintNode* sDstNode = sCG->getSConstraintNode(dstid);
    PointsTo tmpDstPts;
    if (sSrcNode == sDstNode) {
        // NGep Edge in SCC
        NodeBS newReps;
        NodeID oldRep;
        double sccStart = stat->getClk();
        unsigned sccKeep = sCG->sccBreakDetect(srcid, dstid, FConstraintEdge::FNormalGep, newReps, oldRep);
        double sccEnd = stat->getClk();
        timeOfDeletionSCC += (sccEnd - sccStart) / TIMEINTERVAL;
        if (1 == sccKeep) {
            // SCC KEEP
            // SCC KEEP should remove the fEdge
            return;
        }
        // SCC Restore
        // SCC Restore should remove the fEdge and sEdge
        // reset Pts for new rep nodes
        for (NodeID id: newReps) {
            unionPts(id, getPts(oldRep));
        }

        // SCC Restore
        // SCC Restore should remove the fEdge and sEdge

        // NGep Constraint
        for (NodeID o : getPts(srcid))
        {
            if (sCG->isBlkObjOrConstantObj(o) || isFieldInsensitive(o))
            {
                tmpDstPts.set(o);
                continue;
            }
            NodeID fieldSrcPtdNode = sCG->getGepObjVar(o, ap.getConstantFieldIdx());
            tmpDstPts.set(fieldSrcPtdNode);
        }
        STAT_TIME(timeOfDeletionProp, propagateDelPts(tmpDstPts, dstid));   
    }
    else {
        SConstraintEdge* sNGep = sCG->getEdge(sSrcNode, sDstNode, SConstraintEdge::SNormalGep);
        FConstraintNode* fSrcNode = fCG->getFConstraintNode(srcid);
        FConstraintNode* fDstNode = fCG->getFConstraintNode(dstid);
        FConstraintEdge* fNGep = fCG->getEdge(fSrcNode, fDstNode, FConstraintEdge::FNormalGep);

        // 
        sNGep->removeFEdge(fNGep);
        if (sNGep->getFEdgeSet().empty()) {
            sCG->removeDirectEdge(sNGep);
            // NGep Constraint
            for (NodeID o : getPts(sSrcNode->getId()))
            {
                if (sCG->isBlkObjOrConstantObj(o) || isFieldInsensitive(o))
                {
                    tmpDstPts.set(o);
                    continue;
                }
                NodeID fieldSrcPtdNode = sCG->getGepObjVar(o, ap.getConstantFieldIdx());
                tmpDstPts.set(fieldSrcPtdNode);
            }
            STAT_TIME(timeOfDeletionProp, propagateDelPts(tmpDstPts, sDstNode->getId()));      
        }

        fCG->removeDirectEdge(fNGep);
    }
}

// TODO: --wjy
void AndersenInc::propagateDelPts(const PointsTo& pts, NodeID nodeId)
{
    if (pts.empty()) {
        return;
    }

    delNodeNum ++;

    SConstraintNode* node = sCG->getSConstraintNode(nodeId);
    nodeId = node->getId();
    PointsTo dPts; // objs need to be removed from pts(nodeId)
    dPts |= pts;

    // Filter 1: dPts = dPts \intersect pts(nodeId)
    const PointsTo& nodePts = getPts(nodeId);
    dPts &= nodePts;
    PointsTo inAddr;
    for (auto it = node->incomingAddrsBegin(), eit = node->incomingAddrsEnd();
        it != eit; ++it)
    {
        const SConstraintEdge* incomingAddr = *it;
        NodeID src = incomingAddr->getSrcID();
        inAddr.set(src);
    }
    dPts -= inAddr;
    // Filter 2: incoming neighbors check
    for (auto it = node->directInEdgeBegin(), eit = node->directInEdgeEnd();
        it != eit; ++it)
    {
        if (dPts.empty()){
            return;
        }

        const SConstraintEdge* incomingEdge = *it;
        NodeID src = incomingEdge->getSrcID();
        const PointsTo & srcPts = getPts(src);

        if (SVFUtil::isa<CopySCGEdge>(incomingEdge)) {
            dPts -= srcPts;
        }
        else if (SVFUtil::isa<VariantGepSCGEdge>(incomingEdge)) {
            for (NodeID o: srcPts) {
                if (sCG->isBlkObjOrConstantObj(o))
                    dPts.reset(o);
                else 
                    dPts.reset(sCG->getFIObjVar(o));
            }
        }
        else if (SVFUtil::isa<NormalGepSCGEdge>(incomingEdge)) {
            const NormalGepSCGEdge* ngep = SVFUtil::dyn_cast<NormalGepSCGEdge>(incomingEdge);
            for (NodeID o: srcPts) {
                if (sCG->isBlkObjOrConstantObj(o) || isFieldInsensitive(o))
                    dPts.reset(o);
                else {
                    const AccessPath& ap = ngep->getAccessPath();
                    NodeID fieldSrcPtdNode = sCG->getGepObjVar(o, ap.getConstantFieldIdx());
                    dPts.reset(fieldSrcPtdNode);
                }
            }
        }
    }

    // pts(nodeId) = pts(nodeId) - dPts
    for (NodeID o: dPts) {
        clearPts(nodeId, o);
    }

    // process outgoing neighbor propagate
    for (auto it = node->directOutEdgeBegin(), eit = node->directOutEdgeEnd();
        it != eit; ++it)
    {
        const SConstraintEdge* outSEdge = *it;
        NodeID dstId = outSEdge->getDstID();
        if (dstId == nodeId)
            continue; // self circle edge
        if (SVFUtil::isa<CopySCGEdge>(outSEdge)) {
            propagateDelPts(dPts, dstId);
        }
        else if (SVFUtil::isa<VariantGepSCGEdge>(outSEdge)) {
            PointsTo vgepProPts;
            for (NodeID o: dPts) {
                if (sCG->isBlkObjOrConstantObj(o))
                    vgepProPts.set(o);
                else
                    vgepProPts.set(sCG->getFIObjVar(o));
            }
            propagateDelPts(vgepProPts, dstId);
        }
        else if (SVFUtil::isa<NormalGepSCGEdge>(outSEdge)) {
            const NormalGepSCGEdge* ngep = SVFUtil::dyn_cast<NormalGepSCGEdge>(outSEdge);
            PointsTo ngepProPts;
            for (NodeID o: dPts) {
                if (sCG->isBlkObjOrConstantObj(o) || isFieldInsensitive(o))
                    ngepProPts.set(o);
                else {
                    const AccessPath& ap = ngep->getAccessPath();
                    NodeID fieldSrcPtdNode = sCG->getGepObjVar(o, ap.getConstantFieldIdx());
                    ngepProPts.set(fieldSrcPtdNode);
                }
            }
            propagateDelPts(ngepProPts, dstId);
        }
    }

    // process complex constraint 
    for (NodeID o: dPts) {
        if (pag->isConstantObj(o) || isNonPointerObj(o))
            continue;

        FConstraintNode* oNode = fCG->getFConstraintNode(o);

        // load complex constraint
        for (auto it = node->outgoingLoadsBegin(), eit = node->outgoingLoadsEnd();
            it != eit; ++it)
        {
            const SConstraintEdge* sLoad = *it;
            for (FConstraintEdge* fLoad: sLoad->getFEdgeSet()) 
            {
                FConstraintNode* loadDstNode = fLoad->getDstNode();
                if (!fCG->hasEdge(oNode, loadDstNode, FConstraintEdge::FCopy))
                    continue;
                FConstraintEdge* fEdge = fCG->getEdge(oNode, loadDstNode, FConstraintEdge::FCopy);
                CopyFCGEdge* fCopy = SVFUtil::dyn_cast<CopyFCGEdge>(fEdge);
                fCopy->removeComplexEdge(fLoad);
                if (fCopy->getComplexEdgeSet().empty())
                    // pushIntoDelEdgesWL(o, fLoad->getDstID(), FConstraintEdge::FCopy); -- newwl
                    delDirectEdgeVec.push_back(new SDK(o, fLoad->getDstID(), FConstraintEdge::FCopy));
            }
        }

        // store complex constraint
        for (auto it = node->incomingStoresBegin(), eit = node->incomingStoresEnd();
            it != eit; ++it)
        {
            const SConstraintEdge* sStore = *it;
            for (FConstraintEdge* fStore: sStore->getFEdgeSet()) 
            {
                FConstraintNode* storeSrcNode = fStore->getSrcNode();
                if (!fCG->hasEdge(storeSrcNode, oNode, FConstraintEdge::FCopy))
                    continue;
                FConstraintEdge* fEdge = fCG->getEdge(storeSrcNode, oNode, FConstraintEdge::FCopy);
                CopyFCGEdge* fCopy = SVFUtil::dyn_cast<CopyFCGEdge>(fEdge);
                fCopy->removeComplexEdge(fStore);
                if (fCopy->getComplexEdgeSet().empty())
                    // pushIntoDelEdgesWL(fStore->getSrcID(), o, FConstraintEdge::FCopy); -- newwl
                    delDirectEdgeVec.push_back(new SDK(fStore->getSrcID(), o, FConstraintEdge::FCopy));
            }
        }
    }
}

// TODO: --wjy
void AndersenInc::propagateDelPts_IPA(const PointsTo& pts, NodeID nodeId, NodeBS& proping)
{
    if (pts.empty()) {
        return;
    }

    delNodeNum ++;

    SConstraintNode* node = sCG->getSConstraintNode(nodeId);

    nodeId = node->getId();
    if (proping.test(nodeId)) {
        SVFUtil::outs() << "node " << nodeId << "in a cycle!\n";
        return;
    }
    proping.set(nodeId);

    PointsTo dPts; // objs need to be removed from pts(nodeId)
    dPts |= pts;

    // Filter 1: dPts = dPts \intersect pts(nodeId)
    const PointsTo& nodePts = getPts(nodeId);
    dPts &= nodePts;
    
    PointsTo inAddr;
    for (auto it = node->incomingAddrsBegin(), eit = node->incomingAddrsEnd();
        it != eit; ++it)
    {
        const SConstraintEdge* incomingAddr = *it;
        NodeID src = incomingAddr->getSrcID();
        inAddr.set(src);
    }
    dPts -= inAddr;
    // Filter 2: incoming neighbors check
    for (auto it = node->directInEdgeBegin(), eit = node->directInEdgeEnd();
        it != eit; ++it)
    {
        if (dPts.empty()){
            proping.reset(nodeId);
            return;
        }

        const SConstraintEdge* incomingEdge = *it;
        NodeID src = incomingEdge->getSrcID();
        const PointsTo & srcPts = getPts(src);

        if (SVFUtil::isa<CopySCGEdge>(incomingEdge)) {
            dPts -= srcPts;
        }
        else if (SVFUtil::isa<VariantGepSCGEdge>(incomingEdge)) {
            for (NodeID o: srcPts) {
                if (sCG->isBlkObjOrConstantObj(o))
                    dPts.reset(o);
                else 
                    dPts.reset(sCG->getFIObjVar(o));
            }
        }
        else if (SVFUtil::isa<NormalGepSCGEdge>(incomingEdge)) {
            const NormalGepSCGEdge* ngep = SVFUtil::dyn_cast<NormalGepSCGEdge>(incomingEdge);
            for (NodeID o: srcPts) {
                if (sCG->isBlkObjOrConstantObj(o) || isFieldInsensitive(o))
                    dPts.reset(o);
                else {
                    const AccessPath& ap = ngep->getAccessPath();
                    NodeID fieldSrcPtdNode = sCG->getGepObjVar(o, ap.getConstantFieldIdx());
                    dPts.reset(fieldSrcPtdNode);
                }
            }
        }
    }

    // pts(nodeId) = pts(nodeId) - dPts
    for (NodeID o: dPts) {
        clearPts(nodeId, o);
    }

    // process outgoing neighbor propagate
    for (auto it = node->directOutEdgeBegin(), eit = node->directOutEdgeEnd();
        it != eit; ++it)
    {
        const SConstraintEdge* outSEdge = *it;
        NodeID dstId = outSEdge->getDstID();
        if (dstId == nodeId)
            continue; // self circle edge
        if (SVFUtil::isa<CopySCGEdge>(outSEdge)) {
            propagateDelPts_IPA(dPts, dstId, proping);
        }
        else if (SVFUtil::isa<VariantGepSCGEdge>(outSEdge)) {
            PointsTo vgepProPts;
            for (NodeID o: dPts) {
                if (sCG->isBlkObjOrConstantObj(o))
                    vgepProPts.set(o);
                else
                    vgepProPts.set(sCG->getFIObjVar(o));
            }
            propagateDelPts_IPA(vgepProPts, dstId, proping);
        }
        else if (SVFUtil::isa<NormalGepSCGEdge>(outSEdge)) {
            const NormalGepSCGEdge* ngep = SVFUtil::dyn_cast<NormalGepSCGEdge>(outSEdge);
            PointsTo ngepProPts;
            for (NodeID o: dPts) {
                if (sCG->isBlkObjOrConstantObj(o) || isFieldInsensitive(o))
                    ngepProPts.set(o);
                else {
                    const AccessPath& ap = ngep->getAccessPath();
                    NodeID fieldSrcPtdNode = sCG->getGepObjVar(o, ap.getConstantFieldIdx());
                    ngepProPts.set(fieldSrcPtdNode);
                }
            }
            propagateDelPts_IPA(ngepProPts, dstId, proping);
        }
    }

    // process complex constraint 
    for (NodeID o: dPts) {
        if (pag->isConstantObj(o) || isNonPointerObj(o))
            continue;

        FConstraintNode* oNode = fCG->getFConstraintNode(o);

        // load complex constraint
        for (auto it = node->outgoingLoadsBegin(), eit = node->outgoingLoadsEnd();
            it != eit; ++it)
        {
            const SConstraintEdge* sLoad = *it;
            for (FConstraintEdge* fLoad: sLoad->getFEdgeSet()) 
            {
                FConstraintNode* loadDstNode = fLoad->getDstNode();
                if (!fCG->hasEdge(oNode, loadDstNode, FConstraintEdge::FCopy))
                    continue;
                FConstraintEdge* fEdge = fCG->getEdge(oNode, loadDstNode, FConstraintEdge::FCopy);
                CopyFCGEdge* fCopy = SVFUtil::dyn_cast<CopyFCGEdge>(fEdge);
                fCopy->removeComplexEdge(fLoad);
                if (fCopy->getComplexEdgeSet().empty())
                    // pushIntoDelEdgesWL(o, fLoad->getDstID(), FConstraintEdge::FCopy); -- newwl
                    delDirectEdgeVec.push_back(new SDK(o, fLoad->getDstID(), FConstraintEdge::FCopy));
            }
        }

        // store complex constraint
        for (auto it = node->incomingStoresBegin(), eit = node->incomingStoresEnd();
            it != eit; ++it)
        {
            const SConstraintEdge* sStore = *it;
            for (FConstraintEdge* fStore: sStore->getFEdgeSet()) 
            {
                FConstraintNode* storeSrcNode = fStore->getSrcNode();
                if (!fCG->hasEdge(storeSrcNode, oNode, FConstraintEdge::FCopy))
                    continue;
                FConstraintEdge* fEdge = fCG->getEdge(storeSrcNode, oNode, FConstraintEdge::FCopy);
                CopyFCGEdge* fCopy = SVFUtil::dyn_cast<CopyFCGEdge>(fEdge);
                fCopy->removeComplexEdge(fStore);
                if (fCopy->getComplexEdgeSet().empty())
                    // pushIntoDelEdgesWL(fStore->getSrcID(), o, FConstraintEdge::FCopy); -- newwl
                    delDirectEdgeVec.push_back(new SDK(fStore->getSrcID(), o, FConstraintEdge::FCopy));
            }
        }
    }
    proping.reset(nodeId);
}

void AndersenInc::processDeletion_EdgeConstraint_IPA()
{
    initFpPDM();
    SCCBreak = false;
    bool newCallCopyEdge = false;
    // bool needSCCDetect = false;

    // IPA: 1. prosess complex edge to extract direct edge
    while (! delEdgeVec.empty()) {
        SDK* sdk = delEdgeVec.back();
        delEdgeVec.pop_back();

        NodeID src = sdk->src;
        NodeID dst = sdk->dst;
        FConstraintEdge::FConstraintEdgeK kind = sdk->kind;
        if (kind == FConstraintEdge::FAddr) {
            processAddrRemoval_IPA(src, dst);
        }
        else if (kind == FConstraintEdge::FStore) {
            processStoreRemoval(src, dst);
        }
        else if (kind == FConstraintEdge::FLoad) {
            processLoadRemoval(src, dst);
        }
        delete sdk;
    }
    SVFUtil::outs() << "IPA 1 done.\n";

    // IPA: 2. process direct edge
    do {
        newCallCopyEdge = false;
        // needSCCDetect = false;
        int delDirectEdgeCount = 0, delEdgeCount = 0;
        while (! delDirectEdgeVec.empty()) {
            SDK* sdk = delDirectEdgeVec.back();
            delDirectEdgeVec.pop_back();
            delDirectEdgeCount++;
            NodeID src = sdk->src;
            NodeID dst = sdk->dst;
            FConstraintEdge::FConstraintEdgeK kind = sdk->kind;
            if (kind == FConstraintEdge::FCopy)
                // needSCCDetect |= 
                processCopyEdgeRemoval_IPA(src, dst);
            else if (kind == FConstraintEdge::FVariantGep) {
                processVariantGepEdgeRemoval_IPA(src, dst);
            }
            else if (kind == FConstraintEdge::FNormalGep) {
                const AccessPath& ap = sdk->ap;
                processNormalGepEdgeRemoval_IPA(src, dst, ap);
            }
            delete sdk;
        }
        // SVFUtil::outs() << "Deleted DirectEdge Num: " << delDirectEdgeCount << "\n";

        // processSCCRedetection();

        // process simple and comlex constraint...

        // SVFUtil::outs() << "Deleted Edge Num: " << delEdgeCount << "\n";

        double fpstart = stat->getClk();
        computeFpPDM();
        double fpend = stat->getClk();
        timeOfInsComputeFP += (fpend - fpstart) / TIMEINTERVAL;

        if (updateCallGraphDel(getIndirectCallsites()))
            newCallCopyEdge = true;
        if (!delDirectEdgeVec.empty())
            newCallCopyEdge = true;
        // if (!delEdgeVec.empty())
        //     newCallCopyEdge = true;

    } while(newCallCopyEdge);
    // sCG->cleanRep2TempG();
}

void AndersenInc::processDeletion_EdgeConstraint_Lazy()
{
    initFpPDM();
    SCCBreak = false;
    bool newCallCopyEdge = false;

    // Lazy: 1.1 prosess complex edge to extract direct edge
    while (! delEdgeVec.empty()) {
        SDK* sdk = delEdgeVec.back();
        delEdgeVec.pop_back();

        NodeID src = sdk->src;
        NodeID dst = sdk->dst;
        FConstraintEdge::FConstraintEdgeK kind = sdk->kind;
        if (kind == FConstraintEdge::FAddr) {
            processAddrRemoval_Lazy(src, dst);
        }
        else if (kind == FConstraintEdge::FStore) {
            processStoreRemoval(src, dst);
        }
        else if (kind == FConstraintEdge::FLoad) {
            processLoadRemoval(src, dst);
        }
        delete sdk;
    }
    SVFUtil::outs() << "Lazy 1 done.\n";
    int round = 0;
    // Lazy: 1.2 process direct edge
    do {
        round ++;
        newCallCopyEdge = false;
        // needSCCDetect = false;
        int delDirectEdgeCount = 0, delEdgeCount = 0;
        SVFUtil::outs()<< "Round" << round << "begin ...\n"; 
        while (! delDirectEdgeVec.empty()) {
            SDK* sdk = delDirectEdgeVec.back();
            delDirectEdgeVec.pop_back();
            delDirectEdgeCount++;
            NodeID src = sdk->src;
            NodeID dst = sdk->dst;
            FConstraintEdge::FConstraintEdgeK kind = sdk->kind;
            if (kind == FConstraintEdge::FCopy)
                // needSCCDetect |= 
                processCopyEdgeRemoval_Lazy(src, dst);
            else if (kind == FConstraintEdge::FVariantGep) {
                processVariantGepEdgeRemoval_Lazy(src, dst);
            }
            else if (kind == FConstraintEdge::FNormalGep) {
                const AccessPath& ap = sdk->ap;
                processNormalGepEdgeRemoval_Lazy(src, dst, ap);
            }
            delete sdk;
        }
        // SVFUtil::outs() << "Deleted DirectEdge Num: " << delDirectEdgeCount << "\n";
        SVFUtil::outs()<< "Round" << round << "Lazy 2 done.\n"; 
        // Lazy: 1.2 SCC detection
        processSCCRedetection();

        // Lazy: 3 change updating and propagation
        // 2024.4 TODO: Topo order?
        for (auto it = delPropMap.begin(), eit = delPropMap.end(); it != eit; ++it) {
            NodeID dst = sccRepNode(it->first);
            PointsTo& pts = it->second;
            // bool sccflag = unFilterSet.find(dst) != unFilterSet.end();
            // if (sccflag)
            //     unFilterSet.erase(dst);
            STAT_TIME(timeOfDeletionProp, propagateDelPts(pts, dst));  
            // delPropMap[dst].clear();
        }
        delPropMap.clear();
        SVFUtil::outs()<< "Round" << round << "Lazy 3 done.\n"; 
        
        // Lazy: 4 function pointer
        double fpstart = stat->getClk();
        computeFpPDM();
        double fpend = stat->getClk();
        timeOfInsComputeFP += (fpend - fpstart) / TIMEINTERVAL;
        SVFUtil::outs()<< "Round" << round << "FP done.\n"; 
        if (updateCallGraphDel(getIndirectCallsites()))
            newCallCopyEdge = true;
        if (!delDirectEdgeVec.empty())
            newCallCopyEdge = true;
        // if (!delEdgeVec.empty())
        //     newCallCopyEdge = true;

    } while(newCallCopyEdge);
    sCG->cleanRep2TempG();
    SVFUtil::outs()<< "Clean rep2tempg done.\n"; 
}

void AndersenInc::processDeletion_EdgeConstraint()
{
    initFpPDM();
    SCCBreak = false;
    bool newCallCopyEdge = false;
    // bool needSCCDetect = false;
    do {
        newCallCopyEdge = false;
        // needSCCDetect = false;
        int delDirectEdgeCount = 0, delEdgeCount = 0;
        while (! delDirectEdgeVec.empty()) {
            SDK* sdk = delDirectEdgeVec.back();
            delDirectEdgeVec.pop_back();
            delDirectEdgeCount++;
            NodeID src = sdk->src;
            NodeID dst = sdk->dst;
            FConstraintEdge::FConstraintEdgeK kind = sdk->kind;
            if (kind == FConstraintEdge::FCopy)
                // needSCCDetect |= 
                processCopyEdgeRemoval(src, dst);
            else if (kind == FConstraintEdge::FVariantGep) {
                processVariantGepEdgeRemoval(src, dst);
            }
            else if (kind == FConstraintEdge::FNormalGep) {
                const AccessPath& ap = sdk->ap;
                processNormalGepEdgeRemoval(src, dst, ap);
            }
            delete sdk;
        }
        // SVFUtil::outs() << "Deleted DirectEdge Num: " << delDirectEdgeCount << "\n";

        processSCCRedetection();

        // process simple and comlex constraint
        while (! delEdgeVec.empty()) {
            SDK* sdk = delEdgeVec.back();
            delEdgeVec.pop_back();

            NodeID src = sdk->src;
            NodeID dst = sdk->dst;
            FConstraintEdge::FConstraintEdgeK kind = sdk->kind;
            if (kind == FConstraintEdge::FAddr) {
                processAddrRemoval(src, dst);
                delEdgeCount ++;
            }
            else if (kind == FConstraintEdge::FCopy) {
                processCopyConstraintRemoval(src, dst);
            }
            else if (kind == FConstraintEdge::FVariantGep) {
                processVariantGepConstraintRemoval(src, dst);
            }
            else if (kind == FConstraintEdge::FVariantGep) {
                const AccessPath& ap = sdk->ap;
                processNormalGepConstraintRemoval(src, dst, ap);
            }
            else if (kind == FConstraintEdge::FStore) {
                processStoreRemoval(src, dst);
                delEdgeCount ++;
            }
            else if (kind == FConstraintEdge::FLoad) {
                processLoadRemoval(src, dst);
                delEdgeCount ++;
            }
            delete sdk;
        }
        // SVFUtil::outs() << "Deleted Edge Num: " << delEdgeCount << "\n";

        double fpstart = stat->getClk();
        computeFpPDM();
        double fpend = stat->getClk();
        timeOfInsComputeFP += (fpend - fpstart) / TIMEINTERVAL;

        if (updateCallGraphDel(getIndirectCallsites()))
            newCallCopyEdge = true;
        if (!delDirectEdgeVec.empty())
            newCallCopyEdge = true;
        if (!delEdgeVec.empty())
            newCallCopyEdge = true;

    } while(newCallCopyEdge);
    sCG->cleanRep2TempG();
}


void AndersenInc::processDeletion()
{
    initFpPDM();
    bool newCallCopyEdge = false;
    do {
        newCallCopyEdge = false;
        int deletedEdgeCount = 0;
        while (!delEdgesWL.empty()) {
            FConstraintEdge* fEdge = delEdgesWL.pop();
            NodeID src = fEdge->getSrcID();
            NodeID dst = fEdge->getDstID();
            deletedEdgeCount ++;
            
            if (SVFUtil::isa<AddrFCGEdge>(fEdge)) {
                // dumpSDK(src, dst, FConstraintEdge::FAddr);
                processAddrRemoval(src, dst);
            }
            else if (SVFUtil::isa<CopyFCGEdge>(fEdge)) {
                // dumpSDK(src, dst, FConstraintEdge::FCopy);
                processCopyRemoval(src, dst);
            }
            else if (SVFUtil::isa<VariantGepFCGEdge>(fEdge)) {
                // dumpSDK(src, dst, FConstraintEdge::FVariantGep);
                processVariantGepRemoval(src, dst);
            }
            else if (SVFUtil::isa<NormalGepFCGEdge>(fEdge)) {
                // dumpSDK(src, dst, FConstraintEdge::FNormalGep);
                NormalGepFCGEdge* ngep = SVFUtil::dyn_cast<NormalGepFCGEdge>(fEdge);
                const AccessPath& ap = ngep->getAccessPath();
                processNormalGepRemoval(src, dst, ap);
            }
            else if (SVFUtil::isa<StoreFCGEdge>(fEdge)) {
                // dumpSDK(src, dst, FConstraintEdge::FStore);
                processStoreRemoval(src, dst);
            }
            else if (SVFUtil::isa<LoadFCGEdge>(fEdge)) {
                // dumpSDK(src, dst, FConstraintEdge::FLoad);
                processLoadRemoval(src, dst);
            }
        }
        SVFUtil::outs() << "Deleted Edge Num: " << deletedEdgeCount << "\n";
        double fpstart = stat->getClk();
        computeFpPDM();
        double fpend = stat->getClk();
        timeOfDelComputeFP += (fpend - fpstart) / TIMEINTERVAL;
        if (updateCallGraphDel(getIndirectCallsites()))
            newCallCopyEdge = true;
        if (!delEdgesWL.empty())
            newCallCopyEdge = true;

    } while (newCallCopyEdge);
}


void AndersenInc::initFpPDM() {
    for (auto it = fpPtsDiffMap.begin(), eit = fpPtsDiffMap.end(); it != eit; ++it)
    {
        NodeID id = it->first;
        PtsDiff* pd = it->second;
        delete pd;
    }
    fpPtsDiffMap.clear();
    const CallSiteToFunPtrMap& callsites = getIndirectCallsites();
    for(CallSiteToFunPtrMap::const_iterator iter = callsites.begin(), eiter = callsites.end(); 
        iter != eiter; ++iter)
    {
        const CallICFGNode* cs = iter->first;

        if (SVFUtil::getSVFCallSite(cs->getCallSite()).isVirtualCall())
        {
            const SVFValue *vtbl = SVFUtil::getSVFCallSite(cs->getCallSite()).getVtablePtr();
            assert(pag->hasValueNode(vtbl));
            NodeID vtblId = pag->getValueNode(vtbl);

            PtsDiff* pd = new PtsDiff;
            pd->nodeId = vtblId;
            pd->prePts = getPts(vtblId);
            pd->nowPts = getPts(vtblId);
            fpPtsDiffMap[vtblId] = pd;
        }
        else
        {
            PtsDiff* pd = new PtsDiff;
            pd->nodeId = iter->second;
            pd->prePts = getPts(iter->second);
            pd->nowPts = getPts(iter->second);
            fpPtsDiffMap[iter->second] = pd;
        }
    }
}

void AndersenInc::computeFpPDM() {
    for(auto iter = fpPtsDiffMap.begin(), eiter = fpPtsDiffMap.end(); 
        iter != eiter; ++iter)
    {
        NodeID nodeid = iter->first;
        PtsDiff* pd = iter->second;
        PointsTo ptspre = pd->nowPts;
        PointsTo ptsnow = getPts(nodeid);
        pd->insPts.clear();
        pd->delPts.clear();
        pd->insPts |= (ptsnow - ptspre);
        pd->delPts |= (ptspre - ptsnow);
        pd->prePts.clear();
        pd->prePts |= ptspre;
        pd->nowPts.clear();
        pd->nowPts |= ptsnow;
    }
}

void AndersenInc::initAllPDM()
{
    // OrderedNodeSet pagNodes;
    for (auto it = allPtsDiffMap.begin(), eit = allPtsDiffMap.end(); it != eit; ++it)
    {
        // NodeID n = it->first;
        PtsDiff* pd = it->second;
        delete pd;
    }
    allPtsDiffMap.clear();
    for(SVFIR::iterator it = pag->begin(), eit = pag->end(); it!=eit; it++)
    {
        // pagNodes.insert(it->first);
        NodeID n = it->first;
        PtsDiff* pd = new PtsDiff;
        pd->nodeId = n;
        pd->prePts = getPts(n);
        allPtsDiffMap[n] = pd;
    }

}

void AndersenInc::computeAllPDM()
{
    // OrderedNodeSet pagNodes;
    // for(SVFIR::iterator it = pag->begin(), eit = pag->end(); it!=eit; it++)
    // {
    //     pagNodes.insert(it->first);
    // }
    ptsChangeNodes.clear();
    int count = 0;
    for (auto iter = allPtsDiffMap.begin(), eiter = allPtsDiffMap.end(); 
        iter != eiter; ++iter)
    {
        NodeID n = iter->first;
        PtsDiff* pd = iter->second;

        const PointsTo& prepts = pd->prePts;
        const PointsTo& incpts = getPts(n);

        if (prepts == incpts)
        {
            pd->delPts.clear();
            pd->insPts.clear();
        }
        else {
            pd->delPts = prepts - incpts;
            pd->insPts = incpts - prepts;
            ptsChangeNodes.set(n);
            count ++;
            // SVFUtil::outs() << "@@ Pts Diff @@: " << n << "\n";
            // SVFUtil::outs() << "@@@ delPts: ";
            // dumpPts(pd->delPts);
            // SVFUtil::outs() << "@@@ insPts: ";
            // dumpPts(pd->insPts);
            // SVFUtil::outs() << "@@@@@@@@@@@@@@@@@@@@@@@@@@@\n";
        }
    }
    SVFUtil::outs() << "PtsChangeNodes Size: " << count << "\n";
}
// NodeBS& AndersenInc::addRevChainNode(NodeID o)
// {
//     const SVF::NodeSet& revos = getRevPts(o);
//     if (revos.empty()) {
//         return;
//     }
//     for (NodeID revo: revos) {
//         ptsChainChangeNodes |= addRevChainNode(revo);
//     }
// }
// void AndersenInc::computePtsChainChangeNodes()
// {
//     for (NodeID o: ptsChangeNodes) {
//         const SVF::NodeSet& revo = getRevPts(o);
//         for ()
//     }
// }
void AndersenInc::dumpPts(const PointsTo& pts)
{
    SVFUtil::outs() << "{ " ;
    for(NodeID o: pts)
        SVFUtil::outs() << o << ", ";
    SVFUtil::outs() << "}\n" ;
}

bool AndersenInc::updateCallGraphDel(const CallSiteToFunPtrMap& callsites)
{

    double cgUpdateStart = stat->getClk();

    CallEdgeMap newEdges;
    onTheFlyCallGraphSolveDel(callsites,newEdges);
    NodePairSet cpySrcNodes;	/// nodes as a src of a generated new copy edge
    for(CallEdgeMap::iterator it = newEdges.begin(), eit = newEdges.end(); it!=eit; ++it )
    {
        CallSite cs = SVFUtil::getSVFCallSite(it->first->getCallSite());
        for(FunctionSet::iterator cit = it->second.begin(), ecit = it->second.end(); cit!=ecit; ++cit)
        {
            connectCaller2CalleeParamsDel(cs,*cit,cpySrcNodes);
        }
    }
    // for(NodePairSet::iterator it = cpySrcNodes.begin(), eit = cpySrcNodes.end(); it!=eit; ++it)
    // {
    //     pushIntoWorklist(it->first);
    // }

    double cgUpdateEnd = stat->getClk();
    // timeOfUpdateCallGraph += (cgUpdateEnd - cgUpdateStart) / TIMEINTERVAL;
    timeOfUpdateCallGraph += (cgUpdateEnd - cgUpdateStart) / TIMEINTERVAL;

    return (!newEdges.empty());
}

void AndersenInc::onTheFlyCallGraphSolveDel(const CallSiteToFunPtrMap& callsites, CallEdgeMap& newEdges)
{
    for(CallSiteToFunPtrMap::const_iterator iter = callsites.begin(), eiter = callsites.end(); 
        iter != eiter; ++iter)
    {
        const CallICFGNode* cs = iter->first;

        if (SVFUtil::getSVFCallSite(cs->getCallSite()).isVirtualCall())
        {
            const SVFValue *vtbl = SVFUtil::getSVFCallSite(cs->getCallSite()).getVtablePtr();
            assert(pag->hasValueNode(vtbl));
            NodeID vtblId = pag->getValueNode(vtbl);
            PtsDiff* pd = fpPtsDiffMap[vtblId];
            if (pd->delPts.empty())
                continue;   // if no pts changed, then do nothing
            // resolveCPPIndCallsDel(cs, getPts(vtblId), newEdges);
            const PointsTo & delPts = pd->delPts;
            resolveCPPIndCallsDel(cs, delPts, newEdges);
        }
        else {
            PtsDiff* pd = fpPtsDiffMap[iter->second];
            if (pd->delPts.empty())
                continue;   // if no pts changed, then do nothing
            const PointsTo & delPts = pd->delPts;
            // resolveIndCallsDel(iter->first,getPts(iter->second),newEdges);
            resolveIndCallsDel(iter->first, delPts, newEdges);
        }
    }
}

// TODO
void AndersenInc::resolveCPPIndCallsDel(const CallICFGNode* cs, const PointsTo& target, CallEdgeMap& newEdges)
{
    assert(SVFUtil::getSVFCallSite(cs->getCallSite()).isVirtualCall() && "not cpp virtual call");

    VFunSet vfns;
    if (Options::ConnectVCallOnCHA())
        getVFnsFromCHA(cs, vfns);
    else
        getVFnsFromPts(cs, target, vfns);
    connectVCallToVFnsDel(cs, vfns, newEdges); // TODO: handle indirect call edge?
}

void AndersenInc::resolveIndCallsDel(const CallICFGNode* cs, const PointsTo& target, CallEdgeMap& newEdges)
 {
    assert(pag->isIndirectCallSites(cs) && "not an indirect callsite?");
    /// discover indirect pointer target
     for (PointsTo::iterator ii = target.begin(), ie = target.end();
            ii != ie; ii++)
    {

        if(getNumOfResolvedIndCallEdge() >= Options::IndirectCallLimit())
        {
            wrnMsg("Resolved Indirect Call Edges are Out-Of-Budget, please increase the limit");
            return;
        }

        if(ObjVar* objPN = SVFUtil::dyn_cast<ObjVar>(pag->getGNode(*ii)))
        {
            const MemObj* obj = pag->getObject(objPN);

            if(obj->isFunction())
            {
                const SVFFunction* calleefun = SVFUtil::cast<SVFFunction>(obj->getValue());
                const SVFFunction* callee = calleefun->getDefFunForMultipleModule();

                /// if the arg size does not match then we do not need to connect this parameter
                /// even if the callee is a variadic function (the first parameter of variadic function is its paramter number)
                if(SVFUtil::matchArgs(cs->getCallSite(), callee) == false)
                    continue;

                // if(0 == getIndCallMap()[cs].count(callee))
                // {
                //     newEdges[cs].insert(callee);
                //     getIndCallMap()[cs].insert(callee);

                //     ptaCallGraph->addIndirectCallGraphEdge(cs, cs->getCaller(), callee);
                //     // FIXME: do we need to update llvm call graph here?
                //     // The indirect call is maintained by ourself, We may update llvm's when we need to
                //     //CallGraphNode* callgraphNode = callgraph->getOrInsertFunction(cs.getCaller());
                //     //callgraphNode->addCalledFunction(cs,callgraph->getOrInsertFunction(callee));
                // }
                if(0 < getIndCallMap()[cs].count(callee))
                {
                    newEdges[cs].insert(callee);
                    getIndCallMap()[cs].erase(callee);

                    ptaCallGraph->removeIndirectCallGraphEdge(cs, cs->getCaller(), callee);
                }
            }
        }
    }   
}

void AndersenInc::connectVCallToVFnsDel(const CallICFGNode* cs, const VFunSet &vfns, CallEdgeMap& newEdges)
{
    //// connect all valid functions
    for (VFunSet::const_iterator fit = vfns.begin(),
            feit = vfns.end(); fit != feit; ++fit)
    {
        const SVFFunction* callee = *fit;
        callee = callee->getDefFunForMultipleModule();
        // if (getIndCallMap()[cs].count(callee) > 0)
        //     continue;
        if (getIndCallMap()[cs].count(callee) == 0)
            continue;
        if(SVFUtil::getSVFCallSite(cs->getCallSite()).arg_size() == callee->arg_size() ||
                (SVFUtil::getSVFCallSite(cs->getCallSite()).isVarArg() && callee->isVarArg()))
        {
            newEdges[cs].insert(callee);
            getIndCallMap()[cs].erase(callee);
            const CallICFGNode* callBlockNode = pag->getICFG()->getCallICFGNode(cs->getCallSite());
            ptaCallGraph->removeIndirectCallGraphEdge(callBlockNode, cs->getCaller(),callee);
        }
    }
}

void AndersenInc::connectCaller2CalleeParamsDel(CallSite cs, const SVFFunction* F, NodePairSet &cpySrcNodes)
{
    assert(F);

    DBOUT(DAndersen, SVFUtil::outs() << "connect parameters from indirect callsite " << *cs.getInstruction() << " to callee " << *F << "\n");

    CallICFGNode* callBlockNode = pag->getICFG()->getCallICFGNode(cs.getInstruction());
    RetICFGNode* retBlockNode = pag->getICFG()->getRetICFGNode(cs.getInstruction());

    if(SVFUtil::isHeapAllocExtFunViaRet(F) && pag->callsiteHasRet(retBlockNode))
    {
        // heapAllocatorViaIndCall(cs,cpySrcNodes); // TODO 
        SVFUtil::outs() << "Do not handle heapAllocatorViaIndCall in incremental analysis!\n";
    }

    if (pag->funHasRet(F) && pag->callsiteHasRet(retBlockNode))
    {
        const PAGNode* cs_return = pag->getCallSiteRet(retBlockNode);
        const PAGNode* fun_return = pag->getFunRet(F);
        if (cs_return->isPointer() && fun_return->isPointer())
        {
            NodeID dstrec = cs_return->getId();
            NodeID srcret = fun_return->getId();
            // if(addCopyEdge(srcret, dstrec)) // TODO: ret-->rec
            // {
            //     cpySrcNodes.insert(std::make_pair(srcret,dstrec));
            // }
            // pushIntoDelEdgesWL(srcret, dstrec, FConstraintEdge::FCopy); -- newwl
            delDirectEdgeVec.push_back(new SDK(srcret, dstrec, FConstraintEdge::FCopy));
        }
        else
        {
            DBOUT(DAndersen, SVFUtil::outs() << "not a pointer ignored\n");
        }
    }

    if (pag->hasCallSiteArgsMap(callBlockNode) && pag->hasFunArgsList(F))
    {

        // connect actual and formal param
        const SVFIR::SVFVarList& csArgList = pag->getCallSiteArgsList(callBlockNode);
        const SVFIR::SVFVarList& funArgList = pag->getFunArgsList(F);
        //Go through the fixed parameters.
        DBOUT(DPAGBuild, outs() << "      args:");
        SVFIR::SVFVarList::const_iterator funArgIt = funArgList.begin(), funArgEit = funArgList.end();
        SVFIR::SVFVarList::const_iterator csArgIt  = csArgList.begin(), csArgEit = csArgList.end();
        for (; funArgIt != funArgEit; ++csArgIt, ++funArgIt)
        {
            //Some programs (e.g. Linux kernel) leave unneeded parameters empty.
            if (csArgIt  == csArgEit)
            {
                DBOUT(DAndersen, SVFUtil::outs() << " !! not enough args\n");
                break;
            }
            const PAGNode *cs_arg = *csArgIt ;
            const PAGNode *fun_arg = *funArgIt;

            if (cs_arg->isPointer() && fun_arg->isPointer())
            {
                DBOUT(DAndersen, SVFUtil::outs() << "process actual parm  " << cs_arg->toString() << " \n");
                NodeID srcAA = cs_arg->getId();
                NodeID dstFA = fun_arg->getId();
                // if(addCopyEdge(srcAA, dstFA)) // TODO: act arg --> fom arg
                // {
                //     cpySrcNodes.insert(std::make_pair(srcAA,dstFA));
                // }
                // pushIntoDelEdgesWL(srcAA, dstFA, FConstraintEdge::FCopy); -- newwl
                delDirectEdgeVec.push_back(new SDK(srcAA, dstFA, FConstraintEdge::FCopy));
            }
        }

        //Any remaining actual args must be varargs.
        if (F->isVarArg())
        {
            NodeID vaF = pag->getVarargNode(F);
            DBOUT(DPAGBuild, SVFUtil::outs() << "\n      varargs:");
            for (; csArgIt != csArgEit; ++csArgIt)
            {
                const PAGNode *cs_arg = *csArgIt;
                if (cs_arg->isPointer())
                {
                    NodeID vnAA = cs_arg->getId();
                    // if (addCopyEdge(vnAA,vaF)) // TODO: remaining actual arg --> var arg
                    // {
                    //     cpySrcNodes.insert(std::make_pair(vnAA,vaF));
                    // }
                    // pushIntoDelEdgesWL(vnAA, vaF, FConstraintEdge::FCopy); -- newwl
                    delDirectEdgeVec.push_back(new SDK(vnAA, vaF, FConstraintEdge::FCopy));
                }
            }
        }
        if(csArgIt != csArgEit)
        {
            writeWrnMsg("too many args to non-vararg func.");
            writeWrnMsg("(" + cs.getInstruction()->getSourceLoc() + ")");
        }
    }
}

void AndersenInc::processInsertion_pseudo()
{
    for (auto nodeIt = sCG->begin(), nodeEit = sCG->end(); 
        nodeIt != nodeEit; nodeIt++)
        computeDiffPts(nodeIt->first);
        

    while (!insEdgeVec.empty()) {
        SDK* sdk = insEdgeVec.back();
        insEdgeVec.pop_back();

        NodeID src = sdk->src;
        NodeID dst = sdk->dst;
        FConstraintEdge::FConstraintEdgeK kind = sdk->kind;
        if (kind == FConstraintEdge::FAddr) {
            processAddrAddition_pseudo(src, dst);
        }
        else if (kind == FConstraintEdge::FCopy) {
            processCopyAddition_pseudo(src, dst);
        }
        else if (kind == FConstraintEdge::FVariantGep) {
            processVariantGepAddition_pseudo(src, dst);
        }
        else if (kind == FConstraintEdge::FVariantGep) {
            const AccessPath& ap = sdk->ap;
            processNormalGepAddition_pseudo(src, dst, ap);
        }
        else if (kind == FConstraintEdge::FStore) {
            processStoreAddition_pseudo(src, dst);
        }
        else if (kind == FConstraintEdge::FLoad) {
            processLoadAddition_pseudo(src, dst);
        }
        delete sdk;
    }

    initWorklist();
    do
    {
        // numOfIteration++;
        // if (0 == numOfIteration % iterationForPrintStat)
        //     printStat();

        reanalyze = false;

        solveWorklist();

        if (updateCallGraph(getIndirectCallsites()))
            reanalyze = true;

    }
    while (reanalyze);


}

void AndersenInc::processAddrAddition_pseudo(NodeID srcid, NodeID dstid)
{
    fCG->addAddrFCGEdge(srcid, dstid);
    sCG->addAddrSCGEdge(srcid, dstid);
    
    if (addPts(sccRepNode(dstid), srcid))
        pushIntoWorklist(dstid);
}

void AndersenInc::processCopyAddition_pseudo(NodeID srcid, NodeID dstid)
{
    fCG->addCopyFCGEdge(srcid, dstid);
    if (sCG->addCopySCGEdge(srcid, dstid, true))
        updatePropaPts(srcid, dstid);
}

void AndersenInc::processNormalGepAddition_pseudo(NodeID srcid, NodeID dstid, const AccessPath& ap)
{
    fCG->addNormalGepFCGEdge(srcid, dstid, ap);
    NormalGepSCGEdge* newGep = sCG->addNormalGepSCGEdge(srcid, dstid, ap);
    if (newGep)
        processGepPts(getPts(srcid), newGep);
}

void AndersenInc::processVariantGepAddition_pseudo(NodeID srcid, NodeID dstid)
{
    fCG->addVariantGepFCGEdge(srcid, dstid);
    VariantGepSCGEdge* newGep = sCG->addVariantGepSCGEdge(srcid, dstid);
    if (newGep)
        processGepPts(getPts(srcid), newGep);
}

void AndersenInc::processLoadAddition_pseudo(NodeID srcid, NodeID dstid)
{
    fCG->addLoadFCGEdge(srcid, dstid);
    if (sCG->addLoadSCGEdge(srcid, dstid))
        pushIntoWorklist(srcid);
}

void AndersenInc::processStoreAddition_pseudo(NodeID srcid, NodeID dstid)
{
    fCG->addStoreFCGEdge(srcid, dstid);
    if (sCG->addStoreSCGEdge(srcid, dstid))
        pushIntoWorklist(dstid);
}


void AndersenInc::processInsertion()
{
    initFpPDM();
    bool newCallCopyEdge = false;
    bool needSCCDetect = false;
    do {
        newCallCopyEdge = false;
        needSCCDetect = false;
        int insDirectEdgeCount = 0, insEdgeCount = 0;
        // process direct edge insertion
        
        while (!insDirectEdgeVec.empty()) {
            SDK* sdk = insDirectEdgeVec.back();
            insDirectEdgeVec.pop_back();
            insDirectEdgeCount ++;
            NodeID src = sdk->src;
            NodeID dst = sdk->dst;
            FConstraintEdge::FConstraintEdgeK kind = sdk->kind;
            FConstraintEdge* compEdge = sdk->compEdge;
            // dumpSDK(src, dst, kind);
            if (kind == FConstraintEdge::FCopy)
                needSCCDetect |= processCopyAddition(src, dst, compEdge);
            else if (kind == FConstraintEdge::FVariantGep)
                needSCCDetect |= processVariantGepAddition(src, dst);
            else if (kind == FConstraintEdge::FNormalGep) {
                const AccessPath& ap = sdk->ap;
                needSCCDetect |= processNormalGepAddition(src, dst, ap);
            }
            delete sdk;
        }
        // SVFUtil::outs() << "Inserted DirectEdge Num: " << insDirectEdgeCount << "\n";
        
        if (needSCCDetect && SCCBreak) {
            double sccStart = stat->getClk();
            SCCDetect();
            double sccEnd = stat->getClk();
            timeOfInsertionSCC += (sccEnd - sccStart) / TIMEINTERVAL;
            timeOfSCCDetectionIns += (sccEnd - sccStart) / TIMEINTERVAL;
            numOfSCCDetectionIns++;
        }

        // build propagate map
        while (!insDirectConsVec.empty()) {
            SDK* sdk = insDirectConsVec.back();
            insDirectConsVec.pop_back();
            
            NodeID src = sdk->src;
            NodeID dst = sdk->dst;
            FConstraintEdge::FConstraintEdgeK kind = sdk->kind;
            if (kind == FConstraintEdge::FCopy) {
                processCopyConstraintAddition(src, dst);
            }
            else if (kind == FConstraintEdge::FVariantGep) {
                processVariantGepConstraintAddition(src, dst);
            }
            else if (kind == FConstraintEdge::FNormalGep) {
                const AccessPath& ap = sdk->ap;
                processNormalGepConstraintAddition(src, dst, ap);
            }
            delete sdk;
        }

        // process propagate map
        for (auto it = insPropMap.begin(), eit = insPropMap.end(); it != eit; ++it) {
            NodeID dst = it->first;
            PointsTo& pts = it->second;
            bool sccflag = unFilterSet.find(dst) != unFilterSet.end();
            if (sccflag)
                unFilterSet.erase(dst);
            STAT_TIME(timeOfInsertionProp, propagateInsPts(pts, dst, sccflag));  
            insPropMap[dst].clear();
        }
        insPropMap.clear();

        // process simple and complex constraint
        while (!insEdgeVec.empty()) {
            SDK* sdk = insEdgeVec.back();
            insEdgeVec.pop_back();
            
            NodeID src = sdk->src;
            NodeID dst = sdk->dst;
            FConstraintEdge::FConstraintEdgeK kind = sdk->kind;
            // dumpSDK(src, dst, kind);
            if (kind == FConstraintEdge::FAddr) {
                processAddrAddition(src, dst);
                insEdgeCount ++;
            }
            // else if (kind == FConstraintEdge::FCopy) {
            //     processCopyConstraintAddition(src, dst);
            // }
            // else if (kind == FConstraintEdge::FVariantGep) {
            //     processVariantGepConstraintAddition(src, dst);
            // }
            // else if (kind == FConstraintEdge::FNormalGep) {
            //     const AccessPath& ap = sdk->ap;
            //     processNormalGepConstraintAddition(src, dst, ap);
            // }
            else if (kind == FConstraintEdge::FStore) {
                processStoreAddition(src, dst);
                insEdgeCount ++;
            }
            else if (kind == FConstraintEdge::FLoad) {
                processLoadAddition(src, dst);
                insEdgeCount ++;
            }
            delete sdk;
        }
        // SVFUtil::outs() << "Inserted Edge Num: " << insEdgeCount << "\n";

        double fpstart = stat->getClk();
        computeFpPDM();
        double fpend = stat->getClk();
        timeOfInsComputeFP += (fpend - fpstart) / TIMEINTERVAL;

        if (updateCallGraphIns(getIndirectCallsites()))
            newCallCopyEdge = true;
        if (!insDirectEdgeVec.empty())
            newCallCopyEdge = true;
        if (!insEdgeVec.empty())
            newCallCopyEdge = true;
    } while (newCallCopyEdge);
}

void AndersenInc::processInsertion_IPA()
{
    initFpPDM();
    bool newCallCopyEdge = false;
    bool needSCCDetect = false;

    // IPA 1: process complex edge to extract direct edge
    while (!insEdgeVec.empty()) {
            SDK* sdk = insEdgeVec.back();
            insEdgeVec.pop_back();
            
            NodeID src = sdk->src;
            NodeID dst = sdk->dst;
            FConstraintEdge::FConstraintEdgeK kind = sdk->kind;
            // dumpSDK(src, dst, kind);
            if (kind == FConstraintEdge::FAddr) {
                processAddrAddition(src, dst);
            }
            else if (kind == FConstraintEdge::FStore) {
                processStoreAddition(src, dst);
            }
            else if (kind == FConstraintEdge::FLoad) {
                processLoadAddition(src, dst);
            }
            delete sdk;
    }

    // IPA 2: process direct edge
    do {
        newCallCopyEdge = false;
        needSCCDetect = false;
        int insDirectEdgeCount = 0, insEdgeCount = 0;
        // process direct edge insertion
        
        while (!insDirectEdgeVec.empty()) {
            SDK* sdk = insDirectEdgeVec.back();
            insDirectEdgeVec.pop_back();
            insDirectEdgeCount ++;
            NodeID src = sdk->src;
            NodeID dst = sdk->dst;
            FConstraintEdge::FConstraintEdgeK kind = sdk->kind;
            FConstraintEdge* compEdge = sdk->compEdge;
            // dumpSDK(src, dst, kind);
            if (kind == FConstraintEdge::FCopy)
                processCopyAddition_IPA(src, dst, compEdge);
            else if (kind == FConstraintEdge::FVariantGep)
                processVariantGepAddition_IPA(src, dst);
            else if (kind == FConstraintEdge::FNormalGep) {
                const AccessPath& ap = sdk->ap;
                processNormalGepAddition_IPA(src, dst, ap);
            }
            delete sdk;
        }

        double fpstart = stat->getClk();
        computeFpPDM();
        double fpend = stat->getClk();
        timeOfInsComputeFP += (fpend - fpstart) / TIMEINTERVAL;

        if (updateCallGraphIns(getIndirectCallsites()))
            newCallCopyEdge = true;
        if (!insDirectEdgeVec.empty())
            newCallCopyEdge = true;
        if (!insEdgeVec.empty())
            newCallCopyEdge = true;
    } while (newCallCopyEdge);
}


void AndersenInc::processAddrAddition(NodeID srcid, NodeID dstid)
{
    fCG->addAddrFCGEdge(srcid, dstid);
    sCG->addAddrSCGEdge(srcid, dstid);

    const PointsTo& dstPts = getPts(dstid);
    if (!dstPts.test(srcid)) {
        PointsTo ptsChange;
        ptsChange.set(srcid);
        STAT_TIME(timeOfInsertionProp, propagateInsPts(ptsChange, dstid));      
    }
}
void AndersenInc::processAddrAddition_IPA(NodeID srcid, NodeID dstid)
{
    fCG->addAddrFCGEdge(srcid, dstid);
    sCG->addAddrSCGEdge(srcid, dstid);

    const PointsTo& dstPts = getPts(dstid);
    if (!dstPts.test(srcid)) {
        PointsTo ptsChange;
        ptsChange.set(srcid);
        STAT_TIME(timeOfInsertionProp, propagateInsPts_IPA(ptsChange, dstid));      
    }
}

bool AndersenInc::processLoadAddition(NodeID srcid, NodeID dstid)
{
    fCG->addLoadFCGEdge(srcid, dstid);
    sCG->addLoadSCGEdge(srcid, dstid);
    SConstraintNode* sSrcNode = sCG->getSConstraintNode(srcid);
    SConstraintNode* sDstNode = sCG->getSConstraintNode(dstid);
    SConstraintEdge* load = sCG->getEdge(sSrcNode, sDstNode, SConstraintEdge::SLoad);
    const PointsTo& srcPts = getPts(srcid);

    bool newSEdge = false;
    for (NodeID o: srcPts) {
        if (pag->isConstantObj(o) || isNonPointerObj(o))
            continue;
        for (auto it = load->getFEdgeSet().begin(), eit = load->getFEdgeSet().end(); 
            it != eit; ++it)
        {
            FConstraintEdge* fLoad = *it;
            
            if ( !(fLoad->getCompCache().test(o)) ) {
                NodeID loadDst = fLoad->getDstID();
                insDirectEdgeVec.push_back(new SDK(o, loadDst, FConstraintEdge::FCopy, fLoad));
            }
            // fCG->addCopyFCGEdge(o, loadDst, fLoad);
            // newSEdge |= (nullptr != sCG->addCopySCGEdge(o, loadDst));
            // insEdgeVec.push_back(
            //     new SDK(o, loadDst, FConstraintEdge::FCopy, fLoad));
        }
    }
    // if (newSEdge)
    //     SCCDetect();
    return newSEdge;
}

bool AndersenInc::processStoreAddition(NodeID srcid, NodeID dstid)
{
    fCG->addStoreFCGEdge(srcid, dstid);
    sCG->addStoreSCGEdge(srcid, dstid);
    SConstraintNode* sSrcNode = sCG->getSConstraintNode(srcid);
    SConstraintNode* sDstNode = sCG->getSConstraintNode(dstid);
    SConstraintEdge* store = sCG->getEdge(sSrcNode, sDstNode, SConstraintEdge::SStore);
    const PointsTo& dstPts = getPts(dstid);

    bool newSEdge = false;
    for (NodeID o: dstPts) {
        if (pag->isConstantObj(o) || isNonPointerObj(o))
            continue;
        for (auto it = store->getFEdgeSet().begin(), eit = store->getFEdgeSet().end(); 
            it != eit; ++it)
        {
            FConstraintEdge* fStore = *it;
            
            if ( !(fStore->getCompCache().test(o)) ) {
                NodeID storeSrc = fStore->getSrcID();
                insDirectEdgeVec.push_back(new SDK(storeSrc, o, FConstraintEdge::FCopy, fStore));
            }
            // fCG->addCopyFCGEdge(storeSrc, o, fStore);
            // newSEdge |= (nullptr != sCG->(storeSrc, o));
            // insEdgeVec.push_back(
            //     new SDK(storeSrc, o, FConstraintEdge::FCopy, fStore));
        }
    }
    // if (newSEdge)
    //     SCCDetect();
    return newSEdge;
}

bool AndersenInc::processCopyAddition(NodeID srcid, NodeID dstid, FConstraintEdge* complexEdge)
{
    bool newFEdge = fCG->addCopyFCGEdge(srcid, dstid, complexEdge);
    if (!newFEdge)
        return false;
    bool newSEdge = false;
    newSEdge |= (nullptr != sCG->addCopySCGEdge(srcid, dstid, true));
    if (newSEdge)
        insDirectConsVec.push_back(new SDK(srcid, dstid, FConstraintEdge::FCopy));
        // insEdgeVec.push_back(new SDK(srcid, dstid, FConstraintEdge::FCopy));
    return newSEdge;
}
bool AndersenInc::processCopyAddition_IPA(NodeID srcid, NodeID dstid, FConstraintEdge* complexEdge)
{
    bool newFEdge = fCG->addCopyFCGEdge(srcid, dstid, complexEdge);
    if (!newFEdge)
        return false;
    bool newSEdge = false;
    newSEdge |= (nullptr != sCG->addCopySCGEdge(srcid, dstid, true));
    if (newSEdge) {
        // insDirectConsVec.push_back(new SDK(srcid, dstid, FConstraintEdge::FCopy));
        double sbegin = stat->getClk();
        SCCDetect(); //2024.4 TODO
        double send = stat->getClk();
        timeOfInsertionSCC += (send - sbegin) / TIMEINTERVAL;
        timeOfSCCDetectionIPAIns += (send - sbegin) / TIMEINTERVAL;
        numOfSCCDetectionIPAIns++;
        processCopyConstraintAddition_IPA(srcid, dstid);
    }        
        // insEdgeVec.push_back(new SDK(srcid, dstid, FConstraintEdge::FCopy));
    return newSEdge;
}

bool AndersenInc::processVariantGepAddition(NodeID srcid, NodeID dstid)
{
    bool newFEdge = fCG->addVariantGepFCGEdge(srcid, dstid);
    if (!newFEdge)
        return false;
    bool newSEdge = false;
    newSEdge |= (nullptr != sCG->addVariantGepSCGEdge(srcid, dstid));
    if (newSEdge)
        insDirectConsVec.push_back(new SDK(srcid, dstid, FConstraintEdge::FVariantGep));
    return newSEdge;
}
bool AndersenInc::processVariantGepAddition_IPA(NodeID srcid, NodeID dstid)
{
    bool newFEdge = fCG->addVariantGepFCGEdge(srcid, dstid);
    if (!newFEdge)
        return false;
    bool newSEdge = false;
    newSEdge |= (nullptr != sCG->addVariantGepSCGEdge(srcid, dstid));
    if (newSEdge){
        double sbegin = stat->getClk();
        SCCDetect(); //2024.4 TODO
        double send = stat->getClk();
        timeOfInsertionSCC += (send - sbegin) / TIMEINTERVAL;
        timeOfSCCDetectionIPAIns += (send - sbegin) / TIMEINTERVAL;
        numOfSCCDetectionIPAIns++;
        processVariantGepConstraintAddition_IPA(srcid, dstid);
    }
    return newSEdge;
}

bool AndersenInc::processNormalGepAddition(NodeID srcid, NodeID dstid, const AccessPath& ap)
{
    bool newFEdge = fCG->addNormalGepFCGEdge(srcid, dstid, ap);
    if (!newFEdge)
        return false;
    bool newSEdge = false;
    newSEdge |= (nullptr != sCG->addNormalGepSCGEdge(srcid, dstid, ap));
    if (newSEdge) {
        insDirectConsVec.push_back(new SDK(srcid, dstid, FConstraintEdge::FNormalGep, ap));
    }
        
    return newSEdge;
}
bool AndersenInc::processNormalGepAddition_IPA(NodeID srcid, NodeID dstid, const AccessPath& ap)
{
    bool newFEdge = fCG->addNormalGepFCGEdge(srcid, dstid, ap);
    if (!newFEdge)
        return false;
    bool newSEdge = false;
    newSEdge |= (nullptr != sCG->addNormalGepSCGEdge(srcid, dstid, ap));
    if (newSEdge) {
        double sbegin = stat->getClk();
        SCCDetect(); //2024.4 TODO
        double send = stat->getClk();
        timeOfInsertionSCC += (send - sbegin) / TIMEINTERVAL;
        timeOfSCCDetectionIPAIns += (send - sbegin) / TIMEINTERVAL;
        numOfSCCDetectionIPAIns++;
        processNormalGepConstraintAddition_IPA(srcid, dstid, ap);
    }
        
    return newSEdge;
}

void AndersenInc::processCopyConstraintAddition(NodeID srcid, NodeID dstid)
{
    NodeID repdst = sccRepNode(dstid);
    PointsTo changePts = getPts(srcid);
    changePts = changePts - getPts(repdst);
    insPropMap[repdst] |= changePts;
    if (sccRepNode(srcid) == repdst)
        unFilterSet.insert(repdst);
    // STAT_TIME(timeOfInsertionProp, propagateInsPts(getPts(srcid), dstid, sccRepNode(srcid) == sccRepNode(dstid)));  
}
void AndersenInc::processCopyConstraintAddition_IPA(NodeID srcid, NodeID dstid)
{
    NodeID repdst = sccRepNode(dstid);
    PointsTo changePts = getPts(srcid);
    changePts = changePts - getPts(repdst);
    // insPropMap[repdst] |= changePts;
    STAT_TIME(timeOfInsertionProp, propagateInsPts_IPA(changePts,  repdst, sccRepNode(srcid) == repdst)); 
}

void AndersenInc::processVariantGepConstraintAddition(NodeID srcid, NodeID dstid)
{
    PointsTo ptsChange;
    for (NodeID o: getPts(srcid)) {
        if (sCG->isBlkObjOrConstantObj(o))
        {
            ptsChange.set(o);
            continue;
        }
        NodeID baseId = sCG->getFIObjVar(o);
        ptsChange.set(baseId);
    }
    NodeID repdst = sccRepNode(dstid);
    ptsChange = ptsChange - getPts(repdst);
    insPropMap[dstid] |= ptsChange;
    if (sccRepNode(srcid) == repdst)
        unFilterSet.insert(repdst);
    // STAT_TIME(timeOfInsertionProp, propagateInsPts(ptsChange, dstid, sccRepNode(srcid) == sccRepNode(dstid)));     
}

void AndersenInc::processVariantGepConstraintAddition_IPA(NodeID srcid, NodeID dstid)
{
    PointsTo ptsChange;
    for (NodeID o: getPts(srcid)) {
        if (sCG->isBlkObjOrConstantObj(o))
        {
            ptsChange.set(o);
            continue;
        }
        NodeID baseId = sCG->getFIObjVar(o);
        ptsChange.set(baseId);
    }
    NodeID repdst = sccRepNode(dstid);
    ptsChange = ptsChange - getPts(repdst);
    STAT_TIME(timeOfInsertionProp, propagateInsPts_IPA(ptsChange, repdst, sccRepNode(srcid) == repdst));
    // STAT_TIME(timeOfInsertionProp, propagateInsPts(ptsChange, dstid, sccRepNode(srcid) == sccRepNode(dstid)));     
}

void AndersenInc::processNormalGepConstraintAddition(NodeID srcid, NodeID dstid, const AccessPath& ap)
{
    PointsTo ptsChange;
    for (NodeID o: getPts(srcid)) {
        if (sCG->isBlkObjOrConstantObj(o) || isFieldInsensitive(o))
        {
            ptsChange.set(o);
            continue;
        }
        NodeID fieldSrcPtdNode = sCG->getGepObjVar(o, ap.getConstantFieldIdx());
        ptsChange.set(fieldSrcPtdNode);
    }
    dstid = sccRepNode(dstid);
    insPropMap[dstid] |= ptsChange;
    if (sccRepNode(srcid) == sccRepNode(dstid))
        unFilterSet.insert(dstid);
    // STAT_TIME(timeOfInsertionProp, propagateInsPts(ptsChange, dstid, sccRepNode(srcid) == sccRepNode(dstid)));     
}

void AndersenInc::processNormalGepConstraintAddition_IPA(NodeID srcid, NodeID dstid, const AccessPath& ap)
{
    PointsTo ptsChange;
    for (NodeID o: getPts(srcid)) {
        if (sCG->isBlkObjOrConstantObj(o) || isFieldInsensitive(o))
        {
            ptsChange.set(o);
            continue;
        }
        NodeID fieldSrcPtdNode = sCG->getGepObjVar(o, ap.getConstantFieldIdx());
        ptsChange.set(fieldSrcPtdNode);
    }
    NodeID repdst = sccRepNode(dstid);
    ptsChange = ptsChange - getPts(repdst);
    STAT_TIME(timeOfInsertionProp, propagateInsPts_IPA(ptsChange, repdst, sccRepNode(srcid) == repdst));
    // STAT_TIME(timeOfInsertionProp, propagateInsPts(ptsChange, dstid, sccRepNode(srcid) == sccRepNode(dstid)));     
}

void AndersenInc::propagateInsPts(const PointsTo& pts, NodeID nodeId, bool sameSCC)
{
    PointsTo oriPts, dPts;
    oriPts |= pts;
    dPts |= pts;
    
    if (!sameSCC) {
        // Filter 1: dPts = dPts - (dPts \intersect pts(nodeId))
        oriPts &= getPts(nodeId); // oriPts = dPts \intersect pts(nodeId)
        dPts -= oriPts; // dPts = dPts - oriPts
        if (dPts.empty()){
            return;
        }
    }
    
    insNodeNum ++;

    // pts(nodeId) = pts(nodeId) U dPts
    SConstraintNode* node = sCG->getSConstraintNode(nodeId);
    nodeId = node->getId();
    unionPts(nodeId, dPts); 

    // process outgoing neighbor propagate
    
    for (auto it = node->directOutEdgeBegin(), eit = node->directOutEdgeEnd();
        it != eit; ++it)
    {
        SConstraintEdge* outSEdge = *it;
        NodeID dstId = outSEdge->getDstID();
        if (dstId == nodeId)
            continue; // self circle edge
        if (SVFUtil::isa<CopySCGEdge>(outSEdge)) {
            propagateInsPts(dPts, dstId);
        }
        else if (SVFUtil::isa<VariantGepSCGEdge>(outSEdge)) {
            PointsTo vgepProPts;
            for (NodeID o: dPts) {
                if (sCG->isBlkObjOrConstantObj(o))
                    vgepProPts.set(o);
                else
                    vgepProPts.set(sCG->getFIObjVar(o));
            }
            propagateInsPts(vgepProPts, dstId);
        }
        else if (SVFUtil::isa<NormalGepSCGEdge>(outSEdge)) {
            const NormalGepSCGEdge* ngep = SVFUtil::dyn_cast<NormalGepSCGEdge>(outSEdge);
            PointsTo ngepProPts;
            for (NodeID o: dPts) {
                if (sCG->isBlkObjOrConstantObj(o) || isFieldInsensitive(o))
                    ngepProPts.set(o);
                else {
                    const AccessPath& ap = ngep->getAccessPath();
                    NodeID fieldSrcPtdNode = sCG->getGepObjVar(o, ap.getConstantFieldIdx());
                    ngepProPts.set(fieldSrcPtdNode);
                }
            }
            propagateInsPts(ngepProPts, dstId);
        }
    }

    // process complex constraint
    // bool newSEdge = false;
    for (NodeID o: dPts) {
        if (pag->isConstantObj(o) || isNonPointerObj(o))
            continue;

        // load complex constraint
        for (auto it = node->outgoingLoadsBegin(), eit = node->outgoingLoadsEnd();
            it != eit; ++it)
        {
            const SConstraintEdge* sLoad = *it;
            for (FConstraintEdge* fLoad: sLoad->getFEdgeSet()) 
            {
                if ( !(fLoad->getCompCache().test(o)) ) {
                    NodeID loadDst = fLoad->getDstID();
                    insDirectEdgeVec.push_back(new SDK(o, loadDst, FConstraintEdge::FCopy, fLoad));
                }
                // fCG->addCopyFCGEdge(o, loadDst, fLoad);
                // newSEdge |= (nullptr != sCG->addCopySCGEdge(o, loadDst));
                // insEdgeVec.push_back(
                //     new SDK(o, loadDst, FConstraintEdge::FCopy, fLoad));
            }
        }

        // store complex constraint
        for (auto it = node->incomingStoresBegin(), eit = node->incomingStoresEnd();
            it != eit; ++it)
        {
            const SConstraintEdge* sStore = *it;
            for (FConstraintEdge* fStore: sStore->getFEdgeSet()) 
            {
                if ( !(fStore->getCompCache().test(o)) ) {
                    NodeID storeSrc = fStore->getSrcID();
                    insDirectEdgeVec.push_back(new SDK(storeSrc, o, FConstraintEdge::FCopy, fStore));
                }
                // fCG->addCopyFCGEdge(storeSrc, o, fStore);
                // newSEdge |= (nullptr != sCG->addCopySCGEdge(storeSrc, o));
                // insEdgeVec.push_back(
                //     new SDK(storeSrc, o, FConstraintEdge::FCopy, fStore));
            }
        }
    }
    // if (newSEdge)
        // SCCDetect();
}

void AndersenInc::propagateInsPts_IPA(const PointsTo& pts, NodeID nodeId, bool sameSCC)
{
    PointsTo oriPts, dPts;
    oriPts |= pts;
    dPts |= pts;
    
    if (!sameSCC) {
        // Filter 1: dPts = dPts - (dPts \intersect pts(nodeId))
        oriPts &= getPts(nodeId); // oriPts = dPts \intersect pts(nodeId)
        dPts -= oriPts; // dPts = dPts - oriPts
        if (dPts.empty()){
            return;
        }
    }
    
    insNodeNum ++;

    // pts(nodeId) = pts(nodeId) U dPts
    SConstraintNode* node = sCG->getSConstraintNode(nodeId);
    nodeId = node->getId();
    unionPts(nodeId, dPts); 

    // process outgoing neighbor propagate
    
    for (auto it = node->directOutEdgeBegin(), eit = node->directOutEdgeEnd();
        it != eit; ++it)
    {
        SConstraintEdge* outSEdge = *it;
        NodeID dstId = outSEdge->getDstID();
        if (dstId == nodeId)
            continue; // self circle edge
        if (SVFUtil::isa<CopySCGEdge>(outSEdge)) {
            propagateInsPts_IPA(dPts, dstId);
        }
        else if (SVFUtil::isa<VariantGepSCGEdge>(outSEdge)) {
            PointsTo vgepProPts;
            for (NodeID o: dPts) {
                if (sCG->isBlkObjOrConstantObj(o))
                    vgepProPts.set(o);
                else
                    vgepProPts.set(sCG->getFIObjVar(o));
            }
            propagateInsPts_IPA(vgepProPts, dstId);
        }
        else if (SVFUtil::isa<NormalGepSCGEdge>(outSEdge)) {
            const NormalGepSCGEdge* ngep = SVFUtil::dyn_cast<NormalGepSCGEdge>(outSEdge);
            PointsTo ngepProPts;
            for (NodeID o: dPts) {
                if (sCG->isBlkObjOrConstantObj(o) || isFieldInsensitive(o))
                    ngepProPts.set(o);
                else {
                    const AccessPath& ap = ngep->getAccessPath();
                    NodeID fieldSrcPtdNode = sCG->getGepObjVar(o, ap.getConstantFieldIdx());
                    ngepProPts.set(fieldSrcPtdNode);
                }
            }
            propagateInsPts_IPA(ngepProPts, dstId);
        }
    }

    // process complex constraint
    // bool newSEdge = false;
    for (NodeID o: dPts) {
        if (pag->isConstantObj(o) || isNonPointerObj(o))
            continue;

        // load complex constraint
        for (auto it = node->outgoingLoadsBegin(), eit = node->outgoingLoadsEnd();
            it != eit; ++it)
        {
            const SConstraintEdge* sLoad = *it;
            for (FConstraintEdge* fLoad: sLoad->getFEdgeSet()) 
            {
                if ( !(fLoad->getCompCache().test(o)) ) {
                    NodeID loadDst = fLoad->getDstID();
                    insDirectEdgeVec.push_back(new SDK(o, loadDst, FConstraintEdge::FCopy, fLoad));
                }
                // fCG->addCopyFCGEdge(o, loadDst, fLoad);
                // newSEdge |= (nullptr != sCG->addCopySCGEdge(o, loadDst));
                // insEdgeVec.push_back(
                //     new SDK(o, loadDst, FConstraintEdge::FCopy, fLoad));
            }
        }

        // store complex constraint
        for (auto it = node->incomingStoresBegin(), eit = node->incomingStoresEnd();
            it != eit; ++it)
        {
            const SConstraintEdge* sStore = *it;
            for (FConstraintEdge* fStore: sStore->getFEdgeSet()) 
            {
                if ( !(fStore->getCompCache().test(o)) ) {
                    NodeID storeSrc = fStore->getSrcID();
                    insDirectEdgeVec.push_back(new SDK(storeSrc, o, FConstraintEdge::FCopy, fStore));
                }
                // fCG->addCopyFCGEdge(storeSrc, o, fStore);
                // newSEdge |= (nullptr != sCG->addCopySCGEdge(storeSrc, o));
                // insEdgeVec.push_back(
                //     new SDK(storeSrc, o, FConstraintEdge::FCopy, fStore));
            }
        }
    }
    // if (newSEdge)
        // SCCDetect();
}

bool AndersenInc::updateCallGraphIns(const CallSiteToFunPtrMap& callsites)
{

    double cgUpdateStart = stat->getClk();

    CallEdgeMap newEdges;
    onTheFlyCallGraphSolveIns(callsites,newEdges);
    NodePairSet cpySrcNodes;	/// nodes as a src of a generated new copy edge
    for(CallEdgeMap::iterator it = newEdges.begin(), eit = newEdges.end(); it!=eit; ++it )
    {
        CallSite cs = SVFUtil::getSVFCallSite(it->first->getCallSite());
        for(FunctionSet::iterator cit = it->second.begin(), ecit = it->second.end(); cit!=ecit; ++cit)
        {
            connectCaller2CalleeParamsIns(cs,*cit,cpySrcNodes);
        }
    }
    // for(NodePairSet::iterator it = cpySrcNodes.begin(), eit = cpySrcNodes.end(); it!=eit; ++it)
    // {
    //     pushIntoWorklist(it->first);
    // }

    double cgUpdateEnd = stat->getClk();
    // timeOfUpdateCallGraph += (cgUpdateEnd - cgUpdateStart) / TIMEINTERVAL;
    timeOfUpdateCallGraph += (cgUpdateEnd - cgUpdateStart) / TIMEINTERVAL;

    return (!newEdges.empty());
}

void AndersenInc::onTheFlyCallGraphSolveIns(const CallSiteToFunPtrMap& callsites, CallEdgeMap& newEdges)
{
    for(CallSiteToFunPtrMap::const_iterator iter = callsites.begin(), eiter = callsites.end(); 
        iter != eiter; ++iter)
    {
        const CallICFGNode* cs = iter->first;

        if (SVFUtil::getSVFCallSite(cs->getCallSite()).isVirtualCall())
        {
            const SVFValue *vtbl = SVFUtil::getSVFCallSite(cs->getCallSite()).getVtablePtr();
            assert(pag->hasValueNode(vtbl));
            NodeID vtblId = pag->getValueNode(vtbl);
            PtsDiff* pd = fpPtsDiffMap[vtblId];
            if (pd->insPts.empty())
                continue;   // if no pts changed, then do nothing
            // resolveCPPIndCallsIns(cs, getPts(vtblId), newEdges);
            const PointsTo & insPts = pd->insPts;
            resolveCPPIndCallsIns(cs, insPts, newEdges);
        }
        else {
            PtsDiff* pd = fpPtsDiffMap[iter->second];
            if (pd->insPts.empty())
                continue;   // if no pts changed, then do nothing
            const PointsTo & insPts = pd->insPts;
            // resolveIndCallsIns(iter->first,getPts(iter->second),newEdges);
            resolveIndCallsIns(iter->first, insPts, newEdges);
        }
    }
}

void AndersenInc::resolveCPPIndCallsIns(const CallICFGNode* cs, const PointsTo& target, CallEdgeMap& newEdges)
{
    assert(SVFUtil::getSVFCallSite(cs->getCallSite()).isVirtualCall() && "not cpp virtual call");

    VFunSet vfns;
    if (Options::ConnectVCallOnCHA())
        getVFnsFromCHA(cs, vfns);
    else
        getVFnsFromPts(cs, target, vfns);
    connectVCallToVFnsIns(cs, vfns, newEdges); // TODO: handle indirect call edge?
}

void AndersenInc::resolveIndCallsIns(const CallICFGNode* cs, const PointsTo& target, CallEdgeMap& newEdges)
 {
    assert(pag->isIndirectCallSites(cs) && "not an indirect callsite?");
    /// discover indirect pointer target
     for (PointsTo::iterator ii = target.begin(), ie = target.end();
            ii != ie; ii++)
    {

        if(getNumOfResolvedIndCallEdge() >= Options::IndirectCallLimit())
        {
            wrnMsg("Resolved Indirect Call Edges are Out-Of-Budget, please increase the limit");
            return;
        }

        if(ObjVar* objPN = SVFUtil::dyn_cast<ObjVar>(pag->getGNode(*ii)))
        {
            const MemObj* obj = pag->getObject(objPN);

            if(obj->isFunction())
            {
                const SVFFunction* calleefun = SVFUtil::cast<SVFFunction>(obj->getValue());
                const SVFFunction* callee = calleefun->getDefFunForMultipleModule();

                /// if the arg size does not match then we do not need to connect this parameter
                /// even if the callee is a variadic function (the first parameter of variadic function is its paramter number)
                if(SVFUtil::matchArgs(cs->getCallSite(), callee) == false)
                    continue;

                if(0 == getIndCallMap()[cs].count(callee))
                {
                    newEdges[cs].insert(callee);
                    getIndCallMap()[cs].insert(callee);

                    ptaCallGraph->addIndirectCallGraphEdge(cs, cs->getCaller(), callee);
                    // FIXME: do we need to update llvm call graph here?
                    // The indirect call is maintained by ourself, We may update llvm's when we need to
                    //CallGraphNode* callgraphNode = callgraph->getOrInsertFunction(cs.getCaller());
                    //callgraphNode->addCalledFunction(cs,callgraph->getOrInsertFunction(callee));
                }
            }
        }
    }   
 }

void AndersenInc::connectVCallToVFnsIns(const CallICFGNode* cs, const VFunSet &vfns, CallEdgeMap& newEdges)
{
    //// connect all valid functions
    for (VFunSet::const_iterator fit = vfns.begin(),
            feit = vfns.end(); fit != feit; ++fit)
    {
        const SVFFunction* callee = *fit;
        callee = callee->getDefFunForMultipleModule();
        if (getIndCallMap()[cs].count(callee) > 0)
            continue;
        if(SVFUtil::getSVFCallSite(cs->getCallSite()).arg_size() == callee->arg_size() ||
                (SVFUtil::getSVFCallSite(cs->getCallSite()).isVarArg() && callee->isVarArg()))
        {
            newEdges[cs].insert(callee);
            getIndCallMap()[cs].erase(callee);
            const CallICFGNode* callBlockNode = pag->getICFG()->getCallICFGNode(cs->getCallSite());
            ptaCallGraph->addIndirectCallGraphEdge(callBlockNode, cs->getCaller(),callee);
        }
    }
}

void AndersenInc::connectCaller2CalleeParamsIns(CallSite cs, const SVFFunction* F, NodePairSet &cpySrcNodes)
{
    assert(F);

    DBOUT(DAndersen, SVFUtil::outs() << "connect parameters from indirect callsite " << *cs.getInstruction() << " to callee " << *F << "\n");

    CallICFGNode* callBlockNode = pag->getICFG()->getCallICFGNode(cs.getInstruction());
    RetICFGNode* retBlockNode = pag->getICFG()->getRetICFGNode(cs.getInstruction());

    if(SVFUtil::isHeapAllocExtFunViaRet(F) && pag->callsiteHasRet(retBlockNode))
    {
        // heapAllocatorViaIndCall(cs,cpySrcNodes); // TODO 
        SVFUtil::outs() << "Do not handle heapAllocatorViaIndCall in incremental analysis!\n";
    }

    if (pag->funHasRet(F) && pag->callsiteHasRet(retBlockNode))
    {
        const PAGNode* cs_return = pag->getCallSiteRet(retBlockNode);
        const PAGNode* fun_return = pag->getFunRet(F);
        if (cs_return->isPointer() && fun_return->isPointer())
        {
            NodeID dstrec = cs_return->getId();
            NodeID srcret = fun_return->getId();
            // if(addCopyEdge(srcret, dstrec)) // TODO: ret-->rec
            // {
            //     cpySrcNodes.insert(std::make_pair(srcret,dstrec));
            // }
            // pushIntoDelEdgesWL(srcret, dstrec, FConstraintEdge::FCopy);
            insDirectEdgeVec.push_back(
                new SDK(srcret, dstrec, FConstraintEdge::FCopy)
            );
        }
        else
        {
            DBOUT(DAndersen, SVFUtil::outs() << "not a pointer ignored\n");
        }
    }

    if (pag->hasCallSiteArgsMap(callBlockNode) && pag->hasFunArgsList(F))
    {

        // connect actual and formal param
        const SVFIR::SVFVarList& csArgList = pag->getCallSiteArgsList(callBlockNode);
        const SVFIR::SVFVarList& funArgList = pag->getFunArgsList(F);
        //Go through the fixed parameters.
        DBOUT(DPAGBuild, outs() << "      args:");
        SVFIR::SVFVarList::const_iterator funArgIt = funArgList.begin(), funArgEit = funArgList.end();
        SVFIR::SVFVarList::const_iterator csArgIt  = csArgList.begin(), csArgEit = csArgList.end();
        for (; funArgIt != funArgEit; ++csArgIt, ++funArgIt)
        {
            //Some programs (e.g. Linux kernel) leave unneeded parameters empty.
            if (csArgIt  == csArgEit)
            {
                DBOUT(DAndersen, SVFUtil::outs() << " !! not enough args\n");
                break;
            }
            const PAGNode *cs_arg = *csArgIt ;
            const PAGNode *fun_arg = *funArgIt;

            if (cs_arg->isPointer() && fun_arg->isPointer())
            {
                DBOUT(DAndersen, SVFUtil::outs() << "process actual parm  " << cs_arg->toString() << " \n");
                NodeID srcAA = cs_arg->getId();
                NodeID dstFA = fun_arg->getId();
                // if(addCopyEdge(srcAA, dstFA)) // TODO: act arg --> fom arg
                // {
                //     cpySrcNodes.insert(std::make_pair(srcAA,dstFA));
                // }
                // pushIntoDelEdgesWL(srcAA, dstFA, FConstraintEdge::FCopy);
                insDirectEdgeVec.push_back(
                    new SDK(srcAA, dstFA, FConstraintEdge::FCopy)
                );
            }
        }

        //Any remaining actual args must be varargs.
        if (F->isVarArg())
        {
            NodeID vaF = pag->getVarargNode(F);
            DBOUT(DPAGBuild, SVFUtil::outs() << "\n      varargs:");
            for (; csArgIt != csArgEit; ++csArgIt)
            {
                const PAGNode *cs_arg = *csArgIt;
                if (cs_arg->isPointer())
                {
                    NodeID vnAA = cs_arg->getId();
                    // if (addCopyEdge(vnAA,vaF)) // TODO: remaining actual arg --> var arg
                    // {
                    //     cpySrcNodes.insert(std::make_pair(vnAA,vaF));
                    // }
                    // pushIntoDelEdgesWL(vnAA, vaF, FConstraintEdge::FCopy);
                    insDirectEdgeVec.push_back(
                        new SDK(vnAA, vaF, FConstraintEdge::FCopy)
                    );
                }
            }
        }
        if(csArgIt != csArgEit)
        {
            writeWrnMsg("too many args to non-vararg func.");
            writeWrnMsg("(" + cs.getInstruction()->getSourceLoc() + ")");
        }
    }
}

void AndersenInc::dumpSDK(NodeID src, NodeID dst, FConstraintEdge::FConstraintEdgeK kind)
{
    std::string kstr;
    if (kind == FConstraintEdge::FCopy)
        kstr = "Copy";
    else if (kind == FConstraintEdge::FAddr)
        kstr = "Addr";
    else if (kind == FConstraintEdge::FLoad)
        kstr = "Load";
    else if (kind == FConstraintEdge::FStore)
        kstr = "Store";
    else if (kind == FConstraintEdge::FVariantGep)
        kstr = "VGep";
    else if (kind == FConstraintEdge::FNormalGep)
        kstr = "NGep";
    SVFUtil::outs() << "Edge: " << src << " --" << kstr << "--> " << dst << "\n";
}

void AndersenInc::singleIncremental(NodeID srcid, NodeID dstid, FConstraintEdge::FConstraintEdgeK kind, int count)
{
    delEdgesWL.clear();
    insEdgeVec.clear();
    insDirectEdgeVec.clear();
    delEdgeVec.clear();
    delDirectEdgeVec.clear();
    // pushIntoDelEdgesWL(srcid, dstid, kind); -- newwl
    if (kind == FConstraintEdge::FLoad || kind == FConstraintEdge::FStore) {
        delEdgeVec.push_back(new SDK(srcid, dstid, kind));
        insEdgeVec.push_back(new SDK(srcid, dstid, kind));
    }
    else {
        delDirectEdgeVec.push_back(new SDK(srcid, dstid, kind));
        insDirectEdgeVec.push_back(new SDK(srcid, dstid, kind));
    }
    SVFUtil::outs() << "========== Incremental Analysis Round [" << count << "] ==========\n";
    dumpSDK(srcid, dstid, kind);
    
    double iptaStart = stat->getClk();
    // processDeletion(); -- newwl
    if (Options::IPA())
        processDeletion_EdgeConstraint_IPA();
    else
        processDeletion_EdgeConstraint_Lazy();
    // processDeletion_EdgeConstraint();
    double iptaMid = stat->getClk();
    timeOfDeletionPTA += (iptaMid - iptaStart) / TIMEINTERVAL;
    // computeAllPDM();//
    SVFUtil::outs() << "Time of deletion PTA: " << (iptaMid - iptaStart) / TIMEINTERVAL << "\n";
    // 
    if (Options::Pseudo())
        processInsertion_pseudo();
    else
        processInsertion();
    double iptaEnd = stat->getClk();

    timeOfInsertionPTA += (iptaEnd - iptaMid) / TIMEINTERVAL;
    SVFUtil::outs() << "Time of insertion PTA: " << (iptaEnd - iptaMid) / TIMEINTERVAL << "\n";

    timeOfIncrementalPTA += (iptaEnd - iptaStart) / TIMEINTERVAL;
    SVFUtil::outs() << "Time of incremental PTA: " << (iptaEnd - iptaStart) / TIMEINTERVAL << "\n";
    
    // normalizePointsTo();
    computeAllPDM();
}