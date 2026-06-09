//===- MemRegion.cpp -- Memory region-----------------------------------------//
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
//===----------------------------------------------------------------------===//

/*
 * MemRegion.cpp
 *
 *  Created on: Dec 14, 2013
 *      Author: Yulei Sui
 */

#include "SVFIR/SVFValue.h"
#include "Util/Options.h"
#include "SVFIR/SVFModule.h"
#include "MSSA/MemRegion.h"
#include "MSSA/MSSAMuChi.h"
#include "Graphs/SVFGStat.h"
#include "WPA/AndersenInc.h"

using namespace SVF;
using namespace SVFUtil;

u32_t MemRegion::totalMRNum = 0;
u32_t MRVer::totalVERNum = 0;

double MRGenerator::timeOfCollectGlobals = 0;
double MRGenerator::timeOfCollectModRefForLoadStore = 0;
double MRGenerator::timeOfCollectModRefForCall = 0;
double MRGenerator::timeOfPartitionMRs = 0;
double MRGenerator::timeOfUpdateAliasMRs = 0;
double MRGenerator::timeOfCollectCallSitePts = 0;
double MRGenerator::timeOfModRefAnalysis = 0;

MRGenerator::MRGenerator(BVDataPTAImpl* p, bool ptrOnly) :
    pta(p), ptrOnlyMSSA(ptrOnly)
{
    callGraph = pta->getPTACallGraph();
    callGraphSCC = new SCC(callGraph);
}

/*!
 * Clean up memory
 */
void MRGenerator::destroy()
{

    for (MRSet::iterator it = memRegSet.begin(), eit = memRegSet.end();
            it != eit; ++it)
    {
        delete *it;
    }

    delete callGraphSCC;
    callGraphSCC = nullptr;
    callGraph = nullptr;
    pta = nullptr;
}

/*!
 * Generate a memory region and put in into functions which use it
 */
void MRGenerator::createMR(const SVFFunction* fun, const NodeBS& cpts)
{
    const NodeBS& repCPts = getRepPointsTo(cpts);
    MemRegion mr(repCPts);
    MRSet::const_iterator mit = memRegSet.find(&mr);
    if(mit!=memRegSet.end())
    {
        const MemRegion* mr = *mit;
        MRSet& mrs = funToMRsMap[fun];
        if(mrs.find(mr)==mrs.end())
            mrs.insert(mr);
    }
    else
    {
        MemRegion* m = new MemRegion(repCPts);
        memRegSet.insert(m);
        funToMRsMap[fun].insert(m);
    }
}

/*!
 * Generate a memory region and put in into functions which use it
 */
const MemRegion* MRGenerator::getMR(const NodeBS& cpts) const
{
    MemRegion mr(getRepPointsTo(cpts));
    MRSet::iterator mit = memRegSet.find(&mr);
    assert(mit!=memRegSet.end() && "memory region not found!!");
    return *mit;
}


/*!
 * Collect globals for escape analysis
 */
void MRGenerator::collectGlobals()
{
    SVFIR* pag = pta->getPAG();
    for (SVFIR::iterator nIter = pag->begin(); nIter != pag->end(); ++nIter)
    {
        if(ObjVar* obj = SVFUtil::dyn_cast<ObjVar>(nIter->second))
        {
            if (obj->getMemObj()->isGlobalObj())
            {
                allGlobals.set(nIter->first);
                allGlobals |= CollectPtsChain(nIter->first);
            }
        }
    }
}

/*!
 * Generate memory regions according to pointer analysis results
 * Attach regions on loads/stores
 */
void MRGenerator::generateMRs_exh_step1(MemSSAStat* _stat)
{
    MemRegion::resetMRNum();
    stat = _stat;

    DBOUT(DGENERAL, outs() << pasMsg("Generate Memory Regions \n"));

    double cgStart = stat->getClk(true);
    collectGlobals();
    double cgEnd = stat->getClk(true);
    timeOfCollectGlobals += (cgEnd - cgStart)/TIMEINTERVAL;

    callGraphSCC->find();

    DBOUT(DGENERAL, outs() << pasMsg("\tCollect ModRef For Load/Store \n"));

    /// collect mod-ref for loads/stores
    double lsStart = stat->getClk(true);
    collectModRefForLoadStore();
    double lsEnd = stat->getClk(true);
    timeOfCollectModRefForLoadStore += (lsEnd - lsStart)/TIMEINTERVAL;

    DBOUT(DGENERAL, outs() << pasMsg("\tCollect ModRef For const CallICFGNode*\n"));

    /// collect mod-ref for calls
    double mfcStart = stat->getClk(true);
    collectModRefForCall();
    double mfcEnd = stat->getClk(true);
    timeOfCollectModRefForCall += (mfcEnd - mfcStart)/TIMEINTERVAL;

    if (Options::wjydebug()) {
        debugdump();
    }
    saveAfterSnapshot();
}

void MRGenerator::saveAfterSnapshot()
{
    funToRefsMap_snapshot = funToRefsMap;
    funToModsMap_snapshot = funToModsMap;
    csToRefsMap_snapshot = csToRefsMap;
    csToModsMap_snapshot = csToModsMap;
    csToCallSiteArgsPtsMap_snapshot = csToCallSiteArgsPtsMap;
    csToCallSiteRetPtsMap_snapshot = csToCallSiteRetPtsMap;
    allGlobals_snapshot = allGlobals;
}

void MRGenerator::restoreAfterSnapshot()
{
    funToRefsMap = funToRefsMap_snapshot;
    funToModsMap = funToModsMap_snapshot;
    csToRefsMap = csToRefsMap_snapshot;
    csToModsMap = csToModsMap_snapshot;
    csToCallSiteArgsPtsMap = csToCallSiteArgsPtsMap_snapshot;
    csToCallSiteRetPtsMap = csToCallSiteRetPtsMap_snapshot;
    allGlobals = allGlobals_snapshot;
}

void MRGenerator::generateMRs_exh_step2(MemSSAStat* _stat)
{
    stat = _stat;
    DBOUT(DGENERAL, outs() << pasMsg("\tPartition Memory Regions \n"));
    /// Partition memory regions
    double pmrStart = stat->getClk(true);
    partitionMRs();
    double pmrEnd = stat->getClk(true);
    timeOfPartitionMRs += (pmrEnd - pmrStart)/TIMEINTERVAL;
    /// attach memory regions for loads/stores/calls

    double uaStart = stat->getClk(true);
    updateAliasMRs();
    double uaEnd = stat->getClk(true);
    timeOfUpdateAliasMRs += (uaEnd - uaStart)/TIMEINTERVAL;
}

void MRGenerator::generateMRs(MemSSAStat* _stat)
{
    MemRegion::resetMRNum();
    stat = _stat;

    DBOUT(DGENERAL, outs() << pasMsg("Generate Memory Regions \n"));

    double cgStart = stat->getClk(true);
    collectGlobals();
    double cgEnd = stat->getClk(true);
    timeOfCollectGlobals += (cgEnd - cgStart)/TIMEINTERVAL;

    callGraphSCC->find();

    DBOUT(DGENERAL, outs() << pasMsg("\tCollect ModRef For Load/Store \n"));

    /// collect mod-ref for loads/stores
    double lsStart = stat->getClk(true);
    collectModRefForLoadStore();
    double lsEnd = stat->getClk(true);
    timeOfCollectModRefForLoadStore += (lsEnd - lsStart)/TIMEINTERVAL;

    DBOUT(DGENERAL, outs() << pasMsg("\tCollect ModRef For const CallICFGNode*\n"));

    /// collect mod-ref for calls
    double mfcStart = stat->getClk(true);
    collectModRefForCall();
    double mfcEnd = stat->getClk(true);
    timeOfCollectModRefForCall += (mfcEnd - mfcStart)/TIMEINTERVAL;

    DBOUT(DGENERAL, outs() << pasMsg("\tPartition Memory Regions \n"));
    /// Partition memory regions
    double pmrStart = stat->getClk(true);
    partitionMRs();
    double pmrEnd = stat->getClk(true);
    timeOfPartitionMRs += (pmrEnd - pmrStart)/TIMEINTERVAL;
    /// attach memory regions for loads/stores/calls

    double uaStart = stat->getClk(true);
    updateAliasMRs();
    double uaEnd = stat->getClk(true);
    timeOfUpdateAliasMRs += (uaEnd - uaStart)/TIMEINTERVAL;
}

bool MRGenerator::hasSVFStmtList(const SVFInstruction* inst)
{
    SVFIR* pag = pta->getPAG();
    if (ptrOnlyMSSA)
        return pag->hasPTASVFStmtList(pag->getICFG()->getICFGNode(inst));
    else
        return pag->hasSVFStmtList(pag->getICFG()->getICFGNode(inst));
}

SVFIR::SVFStmtList& MRGenerator::getPAGEdgesFromInst(const SVFInstruction* inst)
{
    SVFIR* pag = pta->getPAG();
    if (ptrOnlyMSSA)
        return pag->getPTASVFStmtList(pag->getICFG()->getICFGNode(inst));
    else
        return pag->getSVFStmtList(pag->getICFG()->getICFGNode(inst));
}

/*!
 * Generate memory regions for loads/stores
 */
void MRGenerator::collectModRefForLoadStore()
{

    SVFModule* svfModule = pta->getModule();
    for (SVFModule::const_iterator fi = svfModule->begin(), efi = svfModule->end(); fi != efi;
            ++fi)
    {
        const SVFFunction& fun = **fi;

        /// if this function does not have any caller, then we do not care its MSSA
        if (Options::IgnoreDeadFun() && fun.isUncalledFunction())
            continue;

        for (SVFFunction::const_iterator iter = fun.begin(), eiter = fun.end();
                iter != eiter; ++iter)
        {
            const SVFBasicBlock* bb = *iter;
            for (SVFBasicBlock::const_iterator bit = bb->begin(), ebit = bb->end();
                    bit != ebit; ++bit)
            {
                const SVFInstruction* svfInst = *bit;
                SVFStmtList& pagEdgeList = getPAGEdgesFromInst(svfInst);
                for (SVFStmtList::iterator bit = pagEdgeList.begin(), ebit =
                            pagEdgeList.end(); bit != ebit; ++bit)
                {
                    const PAGEdge* inst = *bit;
                    pagEdgeToFunMap[inst] = &fun;
                    if (const StoreStmt *st = SVFUtil::dyn_cast<StoreStmt>(inst))
                    {
                        NodeBS cpts(pta->getPts(st->getLHSVarID()).toNodeBS());
                        // TODO: change this assertion check later when we have conditional points-to set
                        if (cpts.empty())
                            continue;
                        assert(!cpts.empty() && "null pointer!!");
                        addCPtsToStore(cpts, st, &fun);
                    }

                    else if (const LoadStmt *ld = SVFUtil::dyn_cast<LoadStmt>(inst))
                    {
                        NodeBS cpts(pta->getPts(ld->getRHSVarID()).toNodeBS());
                        // TODO: change this assertion check later when we have conditional points-to set
                        if (cpts.empty())
                            continue;
                        assert(!cpts.empty() && "null pointer!!");
                        addCPtsToLoad(cpts, ld, &fun);
                    }
                }
            }
        }
        // copy the map_ls to map
        funToPointsToMap[&fun] = funToPointsToMap_ls[&fun];
    }
}

void MRGenerator::collectModRefForLoadStore_RR()
{
    SVFModule* svfModule = pta->getModule();
    for (SVFModule::const_iterator fi = svfModule->begin(), efi = svfModule->end(); fi != efi;
            ++fi)
    {
        const SVFFunction& fun = **fi;
        NodeID funid = callGraph->getCallGraphNode(&fun)->getId();
        if (!resetFunctions.test(funid))
            continue; // do not handle unchanged func

        /// if this function does not have any caller, then we do not care its MSSA
        if (Options::IgnoreDeadFun() && fun.isUncalledFunction())
            continue;

        for (SVFFunction::const_iterator iter = fun.begin(), eiter = fun.end();
                iter != eiter; ++iter)
        {
            const SVFBasicBlock* bb = *iter;
            for (SVFBasicBlock::const_iterator bit = bb->begin(), ebit = bb->end();
                    bit != ebit; ++bit)
            {
                const SVFInstruction* svfInst = *bit;
                SVFStmtList& pagEdgeList = getPAGEdgesFromInst(svfInst);
                for (SVFStmtList::iterator bit = pagEdgeList.begin(), ebit =
                            pagEdgeList.end(); bit != ebit; ++bit)
                {
                    const PAGEdge* inst = *bit;
                    pagEdgeToFunMap[inst] = &fun;
                    if (const StoreStmt *st = SVFUtil::dyn_cast<StoreStmt>(inst))
                    {
                        NodeBS cpts(pta->getPts(st->getLHSVarID()).toNodeBS());
                        // TODO: change this assertion check later when we have conditional points-to set
                        if (cpts.empty())
                            continue;
                        assert(!cpts.empty() && "null pointer!!");
                        addCPtsToStore(cpts, st, &fun);
                    }

                    else if (const LoadStmt *ld = SVFUtil::dyn_cast<LoadStmt>(inst))
                    {
                        NodeBS cpts(pta->getPts(ld->getRHSVarID()).toNodeBS());
                        // TODO: change this assertion check later when we have conditional points-to set
                        if (cpts.empty())
                            continue;
                        assert(!cpts.empty() && "null pointer!!");
                        addCPtsToLoad(cpts, ld, &fun);
                    }
                }
            }
        }
        // copy the map_ls to map
        funToPointsToMap[&fun] = funToPointsToMap_ls[&fun];
    }
}

/*!
 * Update memory regions for loads/stores
 */
void MRGenerator::initChangedFunctions()
{

    SVFModule* svfModule = incpta->getModule();

    // Collect store changed functions
    for (SVFModule::const_iterator fi = svfModule->begin(), efi = svfModule->end(); fi != efi;
            ++fi)
    {
        bool nextFunc = false;
        const SVFFunction& fun = **fi;

        /// if this function does not have any caller, then we do not care its MSSA
        if (Options::IgnoreDeadFun() && fun.isUncalledFunction())
            continue;

        for (SVFFunction::const_iterator iter = fun.begin(), eiter = fun.end();
                iter != eiter; ++iter)
        {
            const SVFBasicBlock* bb = *iter;
            for (SVFBasicBlock::const_iterator bit = bb->begin(), ebit = bb->end();
                    bit != ebit; ++bit)
            {
                const SVFInstruction* svfInst = *bit;
                SVFStmtList& pagEdgeList = getPAGEdgesFromInst(svfInst);
                for (SVFStmtList::iterator bit = pagEdgeList.begin(), ebit =
                            pagEdgeList.end(); bit != ebit; ++bit)
                {
                    const PAGEdge* inst = *bit;
                    // pagEdgeToFunMap[inst] = &fun;
                    if (const StoreStmt *st = SVFUtil::dyn_cast<StoreStmt>(inst))
                    {
                        NodeID id = st->getLHSVarID();
                        if (incpta->isPtsChange(id) || st->isDeleted || st->isInserted) 
                        {
                            storeChangedFunctions.insert(&fun);
                            nextFunc = true;
                            break;
                        }
                    }
                }
                if (nextFunc)
                    break;
            }
            if (nextFunc)
                break;
        }
    }

    // Collect load changed functions
    for (SVFModule::const_iterator fi = svfModule->begin(), efi = svfModule->end(); fi != efi;
            ++fi)
    {
        bool nextFunc = false;
        const SVFFunction& fun = **fi;

        /// if this function does not have any caller, then we do not care its MSSA
        if (Options::IgnoreDeadFun() && fun.isUncalledFunction())
            continue;
        
        for (SVFFunction::const_iterator iter = fun.begin(), eiter = fun.end();
                iter != eiter; ++iter)
        {
            const SVFBasicBlock* bb = *iter;
            for (SVFBasicBlock::const_iterator bit = bb->begin(), ebit = bb->end();
                    bit != ebit; ++bit)
            {
                const SVFInstruction* svfInst = *bit;
                SVFStmtList& pagEdgeList = getPAGEdgesFromInst(svfInst);
                for (SVFStmtList::iterator bit = pagEdgeList.begin(), ebit =
                            pagEdgeList.end(); bit != ebit; ++bit)
                {
                    const PAGEdge* inst = *bit;
                    // pagEdgeToFunMap[inst] = &fun;

                    if (const LoadStmt *ld = SVFUtil::dyn_cast<LoadStmt>(inst))
                    {
                        NodeID id = ld->getRHSVarID();
                        if (incpta->isPtsChange(id) || ld->isDeleted || ld->isInserted) 
                        {
                            loadChangedFunctions.insert(&fun);
                            nextFunc = true;
                            break;
                        }
                    }
                }
                if (nextFunc)
                    break;
            }
            if (nextFunc)
                break;
        }
    }

}
void MRGenerator::initChangedFunctions_RR()
{
    initChangedFunctions();
    WorkList storeWL;
    WorkList loadWL;
    // WorkList storeResetFunctions;
    // WorkList loadResetFunctions;
    SVFUtil::outs() << "Store changed functions: " << storeChangedFunctions.size() << "\n";
    SVFUtil::outs() << "Load changed functions: " << loadChangedFunctions.size() << "\n";
    for (auto it = storeChangedFunctions.begin(), eit = storeChangedFunctions.end();
        it != eit; ++it)
    {
        const SVFFunction* f = *it;
        const PTACallGraphNode* callGraphNode = callGraph->getCallGraphNode(f);
        NodeID id = callGraphNode->getId();
        storeWL.push(id);
        storeResetFunctions.set(id);
    }
    for (auto it = loadChangedFunctions.begin(), eit = loadChangedFunctions.end();
        it != eit; ++it)
    {
        const SVFFunction* f = *it;
        const PTACallGraphNode* callGraphNode = callGraph->getCallGraphNode(f);
        NodeID id = callGraphNode->getId();
        loadWL.push(id);
        loadResetFunctions.set(id);
    }
    // SVFUtil::outs() << "store reset functions num: " << storeResetFunctions.count() << "\n";
    // SVFUtil::outs() << "load reset functions num: " << loadResetFunctions.count() << "\n";
    while (!storeWL.empty()) {
        NodeID callGraphNodeID = storeWL.pop();
        PTACallGraphNode* callGraphNode = callGraph->getCallGraphNode(callGraphNodeID);
        for(PTACallGraphNode::iterator it = callGraphNode->InEdgeBegin(), eit = callGraphNode->InEdgeEnd();
            it!=eit; ++it)
        {
            PTACallGraphEdge* edge = *it;
            NodeID callerID = edge->getSrcID();
            if (storeResetFunctions.test_and_set(callerID))
            {
                storeWL.push(callerID);
            }
        }
    }

    while (!loadWL.empty()) {
        NodeID callGraphNodeID = loadWL.pop();
        PTACallGraphNode* callGraphNode = callGraph->getCallGraphNode(callGraphNodeID);
        for(PTACallGraphNode::iterator it = callGraphNode->InEdgeBegin(), eit = callGraphNode->InEdgeEnd();
            it!=eit; ++it)
        {
            PTACallGraphEdge* edge = *it;
            NodeID callerID = edge->getSrcID();
            if (loadResetFunctions.test_and_set(callerID))
            {
                loadWL.push(callerID);
            }
        }
    }
    // SVFUtil::outs() << "store reset functions num: " << storeResetFunctions.count() << "\n";
    // SVFUtil::outs() << "load reset functions num: " << loadResetFunctions.count() << "\n";
    
}
/*!
 * Generate memory regions for loads/stores
 */
void MRGenerator::collectModRefForLoadStore_inc()
{
    mods_lsChangedFunctions.clear();
    refs_lsChangedFunctions.clear();
    SVFModule* svfModule = incpta->getModule();
    for (SVFModule::const_iterator fi = svfModule->begin(), efi = svfModule->end(); fi != efi;
            ++fi)
    {
        const SVFFunction& fun = **fi;

        /// if this function does not have any caller, then we do not care its MSSA
        if (Options::IgnoreDeadFun() && fun.isUncalledFunction())
            continue;
        bool storeUnchanged = storeChangedFunctions.find(&fun) == storeChangedFunctions.end();
        bool loadUnchanged = loadChangedFunctions.find(&fun) == loadChangedFunctions.end();
        if (storeUnchanged && loadUnchanged)
        {
            // Do not handle unchanged function
            // funToPointsToMap already contains both load/store and call contributions
            // from generateMRs_exh_step1(); do not overwrite with funToPointsToMap_ls
            // which only has load/store contributions.
            continue;
        }
        // handle changed function
        funToPointsToMap_ls[&fun].clear(); // clear funToPointsToMap_ls here
        // 1. handle store changed function
        if (!storeUnchanged) {
            // 1.1 save old funToMods_ls and recompute funToMods_ls
            NodeBS oldFunToMods_ls = funToModsMap_ls[&fun];
            funToModsMap_ls[&fun].clear();
            // 1.2 recompute funToMods
            for (SVFFunction::const_iterator iter = fun.begin(), eiter = fun.end();
                    iter != eiter; ++iter)
            {                
                const SVFBasicBlock* bb = *iter;
                for (SVFBasicBlock::const_iterator bit = bb->begin(), ebit = bb->end();
                        bit != ebit; ++bit)
                {
                    const SVFInstruction* svfInst = *bit;
                    SVFStmtList& pagEdgeList = getPAGEdgesFromInst(svfInst);
                    for (SVFStmtList::iterator bit = pagEdgeList.begin(), ebit =
                                pagEdgeList.end(); bit != ebit; ++bit)
                    {
                        const PAGEdge* inst = *bit;
                        if (inst->isDeleted) //TODO: handle del inst? -- wjy
                            continue;
                        // pagEdgeToFunMap[inst] = &fun; 
                        if (const StoreStmt *st = SVFUtil::dyn_cast<StoreStmt>(inst))
                        {
                            NodeBS cpts(incpta->getPts(st->getLHSVarID()).toNodeBS());
                            // TODO: change this assertion check later when we have conditional points-to set
                            if (cpts.empty())
                                continue;
                            assert(!cpts.empty() && "null pointer!!");
                            addCPtsToStore_inc(cpts, st, &fun);
                        }
                    } // end for each inst in pagedgelist
                } // end for each svfinst in bb
            } // end for each bb in fun (end 1.2)

            // 1.3 compare new/old funToMods_ls
            NodeBS& newFunToMods_ls = funToModsMap_ls[&fun];
            if (oldFunToMods_ls != newFunToMods_ls) {
                mods_lsChangedFunctions.insert(&fun);
                funToModsMap_ls_t[&fun] = oldFunToMods_ls;
            }
        }   // end if (!storeUnchanged)

        // 2. handle changed function
        if (!loadUnchanged) {
            // 2.1. save old funToRefs and recompute funToRefs_ls
            NodeBS oldFunToRefs_ls = funToRefsMap_ls[&fun];
            funToRefsMap_ls[&fun].clear();

            // 2.2 recompute funToRefs/funToMods
            for (SVFFunction::const_iterator iter = fun.begin(), eiter = fun.end();
                    iter != eiter; ++iter)
            { 
                const SVFBasicBlock* bb = *iter;
                for (SVFBasicBlock::const_iterator bit = bb->begin(), ebit = bb->end();
                        bit != ebit; ++bit)
                {
                    const SVFInstruction* svfInst = *bit;
                    SVFStmtList& pagEdgeList = getPAGEdgesFromInst(svfInst);
                    for (SVFStmtList::iterator bit = pagEdgeList.begin(), ebit =
                                pagEdgeList.end(); bit != ebit; ++bit)
                    {
                        const PAGEdge* inst = *bit;
                        if (inst->isDeleted) //TODO: handle del inst? -- wjy
                            continue;
                        // pagEdgeToFunMap[inst] = &fun; 
                        if (const LoadStmt *ld = SVFUtil::dyn_cast<LoadStmt>(inst))
                        {
                            NodeBS cpts(incpta->getPts(ld->getRHSVarID()).toNodeBS());
                            // TODO: change this assertion check later when we have conditional points-to set
                            if (cpts.empty())
                                continue;
                            assert(!cpts.empty() && "null pointer!!");
                            addCPtsToLoad_inc(cpts, ld, &fun);
                        } 
                    } // end for each inst in pagedgelist
                } // end for each svfinst in bb
            } // end for each bb in fun

            // 2.3
            NodeBS& newFunToRefs_ls = funToRefsMap_ls[&fun];
            if (oldFunToRefs_ls != newFunToRefs_ls) {
                refs_lsChangedFunctions.insert(&fun);
                funToRefsMap_ls_t[&fun] = oldFunToRefs_ls;
            }
        } // end if (!loadUnchanged)
        
        // 3. reset the funToPointsToMap of changed function.
        // the funToPointsToMap_ls is computed in addCPtsToStore/Load_inc
        funToPointsToMap[&fun].clear();
        funToPointsToMap[&fun] = funToPointsToMap_ls[&fun];
    }
    SVFUtil::outs() << "Mods_ls changed functions num: " << mods_lsChangedFunctions.size() << "\n";
    SVFUtil::outs() << "Refs_ls changed functions num: " << refs_lsChangedFunctions.size() << "\n";
}

void MRGenerator::resetModRefInfo_RR()
{
    mods_lsChangedFunctions.clear();
    refs_lsChangedFunctions.clear();
    SVFModule* svfModule = incpta->getModule();
    resetFunctions.clear();
    resetFunctions |= storeResetFunctions;
    resetFunctions |= loadResetFunctions;
    SVFUtil::outs() << "store reset functions num: " << storeResetFunctions.count() << "\n";
    SVFUtil::outs() << "load reset functions num: " << loadResetFunctions.count() << "\n";
    // SVFUtil::outs() << "reset functions num: " << resetFunctions.count() << "\n";
    int funcount = 0;
    unsigned modSum = 0;
    unsigned refSum = 0; 
    // reset fun's mod ref info: funToPointsToMap & funToMods/RefsMap
    for (NodeID funid: resetFunctions) {
        funcount++;
        const SVFFunction* fun = callGraph->getCallGraphNode(funid)->getFunction();
        // if (Options::IgnoreDeadFun() && fun->isUncalledFunction())
        //     continue;
        funToPointsToMap[fun].clear();
        funToPointsToMap_ls[fun].clear();
        modSum += funToModsMap[fun].count();
        funToModsMap[fun].clear();
        funToModsMap_ls[fun].clear();
        funToModsMap_cs[fun].clear();
        refSum += funToRefsMap[fun].count();
        funToRefsMap[fun].clear();
        funToRefsMap_ls[fun].clear();
        funToRefsMap_cs[fun].clear();
    }
    SVFUtil::outs() << "reset functions num: " << funcount << "\n";
    SVFUtil::outs() << "reset Mod elements num: " << modSum << "\n";
    SVFUtil::outs() << "reset Ref elements num: " << refSum << "\n";
    int cscount = 0;
    // reset cs's mod ref info: csToArgs/RetPtsMap & csToMods/RefsMap
    for(SVFIR::CallSiteSet::const_iterator it =  pta->getPAG()->getCallSiteSet().begin(),
            eit = pta->getPAG()->getCallSiteSet().end(); it!=eit; ++it)
    {
        const CallICFGNode* cs = (*it);
        const SVFFunction* caller = cs->getCaller();
        NodeID callerID = callGraph->getCallGraphNode(caller)->getId();
        if (resetFunctions.test(callerID)) {
            cscount++;
            csToCallSiteArgsPtsMap[cs].clear();
            csToCallSiteRetPtsMap[cs].clear();
            csToModsMap[cs].clear();
            csToRefsMap[cs].clear();
        }
    }
    SVFUtil::outs() << "reset callsites num: " << cscount << "\n";
}

void MRGenerator::incrementalModRefAnalysis_RR()
{
    SVFUtil::outs() << "Incremental Mod Ref Analysis RR Starts...\n";
    double incStart = stat->getClk();
    incpta = dyn_cast<AndersenInc>(pta);
    // 1. clear glob and recompute
    cachedPtsChainMap.clear();
    allGlobals.clear();
    collectGlobals();

    callGraphSCC->find();

    double initCFStart = stat->getClk();
    initChangedFunctions_RR();
    double initCFEnd = stat->getClk();
    SVFUtil::outs() << "Time of init changed func MRA-RR: " << (initCFEnd - initCFStart)/TIMEINTERVAL << "\n";

    // 2. reset mod ref info
    double resetStart = stat->getClk();
    resetModRefInfo_RR();
    double resetEnd = stat->getClk();
    SVFUtil::outs() << "Time of reset MRA-RR: " << (resetEnd - resetStart)/TIMEINTERVAL << "\n";

    // 3. recollect mod-ref for loads/stores
    double recomputeLSStart = stat->getClk();
    collectModRefForLoadStore_RR();
    double recomputeLSEnd = stat->getClk();
    SVFUtil::outs() << "Time of recompute intra: " << (recomputeLSEnd - recomputeLSStart)/TIMEINTERVAL << "\n";

    // 4. recollect mod-ref for call
    double recomputeCallStart = stat->getClk();
    collectModRefForCall_RR();
    double recomputeCallEnd = stat->getClk();
    SVFUtil::outs() << "Time of recompute intra: " << (recomputeCallEnd - recomputeCallStart)/TIMEINTERVAL << "\n";

    double incEnd = stat->getClk();
    SVFUtil::outs() << "Time of incremental MRA-RR: " << (incEnd - incStart)/TIMEINTERVAL << "\n";
}

void MRGenerator::incrementalModRefAnalysis()
{
    SVFUtil::outs() << "Incremental Mod Ref Analysis Starts...\n";
    double incStart = stat->getClk();
    incpta = dyn_cast<AndersenInc>(pta);

    SVFUtil::outs() << "Incremental Changed Functions Initialization Starts...\n";
    double initCFStart = stat->getClk();
    initChangedFunctions();
    double initCFEnd = stat->getClk();
    SVFUtil::outs() << "Time of initChangedFunctions: " << (initCFEnd - initCFStart) / TIMEINTERVAL << "\n";
    SVFUtil::outs() << "Store changed functions: " << storeChangedFunctions.size() << "\n";
    SVFUtil::outs() << "Load changed functions: " << loadChangedFunctions.size() << "\n";

    SVFUtil::outs() << "Incremental Mod Ref for Load Store Starts...\n";
    double incLSStart = stat->getClk();
    collectModRefForLoadStore_inc();
    double incLSEnd = stat->getClk();
    SVFUtil::outs() << "Time of CollectModRefForLoadStore_inc: " << (incLSEnd - incLSStart) / TIMEINTERVAL << "\n";
    
    // Restore after-state snapshots to avoid before-state PTA corruption
    restoreAfterSnapshot();
    
    // U/D_callsite |= (N_callsite \intersection U/D_callee) \union (G \intersection U/D_callee)
    // 
    // 1. reset and propagate del ref and del mod from the bottom up based on old globs and argspts/retspts
    // 2. recompute globs and get del globs
    // 3. recompute argspts/retspts and get del argspts/retspts
    // 4. reset and propagate del globs and del argspts/retspts 
    //      from the bottom up base on the function mod/ref set changed
    // 5. recompute mod/ref from the bottom up

    // If no local mod/ref changed, skip steps 1-5 to avoid corrupting
    // csToRefsMap/csToModsMap with before-state PTA during reset round.
    bool skipSteps1To5 = mods_lsChangedFunctions.empty() && refs_lsChangedFunctions.empty();
    if (skipSteps1To5) {
        SVFUtil::outs() << "Skipping steps 1-5: no local mod/ref changes detected.\n";
    } else {
    // 1. reset and propagate del ref and del mod from the bottom up based on old globs and argspts/retspts
    SVFUtil::outs() << "Deletion Mod Ref Reset and Propagation Starts...\n";
    resetModSum = 0;
    resetRefSum = 0;
    recomModSum = 0;
    resetRefSum = 0;
    impactFunc = 0;
    double resetStart = stat->getClk();
    WorkList worklist;
    WorkList modlist;
    WorkList reflist;
    getCallGraphSCCRevTopoOrder(worklist);
    while (!worklist.empty()) {
        NodeID callGraphNodeID = worklist.pop();
        const NodeBS& subNodes = callGraphSCC->subNodes(callGraphNodeID);
        for(NodeBS::iterator it = subNodes.begin(), eit = subNodes.end(); it!=eit; ++it)
        {
            PTACallGraphNode* subCallGraphNode = callGraph->getCallGraphNode(*it);
            /// Get mod-ref of all callsites calling callGraphNode
            const SVFFunction* fun = subCallGraphNode->getFunction();
            // 1. modChanged
            // 2. refChanged
            // 3. argsPtsChanged
            if (mods_lsChangedFunctions.find(fun) != mods_lsChangedFunctions.end())
            {
                const NodeBS newMod_ls = funToModsMap_ls[fun];
                const NodeBS oldMod_ls = funToModsMap_ls_t[fun];
                NodeBS dMod = oldMod_ls - newMod_ls;
                resetModSum += dMod.count();
                if (!dMod.empty()) {
                    for (NodeID o: dMod) {
                        funToModsMap[fun].reset(o);
                    }
                    funToDelModsMap[fun] |= dMod;
                    modlist.push((*it));
                }
            }
            if (refs_lsChangedFunctions.find(fun) != refs_lsChangedFunctions.end())
            {
                const NodeBS newRef_ls = funToRefsMap_ls[fun];
                const NodeBS oldRef_ls = funToRefsMap_ls_t[fun];
                NodeBS dRef = oldRef_ls - newRef_ls;
                resetRefSum += dRef.count();
                if (!dRef.empty()) {
                    for (NodeID o: dRef) {
                        funToRefsMap[fun].reset(o);
                    }
                    funToDelRefsMap[fun] |= dRef;
                    reflist.push((*it));
                }
            }
            
        }
    }
    while (!modlist.empty()) {
        NodeID callGraphNodeID = modlist.pop();
        PTACallGraphNode* callGraphNode = callGraph->getCallGraphNode(callGraphNodeID);
        delModAnalysis(callGraphNode, modlist);
    }
    while (!reflist.empty()) {
        NodeID callGraphNodeID = reflist.pop();
        PTACallGraphNode* callGraphNode = callGraph->getCallGraphNode(callGraphNodeID);
        delRefAnalysis(callGraphNode, reflist);
    }
    double resetEnd = stat->getClk();
    SVFUtil::outs() << "Time of Resetting and Propatating Deletion Mod Ref Set: " << 
        (resetEnd - resetStart) / TIMEINTERVAL << "\n";

    Set<const SVFFunction*> impactFuncSet;
    for (auto it = funToDelModsMap.begin(), eit = funToDelModsMap.end(); it != eit; ++it)
    {
        impactFuncSet.insert(it->first);
    }
    for (auto it = funToDelRefsMap.begin(), eit = funToDelRefsMap.end(); it != eit; ++it)
    {
        impactFuncSet.insert(it->first);
    }
    impactFunc = impactFuncSet.size();
    funToDelModsMap.clear();
    funToDelRefsMap.clear();

    // Re-populate funToDelModsMap/RefsMap for step 4 (delGlobsArgsRetsPtsAnalysis),
    // because delModAnalysis/delRefAnalysis in step 1 already consumed them.
    for (auto it = mods_lsChangedFunctions.begin(), eit = mods_lsChangedFunctions.end(); it != eit; ++it) {
        const SVFFunction* fun = *it;
        NodeBS dMod = funToModsMap_ls_t[fun] - funToModsMap_ls[fun];
        if (!dMod.empty()) {
            funToDelModsMap[fun] |= dMod;
        }
    }
    for (auto it = refs_lsChangedFunctions.begin(), eit = refs_lsChangedFunctions.end(); it != eit; ++it) {
        const SVFFunction* fun = *it;
        NodeBS dRef = funToRefsMap_ls_t[fun] - funToRefsMap_ls[fun];
        if (!dRef.empty()) {
            funToDelRefsMap[fun] |= dRef;
        }
    }

    SVFUtil::outs() << "Impacted function num: " << impactFunc << "\n";

    // 2. recompute globs and get del globs
    SVFUtil::outs() << "Globs and Deletion Globs Recomputation Starts...\n";
    double globStart = stat->getClk();
    allGlobals_t = allGlobals;
    if (hasPtsChange(allGlobals)) {
        allGlobals.clear();
        collectGlobals();       
    }
    dGlobs = allGlobals_t - allGlobals;
    double globEnd = stat->getClk();
    SVFUtil::outs() << "dGlobs.count(): " << dGlobs.count() << "\n";
    SVFUtil::outs() << "Time of Recompute Globs: " << (globEnd - globStart) / TIMEINTERVAL << "\n";

    // 3. recompute argspts/retspts and get del argspts/retspts
    SVFUtil::outs() << "ArgsPts/RetsPts and Deletion ArgsPts/RetsPts Recomputation Starts...\n";
    double argStart = stat->getClk();
    cachedPtsChainMap.clear(); // clear cachedPtsChainMap
    for(SVFIR::CallSiteSet::const_iterator it =  pta->getPAG()->getCallSiteSet().begin(),
            eit = pta->getPAG()->getCallSiteSet().end(); it!=eit; ++it)
    {
        const CallICFGNode* cnode = (*it);
        collectCallSitePts_inc((*it));
    }
    double argEnd = stat->getClk();
    SVFUtil::outs() << "Time of Recompute ArgsPts/RetsPts: " << (argEnd - argStart) / TIMEINTERVAL << "\n";

    // 4. reset and propagate del globs and del argspts/retspts 
    //      from the bottom up base on the function mod/ref set changed
    SVFUtil::outs() << "Deletion Globs and ArgsPts/RetPts Reset and Propagation Starts...\n";
    double gaStart = stat->getClk();
    getCallGraphSCCRevTopoOrder(worklist);
    while(!worklist.empty())
    {
        NodeID callGraphNodeID = worklist.pop();
        /// handle all sub scc nodes of this rep node
        const NodeBS& subNodes = callGraphSCC->subNodes(callGraphNodeID);
        for(NodeBS::iterator it = subNodes.begin(), eit = subNodes.end(); it!=eit; ++it)
        {
            PTACallGraphNode* subCallGraphNode = callGraph->getCallGraphNode(*it);
            /// Get mod-ref of all callsites calling callGraphNode
            // const SVFFunction* fun = subCallGraphNode->getFunction();
            delGlobsArgsRetsPtsAnalysis(subCallGraphNode, worklist);
        }
    }
    double gaEnd = stat->getClk();
    SVFUtil::outs() << "Time of Resetting and Propagating Deletion Globs/ArgsPts/RetsPts: " 
        << (gaEnd - gaStart) / TIMEINTERVAL << "\n";
    // TODO: handle changed callsite/calledge?
    
    // 5.  recompute mod/ref from the bottom up
    SVFUtil::outs() << "Recomputation Mod Ref Starts...\n";
    double rcStart = stat->getClk();
    getCallGraphSCCRevTopoOrder(worklist);

    while(!worklist.empty())
    {
        NodeID callGraphNodeID = worklist.pop();
        /// handle all sub scc nodes of this rep node
        const NodeBS& subNodes = callGraphSCC->subNodes(callGraphNodeID);
        for(NodeBS::iterator it = subNodes.begin(), eit = subNodes.end(); it!=eit; ++it)
        {
            PTACallGraphNode* subCallGraphNode = callGraph->getCallGraphNode(*it);
            /// Get mod-ref of all callsites calling callGraphNode
            modRefAnalysis(subCallGraphNode,worklist);
        }
    }
    double rcEnd = stat->getClk();
    SVFUtil::outs() << "Time of recomputing mod ref: " << (rcEnd - rcStart)/TIMEINTERVAL << "\n";
    SVFUtil::outs() << "reset Mod element num: " << resetModSum << "\n";
    SVFUtil::outs() << "reset Ref element num: " << resetRefSum << "\n";
    }
    
    // 6. addCPtsToCallSite
    SVFUtil::outs() << "AddCPtsToCallSite Starts...\n";
    double acpStart = stat->getClk();
    callsiteToRefPointsToMap.clear();
    callsiteToModPointsToMap.clear();
    for (const CallICFGNode* callBlockNode : pta->getPAG()->getCallSiteSet())
    {   
        if(hasRefSideEffectOfCallSite(callBlockNode))
        {
            NodeBS refs = getRefSideEffectOfCallSite(callBlockNode);
            addCPtsToCallSiteRefs(refs,callBlockNode);
        }
        if(hasModSideEffectOfCallSite(callBlockNode))
        {
            NodeBS mods = getModSideEffectOfCallSite(callBlockNode);
            /// mods are treated as both def and use of memory objects
            addCPtsToCallSiteMods(mods,callBlockNode);
            addCPtsToCallSiteRefs(mods,callBlockNode);
        }
    }
    double acpEnd = stat->getClk();
    SVFUtil::outs() << "Time of addCPtsToCallSite: " << (acpEnd - acpStart)/TIMEINTERVAL << "\n";

    double incEnd = stat->getClk();
    SVFUtil::outs() << "Mod Ref (Inc): " << (incEnd - incStart)/TIMEINTERVAL << "\n";
    
    if (Options::wjydebug())
        debugdump();
}
/*!
 * Generate memory regions for calls
 */
void MRGenerator::collectModRefForCall()
{

    DBOUT(DGENERAL, outs() << pasMsg("\t\tCollect Callsite PointsTo \n"));
    double ccStart = stat->getClk(true);
    /// collect points-to information for callsites
    for(SVFIR::CallSiteSet::const_iterator it =  pta->getPAG()->getCallSiteSet().begin(),
            eit = pta->getPAG()->getCallSiteSet().end(); it!=eit; ++it)
    {
        collectCallSitePts((*it));
    }
    double ccEnd = stat->getClk(true);
    timeOfCollectCallSitePts += (ccEnd - ccStart)/TIMEINTERVAL;

    DBOUT(DGENERAL, outs() << pasMsg("\t\tPerform Callsite Mod-Ref \n"));

    WorkList worklist;
    getCallGraphSCCRevTopoOrder(worklist);

    double mrStart = stat->getClk(true);
    while(!worklist.empty())
    {
        NodeID callGraphNodeID = worklist.pop();
        /// handle all sub scc nodes of this rep node
        const NodeBS& subNodes = callGraphSCC->subNodes(callGraphNodeID);
        for(NodeBS::iterator it = subNodes.begin(), eit = subNodes.end(); it!=eit; ++it)
        {
            PTACallGraphNode* subCallGraphNode = callGraph->getCallGraphNode(*it);
            /// Get mod-ref of all callsites calling callGraphNode
            modRefAnalysis(subCallGraphNode,worklist);
        }
    }
    double mrEnd = stat->getClk(true);
    SVFUtil::outs() << "Mod Ref (collectModRefForCall): " << (mrEnd - mrStart)/TIMEINTERVAL << "\n";

    timeOfModRefAnalysis += (mrEnd - mrStart)/TIMEINTERVAL;

    DBOUT(DGENERAL, outs() << pasMsg("\t\tAdd PointsTo to Callsites \n"));

    for (const CallICFGNode* callBlockNode : pta->getPAG()->getCallSiteSet())
    {
        if(hasRefSideEffectOfCallSite(callBlockNode))
        {
            NodeBS refs = getRefSideEffectOfCallSite(callBlockNode);
            addCPtsToCallSiteRefs(refs,callBlockNode);
        }
        if(hasModSideEffectOfCallSite(callBlockNode))
        {
            NodeBS mods = getModSideEffectOfCallSite(callBlockNode);
            /// mods are treated as both def and use of memory objects
            addCPtsToCallSiteMods(mods,callBlockNode);
            addCPtsToCallSiteRefs(mods,callBlockNode);
        }
    }
}

void MRGenerator::collectModRefForCall_RR()
{
    DBOUT(DGENERAL, outs() << pasMsg("\t\tCollect Callsite PointsTo \n"));
    double ccStart = stat->getClk(true);
    /// collect points-to information for callsites
    for(SVFIR::CallSiteSet::const_iterator it =  pta->getPAG()->getCallSiteSet().begin(),
            eit = pta->getPAG()->getCallSiteSet().end(); it!=eit; ++it)
    {
        const CallICFGNode* cs = (*it);
        const SVFFunction* caller = cs->getCaller();
        NodeID callerID = callGraph->getCallGraphNode(caller)->getId();
        if (resetFunctions.test(callerID)) {
            collectCallSitePts((*it));
        }
    }
    double ccEnd = stat->getClk(true);
    SVFUtil::outs() << "Time of RR collectCallSitePts: " 
        << (ccEnd - ccStart)/TIMEINTERVAL << "\n";


    DBOUT(DGENERAL, outs() << pasMsg("\t\tPerform Callsite Mod-Ref \n"));
    WorkList worklist;
    getCallGraphSCCRevTopoOrder(worklist);
    double mrStart = stat->getClk(true);
    while(!worklist.empty())
    {
        NodeID callGraphNodeID = worklist.pop();
        /// handle all sub scc nodes of this rep node
        const NodeBS& subNodes = callGraphSCC->subNodes(callGraphNodeID);
        for(NodeBS::iterator it = subNodes.begin(), eit = subNodes.end(); it!=eit; ++it)
        {
            PTACallGraphNode* subCallGraphNode = callGraph->getCallGraphNode(*it);
            /// Get mod-ref of all callsites calling callGraphNode
            modRefAnalysis(subCallGraphNode,worklist);
        }
    }
    double mrEnd = stat->getClk(true);
    SVFUtil::outs() << "Mod Ref (RR): " << (mrEnd - mrStart)/TIMEINTERVAL << "\n";

    // timeOfModRefAnalysis += (mrEnd - mrStart)/TIMEINTERVAL;

    DBOUT(DGENERAL, outs() << pasMsg("\t\tAdd PointsTo to Callsites \n"));
    SVFUtil::outs() << "AddCPtsToCallSite Starts...\n";
    double acpStart = stat->getClk();
    callsiteToRefPointsToMap.clear();
    callsiteToModPointsToMap.clear();
    for (const CallICFGNode* callBlockNode : pta->getPAG()->getCallSiteSet())
    {
        if(hasRefSideEffectOfCallSite(callBlockNode))
        {
            NodeBS refs = getRefSideEffectOfCallSite(callBlockNode);
            addCPtsToCallSiteRefs(refs,callBlockNode);
        }
        if(hasModSideEffectOfCallSite(callBlockNode))
        {
            NodeBS mods = getModSideEffectOfCallSite(callBlockNode);
            /// mods are treated as both def and use of memory objects
            addCPtsToCallSiteMods(mods,callBlockNode);
            addCPtsToCallSiteRefs(mods,callBlockNode);
        }
    }
    double acpEnd = stat->getClk();
    SVFUtil::outs() << "Time of addCPtsToCallSite (RR): " << (acpEnd - acpStart)/TIMEINTERVAL << "\n";
}

/*!
 * Given a condition pts, insert into cptsToRepCPtsMap
 * Always map it to its superset(rep) cpts according to existing items
 * 1) map cpts to its superset(rep) which exists in the map, otherwise its superset is itself
 * 2) adjust existing items in the map if their supersets are cpts
 */
void MRGenerator::sortPointsTo(const NodeBS& cpts)
{

    if(cptsToRepCPtsMap.find(cpts)!=cptsToRepCPtsMap.end())
        return;

    PointsToList subSetList;
    NodeBS repCPts = cpts;
    for(PtsToRepPtsSetMap::iterator it = cptsToRepCPtsMap.begin(),
            eit = cptsToRepCPtsMap.end(); it!=eit; ++it)
    {
        NodeBS& existCPts = it->second;
        if(cpts.contains(existCPts))
        {
            subSetList.insert(it->first);
        }
        else if(existCPts.contains(cpts))
        {
            repCPts = existCPts;
        }
    }

    for(PointsToList::iterator it = subSetList.begin(), eit = subSetList.end(); it!=eit; ++it)
    {
        cptsToRepCPtsMap[*it] = cpts;
    }

    cptsToRepCPtsMap[cpts] = repCPts;
}

/*!
 * Partition memory regions
 */
void MRGenerator::partitionMRs()
{

    /// Compute all superset of all condition points-to sets
    /// TODO: we may need some refined region partitioning algorithm here
    /// For now, we just collapse all refs/mods objects at callsites into one region
    /// Consider modularly partition memory regions to speed up analysis (only partition regions within function scope)
    for(FunToPointsTosMap::iterator it = getFunToPointsToList().begin(), eit = getFunToPointsToList().end();
            it!=eit; ++it)
    {
        for(PointsToList::iterator cit = it->second.begin(), ecit = it->second.end(); cit!=ecit; ++cit)
        {
            sortPointsTo(*cit);
        }
    }
    /// Generate memory regions according to condition pts after computing superset
    for(FunToPointsTosMap::iterator it = getFunToPointsToList().begin(), eit = getFunToPointsToList().end();
            it!=eit; ++it)
    {
        const SVFFunction* fun = it->first;
        for(PointsToList::iterator cit = it->second.begin(), ecit = it->second.end(); cit!=ecit; ++cit)
        {
            createMR(fun,*cit);
        }
    }

}

/*!
 * Update aliased regions for loads/stores/callsites
 */
void MRGenerator::updateAliasMRs()
{

    /// update stores with its aliased regions
    for(StoresToPointsToMap::const_iterator it = storesToPointsToMap.begin(), eit = storesToPointsToMap.end(); it!=eit; ++it)
    {
        MRSet aliasMRs;
        const SVFFunction* fun = getFunction(it->first);
        const NodeBS& storeCPts = it->second;
        getAliasMemRegions(aliasMRs,storeCPts,fun);
        for(MRSet::iterator ait = aliasMRs.begin(), eait = aliasMRs.end(); ait!=eait; ++ait)
        {
            storesToMRsMap[it->first].insert(*ait);
        }
    }

    for(LoadsToPointsToMap::const_iterator it = loadsToPointsToMap.begin(), eit = loadsToPointsToMap.end(); it!=eit; ++it)
    {
        MRSet aliasMRs;
        const SVFFunction* fun = getFunction(it->first);
        const NodeBS& loadCPts = it->second;
        getMRsForLoad(aliasMRs, loadCPts, fun);
        for(MRSet::iterator ait = aliasMRs.begin(), eait = aliasMRs.end(); ait!=eait; ++ait)
        {
            loadsToMRsMap[it->first].insert(*ait);
        }
    }

    /// update callsites with its aliased regions
    for(CallSiteToPointsToMap::const_iterator it =  callsiteToModPointsToMap.begin(),
            eit = callsiteToModPointsToMap.end(); it!=eit; ++it)
    {
        const SVFFunction* fun = it->first->getCaller();
        MRSet aliasMRs;
        const NodeBS& callsiteModCPts = it->second;
        getAliasMemRegions(aliasMRs,callsiteModCPts,fun);
        for(MRSet::iterator ait = aliasMRs.begin(), eait = aliasMRs.end(); ait!=eait; ++ait)
        {
            callsiteToModMRsMap[it->first].insert(*ait);
        }
    }
    for(CallSiteToPointsToMap::const_iterator it =  callsiteToRefPointsToMap.begin(),
            eit = callsiteToRefPointsToMap.end(); it!=eit; ++it)
    {
        const SVFFunction* fun = it->first->getCaller();
        MRSet aliasMRs;
        const NodeBS& callsiteRefCPts = it->second;
        getMRsForCallSiteRef(aliasMRs, callsiteRefCPts, fun);
        for(MRSet::iterator ait = aliasMRs.begin(), eait = aliasMRs.end(); ait!=eait; ++ait)
        {
            callsiteToRefMRsMap[it->first].insert(*ait);
        }
    }
}


/*!
 * Add indirect uses an memory object in the function
 */
void MRGenerator::addRefSideEffectOfFunction(const SVFFunction* fun, const NodeBS& refs)
{
    for(NodeBS::iterator it = refs.begin(), eit = refs.end(); it!=eit; ++it)
    {
        if(isNonLocalObject(*it,fun))
            funToRefsMap[fun].set(*it);
    }
}
void MRGenerator::addRefSideEffectOfFunction_loadStore(const SVFFunction* fun, const NodeBS& refs)
{
    for(NodeBS::iterator it = refs.begin(), eit = refs.end(); it!=eit; ++it)
    {
        if(isNonLocalObject(*it,fun)) {
            funToRefsMap[fun].set(*it);
            funToRefsMap_ls[fun].set(*it);
        }
    }
}
void MRGenerator::addRefSideEffectOfFunction_callSite(const SVFFunction* fun, const NodeBS& refs)
{
    for(NodeBS::iterator it = refs.begin(), eit = refs.end(); it!=eit; ++it)
    {
        if(isNonLocalObject(*it,fun)) {
            funToRefsMap[fun].set(*it);
            funToRefsMap_cs[fun].set(*it);
        }
    }
}
void MRGenerator::delRefSideEffectOfFunction_callSite(const SVFFunction* fun, const NodeBS& refs, NodeBS& delRefs)
{
    for(NodeBS::iterator it = refs.begin(), eit = refs.end(); it!=eit; ++it)
    {
        if(isNonLocalObject(*it,fun)) {
            // funToRefsMap[fun].reset(*it);
            funToRefsMap_cs[fun].reset(*it);
            if ( (!funToRefsMap_ls[fun].test(*it)) && (funToRefsMap[fun].test(*it)) ) {
                // not by load/store  (aka. only by callsite)
                funToRefsMap[fun].reset(*it);
                delRefs.set(*it);
            }
        }
    }
    resetRefSum += delRefs.count();
}
void MRGenerator::addRefSideEffectOfFunction_loadStore_inc(const SVFFunction* fun, const NodeBS& refs)
{
    for(NodeBS::iterator it = refs.begin(), eit = refs.end(); it!=eit; ++it)
    {
        if(isNonLocalObject(*it,fun)) {
            funToRefsMap[fun].set(*it); // 1.19
            funToRefsMap_ls[fun].set(*it);
        }
    }
}

/*!
 * Add indirect def an memory object in the function
 */
void MRGenerator::addModSideEffectOfFunction(const SVFFunction* fun, const NodeBS& mods)
{
    for(NodeBS::iterator it = mods.begin(), eit = mods.end(); it!=eit; ++it)
    {
        if(isNonLocalObject(*it,fun))
            funToModsMap[fun].set(*it);
    }
}
void MRGenerator::addModSideEffectOfFunction_loadStore(const SVFFunction* fun, const NodeBS& mods)
{
    for(NodeBS::iterator it = mods.begin(), eit = mods.end(); it!=eit; ++it)
    {
        if(isNonLocalObject(*it,fun)) {
            funToModsMap[fun].set(*it);
            funToModsMap_ls[fun].set(*it);
        }
    }
}
void MRGenerator::addModSideEffectOfFunction_callSite(const SVFFunction* fun, const NodeBS& mods)
{
    for(NodeBS::iterator it = mods.begin(), eit = mods.end(); it!=eit; ++it)
    {
        if(isNonLocalObject(*it,fun)) {
            funToModsMap[fun].set(*it);
            funToModsMap_cs[fun].set(*it);
        }
    }
}
void MRGenerator::delModSideEffectOfFunction_callSite(const SVFFunction* fun, const NodeBS& mods, NodeBS& delMods)
{
    for(NodeBS::iterator it = mods.begin(), eit = mods.end(); it!=eit; ++it)
    {
        if(isNonLocalObject(*it,fun)) {
            // funToModsMap[fun].reset(*it);
            funToModsMap_cs[fun].reset(*it);
            if ((!funToModsMap_ls[fun].test(*it)) && (funToModsMap[fun].test(*it))) {
                // not by load/store  (aka. only by callsite)
                funToModsMap[fun].reset(*it);
                delMods.set(*it);
            }
        }
    }
    resetModSum += delMods.count();
}
void MRGenerator::addModSideEffectOfFunction_loadStore_inc(const SVFFunction* fun, const NodeBS& mods)
{
    for(NodeBS::iterator it = mods.begin(), eit = mods.end(); it!=eit; ++it)
    {
        if(isNonLocalObject(*it,fun)) {
            funToModsMap[fun].set(*it); // 1.19
            funToModsMap_ls[fun].set(*it);
        }
    }
}

/*!
 * Add indirect uses an memory object in the function
 */
bool MRGenerator::addRefSideEffectOfCallSite(const CallICFGNode* cs, const NodeBS& refs)
{
    if(!refs.empty())
    {
        NodeBS refset = refs;
        refset &= getCallSiteArgsPts(cs);
        getEscapObjviaGlobals(refset,refs);
        // addRefSideEffectOfFunction(cs->getCaller(),refset);
        addRefSideEffectOfFunction_callSite(cs->getCaller(),refset);
        return csToRefsMap[cs] |= refset;
    }
    return false;
}

/*!
 * Add indirect def an memory object in the function
 */
bool MRGenerator::addModSideEffectOfCallSite(const CallICFGNode* cs, const NodeBS& mods)
{
    if(!mods.empty())
    {
        NodeBS modset = mods;
        modset &= (getCallSiteArgsPts(cs) | getCallSiteRetPts(cs));
        getEscapObjviaGlobals(modset,mods);
        // addModSideEffectOfFunction(cs->getCaller(),modset);
        addModSideEffectOfFunction_callSite(cs->getCaller(),modset);
        return csToModsMap[cs] |= modset;
    }
    return false;
}


/*!
 * Get the reverse topo order of scc call graph
 */
void MRGenerator::getCallGraphSCCRevTopoOrder(WorkList& worklist)
{
    NodeStack revTopoOrder;
    NodeStack& topoOrder = callGraphSCC->topoNodeStack();
    while(!topoOrder.empty())
    {
        NodeID callgraphNodeID = topoOrder.top();
        topoOrder.pop();
        revTopoOrder.push(callgraphNodeID);
        worklist.push(callgraphNodeID);
    }
    // restore the topological order for later solving.
    while (!revTopoOrder.empty())
    {
        NodeID nodeId = revTopoOrder.top();
        revTopoOrder.pop();
        topoOrder.push(nodeId);
    }
}

/*!
 * Get all objects might pass into and pass out of callee(s) from a callsite
 */
bool MRGenerator::hasPtsChange(NodeBS& nodes)
{
    bool res = false;
    const NodeBS& ptsChangeNodes = incpta->getPtsChangeNodes();
    for(NodeID o: nodes) {
        if (ptsChangeNodes.test(o)) {
            res = true;
            break;
        }
    }
    return res;
}

void MRGenerator::collectCallSitePts_inc(const CallICFGNode* cs)
{

    NodeBS& argsPts = csToCallSiteArgsPtsMap[cs];
    SVFIR* pag = incpta->getPAG();
    CallICFGNode* callBlockNode = pag->getICFG()->getCallICFGNode(cs->getCallSite());
    RetICFGNode* retBlockNode = pag->getICFG()->getRetICFGNode(cs->getCallSite());

    NodeBS argsBS;
    if (pag->hasCallSiteArgsMap(callBlockNode))
    {
        const SVFIR::SVFVarList& args = incpta->getPAG()->getCallSiteArgsList(callBlockNode);
        for(SVFIR::SVFVarList::const_iterator itA = args.begin(), ieA = args.end(); itA!=ieA; ++itA)
        {
            const PAGNode* node = *itA;
            if(node->isPointer())
                argsBS.set(node->getId());
        }
    }
    NodeBS args_U_argsPts = argsBS|argsPts;
    if (hasPtsChange(args_U_argsPts)) {
        // Recompute ptschain if argsPts contains node whose pts is changed.
        csToCallSiteArgsPtsMap_t[cs] = argsPts;
        argsPts.clear();
        for (NodeID arg: argsBS) {
            for (NodeID o: incpta->getPts(arg)) {
                argsPts |= CollectPtsChain(o);
            }
        }
    }
    
    /// collect the pts chain of the return argument
    NodeBS& retPts = csToCallSiteRetPtsMap[cs];
    NodeBS retBS;
    if (pta->getPAG()->callsiteHasRet(retBlockNode))
    {
        const PAGNode* node = pta->getPAG()->getCallSiteRet(retBlockNode);
        if(node->isPointer())
        {
            retBS.set(node->getId());
        }
    }
    NodeBS ret_U_retPts = retBS|retPts;
    if (hasPtsChange(ret_U_retPts)) {
        csToCallSiteRetPtsMap_t[cs] = argsPts;
        retPts.clear();
        for (NodeID ret: retBS) {
            for (NodeID o: incpta->getPts(ret)) {
                retPts |= CollectPtsChain(o);
            }
        }
    }

}
void MRGenerator::collectCallSitePts(const CallICFGNode* cs)
{
    /// collect the pts chain of the callsite arguments
    NodeBS& argsPts = csToCallSiteArgsPtsMap[cs];
    SVFIR* pag = pta->getPAG();
    CallICFGNode* callBlockNode = pag->getICFG()->getCallICFGNode(cs->getCallSite());
    RetICFGNode* retBlockNode = pag->getICFG()->getRetICFGNode(cs->getCallSite());

    WorkList worklist;
    if (pag->hasCallSiteArgsMap(callBlockNode))
    {
        const SVFIR::SVFVarList& args = pta->getPAG()->getCallSiteArgsList(callBlockNode);
        for(SVFIR::SVFVarList::const_iterator itA = args.begin(), ieA = args.end(); itA!=ieA; ++itA)
        {
            const PAGNode* node = *itA;
            if(node->isPointer())
                worklist.push(node->getId());
        }
    }

    while(!worklist.empty())
    {
        NodeID nodeId = worklist.pop();
        const NodeBS& tmp = pta->getPts(nodeId).toNodeBS();
        for(NodeBS::iterator it = tmp.begin(), eit = tmp.end(); it!=eit; ++it)
            argsPts |= CollectPtsChain(*it);
    }

    /// collect the pts chain of the return argument
    NodeBS& retPts = csToCallSiteRetPtsMap[cs];

    if (pta->getPAG()->callsiteHasRet(retBlockNode))
    {
        const PAGNode* node = pta->getPAG()->getCallSiteRet(retBlockNode);
        if(node->isPointer())
        {
            const NodeBS& tmp = pta->getPts(node->getId()).toNodeBS();
            for(NodeBS::iterator it = tmp.begin(), eit = tmp.end(); it!=eit; ++it)
                retPts |= CollectPtsChain(*it);
        }
    }

}


/*!
 * Recursively collect all points-to of the whole struct fields
 */
NodeBS& MRGenerator::CollectPtsChain(NodeID id)
{
    NodeID baseId = pta->getPAG()->getBaseObjVar(id);
    NodeToPTSSMap::iterator it = cachedPtsChainMap.find(baseId);
    if(it!=cachedPtsChainMap.end())
        return it->second;
    else
    {
        NodeBS& pts = cachedPtsChainMap[baseId];
        pts |= pta->getPAG()->getFieldsAfterCollapse(baseId);

        WorkList worklist;
        for(NodeBS::iterator it = pts.begin(), eit = pts.end(); it!=eit; ++it)
            worklist.push(*it);

        while(!worklist.empty())
        {
            NodeID nodeId = worklist.pop();
            const NodeBS& tmp = pta->getPts(nodeId).toNodeBS();
            for(NodeBS::iterator it = tmp.begin(), eit = tmp.end(); it!=eit; ++it)
            {
                pts |= CollectPtsChain(*it);
            }
        }
        return pts;
    }

}

/*!
 * Get all the objects in callee's modref escaped via global objects (the chain pts of globals)
 * Otherwise, the object in callee's modref would not escape through globals
 */

void MRGenerator::getEscapObjviaGlobals(NodeBS& globs, const NodeBS& calleeModRef)
{
    for(NodeBS::iterator it = calleeModRef.begin(), eit = calleeModRef.end(); it!=eit; ++it)
    {
        const MemObj* obj = pta->getPAG()->getObject(*it);
        (void)obj; // Suppress warning of unused variable under release build
        assert(obj && "object not found!!");
        if(allGlobals.test(*it))
            globs.set(*it);
    }
}

/*!
 * Whether the object node is a non-local object
 * including global, heap, and stack variable in recursions
 */
bool MRGenerator::isNonLocalObject(NodeID id, const SVFFunction* curFun) const
{
    const MemObj* obj = pta->getPAG()->getObject(id);
    assert(obj && "object not found!!");
    /// if the object is heap or global
    if(obj->isGlobalObj() || obj->isHeap())
        return true;
    /// or if the local variable of its callers
    /// or a local variable is in function recursion cycles
    else if(obj->isStack())
    {
        if(const SVFFunction* svffun = pta->getPAG()->getGNode(id)->getFunction())
        {
            if(svffun!=curFun)
                return true;
            else
                return callGraphSCC->isInCycle(callGraph->getCallGraphNode(svffun)->getId());
        }
    }

    return false;
}

/*!
 * Get Mod-Ref of a callee function
 */
bool MRGenerator::handleCallsiteModRef(NodeBS& mod, NodeBS& ref, const CallICFGNode* cs, const SVFFunction* callee)
{
    /// if a callee is a heap allocator function, then its mod set of this callsite is the heap object.
    if(isHeapAllocExtCall(cs->getCallSite()))
    {
        SVFStmtList& pagEdgeList = getPAGEdgesFromInst(cs->getCallSite());
        for (SVFStmtList::const_iterator bit = pagEdgeList.begin(),
                ebit = pagEdgeList.end(); bit != ebit; ++bit)
        {
            const PAGEdge* edge = *bit;
            if (const AddrStmt* addr = SVFUtil::dyn_cast<AddrStmt>(edge))
                mod.set(addr->getRHSVarID());
        }
    }
    /// otherwise, we find the mod/ref sets from the callee function, who has definition and been processed
    else
    {
        mod = getModSideEffectOfFunction(callee);
        ref = getRefSideEffectOfFunction(callee);
    }
    // add ref set
    bool refchanged = addRefSideEffectOfCallSite(cs, ref);
    // add mod set
    bool modchanged = addModSideEffectOfCallSite(cs, mod);

    return refchanged || modchanged;
}
bool MRGenerator::delRefSideEffectOfCallSite(const CallICFGNode* cs, const NodeBS& refs, NodeBS& delRefToProp)
{
    if(!refs.empty())
    {
        NodeBS refset = refs;
        refset &= (getCallSiteArgsPts(cs));
        getEscapObjviaGlobals(refset,refs);

        // compute fun refs diff
        NodeBS funDelRefs;
        delRefSideEffectOfFunction_callSite(cs->getCaller(), refset, funDelRefs);

        // reset callsite refs
        csToRefsMap[cs] = csToRefsMap[cs] - refset;

        if (!funDelRefs.empty()) {
            funToDelRefsMap[cs->getCaller()] |= funDelRefs;
            return true;
        }
    }
    return false;
}
bool MRGenerator::delModSideEffectOfCallSite(const CallICFGNode* cs, const NodeBS& mods, NodeBS& delModToProp)
{
    if(!mods.empty())
    {
        NodeBS modset = mods;
        modset &= (getCallSiteArgsPts(cs) | getCallSiteRetPts(cs));
        getEscapObjviaGlobals(modset,mods);

        // compute fun mods diff
        NodeBS funDelMods;
        delModSideEffectOfFunction_callSite(cs->getCaller(), modset, funDelMods);

        // reset callsite mods
        csToModsMap[cs] = csToModsMap[cs] - modset;

        if (!funDelMods.empty()) {
            funToDelModsMap[cs->getCaller()] |= funDelMods;
            return true;
        }
    }
    return false;
}
bool MRGenerator::delGlobsArgsRetsPtsOfCallSite(const CallICFGNode* cs, const NodeBS& calleeDMods, const NodeBS& calleeDRefs)
{
    const SVFFunction* caller = cs->getCaller();
    const NodeBS& oldCsArgsPts = csToCallSiteArgsPtsMap_t[cs];
    const NodeBS& oldCsRetsPts = csToCallSiteRetPtsMap_t[cs];
    const NodeBS& csArgsPts = csToCallSiteArgsPtsMap[cs];
    const NodeBS& csRetsPts = csToCallSiteRetPtsMap[cs];
    // compute dRetPts/dArgsPts
    NodeBS dRets = oldCsRetsPts - csRetsPts;
    NodeBS dArgs = oldCsArgsPts - csArgsPts;
    if (dRets.empty()) {
        csToCallSiteRetPtsMap_t.erase(cs);
    }
    if (dArgs.empty()) {
        csToCallSiteArgsPtsMap_t.erase(cs);
    }
    // compute callsite's dRefs/dMods
    NodeBS dRefs = dGlobs | dArgs | calleeDRefs;
    NodeBS dMods = dGlobs | dArgs | dRets | calleeDMods;

    // compute fun refs/mods diff
    NodeBS funDelRefs;
    NodeBS funDelMods;
    delRefSideEffectOfFunction_callSite(caller, dRefs, funDelRefs);
    delModSideEffectOfFunction_callSite(caller, dMods, funDelMods);

    // reset callsite refs/mods
    csToModsMap[cs] = csToModsMap[cs] - dMods;
    csToRefsMap[cs] = csToRefsMap[cs] - dRefs;
    bool callerRefsChanged = false;
    bool callerModsChanged = false;
    if (!funDelRefs.empty()) {
        funToDelRefsMap[caller] |= funDelRefs;
        callerRefsChanged = true;
    }
    if (!funDelMods.empty()) {
        funToDelModsMap[caller] |= funDelMods;
        callerModsChanged = true;
    }
    return callerRefsChanged || callerModsChanged;
}
/*!
 * Call site mod-ref analysis
 * Compute mod-ref of all callsites invoking this call graph node
 */
void MRGenerator::delRefAnalysis(PTACallGraphNode* callGraphNode, WorkList& worklist)
{
    const SVFFunction* fun = callGraphNode->getFunction();
    NodeBS dRef = funToDelRefsMap[fun]; // copy del ref set
    funToDelRefsMap[fun].clear();
    funToDelRefsMap.erase(fun);
    if (dRef.empty())
        return;
    /// del ref set of callee to its invocation callsites at caller
    for(PTACallGraphNode::iterator it = callGraphNode->InEdgeBegin(), eit = callGraphNode->InEdgeEnd();
            it!=eit; ++it)
    {
        PTACallGraphEdge* edge = *it;

        /// handle direct callsites
        for(PTACallGraphEdge::CallInstSet::iterator cit = edge->getDirectCalls().begin(),
                ecit = edge->getDirectCalls().end(); cit!=ecit; ++cit)
        {
            NodeBS ref;
            const CallICFGNode* cs = (*cit);
            NodeBS delRefToProp;
            bool changed = delRefSideEffectOfCallSite(cs, dRef, delRefToProp);
            if (changed) {
                worklist.push(edge->getSrcID());
            }
        }
        /// handle indirect callsites
        for(PTACallGraphEdge::CallInstSet::iterator cit = edge->getIndirectCalls().begin(),
                ecit = edge->getIndirectCalls().end(); cit!=ecit; ++cit)
        {
            NodeBS ref;
            const CallICFGNode* cs = (*cit);
            NodeBS delRefToProp;
            bool changed = delRefSideEffectOfCallSite(cs, dRef, delRefToProp);
            if (changed) {
                worklist.push(edge->getSrcID());
            }
        }
    }

}
void MRGenerator::delModAnalysis(PTACallGraphNode* callGraphNode, WorkList& worklist)
{
    const SVFFunction* fun = callGraphNode->getFunction();
    NodeBS dMod = funToDelModsMap[fun]; // copy del mod set
    funToDelModsMap[fun].clear();
    funToDelModsMap.erase(fun);
    if (dMod.empty())
        return;

    /// del mod set of callee to its invocation callsites at caller
    for(PTACallGraphNode::iterator it = callGraphNode->InEdgeBegin(), eit = callGraphNode->InEdgeEnd();
            it!=eit; ++it)
    {
        PTACallGraphEdge* edge = *it;

        /// handle direct callsites
        for(PTACallGraphEdge::CallInstSet::iterator cit = edge->getDirectCalls().begin(),
                ecit = edge->getDirectCalls().end(); cit!=ecit; ++cit)
        {
            NodeBS mod;
            const CallICFGNode* cs = (*cit);
            NodeBS delModToProp;
            bool changed = delModSideEffectOfCallSite(cs, dMod, delModToProp);
            if (changed) {
                worklist.push(edge->getSrcID());
            }
        }
        /// handle indirect callsites
        for(PTACallGraphEdge::CallInstSet::iterator cit = edge->getIndirectCalls().begin(),
                ecit = edge->getIndirectCalls().end(); cit!=ecit; ++cit)
        {
            NodeBS mod;
            const CallICFGNode* cs = (*cit);
            NodeBS delModToProp;
            bool changed = delModSideEffectOfCallSite(cs, dMod, delModToProp);
            if (changed) {
                worklist.push(edge->getSrcID());
            }
        }
    }

}
void MRGenerator::delGlobsArgsRetsPtsAnalysis(PTACallGraphNode* callGraphNode, WorkList& worklist)
{
    const SVFFunction* fun = callGraphNode->getFunction();
    NodeBS calleeDMods = funToDelModsMap[fun];
    NodeBS calleeDRefs = funToDelRefsMap[fun];
    funToDelModsMap[fun].clear();
    funToDelModsMap.erase(fun);
    funToDelRefsMap[fun].clear();
    funToDelRefsMap.erase(fun);
    for(PTACallGraphNode::iterator it = callGraphNode->InEdgeBegin(), eit = callGraphNode->InEdgeEnd();
            it!=eit; ++it)
    {
        PTACallGraphEdge* edge = *it;

        /// handle direct callsites
        for(PTACallGraphEdge::CallInstSet::iterator cit = edge->getDirectCalls().begin(),
                ecit = edge->getDirectCalls().end(); cit!=ecit; ++cit)
        {
            const CallICFGNode* cs = (*cit);
            bool changed = delGlobsArgsRetsPtsOfCallSite(cs, calleeDMods, calleeDRefs);
            if (changed) {
                worklist.push(edge->getSrcID());
            }
        }
        /// handle indirect callsites
        for(PTACallGraphEdge::CallInstSet::iterator cit = edge->getIndirectCalls().begin(),
                ecit = edge->getIndirectCalls().end(); cit!=ecit; ++cit)
        {
            const CallICFGNode* cs = (*cit);
            bool changed = delGlobsArgsRetsPtsOfCallSite(cs, calleeDMods, calleeDRefs);
            if (changed) {
                worklist.push(edge->getSrcID());
            }
        }
    }
    
}
void MRGenerator::modRefAnalysis(PTACallGraphNode* callGraphNode, WorkList& worklist)
{

    /// add ref/mod set of callee to its invocation callsites at caller
    for(PTACallGraphNode::iterator it = callGraphNode->InEdgeBegin(), eit = callGraphNode->InEdgeEnd();
            it!=eit; ++it)
    {
        PTACallGraphEdge* edge = *it;

        /// handle direct callsites
        for(PTACallGraphEdge::CallInstSet::iterator cit = edge->getDirectCalls().begin(),
                ecit = edge->getDirectCalls().end(); cit!=ecit; ++cit)
        {
            NodeBS mod, ref;
            const CallICFGNode* cs = (*cit);
            bool modrefchanged = handleCallsiteModRef(mod, ref, cs, callGraphNode->getFunction());
            if(modrefchanged)
                worklist.push(edge->getSrcID());
        }
        /// handle indirect callsites
        for(PTACallGraphEdge::CallInstSet::iterator cit = edge->getIndirectCalls().begin(),
                ecit = edge->getIndirectCalls().end(); cit!=ecit; ++cit)
        {
            NodeBS mod, ref;
            const CallICFGNode* cs = (*cit);
            bool modrefchanged = handleCallsiteModRef(mod, ref, cs, callGraphNode->getFunction());
            if(modrefchanged)
                worklist.push(edge->getSrcID());
        }
    }
}

/*!
 * Obtain the mod sets for a call, used for external ModRefInfo queries
 */
NodeBS MRGenerator::getModInfoForCall(const CallICFGNode* cs)
{
    if (isExtCall(cs->getCallSite()) && !isHeapAllocExtCall(cs->getCallSite()))
    {
        SVFStmtList& pagEdgeList = getPAGEdgesFromInst(cs->getCallSite());
        NodeBS mods;
        for (SVFStmtList::const_iterator bit = pagEdgeList.begin(), ebit =
                    pagEdgeList.end(); bit != ebit; ++bit)
        {
            const PAGEdge* edge = *bit;
            if (const StoreStmt* st = SVFUtil::dyn_cast<StoreStmt>(edge))
                mods |= pta->getPts(st->getLHSVarID()).toNodeBS();
        }
        return mods;
    }
    else
    {
        return getModSideEffectOfCallSite(cs);
    }
}

/*!
 * Obtain the ref sets for a call, used for external ModRefInfo queries
 */
NodeBS MRGenerator::getRefInfoForCall(const CallICFGNode* cs)
{
    if (isExtCall(cs->getCallSite()) && !isHeapAllocExtCall(cs->getCallSite()))
    {
        SVFStmtList& pagEdgeList = getPAGEdgesFromInst(cs->getCallSite());
        NodeBS refs;
        for (SVFStmtList::const_iterator bit = pagEdgeList.begin(), ebit =
                    pagEdgeList.end(); bit != ebit; ++bit)
        {
            const PAGEdge* edge = *bit;
            if (const LoadStmt* ld = SVFUtil::dyn_cast<LoadStmt>(edge))
                refs |= pta->getPts(ld->getRHSVarID()).toNodeBS();
        }
        return refs;
    }
    else
    {
        return getRefSideEffectOfCallSite(cs);
    }
}

/*!
 * Determine whether a CallSite instruction can mod or ref
 * any memory location
 */
ModRefInfo MRGenerator::getModRefInfo(const CallICFGNode* cs)
{
    bool ref = !getRefInfoForCall(cs).empty();
    bool mod = !getModInfoForCall(cs).empty();

    if (mod && ref)
        return ModRefInfo::ModRef;
    else if (ref)
        return ModRefInfo::Ref;
    else if (mod)
        return ModRefInfo::Mod;
    else
        return ModRefInfo::NoModRef;
}

/*!
 * Determine whether a const CallICFGNode* instruction can mod or ref
 * a specific memory location pointed by V
 */
ModRefInfo MRGenerator::getModRefInfo(const CallICFGNode* cs, const SVFValue* V)
{
    bool ref = false;
    bool mod = false;

    if (pta->getPAG()->hasValueNode(V))
    {
        const NodeBS pts(pta->getPts(pta->getPAG()->getValueNode(V)).toNodeBS());
        const NodeBS csRef = getRefInfoForCall(cs);
        const NodeBS csMod = getModInfoForCall(cs);
        NodeBS ptsExpanded, csRefExpanded, csModExpanded;
        pta->expandFIObjs(pts, ptsExpanded);
        pta->expandFIObjs(csRef, csRefExpanded);
        pta->expandFIObjs(csMod, csModExpanded);

        if (csRefExpanded.intersects(ptsExpanded))
            ref = true;
        if (csModExpanded.intersects(ptsExpanded))
            mod = true;
    }

    if (mod && ref)
        return ModRefInfo::ModRef;
    else if (ref)
        return ModRefInfo::Ref;
    else if (mod)
        return ModRefInfo::Mod;
    else
        return ModRefInfo::NoModRef;
}

/*!
 * Determine mod-ref relations between two const CallICFGNode* instructions
 */
ModRefInfo MRGenerator::getModRefInfo(const CallICFGNode* cs1, const CallICFGNode* cs2)
{
    bool ref = false;
    bool mod = false;

    /// return NoModRef neither two callsites ref or mod any memory
    if (getModRefInfo(cs1) == ModRefInfo::NoModRef || getModRefInfo(cs2) == ModRefInfo::NoModRef)
        return ModRefInfo::NoModRef;

    const NodeBS cs1Ref = getRefInfoForCall(cs1);
    const NodeBS cs1Mod = getModInfoForCall(cs1);
    const NodeBS cs2Ref = getRefInfoForCall(cs2);
    const NodeBS cs2Mod = getModInfoForCall(cs2);
    NodeBS cs1RefExpanded, cs1ModExpanded, cs2RefExpanded, cs2ModExpanded;
    pta->expandFIObjs(cs1Ref, cs1RefExpanded);
    pta->expandFIObjs(cs1Mod, cs1ModExpanded);
    pta->expandFIObjs(cs2Ref, cs2RefExpanded);
    pta->expandFIObjs(cs2Mod, cs2ModExpanded);

    /// Ref: cs1 ref memory mod by cs2
    if (cs1RefExpanded.intersects(cs2ModExpanded))
        ref = true;
    /// Mod: cs1 mod memory ref or mod by cs2
    if (cs1ModExpanded.intersects(cs2RefExpanded) || cs1ModExpanded.intersects(cs2ModExpanded))
        mod = true;
    /// ModRef: cs1 ref and mod memory mod by cs2
    if (cs1RefExpanded.intersects(cs2ModExpanded) && cs1ModExpanded.intersects(cs2ModExpanded))
        ref = mod = true;

    if (ref && mod)
        return ModRefInfo::ModRef;
    else if (ref)
        return ModRefInfo::Ref;
    else if (mod)
        return ModRefInfo::Mod;
    else
        return ModRefInfo::NoModRef;
}

std::ostream& SVF::operator<<(std::ostream &o, const MRVer& mrver)
{
    o << "MRVERID: " << mrver.getID() <<" MemRegion: " << mrver.getMR()->dumpStr() << " MRVERSION: " << mrver.getSSAVersion() << " MSSADef: " << mrver.getDef()->getType() << ", "
      << mrver.getDef()->getMR()->dumpStr() ;
    return o;
}
