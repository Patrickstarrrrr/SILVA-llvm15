//===- MemRegion.h -- Memory region -----------------------------------------//
//
//                     SVF: Static Value-Flow Analysis
//
// Copyright (C) <2013->  <Yulei Sui>
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
 * MemoryRegion.h
 *
 *  Created on: Jul 14, 2013
 *      Author: Yulei Sui
 * The implementation is based on
 * Yulei Sui, Hua Yan, Yunpeng Zhang, Jingling Xue and Zheng Zheng.
 * "Parallel Construction of Interprocedural Memory SSA Form".
 * Journal of Systems and Software (JSS'16)
 */

#ifndef MEMORYREGION_H_
#define MEMORYREGION_H_

#include "SVFIR/SVFIR.h"
#include "Graphs/PTACallGraph.h"
#include "Util/SCC.h"
#include "Util/WorkList.h"
#include "Graphs/ICFG.h"

#include <set>

namespace SVF
{

class BVDataPTAImpl;
class MemSSAStat;
class AndersenInc;

typedef NodeID MRID;
typedef NodeID MRVERID;
typedef NodeID MRVERSION;

/// Memory Region class
class MemRegion
{

public:
    typedef bool Condition;
private:
    /// region ID 0 is reserved
    static u32_t totalMRNum;
    MRID rid;
    const NodeBS cptsSet;

public:
    /// Reset totalMRNum for deterministic region IDs
    static inline void resetMRNum()
    {
        totalMRNum = 0;
    }
    /// Constructor
    MemRegion(const NodeBS& cp) :
        rid(++totalMRNum), cptsSet(cp)
    {
    }
    /// Destructor
    ~MemRegion()
    {
    }

    /// Return memory region ID
    inline MRID getMRID() const
    {
        return rid;
    }
    /// Return points-to
    inline const NodeBS &getPointsTo() const
    {
        return cptsSet;
    }
    /// Operator== overriding
    inline bool operator==(const MemRegion* rhs) const
    {
        return this->getPointsTo() == rhs->getPointsTo();
    }
    /// Dump string
    inline std::string dumpStr() const
    {
        std::string str;
        str += "pts{";
        for (NodeBS::iterator ii = cptsSet.begin(), ie = cptsSet.end();
                ii != ie; ii++)
        {
            char int2str[16];
            snprintf(int2str, sizeof(int2str), "%d", *ii);
            str += int2str;
            str += " ";
        }
        str += "}";
        return str;
    }

    /// add the hash function here to sort elements and remove
    /// and remove duplicated element in the set (binary tree comparison)
    //@{
    typedef struct equalMemRegion
    {
        bool operator()(const MemRegion* lhs, const MemRegion* rhs) const
        {
            return SVFUtil::cmpNodeBS(lhs->getPointsTo(), rhs->getPointsTo());
        }
    } equalMemRegion;
    //@}

    /// Return memory object number inside a region
    inline u32_t getRegionSize() const
    {
        return cptsSet.count();
    }
};

/*!
 * Memory Region Partitioning
 */
class MRGenerator
{

public:
    static double timeOfCollectGlobals;
    static double timeOfCollectModRefForLoadStore;
    static double timeOfCollectModRefForCall;
    static double timeOfPartitionMRs;
    static double timeOfUpdateAliasMRs;
    static double timeOfCollectCallSitePts;
    static double timeOfModRefAnalysis;

    MemSSAStat* stat;

    typedef FIFOWorkList<NodeID> WorkList;

    /// Get typedef from Pointer Analysis
    //@{
    //@}
    ///Define mem region set
    typedef OrderedSet<const MemRegion*, MemRegion::equalMemRegion> MRSet;
    typedef Map<const PAGEdge*, const SVFFunction*> PAGEdgeToFunMap;
    typedef OrderedSet<NodeBS, SVFUtil::equalNodeBS> PointsToList;
    typedef Map<const SVFFunction*, NodeBS> FunToPointsToMap;
    typedef Map<const SVFFunction*, PointsToList> FunToPointsTosMap;
    typedef OrderedMap<NodeBS, NodeBS, SVFUtil::equalNodeBS> PtsToRepPtsSetMap;

    /// Map a function to its region set
    typedef Map<const SVFFunction*, MRSet> FunToMRsMap;
    /// Map loads/stores to its mem regions,
    /// TODO:visitAtomicCmpXchgInst, visitAtomicRMWInst??
    //@{
    typedef Map<const LoadStmt*, MRSet> LoadsToMRsMap;
    typedef Map<const StoreStmt*, MRSet> StoresToMRsMap;
    typedef Map<const CallICFGNode*, MRSet> CallSiteToMRsMap;
    //@}

    /// Map loads/stores/callsites to their cpts set
    //@{
    typedef Map<const LoadStmt*, NodeBS> LoadsToPointsToMap;
    typedef Map<const StoreStmt*, NodeBS> StoresToPointsToMap;
    typedef Map<const CallICFGNode*, NodeBS> CallSiteToPointsToMap;
    //@}

    /// Maps Mod-Ref analysis
    //@{
    /// Map a function to its indirect refs/mods of memory objects
    typedef Map<const SVFFunction*, NodeBS> FunToNodeBSMap;
    /// Map a callsite to its indirect refs/mods of memory objects
    typedef Map<const CallICFGNode*, NodeBS> CallSiteToNodeBSMap;
    //@}

    typedef Map<NodeID, NodeBS> NodeToPTSSMap;

    /// SVFIR edge list
    typedef SVFIR::SVFStmtList SVFStmtList;
    /// Call Graph SCC
    typedef SCCDetection<PTACallGraph*> SCC;
    typedef std::unordered_set<const SVFFunction*> FunctionSet;
    MRSet& getMRSet()
    {
        return memRegSet;
    }

    /// Get superset cpts set
    inline const NodeBS& getRepPointsTo(const NodeBS& cpts) const
    {
        PtsToRepPtsSetMap::const_iterator it = cptsToRepCPtsMap.find(cpts);
        assert(it!=cptsToRepCPtsMap.end() && "can not find superset of cpts??");
        return it->second;
    }
    /// Get a memory region according to cpts
    const MemRegion* getMR(const NodeBS& cpts) const;

private:
    FunctionSet storeChangedFunctions;
    FunctionSet loadChangedFunctions;
    NodeBS storeResetFunctions;
    NodeBS loadResetFunctions;
    NodeBS resetFunctions;
    FunctionSet mods_lsChangedFunctions;
    FunctionSet refs_lsChangedFunctions;
    unsigned resetModSum;
    unsigned recomModSum;
    unsigned resetRefSum;
    unsigned recomRefSum;
    unsigned impactFunc;
    
    void initChangedFunctions();
    void initChangedFunctions_RR();
    
    BVDataPTAImpl* pta;
    AndersenInc* incpta;
    SCC* callGraphSCC;
    PTACallGraph* callGraph;
    bool ptrOnlyMSSA;

    /// Map a function to all its memory regions
    FunToMRsMap funToMRsMap;
    /// Map a load SVFIR Edge to its memory regions sets in order for inserting mus in Memory SSA
    LoadsToMRsMap loadsToMRsMap;
    /// Map a store SVFIR Edge to its memory regions sets in order for inserting chis in Memory SSA
    StoresToMRsMap storesToMRsMap;
    /// Map a callsite to its refs regions
    CallSiteToMRsMap callsiteToRefMRsMap;
    /// Map a callsite to its mods regions
    CallSiteToMRsMap callsiteToModMRsMap;
    /// Map a load SVFIR Edge to its CPts set map
    LoadsToPointsToMap loadsToPointsToMap;
    /// Map a store SVFIR Edge to its CPts set map
    StoresToPointsToMap	storesToPointsToMap;
    /// Map a callsite to it refs cpts set
    CallSiteToPointsToMap callsiteToRefPointsToMap;
    CallSiteToPointsToMap newCallsiteToRefPointsToMap;
    /// Map a callsite to it mods cpts set
    CallSiteToPointsToMap callsiteToModPointsToMap;
    CallSiteToPointsToMap newCallsiteToModPointsToMap;

    /// Map a function to all of its conditional points-to sets
    FunToPointsTosMap funToPointsToMap;
    FunToPointsTosMap funToPointsToMap_ls;
    
    /// Map a PAGEdge to its fun
    PAGEdgeToFunMap pagEdgeToFunMap;

    /// Map a function to its indirect uses of memory objects
    FunToPointsToMap funToRefsMap; // funToRefs := funToRefs_ls \union funToRefs_cs
    FunToPointsToMap funToRefsMap_t;

    FunToPointsToMap funToRefsMap_ls;
    FunToPointsToMap funToRefsMap_ls_t;

    FunToPointsToMap funToRefsMap_cs;
public:
    inline void dump_funToPointsToMap() {
        SVFUtil::outs() << "funtoPointsToMap:\n";
        for(auto iter = funToPointsToMap.begin(); iter != funToPointsToMap.end(); ++iter) {
            const SVFFunction* svffun = iter->first;
            PointsToList ptsl = iter->second;
            SVFUtil::outs() << "fun: " << svffun->getName() << "\n";
            SVFUtil::outs() << "pts list: [\n";
            for (auto pit = ptsl.begin(); pit != ptsl.end(); pit++) {
                SVFUtil::dumpSparseSet(*pit);
            }
            SVFUtil::outs() << "]\n";
            SVFUtil::outs() << "\n";
        }
        SVFUtil::outs() << "funtoPointsToMap_ls:\n";
        for(auto iter = funToPointsToMap_ls.begin(); iter != funToPointsToMap_ls.end(); ++iter) {
            const SVFFunction* svffun = iter->first;
            PointsToList ptsl = iter->second;
            SVFUtil::outs() << "fun: " << svffun->getName() << "\n";
            SVFUtil::outs() << "pts list ls: [\n";
            for (auto pit = ptsl.begin(); pit != ptsl.end(); pit++) {
                SVFUtil::dumpSparseSet(*pit);
            }
            SVFUtil::outs() << "]\n";
            SVFUtil::outs() << "\n";
        }
    }
    inline void dump_funToRefsMap() {
        SVFUtil::outs() << "funtoRefsMap:\n";
        for(auto iter = funToRefsMap.begin(); iter != funToRefsMap.end(); ++iter) {
            const SVFFunction* svffun = iter->first;
            NodeBS refs = iter->second;
            SVFUtil::outs() << "fun: " << svffun->getName() << "\n";
            SVFUtil::outs() << "refs: ";
            SVFUtil::dumpSparseSet(refs);
            SVFUtil::outs() << "\n";
        }
        SVFUtil::outs() << "funtoRefsMap_ls:\n";
        for(auto iter = funToRefsMap_ls.begin(); iter != funToRefsMap_ls.end(); ++iter) {
            const SVFFunction* svffun = iter->first;
            NodeBS refs = iter->second;
            SVFUtil::outs() << "fun: " << svffun->getName() << "\n";
            SVFUtil::outs() << "refs: ";
            SVFUtil::dumpSparseSet(refs);
            SVFUtil::outs() << "\n";
        }
        SVFUtil::outs() << "funtoRefsMap_cs:\n";
        for(auto iter = funToRefsMap_cs.begin(); iter != funToRefsMap_cs.end(); ++iter) {
            const SVFFunction* svffun = iter->first;
            NodeBS refs = iter->second;
            SVFUtil::outs() << "fun: " << svffun->getName() << "\n";
            SVFUtil::outs() << "refs: ";
            SVFUtil::dumpSparseSet(refs);
            SVFUtil::outs() << "\n";
        }
    }
  private:
    /// Map a function to its indirect defs of memory objects
    FunToPointsToMap funToModsMap; // funToMods := funToMods_ls \union funToMods_cs
    FunToPointsToMap funToModsMap_t;

    FunToPointsToMap funToModsMap_ls;
    FunToPointsToMap funToModsMap_ls_t;

    FunToPointsToMap funToModsMap_cs;

    FunToPointsToMap funToDelModsMap;
    FunToPointsToMap funToDelRefsMap;
public:
    inline void dump_funToModsMap() {
        SVFUtil::outs() << "funtoModsMap:\n";
        for(auto iter = funToModsMap.begin(); iter != funToModsMap.end(); ++iter) {
            const SVFFunction* svffun = iter->first;
            NodeBS mods = iter->second;
            SVFUtil::outs() << "fun: " << svffun->getName() << "\n";
            SVFUtil::outs() << "mods: ";
            SVFUtil::dumpSparseSet(mods);
            SVFUtil::outs() << "\n";
        }
        SVFUtil::outs() << "funtoModsMap_ls:\n";
        for(auto iter = funToModsMap_ls.begin(); iter != funToModsMap_ls.end(); ++iter) {
            const SVFFunction* svffun = iter->first;
            NodeBS mods = iter->second;
            SVFUtil::outs() << "fun: " << svffun->getName() << "\n";
            SVFUtil::outs() << "mods: ";
            SVFUtil::dumpSparseSet(mods);
            SVFUtil::outs() << "\n";
        }
        SVFUtil::outs() << "funtoModsMap_cs:\n";
        for(auto iter = funToModsMap_cs.begin(); iter != funToModsMap_cs.end(); ++iter) {
            const SVFFunction* svffun = iter->first;
            NodeBS mods = iter->second;
            SVFUtil::outs() << "fun: " << svffun->getName() << "\n";
            SVFUtil::outs() << "mods: ";
            SVFUtil::dumpSparseSet(mods);
            SVFUtil::outs() << "\n";
        }
    }
  private:
    /// Map a callsite to its indirect uses of memory objects
    CallSiteToPointsToMap csToRefsMap;
    /// Map a callsite to its indirect defs of memory objects
    CallSiteToPointsToMap csToModsMap;

public:
    inline void dump_csToModRef() {
        SVFUtil::outs() << "csToRefsMap_cs:\n";
        for (auto iter = csToRefsMap.begin(); iter != csToRefsMap.end(); ++iter) {
            const CallICFGNode* callnode = iter->first;
            NodeBS refs = iter->second;
            SVFUtil::outs() << "callnode: ";
            callnode->dump();
            SVFUtil::outs() << "\n";
            SVFUtil::outs() << "refs: ";
            SVFUtil::dumpSparseSet(refs);
            SVFUtil::outs() << "\n";
        }
        SVFUtil::outs() << "csToModsMap_cs:\n";
        for (auto iter = csToModsMap.begin(); iter != csToModsMap.end(); ++iter) {
            const CallICFGNode* callnode = iter->first;
            NodeBS mods = iter->second;
            SVFUtil::outs() << "callnode: ";
            callnode->dump();
            SVFUtil::outs() << "\n";
            SVFUtil::outs() << "mods: ";
            SVFUtil::dumpSparseSet(mods);
            SVFUtil::outs() << "\n";
        }
    }
    inline void debugdump() {
        SVFUtil::outs() << "$$$$$$$$$$$$$$$$$$$$$$$$$\n";
        dump_funToPointsToMap();
        dump_funToRefsMap();
        dump_funToModsMap();
        dump_csToModRef();
        SVFUtil::outs() << "globs: ";
        SVFUtil::dumpSparseSet(allGlobals);
        SVFUtil::outs() << "\n";
        SVFUtil::outs() << "$$$$$$$$$$$$$$$$$$$$$$$$$\n";
    }
private:
    /// Map a callsite to all its object might pass into its callees
    CallSiteToPointsToMap csToCallSiteArgsPtsMap;
    CallSiteToPointsToMap csToCallSiteArgsPtsMap_t;
    /// Map a callsite to all its object might return from its callees
    CallSiteToPointsToMap csToCallSiteRetPtsMap;
    CallSiteToPointsToMap csToCallSiteRetPtsMap_t;

    /// Map a pointer to its cached points-to chain;
    NodeToPTSSMap cachedPtsChainMap;

    /// All global variable SVFIR node ids
    NodeBS allGlobals;
    NodeBS allGlobals_t;
    NodeBS dGlobs;
    /// Clean up memory
    void destroy();

    //Get all objects might pass into callee from a callsite
    void collectCallSitePts(const CallICFGNode* cs);
    void collectCallSitePts_inc(const CallICFGNode* cs);
    bool hasPtsChange(NodeBS& nodes);
    //Recursive collect points-to chain
    NodeBS& CollectPtsChain(NodeID id);

    /// Return the pts chain of all callsite arguments
    inline NodeBS& getCallSiteArgsPts(const CallICFGNode* cs)
    {
        return csToCallSiteArgsPtsMap[cs];
    }
    /// Return the pts chain of the return parameter of the callsite
    inline NodeBS& getCallSiteRetPts(const CallICFGNode* cs)
    {
        return csToCallSiteRetPtsMap[cs];
    }
    /// Whether the object node is a non-local object
    /// including global, heap, and stack variable in recursions
    bool isNonLocalObject(NodeID id, const SVFFunction* curFun) const;

    /// Get all the objects in callee's modref escaped via global objects (the chain pts of globals)
    void getEscapObjviaGlobals(NodeBS& globs, const NodeBS& pts);

    /// Get reverse topo call graph scc
    void getCallGraphSCCRevTopoOrder(WorkList& worklist);

protected:
    MRGenerator(BVDataPTAImpl* p, bool ptrOnly);

    /// A set of All memory regions
    MRSet memRegSet;
    /// Map a condition pts to its rep conditional pts (super set points-to)
    PtsToRepPtsSetMap cptsToRepCPtsMap;

    /// Generate a memory region and put in into functions which use it
    void createMR(const SVFFunction* fun, const NodeBS& cpts);

    /// Collect all global variables for later escape analysis
    void collectGlobals();

    /// Generate regions for loads/stores
    virtual void collectModRefForLoadStore();
    void collectModRefForLoadStore_inc();
    void collectModRefForLoadStore_RR();
    void resetModRefInfo_RR();
    /// Generate regions for calls/rets
    virtual void collectModRefForCall();
    void collectModRefForCall_RR();
public:
    void incrementalModRefAnalysis();
    void incrementalModRefAnalysis_RR();
protected:
    /// Partition regions
    virtual void partitionMRs();

    /// Update aliased regions for loads/stores/callsites
    virtual void updateAliasMRs();

    /// Given a condition pts, insert into cptsToRepCPtsMap for region generation
    virtual void sortPointsTo(const NodeBS& cpts);

    /// Whether a region is aliased with a conditional points-to
    virtual inline bool isAliasedMR(const NodeBS& cpts, const MemRegion* mr)
    {
        return mr->getPointsTo().intersects(cpts);
    }
    /// Get all aliased mem regions from function fun according to cpts
    virtual inline void getAliasMemRegions(MRSet& aliasMRs, const NodeBS& cpts, const SVFFunction* fun)
    {
        for(MRSet::const_iterator it = funToMRsMap[fun].begin(), eit = funToMRsMap[fun].end(); it!=eit; ++it)
        {
            if(isAliasedMR(cpts,*it))
                aliasMRs.insert(*it);
        }
    }

    /// Get memory regions for a load statement according to cpts.
    virtual inline void getMRsForLoad(MRSet& aliasMRs, const NodeBS& cpts, const SVFFunction*)
    {
        const MemRegion* mr = getMR(cpts);
        aliasMRs.insert(mr);
    }

    /// Get memory regions for call site ref according to cpts.
    virtual inline void getMRsForCallSiteRef(MRSet& aliasMRs, const NodeBS& cpts, const SVFFunction*)
    {
        const MemRegion* mr = getMR(cpts);
        aliasMRs.insert(mr);
    }

    /// Mod-Ref analysis for callsite invoking this callGraphNode
    virtual void modRefAnalysis(PTACallGraphNode* callGraphNode, WorkList& worklist);
    void delModAnalysis(PTACallGraphNode* callGraphNode, WorkList& worklist);
    void delRefAnalysis(PTACallGraphNode* callGraphNode, WorkList& worklist);
    void delGlobsArgsRetsPtsAnalysis(PTACallGraphNode* callGraphNode, WorkList& worklist);
    /// Get Mod-Ref of a callee function
    virtual bool handleCallsiteModRef(NodeBS& mod, NodeBS& ref, const CallICFGNode* cs, const SVFFunction* fun);
    bool handleCallsiteModDel(NodeBS& mods, const CallICFGNode* cs, const SVFFunction* fun);
    bool handleCallsiteRefDel(NodeBS& refs, const CallICFGNode* cs, const SVFFunction* fun);

    /// Add cpts to store/load
    //@{
    inline void addCPtsToStore_inc(NodeBS& cpts, const StoreStmt *st, const SVFFunction* fun)
    {
        storesToPointsToMap[st] = cpts;
        // funToPointsToMap[fun].insert(cpts); // TODO: handle funToPointsToMap -- wjy
        funToPointsToMap_ls[fun].insert(cpts);
        addModSideEffectOfFunction_loadStore_inc(fun, cpts);
    }
    inline void addCPtsToStore(NodeBS& cpts, const StoreStmt *st, const SVFFunction* fun)
    {
        storesToPointsToMap[st] = cpts;
        // funToPointsToMap[fun].insert(cpts);
        funToPointsToMap_ls[fun].insert(cpts);
        // addModSideEffectOfFunction(fun,cpts);
        addModSideEffectOfFunction_loadStore(fun, cpts);
    }

    inline void addCPtsToLoad_inc(NodeBS& cpts, const LoadStmt *ld, const SVFFunction* fun)
    {
        loadsToPointsToMap[ld] = cpts;
        // funToPointsToMap[fun].insert(cpts); // TODO: handle funToPointsToMap -- wjy
        funToPointsToMap_ls[fun].insert(cpts);
        // addRefSideEffectOfFunction(fun,cpts);
        addRefSideEffectOfFunction_loadStore_inc(fun, cpts);
    }
    inline void addCPtsToLoad(NodeBS& cpts, const LoadStmt *ld, const SVFFunction* fun)
    {
        loadsToPointsToMap[ld] = cpts;
        // funToPointsToMap[fun].insert(cpts);
        funToPointsToMap_ls[fun].insert(cpts);
        // addRefSideEffectOfFunction(fun,cpts);
        addRefSideEffectOfFunction_loadStore(fun, cpts);
    }
    
    inline void addCPtsToCallSiteRefs(NodeBS& cpts, const CallICFGNode* cs)
    {
        callsiteToRefPointsToMap[cs] |= cpts;
        funToPointsToMap[cs->getCaller()].insert(cpts);
    }
    inline void addCPtsToCallSiteMods(NodeBS& cpts, const CallICFGNode* cs)
    {
        callsiteToModPointsToMap[cs] |= cpts;
        funToPointsToMap[cs->getCaller()].insert(cpts);
    }
    inline bool hasCPtsList(const SVFFunction* fun) const
    {
        return funToPointsToMap.find(fun)!=funToPointsToMap.end();
    }
    inline PointsToList& getPointsToList(const SVFFunction* fun)
    {
        return funToPointsToMap[fun];
    }
    inline FunToPointsTosMap& getFunToPointsToList()
    {
        return funToPointsToMap;
    }
    //@}
    /// Add/Get methods for side-effect of functions and callsites
    //@{
    /// Add indirect uses an memory object in the function
    void addRefSideEffectOfFunction(const SVFFunction* fun, const NodeBS& refs);
    void addRefSideEffectOfFunction_loadStore(const SVFFunction* fun, const NodeBS& refs);
    void addRefSideEffectOfFunction_callSite(const SVFFunction* fun, const NodeBS& refs);
    void delRefSideEffectOfFunction_callSite(const SVFFunction* fun, const NodeBS& refs, NodeBS& delRefs);
    void addRefSideEffectOfFunction_loadStore_inc(const SVFFunction* fun, const NodeBS& refs);
    
    /// Add indirect def an memory object in the function
    void addModSideEffectOfFunction(const SVFFunction* fun, const NodeBS& mods);
    void addModSideEffectOfFunction_loadStore(const SVFFunction* fun, const NodeBS& mods);
    void addModSideEffectOfFunction_callSite(const SVFFunction* fun, const NodeBS& mods);
    void delModSideEffectOfFunction_callSite(const SVFFunction* fun, const NodeBS& mods, NodeBS& delMods);
    void addModSideEffectOfFunction_loadStore_inc(const SVFFunction* fun, const NodeBS& mods);
    
    /// Add indirect uses an memory object in the function
    bool addRefSideEffectOfCallSite(const CallICFGNode* cs, const NodeBS& refs);
    bool delRefSideEffectOfCallSite(const CallICFGNode* cs, const NodeBS& refs, NodeBS& delRefToProp);
    /// Add indirect def an memory object in the function
    bool addModSideEffectOfCallSite(const CallICFGNode* cs, const NodeBS& mods);
    bool delModSideEffectOfCallSite(const CallICFGNode* cs, const NodeBS& mods, NodeBS& delModToProp);

    bool delGlobsArgsRetsPtsOfCallSite(const CallICFGNode* cs, const NodeBS& calleeDMods, const NodeBS& calleeDRefs);
    /// Get indirect refs of a function
    inline const NodeBS& getRefSideEffectOfFunction(const SVFFunction* fun)
    {
        return funToRefsMap[fun];
    }
    /// Get indirect mods of a function
    inline const NodeBS& getModSideEffectOfFunction(const SVFFunction* fun)
    {
        return funToModsMap[fun];
    }
    /// Get indirect refs of a callsite
    inline const NodeBS& getRefSideEffectOfCallSite(const CallICFGNode* cs)
    {
        return csToRefsMap[cs];
    }
    /// Get indirect mods of a callsite
    inline const NodeBS& getModSideEffectOfCallSite(const CallICFGNode* cs)
    {
        return csToModsMap[cs];
    }
    /// Has indirect refs of a callsite
    inline bool hasRefSideEffectOfCallSite(const CallICFGNode* cs)
    {
        return csToRefsMap.find(cs) != csToRefsMap.end();
    }
    /// Has indirect mods of a callsite
    inline bool hasModSideEffectOfCallSite(const CallICFGNode* cs)
    {
        return csToModsMap.find(cs) != csToModsMap.end();
    }
    //@}

public:
    inline u32_t getMRNum() const
    {
        return memRegSet.size();
    }

    /// Destructor
    virtual ~MRGenerator()
    {
        destroy();
    }

    /// Start generating memory regions
    virtual void generateMRs_exh_step1(MemSSAStat* _stat); // collectGlobals() + collectMR4LS() + collectMR4C()
    virtual void generateMRs(MemSSAStat* _stat);
    virtual void generateMRs_exh_step2(MemSSAStat* _stat); // partitionMRs() + updateAliasMRs()
    /// Get the function which SVFIR Edge located
    const SVFFunction* getFunction(const PAGEdge* pagEdge) const
    {
        PAGEdgeToFunMap::const_iterator it = pagEdgeToFunMap.find(pagEdge);
        assert(it!=pagEdgeToFunMap.end() && "can not find its function, it is a global SVFIR edge");
        return it->second;
    }
    /// Get Memory Region set
    //@{
    inline MRSet& getFunMRSet(const SVFFunction* fun)
    {
        return funToMRsMap[fun];
    }
    inline MRSet& getLoadMRSet(const LoadStmt* load)
    {
        return loadsToMRsMap[load];
    }
    inline MRSet& getStoreMRSet(const StoreStmt* store)
    {
        return storesToMRsMap[store];
    }
    inline bool hasRefMRSet(const CallICFGNode* cs)
    {
        return callsiteToRefMRsMap.find(cs)!=callsiteToRefMRsMap.end();
    }
    inline bool hasModMRSet(const CallICFGNode* cs)
    {
        return callsiteToModMRsMap.find(cs)!=callsiteToModMRsMap.end();
    }
    inline MRSet& getCallSiteRefMRSet(const CallICFGNode* cs)
    {
        return callsiteToRefMRsMap[cs];
    }
    inline MRSet& getCallSiteModMRSet(const CallICFGNode* cs)
    {
        return callsiteToModMRsMap[cs];
    }
    //@}
    /// Whether this instruction has SVFIR Edge
    bool hasSVFStmtList(const SVFInstruction* inst);
    /// Given an instruction, get all its the PAGEdge (statement) in sequence
    SVFStmtList& getPAGEdgesFromInst(const SVFInstruction* inst);

    /// getModRefInfo APIs
    //@{
    /// Collect mod ref for external callsite other than heap alloc external call
    NodeBS getModInfoForCall(const CallICFGNode* cs);
    NodeBS getRefInfoForCall(const CallICFGNode* cs);
    ModRefInfo getModRefInfo(const CallICFGNode* cs);
    ModRefInfo getModRefInfo(const CallICFGNode* cs, const SVFValue* V);
    ModRefInfo getModRefInfo(const CallICFGNode* cs1, const CallICFGNode* cs2);
    //@}

};

} // End namespace SVF

#endif /* MEMORYREGION_H_ */
