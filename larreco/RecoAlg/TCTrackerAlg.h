////////////////////////////////////////////////////////////////////////
//
//
// TCTrackerAlg, formerly TCTrackerAlg
//
// Bruce Baller
//
///////////////////////////////////////////////////////////////////////
#ifndef TCTRACKERALG_H
#define TCTRACKERALG_H

// C/C++ standard libraries
#include <string>
#include <utility> // std::pair<>
#include <vector>

// framework libraries
#include "fhiclcpp/fwd.h"

// LArSoft libraries
#include "lardataobj/RecoBase/Hit.h"
#include "lardataobj/RecoBase/SpacePoint.h"
#include "larreco/Calorimetry/CalorimetryAlg.h"
#include "larreco/RecoAlg/TCAlg/DataStructs.h"
#include "larreco/RecoAlg/TCAlg/TCVertex.h"
#include "lardataobj/RecoBase/Track.h"
#include "lardataobj/RecoBase/TrackHitMeta.h"
namespace detinfo {
  class DetectorClocksData;
  class DetectorPropertiesData;
}

class TTree;

namespace tca {

  class TCTrackerAlg {
    public:


    /// @{
    /// @name Data structures for the reconstruction results

    explicit TCTrackerAlg(fhicl::ParameterSet const& pset);

    void reconfigure(fhicl::ParameterSet const& pset);

    void GetdEdxTemplates(std::string const& fileName);
    bool SetInputHits(std::vector<recob::Hit> const& inputHits, unsigned int run, unsigned int event);
    void SetInputSpts(std::vector<recob::SpacePoint> const& sptHandle) { evt.sptHandle = &sptHandle; }
    void SetSourceHits(std::vector<recob::Hit> const& srcHits);
    void ExpectSlicedHits() { evt.expectSlicedHits = true; }
    void RunTCTrackerAlg(detinfo::DetectorClocksData const& clockData,
                           detinfo::DetectorPropertiesData const& detProp,
                           std::vector<unsigned int>& hitsInSlice, int sliceID);
    bool CreateSlice(detinfo::DetectorClocksData const& clockData,
                     detinfo::DetectorPropertiesData const& detProp,
                     std::vector<unsigned int>& hitsInSlice, int sliceID);
    void MakeSpacePointsFromPFP(const tca::PFPStruct& pfp,
                                const std::vector<unsigned int>& newHitIndex, 
                                std::vector<recob::SpacePoint>& spts, 
                                std::vector<unsigned int>& sptsHit);
    void MakeTrackFromPFP(const tca::PFPStruct& pfp, const std::vector<unsigned int>& newHitIndex,
                          recob::Track& trk, std::vector<unsigned int>& trkHits);

    void FinishEvent(detinfo::DetectorPropertiesData const& detProp);

    unsigned short
    GetSlicesSize() const
    {
      return slices.size();
    }
    TCSlice const&
    GetSlice(unsigned short sliceIndex) const
    {
      return slices[sliceIndex];
    }
    void MergeTPHits(std::vector<unsigned int>& tpHits,
                     std::vector<recob::Hit>& newHitCol,
                     std::vector<unsigned int>& newHitAssns) const;

    std::vector<unsigned int> const&
    GetAlgModCount() const
    {
      return fAlgModCount;
    }
    std::vector<std::string> const&
    GetAlgBitNames() const
    {
      return AlgBitNames;
    }

    /// Deletes all the results
    void
    ClearResults()
    {
      slices.resize(0);
      seeds.resize(0);
      tjfs.resize(0);
      evt.TPCID.TPC = UINT_MAX;
      evt.sptHits.resize(0);
      evt.wireHitRange.resize(0);
      evt.tpcSrcHitRange.resize(0);
      evt.goodWire.resize(0);
      evt.wireIntersections.resize(0);
    }

  private:
    recob::Hit MergeTPHitsOnWire(std::vector<unsigned int>& tpHits) const;

    calo::CalorimetryAlg fCaloAlg;

    std::vector<unsigned int> fAlgModCount;

    void ReconstructAllTraj(detinfo::DetectorPropertiesData const& detProp,
                            TCSlice& slc,
                            CTP_t inCTP);
    // Finds junk trajectories using unassigned hits
    void FindJunkTraj(TCSlice& slc, CTP_t inCTP);
    std::vector<unsigned int> FindJTHits(const TCSlice& slc, unsigned int iht);

  }; // class TCTrackerAlg

} // namespace cluster

#endif // ifndef TCTRACKERALG_H
