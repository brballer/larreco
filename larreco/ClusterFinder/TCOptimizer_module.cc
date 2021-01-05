////////////////////////////////////////////////////////////////////////
// Class:       TCOptimizer
// Plugin Type: analyzer (art v3_05_01)
// File:        TCOptimizer_module.cc
//
// Generated at Thu Dec 17 11:45:43 2020 by Bruce Baller using cetskelgen
// from cetlib version v3_10_00.
////////////////////////////////////////////////////////////////////////

#include "art/Framework/Core/EDAnalyzer.h"
#include "art/Framework/Core/ModuleMacros.h"
#include "art/Framework/Principal/Event.h"
#include "art/Framework/Principal/Handle.h"
#include "art/Framework/Principal/Run.h"
#include "art/Framework/Principal/SubRun.h"
#include "canvas/Utilities/InputTag.h"
#include "fhiclcpp/ParameterSet.h"
#include "messagefacility/MessageLogger/MessageLogger.h"

#include <boost/algorithm/string/classification.hpp>   // Include boost::for is_any_of
#include <boost/algorithm/string/split.hpp>            // Include for boost::split


#include "lardata/DetectorInfoServices/DetectorClocksService.h"
#include "lardata/DetectorInfoServices/DetectorPropertiesService.h"
#include "larsim/MCCheater/BackTrackerService.h"
#include "nug4/ParticleNavigation/ParticleList.h"
#include "larreco/RecoAlg/TrajClusterAlg.h"
#include "larreco/RecoAlg/TCAlg/DataStructs.h"


namespace tco {

  class TCOptimizer;

  struct MatchStruct {
    int G4TrkID {INT_MAX};
    std::vector<float> nTruHits {0};
    std::vector<int> TjIDs {0};
    std::vector<float> nRecoHits {0};     ///< count of MC-matched hits used in the Tj
    std::vector<float> nTruRecoHits {0};  ///< count of the correct MC-matched hits used in the Tj
  };

  struct SumStruct {
    unsigned short parent {0};  /// The index of the FltVarStruct
    float value {0};          ///< the fcl variable setting
    float epCnt {0};          ///< count of EP entries for all events in the job
    float epSum {0};          ///< Sum of EP values for all events in the job
  };

    typedef enum {
      kHitEffFac,
      kKinkCuts0,
      kKinkCuts1,
      kKinkCuts2,
      kAngleRanges0,
      kMaxChi,
      kMaxWireSkipNoSignal,
      kProjectionErrFactor
    } VarID_t;

  // struct for a TrajClusterAlg (TCA) float fcl variable
  struct FltVarStruct {
    std::string varName;
    VarID_t varID;
    float *ptr;             ///< pointer to the TCA fcl variable
    float startValue;          ///< fcl variable value at the start of the job
    std::vector<SumStruct> ss;  ///< EP count and sum for each value
  };

  class TCOptimizer : public art::EDAnalyzer {
  public:
    explicit TCOptimizer(fhicl::ParameterSet const& pset);
    TCOptimizer(TCOptimizer const&) = delete;
    TCOptimizer(TCOptimizer&&) = delete;
    TCOptimizer& operator=(TCOptimizer const&) = delete;
    TCOptimizer& operator=(TCOptimizer&&) = delete;

    bool Initialize();
    void Restore();
    void analyze(art::Event const& e) override;

    void beginJob() override;
    void endJob() override;

  private:

    art::InputTag fInputHitsModuleLabel;
    art::Handle<std::vector<recob::Hit>> fInputHits;
    std::vector<std::string> fFclVarNames;  ///< FCL variable names defined by the user
    std::vector<FltVarStruct> fFVSs;         ///< Set of SumStructs that should be evaluated

    tca::TrajClusterAlg fTCAlg;
    void MatchHitsToTruth(art::Event const& evt, unsigned int& nMatchedHits);
    void Reconstruct(art::Event const& evt, SumStruct& ss);
    float * FclVarPtr(std::string strng, VarID_t& varID);
    void SumResults(art::Event const& evt, SumStruct& ss,
                    std::vector<unsigned int> const& tpcHits);

    std::vector<int> fHitG4TrkID;
    std::vector<MatchStruct> fG4TrkTjMatch;
    unsigned short fNPlanes;
    unsigned short fPrintLevel;
    unsigned short fMinTruHitsInPlane;
    unsigned short fWeightOption;
  };

  // struct used to sort hits by Cryo:TPC:Plane:Wire:Tick:LocalIndex
  struct HitLoc {
    unsigned int index; // index of this entry in a sort vector
    unsigned int ctp;   // encoded Cryostat, TPC and Plane
    unsigned int wire;
    int tick;         // hit StartTick using typedef int TDCtick_t in RawTypes.h
    short localIndex; // defined in Hit.h
  };

  bool
  SortHits(HitLoc const& h1, HitLoc const& h2)
  {
    // sort by hit location (Cryostat, TPC, Plane, Wire, StartTick, LocalIndex)
    if (h1.ctp != h2.ctp) return h1.ctp < h2.ctp;
    if (h1.wire != h2.wire) return h1.wire < h2.wire;
    if (h1.tick != h2.tick) return h1.tick < h2.tick;
    return h1.localIndex < h2.localIndex;
  } // SortHits

  TCOptimizer::TCOptimizer(fhicl::ParameterSet const& pset)
    : EDAnalyzer{pset}
    , fTCAlg{pset.get<fhicl::ParameterSet>("TrajClusterAlg")}
  {
    // Call appropriate consumes<>() for any products to be retrieved by this module.
    fInputHitsModuleLabel = pset.get<art::InputTag>("InputHitsModuleLabel");
    fFclVarNames = pset.get<std::vector<std::string>>("FclVarNames");
    fPrintLevel = pset.get<unsigned short>("PrintLevel", 0);
    fMinTruHitsInPlane = pset.get<unsigned short>("MinTruHitsInPlane", 3);
    fWeightOption = pset.get<unsigned short>("WeightOption", 0);
  } // constructor

  //----------------------------------------------------------------------------
  bool 
  TCOptimizer::Initialize()
  {
    // Parse the FclVarNames vector and define a vector of FCL variables whose
    // values should be optimized. This should be done pnce at job begin
    if (fFclVarNames.empty()) return false;

    unsigned short nReco = 0;
    for(auto strng : fFclVarNames) {
      VarID_t varID;
      auto varPtr = FclVarPtr(strng, varID);
      if(!varPtr) return false;
      float fromVal = 0;
      float toVal = 0;
      float step = 0;
      if (varID == kHitEffFac) {
        fromVal = 0.1; toVal = 0.8; step = 0.1;
      } else if (varID == kKinkCuts0) {
        // number of points in the kink fit
        fromVal = 4.; toVal = 10; step = 2;
      } else if (varID == kKinkCuts1) {
        // kink significance
        fromVal = 2.; toVal = 16; step = 2;
      } else if (varID == kKinkCuts2) {
        // use Charge?
        fromVal = 0.; toVal = 1; step = 1;
      } else if (varID == kAngleRanges0) {
        // Small angle tracking range boundary (degrees)
        fromVal = 30; toVal = 70; step = 10;
      } else if (varID == kMaxChi) {
        // maximum Chi/DOF for 2D trajectory fit
        fromVal = 10; toVal = 40; step = 10;
      } else if (varID == kMaxWireSkipNoSignal) {
        // Handling dead wires
        fromVal = 0; toVal = 10; step = 2;
      } else if (varID == kProjectionErrFactor) {
        // Factor applied to the angle fit error to define a window for
        // associating hits with a trajectory point
        fromVal = 1; toVal = 3; step = 1;
      } else {
        std::cout<<"invalid varID "<<varID<<"\n";
        exit(1);
      }
      FltVarStruct fvs;
      fvs.varName = strng;
      fvs.varID = varID;
      fvs.ptr = varPtr;
      fvs.startValue = *(varPtr);
      SumStruct ss;
      ss.parent = fFVSs.size();
      // create a SumStruct in a reasonable range
      for (float value = fromVal; value <= toVal; value += step) {
        if (varID == kProjectionErrFactor) std::cout<<"chk "<<value<<"\n";
        ss.value = value;
        if (varID == kAngleRanges0) ss.value *= M_PI / 180;
        fvs.ss.push_back(ss);
        ++nReco;
      } // val
      fFVSs.push_back(fvs);
    } // strng
    std::cout <<"TCO Initialize: This data set will be reconstructed ";
    std::cout << nReco << " times\n";
    return true;
  } // Initialize

  //----------------------------------------------------------------------------
  float * 
  TCOptimizer::FclVarPtr(std::string strng, VarID_t& varID )
  {
    // returns a pointer to tca::tcc.<fclVarName>

    // single variables
    if(strng.compare("HitErrFac") == 0) {
      varID = kHitEffFac;
      return &tca::tcc.hitErrFac;
    }
    if(strng.compare("MaxChi") == 0) {
      varID = kMaxChi;
      return &tca::tcc.maxChi;
    }
    if(strng.compare("MaxWireSkipNoSignal") == 0) {
      varID = kMaxWireSkipNoSignal;
      return &tca::tcc.maxWireSkipNoSignal;
    }
    if(strng.compare("ProjectionErrFactor") == 0) {
      varID = kProjectionErrFactor;
      return &tca::tcc.projectionErrFactor;
    }

    // vector elements
    std::vector<std::string> words;
    boost::split(words, strng, boost::is_any_of("[ ]"), boost::token_compress_on);
    if(words.size() > 1 && words[0].compare("KinkCuts") == 0) {
      unsigned short indx = std::stoi(words[1]);
      if (indx >= tca::tcc.kinkCuts.size()) return NULL;
      if (indx == 0) varID = kKinkCuts0;
      if (indx == 1) varID = kKinkCuts1;
      if (indx == 2) varID = kKinkCuts2;
      return &tca::tcc.kinkCuts[indx];
    } // kinkCuts
    if(words.size() > 1 && words[0].compare("AngleRanges") == 0) {
      unsigned short indx = std::stoi(words[1]);
      if (indx != 0) return NULL;
      varID = kAngleRanges0;
      return &tca::tcc.angleRanges[0];
    } // AngleRanges
    return NULL;
  } // FclVarPtr

  //----------------------------------------------------------------------------
  void
  TCOptimizer::analyze(art::Event const& evt)
  {
    std::cout<<"inside analyze "<<fInputHitsModuleLabel.label()<<"\n";
    if (!evt.getByLabel(fInputHitsModuleLabel, fInputHits))
      throw cet::exception("TCOptimizer")
        << "Failed to get a handle to hit collection '" << fInputHitsModuleLabel.label() << "'\n";
    if((*fInputHits).empty()) return;

    if (!fTCAlg.SetInputHits(*fInputHits, evt.run(), evt.event()))
      throw cet::exception("TCOptimizer")
        << "Failed to process hits from '" << fInputHitsModuleLabel.label() << "'\n";

    unsigned int nhm;
    MatchHitsToTruth(evt, nhm);
    if (fG4TrkTjMatch.empty()) return;
    if (nhm == 0) return;

    std::cout<<"update the pointers";
    // update the pointers
    for (auto& fvs : fFVSs) {
      VarID_t varID;
      fvs.ptr = FclVarPtr(fvs.varName, varID);
      std::cout<<" "<<varID;
      if(!fvs.ptr) throw cet::exception("TCOptimizer")
          << "Failed to update the pointers on event " << evt.event() << "\n";
    } // fvs
    std::cout<<" done\n";
    std::cout<<"Reco set";
    // iterate through all of the variables
    for (auto& fvs : fFVSs) {
      // iterate through all variable settings
      for (auto& ss : fvs.ss) {
        *(fvs.ptr) = ss.value;
        std::cout<<" " << *(fvs.ptr);
        Reconstruct(evt, ss);
      }
      std::cout<<" Restore";
      Restore();
      std::cout<<" done\n";
    } // fvs
    fG4TrkTjMatch.resize(0);
    fHitG4TrkID.resize(0);
    fTCAlg.ClearResults();
  } // analyze

  //----------------------------------------------------------------------------
  void
  TCOptimizer::MatchHitsToTruth(art::Event const& evt, unsigned int& nMatchedHits)
  {
    // Define the set of MCParticles (indexed by G4 Track ID) that will be used
    // to monitor performance in all TPCs
    fG4TrkTjMatch.clear();
    nMatchedHits = 0;
    auto mcps = art::Handle<std::vector<simb::MCParticle>>();
    art::InputTag mcpLabel = "largeant";
    if (!evt.getByLabel(mcpLabel, mcps)) return;
    if ((*mcps).empty()) return;
    auto const clockData = art::ServiceHandle<detinfo::DetectorClocksService const>()->DataFor(evt);
    art::ServiceHandle<cheat::BackTrackerService const> bt_serv;

    for (unsigned int mcpi = 0; mcpi < (*mcps).size(); ++mcpi) {
      auto& mcp = (*mcps)[mcpi];
      int pdg = abs(mcp.PdgCode());
      bool useIt = (pdg == 11 || pdg == 13 || pdg == 211 || pdg == 321 || pdg == 2212);
      if (!useIt) continue;
      float TMeV = 1000 * (mcp.E() - mcp.Mass());
      if (TMeV < 1) continue;
      if (pdg == 11 && TMeV < 50) continue;
      MatchStruct ms;
      ms.G4TrkID = mcp.TrackId();
      ms.TjIDs.resize(fNPlanes);
      ms.nTruHits.resize(fNPlanes);
      ms.nTruRecoHits.resize(fNPlanes);
      ms.nRecoHits.resize(fNPlanes);
      fG4TrkTjMatch.push_back(ms);
    } //  mcpi
    if(fG4TrkTjMatch.empty()) return;

    // now do the matching
    fHitG4TrkID.resize((*fInputHits).size(), INT_MAX);
    for (size_t iht = 0; iht < (*fInputHits).size(); ++iht) {
      auto& hit = (*fInputHits)[iht];
      auto tides = bt_serv->HitToTrackIDEs(clockData, hit);
      for (auto& tide : tides) {
        if (tide.energyFrac > 0.5) {
          // define the hit -> G4 track assn
          fHitG4TrkID[iht] = tide.trackID;
          ++nMatchedHits;
          break;
        } // tide.energyFrac > 0.5
      } // tide
    } // iht
    if(fPrintLevel > 2) {
      std::cout<<"MC-matched hits/total "<<nMatchedHits<<"/"<<(*fInputHits).size();
      std::cout<<" to "<<fG4TrkTjMatch.size()<<" MCParticles\n";
    }
  } // MatchHitsToTruth

  //----------------------------------------------------------------------------
  void
  TCOptimizer::Reconstruct(art::Event const& evt, SumStruct& ss)
  {
    // Reconstruct hits in all TPCs with the current fcl configuration and calculate
    // Efficiency * Purity

    auto const* geom = lar::providerFrom<geo::Geometry>();
    fNPlanes = geom->Nplanes();
    auto const clockData = 
      art::ServiceHandle<detinfo::DetectorClocksService const>()->DataFor(evt);
    auto const detProp =
      art::ServiceHandle<detinfo::DetectorPropertiesService const>()->DataFor(evt, clockData);
    art::ServiceHandle<cheat::BackTrackerService const> bt_serv;

    int sliceID = 0;
    for (const auto& tpcid : geom->IterateTPCIDs()) {
      // ignore protoDUNE dummy TPCs
      if (geom->TPC(tpcid).DriftDistance() < 25.0) continue;
      // find all hits in this tpc
      unsigned int tpc = tpcid.TPC;
      std::vector<unsigned int> tpcHits;
      for(size_t iht = 0; iht < (*fInputHits).size(); ++iht) {
        auto& hit = (*fInputHits)[iht];
        if (hit.WireID().TPC == tpc) tpcHits.push_back(iht);
      } // iht
      if(tpcHits.size() < 3) continue;
      // sort the slice hits by Cryostat, TPC, Wire, Plane, Start Tick and LocalIndex.
      // This assumes that hits with larger LocalIndex are at larger Tick.
      std::vector<HitLoc> sortVec(tpcHits.size());
      for (unsigned int indx = 0; indx < tpcHits.size(); ++indx) {
        auto& hit = (*fInputHits)[tpcHits[indx]];
        sortVec[indx].index = indx;
        sortVec[indx].ctp = tca::EncodeCTP(hit.WireID());
        sortVec[indx].wire = hit.WireID().Wire;
        sortVec[indx].tick = hit.StartTick();
        sortVec[indx].localIndex = hit.LocalIndex();
      } // iht
      std::sort(sortVec.begin(), sortVec.end(), SortHits);
      std::vector tmp = tpcHits;
      for (unsigned int ii = 0; ii < tpcHits.size(); ++ii)
        tpcHits[ii] = tmp[sortVec[ii].index];
      // clear the temp vector
      tmp.resize(0);
      sortVec.resize(0);
      ++sliceID;
      fTCAlg.RunTrajClusterAlg(clockData, detProp, tpcHits, sliceID);
      SumResults(evt, ss, tpcHits);
      tca::slices.clear();
    } // tpcid
  } // Reconstruct


  //----------------------------------------------------------------------------
  void
  TCOptimizer::SumResults(art::Event const& evt, SumStruct& ss,
                                std::vector<unsigned int> const& tpcHits)
  {
    // Calculates EP for Tjs reconstructed using tpcHits and sums the results

    // initialize the contents of fG4TrkTjMatch
    for (auto& ms : fG4TrkTjMatch) {
      for (unsigned short plane = 0; plane < fNPlanes; ++plane) {
        ms.nTruHits[plane] = 0;
        ms.TjIDs[plane] = 0;
        ms.nRecoHits[plane] = 0;
        ms.nTruRecoHits[plane] = 0;
      } // plane
    } // ms

    // count the truth hits
    for (auto iht : tpcHits) {
      if (fHitG4TrkID[iht] == INT_MAX) continue;
      unsigned short indx = 0;
      for (indx = 0; indx < fG4TrkTjMatch.size(); ++indx) 
        if (fG4TrkTjMatch[indx].G4TrkID == fHitG4TrkID[iht]) break;
      if (indx == fG4TrkTjMatch.size()) continue;
      auto& hit = (*fInputHits)[iht];
      ++fG4TrkTjMatch[indx].nTruHits[hit.WireID().Plane];
    } // iht

    // account for a major reco failure
    if (tca::slices.empty() || (!tca::slices.empty() && !tca::slices.back().isValid)) {
      for (auto& ms : fG4TrkTjMatch) {
        // increment epCnt if there were enuf hits to reconstruct
        for (auto thc : ms.nTruHits) {
          if (thc < fMinTruHitsInPlane) continue;
          if (fWeightOption == 0) {
            ++ss.epCnt;
          } else {
            ss.epCnt += thc;
          }
        } // thc
      } // ms
      return;
    } // tca::slices.empty()

    auto& slc = tca::slices.back();
    for (auto& tj : slc.tjs) {
      if (tj.AlgMod[tca::kKilled]) continue;
      // vector of trajectory hit -> G4 TrackID + count
      std::vector<std::pair<int, float>> tidCnt;
      unsigned short plane = USHRT_MAX;
      float nRecoHits = 0;
      for (auto& tp : tj.Pts) {
        if (tp.Chg <= 0) continue;
        for (unsigned short ii = 0; ii < tp.Hits.size(); ++ii) {
          if (!tp.UseHit[ii]) continue;
          if (tp.Hits[ii] >= slc.slHits.size()) {
            std::cout<<"Bad tp.Hits T"<<tj.WorkID<<" in CTP "<<tj.CTP;
            std::cout<<" "<<tp.Hits[ii]<<" max "<<slc.slHits.size();
            std::cout<<" tp.Hits size "<<tp.Hits.size()<<" ii "<<ii;
            std::cout<<"\n";
            continue;
          }
          size_t iht = slc.slHits[tp.Hits[ii]].allHitsIndex;
          if (iht >= (*fInputHits).size()) {
            std::cout<<"Bad inputHits ref "<<iht<<" "<<(*fInputHits).size()<<"\n";
            continue;
          }
          if (plane == USHRT_MAX) plane = (*fInputHits)[iht].WireID().Plane;
          if(fHitG4TrkID[iht] == INT_MAX) continue;
          int g4Tid = fHitG4TrkID[iht];
          unsigned short indx = 0;
          for (indx = 0; indx < tidCnt.size(); ++indx) if (tidCnt[indx].first == g4Tid) break;
          if (indx == tidCnt.size()) tidCnt.push_back(std::make_pair(g4Tid, 0));
          ++tidCnt[indx].second;
          // Note that only MC-matched hits are counted (to calculate purity)
          ++nRecoHits;
        } // ii
      } // tp
      if(tidCnt.empty()) continue;
      // find the match with the highest count
      unsigned short indx = 0;
      float maxCnt = 0;
      for (unsigned short ii = 0; ii < tidCnt.size(); ++ii) {
        if (tidCnt[ii].second < maxCnt) continue;
        maxCnt = tidCnt[ii].second;
        indx = ii;
      } // ii
      auto& bestTidCnt = tidCnt[indx];
      // now compare with an existing tj -> G4 track
      for (auto& g4TrkMatch : fG4TrkTjMatch) {
        if (g4TrkMatch.G4TrkID != bestTidCnt.first) continue;
        if (g4TrkMatch.nTruRecoHits[plane] > bestTidCnt.second) continue;
        // update
        g4TrkMatch.nTruRecoHits[plane] = bestTidCnt.second;
        g4TrkMatch.nRecoHits[plane] = nRecoHits;
        g4TrkMatch.TjIDs[plane] = tj.ID;
      } // g4TrkMatch
    } // tj

    // now sum the results in the SumStruct
    for (auto& g4tktjm : fG4TrkTjMatch) {
      for (unsigned short plane = 0; plane < fNPlanes; ++plane) {
        if (g4tktjm.nTruHits[plane] < fMinTruHitsInPlane) continue;
        float eff = 0;
        float pur = 0;
        if (g4tktjm.nRecoHits[plane] > 0) {
          eff = g4tktjm.nTruRecoHits[plane] / g4tktjm.nTruHits[plane];
          pur = g4tktjm.nTruRecoHits[plane] / g4tktjm.nRecoHits[plane];
        }
        float wght = 1;
        if (fWeightOption == 1) wght = g4tktjm.nTruHits[plane];
        ss.epCnt += wght;
        ss.epSum += wght * eff * pur;
        if(fPrintLevel > 1) {
          mf::LogVerbatim myprt("TCO");
          myprt << "Parent " << ss.parent;
          myprt << " G4TID "<< g4tktjm.G4TrkID;
          myprt << " tpc " << slc.TPCID.TPC;
          myprt << " pln " << plane << " nTruHits " << g4tktjm.nTruHits[plane];
          int tid = g4tktjm.TjIDs[plane];
          myprt << " T" <<tid;
          myprt << " EP " << std::setprecision(5) << eff * pur;
        } // fPrintLevel > 1
      } // plane
    } // g4TrkMatch
  } // SumResults

  //----------------------------------------------------------------------------
  void
  TCOptimizer::Restore()
  {
    // restore all variables to their starting values
    for (auto& fvs : fFVSs) *(fvs.ptr) = fvs.startValue;
  } // Restore

  //----------------------------------------------------------------------------
  void TCOptimizer::beginJob()
  {
    // Turn off 3D matching.
    std::cout<<"beginJob ";
    tca::tcc.match3DCuts[0] = -1;
    std::cout<<"Turned off 3D matching \n";
    if(!Initialize()) {
      throw cet::exception("TCOptimizer") << " Invalid FclVarNames definition";
    }
  } // beginJob

  //----------------------------------------------------------------------------
  void
  TCOptimizer::endJob()
  {
    mf::LogVerbatim myprt("TCO");
    myprt<<"TCOptimizer results using WeightOption "<<fWeightOption<<"\n";
    for (unsigned short ivar = 0; ivar < fFVSs.size(); ++ivar) {
      auto& fvs = fFVSs[ivar];
      myprt << fFclVarNames[ivar] << " EP   Count = " << fvs.ss[0].epCnt;
      myprt << " Job configuration value = ";
      if (fvs.varID == kAngleRanges0) {
        myprt << std::nearbyint(fvs.startValue*180/M_PI);
      } else {
        myprt << fvs.startValue << "\n";
      }
      // Calculate average EP
      float maxEP = 0;
      for (auto& ss : fvs.ss) {
        if (ss.epCnt < fMinTruHitsInPlane) continue;
        ss.epSum /= ss.epCnt;
        if (ss.epSum > maxEP) maxEP = ss.epSum;
      } // ss
      // check for a significant improvement (> 1%) in the range
      unsigned short nSig = 0;
      for (auto& ss : fvs.ss) {
        if (std::abs(ss.epSum - maxEP) > 0.01) ++nSig;
      } // ss
      bool significantImprovement = (nSig > 0);
      for (auto& ss : fvs.ss) {
        if (ss.epCnt < fMinTruHitsInPlane) continue;
        // convert AngleRanges to degrees
        if(fvs.varID == kAngleRanges0) {
          myprt << std::setw(5) << std::left << std::nearbyint(ss.value*180/M_PI);
        } else {
          myprt << std::setw(5) << std::left << std::setprecision(3) << ss.value;
        }
        myprt << std::setw(5) << std::right << std::setprecision(3) << ss.epSum;
        myprt << std::setw(10) << std::right << (int)ss.epCnt;
        if (ss.epSum == maxEP && significantImprovement) myprt <<" <--- use this value";
        myprt<< "\n";
      } // ss
      if (!significantImprovement) {
        myprt << " No significant optimum value for this fcl variable in the range\n";
      }
    } // fvs
  } // endJob

  DEFINE_ART_MODULE(TCOptimizer)

} // namespace tco
