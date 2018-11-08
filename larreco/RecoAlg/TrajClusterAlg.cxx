//////////////////////////////////////////////////////////////////////
///
/// TrajClusterAlg
///
/// Bruce Baller, baller@fnal.gov
/// Citation: Liquid argon TPC signal formation, signal processing and reconstruction techniques
/// B. Baller 2017 JINST 12 P07010
///
///
////////////////////////////////////////////////////////////////////////

#include "larreco/RecoAlg/TrajClusterAlg.h"

namespace tca {

  //------------------------------------------------------------------------------

  TrajClusterAlg::TrajClusterAlg(fhicl::ParameterSet const& pset)
  :fCaloAlg(pset.get<fhicl::ParameterSet>("CaloAlg")), fMVAReader("Silent")
  {
    tcc.showerParentReader = &fMVAReader;
    reconfigure(pset);
    tcc.caloAlg = &fCaloAlg;
//    art::ServiceHandle<art::TFileService> tfs;
//    hist.CreateHists(*tfs);
//    tm.Initialize();
  }

  //------------------------------------------------------------------------------
  void TrajClusterAlg::reconfigure(fhicl::ParameterSet const& pset)
  {

    bool badinput = false;
    // set all configurable modes false
    tcc.modes.reset();
    
    // default mode is stepping US -> DS
    tcc.modes[kStepDir] = true;
    short userMode = 1;
    if(pset.has_key("Mode")) userMode = pset.get< short >("Mode");
    if(userMode < 0) tcc.modes[kStepDir] = false;
    if(pset.has_key("DoForecast")) tcc.doForecast = pset.get< bool >("DoForecast");
    if(pset.has_key("StudyMode")) {
      std::cout<<"StudyMode is not valid anymore. Try Study: 1 or Study: 2, etc/n";
    } // old StudyMode
    if(pset.has_key("Study")) {
      userMode = pset.get< short >("Study");
      if(userMode == 1) tcc.modes[kStudy1] = true;
      if(userMode == 2) tcc.modes[kStudy2] = true;
      if(userMode == 3) tcc.modes[kStudy3] = true;
      if(userMode == 4) tcc.modes[kStudy4] = true;
    } // new Study mode
    if(pset.has_key("SaveShowerTree")) tcc.modes[kSaveShowerTree] = pset.get<bool>("SaveShowerTree");
    if(pset.has_key("SaveCRTree")) tcc.modes[kSaveCRTree] = pset.get<bool>("SaveCRTree");
    if(pset.has_key("TagCosmics")) tcc.modes[kTagCosmics] = pset.get<bool>("TagCosmics");
    std::vector<std::string> skipAlgsVec;
    if(pset.has_key("SkipAlgs")) skipAlgsVec = pset.get<std::vector<std::string>>("SkipAlgs");
    std::vector<std::string> debugConfigVec;
    if(pset.has_key("DebugConfig")) debugConfigVec = pset.get<std::vector<std::string>>("DebugConfig");
    std::vector<std::string> specialAlgsVec;
    if(pset.has_key("SpecialAlgs")) specialAlgsVec = pset.get<std::vector<std::string>>("SpecialAlgs");
    
    tcc.hitErrFac = pset.get< float >("HitErrFac", 0.4);
    // Allow the user to specify the typical hit rms for small-angle tracks
    std::vector<float> aveHitRMS;
    if(pset.has_key("AveHitRMS")) aveHitRMS = pset.get<std::vector<float>>("AveHitRMS");
    // Turn off the call to AnalyzeHits
    if(!aveHitRMS.empty()) {
      evt.aveHitRMSValid = true;
      evt.aveHitRMS = aveHitRMS;
    }
    tcc.angleRanges         = pset.get< std::vector<float>>("AngleRanges");
    tcc.nPtsAve             = pset.get< short >("NPtsAve", 20);
    tcc.minPtsFit            = pset.get< std::vector<unsigned short >>("MinPtsFit");
    tcc.minPts               = pset.get< std::vector<unsigned short >>("MinPts");
    tcc.maxAngleCode         = pset.get< std::vector<unsigned short>>("MaxAngleCode");
    tcc.minMCSMom            = pset.get< std::vector<short >>("MinMCSMom");
    tcc.maxChi               = pset.get< float >("MaxChi", 10);
    tcc.chargeCuts           = pset.get< std::vector<float >>("ChargeCuts", {3, 0.15, 0.25});
    tcc.multHitSep           = pset.get< float >("MultHitSep", 2.5);
    tcc.kinkCuts             = pset.get< std::vector<float >>("KinkCuts", {0.4, 1.5, 4});
    tcc.qualityCuts          = pset.get< std::vector<float >>("QualityCuts", {0.8, 3});
    tcc.maxWireSkipNoSignal  = pset.get< float >("MaxWireSkipNoSignal", 1);
    tcc.maxWireSkipWithSignal= pset.get< float >("MaxWireSkipWithSignal", 100);
    tcc.projectionErrFactor  = pset.get< float >("ProjectionErrFactor", 2);
    tcc.VLAStepSize        = pset.get< float >("VLAStepSize", 1.5);
    tcc.JTMaxHitSep2         = pset.get< float >("JTMaxHitSep", 2);
    tcc.deltaRayTag       = pset.get< std::vector<short>>("DeltaRayTag", {-1, -1, -1});
    tcc.muonTag           = pset.get< std::vector<short>>("MuonTag", {-1, -1, -1, - 1});
    if(pset.has_key("ElectronTag")) tcc.electronTag = pset.get<std::vector<float>>("ElectronTag");
    tcc.showerTag         = pset.get< std::vector<float>>("ShowerTag", {-1, -1, -1, -1, -1, -1});
    std::string fMVAShowerParentWeights = "NA";
    pset.get_if_present<std::string>("MVAShowerParentWeights", fMVAShowerParentWeights);    
    tcc.chkStopCuts          = pset.get< std::vector<float>>("ChkStopCuts", {-1, -1, -1});    
    tcc.matchTruth        = pset.get< std::vector<float> >("MatchTruth", {-1, -1, -1, -1});
    tcc.vtx2DCuts      = pset.get< std::vector<float >>("Vertex2DCuts", {-1, -1, -1, -1, -1, -1, -1});
    tcc.vtx3DCuts      = pset.get< std::vector<float >>("Vertex3DCuts", {-1, -1});
    tcc.vtxScoreWeights = pset.get< std::vector<float> >("VertexScoreWeights");
    tcc.match3DCuts       = pset.get< std::vector<float >>("Match3DCuts", {-1, -1, -1, -1, -1});
    tcc.pfpStitchCuts     = pset.get< std::vector<float >>("PFPStitchCuts", {-1});
    pset.get_if_present<std::vector<float>>("TestBeamCuts", tcc.testBeamCuts);
    pset.get_if_present<std::vector<float>>("NeutralVxCuts", tcc.neutralVxCuts);
    if(tcc.JTMaxHitSep2 > 0) tcc.JTMaxHitSep2 *= tcc.JTMaxHitSep2;
    
    // in the following section we ensure that the fcl vectors are appropriately sized so that later references are valid
    if(tcc.minPtsFit.size() != tcc.minPts.size()) badinput = true;
    if(tcc.maxAngleCode.size() != tcc.minPts.size()) badinput = true;
    if(tcc.minMCSMom.size() != tcc.minPts.size()) badinput = true;
    if(badinput) throw art::Exception(art::errors::Configuration)<< "Bad input from fcl file. Vector lengths for MinPtsFit, MaxAngleRange and MinMCSMom should be defined for each reconstruction pass";
    
    if(tcc.vtx2DCuts.size() < 10) throw art::Exception(art::errors::Configuration)<<"Vertex2DCuts must be size 10\n 0 = Max length definition for short TJs\n 1 = Max vtx-TJ sep short TJs\n 2 = Max vtx-TJ sep long TJs\n 3 = Max position pull for >2 TJs\n 4 = Max vtx position error\n 5 = Min MCSMom for one of two TJs\n 6 = Min fraction of wires hit btw vtx and Tjs\n 7 = Min Score\n 8 = min ChgFrac at a vtx or merge point\n 9 = max MCSMom asymmetry, 10 = require chg on wires btw vtx and tjs in induction planes?";
    if(tcc.vtx2DCuts.size() == 10) {
      // User didn't specify a requirement for the presence of charge between a vertex and the start of the
      // vertex Tjs in induction planes. Assume that it is required
      tcc.vtx2DCuts.resize(11, 1.);
    }
    if(tcc.vtx3DCuts.size() < 2)  throw art::Exception(art::errors::Configuration)<<"Vertex3DCuts must be size 2\n 0 = Max dX (cm)\n 1 = Max pull";
    if(tcc.kinkCuts.size() != 3) throw art::Exception(art::errors::Configuration)<<"KinkCuts must be size 2\n 0 = Hard kink angle cut\n 1 = Kink angle significance\n 2 = nPts fit";
    if(tcc.kinkCuts[2] < 2) throw art::Exception(art::errors::Configuration)<<"KinkCuts[2] must be > 1";
    if(tcc.chargeCuts.size() != 3) throw art::Exception(art::errors::Configuration)<<"ChargeCuts must be size 3\n 0 = Charge pull cut\n 1 = Min allowed fractional chg RMS\n 2 = Max allowed fractional chg RMS";
    
    if(tcc.muonTag.size() != 4) throw art::Exception(art::errors::Configuration)<<"MuonTag must be size 4\n 0 = minPtsFit\n 1 = minMCSMom\n 2= maxWireSkipNoSignal\n 3 = min delta ray length for tagging";
    if(tcc.deltaRayTag.size() != 3) throw art::Exception(art::errors::Configuration)<<"DeltaRayTag must be size 3\n 0 = Max endpoint sep\n 1 = min MCSMom\n 2 = max MCSMom";
    if(tcc.chkStopCuts.size() != 3) throw art::Exception(art::errors::Configuration)<<"ChkStopCuts must be size 3\n 0 = Min Charge ratio\n 1 = Charge slope pull cut\n 2 = Charge fit chisq cut";
    if(tcc.showerTag.size() < 13) {
      std::cout<< "ShowerTag must be size 13\n 0 = Mode\n 1 = max MCSMom\n 2 = max separation (WSE units)\n 3 = Max angle diff\n 4 = Factor * rms width\n 5 = Min half width\n 6 = min total Tps\n 7 = Min Tjs\n 8 = max parent FOM\n 9 = max direction FOM 10 = max AspectRatio\n 11 = min Score to preserve a vertex\n 12 = Debug showers in CTP\n";
      std::cout<<" Fixing this problem...";
      tcc.showerTag.resize(13);
      // set the min score to 0
      tcc.showerTag[11] = 0;
      // turn off printing
      tcc.showerTag[12] = -1;
    }
    if(tcc.match3DCuts.size() < 7) {
      std::cout<<">>>>>>>> Match3DCuts has been expanded to size 7. Please update your fcl file\n";
      unsigned short oldsize = tcc.match3DCuts.size();
      tcc.match3DCuts.resize(7);
      if(oldsize < 5) std::cout<<" Setting Match3DCuts[4] = 2000 combinations\n";
      if(oldsize < 6) std::cout<<" Setting Match3DCuts[5] = 1, which disables charge asymmetry checking\n";
      tcc.match3DCuts[4] = 2000;
      tcc.match3DCuts[5] = 1;
      tcc.match3DCuts[6] = tcc.kinkCuts[0];
    }
    // convert Match3DCuts[6] from angle to cos(angle)
    tcc.match3DCuts[6] = cos(tcc.match3DCuts[6]);
    
    // check the angle ranges and convert from degrees to radians
    if(tcc.angleRanges.back() < 90) {
      mf::LogVerbatim("TC")<<"Last element of AngleRange != 90 degrees. Fixing it\n";
      tcc.angleRanges.back() = 90;
    }
    
    // convert PFP stitch cuts
    if(tcc.pfpStitchCuts.size() > 1 && tcc.pfpStitchCuts[0] > 0) {
      // square the separation cut
      tcc.pfpStitchCuts[0] *= tcc.pfpStitchCuts[0];
      // convert angle to cos
      tcc.pfpStitchCuts[1] = cos(tcc.pfpStitchCuts[1]);
    }
    // turn on TestBeam mode?
    tcc.modes[kTestBeam] = (!tcc.testBeamCuts.empty());

    
    // configure algorithm debugging. Configuration for debugging standard stepping
    // is done in Utils/AnalyzeHits when the input hit collection is passed to SetInputHits
    tcc.modes[kDebug] = false;
    tcc.dbgAlg.reset();
    for(auto strng : debugConfigVec) {
      // try to interpret this as a C:T:P:W:Tick specification or something similar
      if(!DecodeDebugString(strng)) {
        // try to set a dbgAlg bit
        for(unsigned short ib = 0; ib < AlgBitNames.size(); ++ib) {
          if(strng == AlgBitNames[ib]) {
            tcc.dbgAlg[ib] = true;
            tcc.modes[kDebug] = true;
            break;
          }
        } // ib
        // print some instructions and quit if there was a failure
        if(!tcc.modes[kDebug]) {
          DecodeDebugString("instruct");
          exit(1);
        }
      } // DecodeDebugString failed
    } // strng
/*
    if(tcc.modes[kDebug] && debug.Cryostat >= 0 && debug.TPC >= 0 && debug.Plane >= 0) {
      debug.CTP = EncodeCTP((unsigned int)debug.Cryostat, (unsigned int)debug.TPC, (unsigned int)debug.Plane);
    }
*/
    for(auto& range : tcc.angleRanges) {
      if(range < 0 || range > 90) throw art::Exception(art::errors::Configuration)<< "Invalid angle range "<<range<<" Must be 0 - 90 degrees";
      range *= M_PI / 180;
    } // range
    
    // Ensure that the size of the AlgBitNames vector is consistent with the AlgBit typedef enum
    if(kAlgBitSize != AlgBitNames.size()) throw art::Exception(art::errors::Configuration)<<"kAlgBitSize "<<kAlgBitSize<<" != AlgBitNames size "<<AlgBitNames.size();
    if(kAlgBitSize > 128) throw art::Exception(art::errors::Configuration)<<"Increase the size of UseAlgs to at least "<<kAlgBitSize;
    fAlgModCount.resize(kAlgBitSize);

    if(kFlagBitSize != StopFlagNames.size()) throw art::Exception(art::errors::Configuration)<<"kFlagBitSize "<<kFlagBitSize<<" != StopFlagNames size "<<StopFlagNames.size();
    
    if(kFlagBitSize > 8) throw art::Exception(art::errors::Configuration)<<"Increase the size of StopFlag to at least "<<kFlagBitSize;
    
    bool printHelp = false;
    for(unsigned short ib = 0; ib < AlgBitNames.size(); ++ib) tcc.useAlg[ib] = true;
    
    // turn off the special algs
    // A lightly tested algorithm that should only be turned on for testing
    tcc.useAlg[kStopAtTj] = false;
    // Do an exhaustive (and slow) check of the hit -> trajectory associations
    tcc.useAlg[kChkInTraj] = false;
    
    for(auto strng : skipAlgsVec) {
      bool gotit = false;
      if(strng == "All") {
        // turn everything off
        for(unsigned short ib = 0; ib < AlgBitNames.size(); ++ib) tcc.useAlg[ib] = false;
        gotit = true;
        break;
      } // All off
      for(unsigned short ib = 0; ib < AlgBitNames.size(); ++ib) {
        if(strng == AlgBitNames[ib]) {
          tcc.useAlg[ib] = false;
          gotit = true;
          break;
        }
      } // ib
      if(!gotit) {
        std::cout<<"******* Unknown SkipAlgs input string '"<<strng<<"'\n";
        printHelp = true;
      }
    } // strng
    if(printHelp) {
      std::cout<<"Valid AlgNames:";
      for(auto strng : AlgBitNames) std::cout<<" "<<strng;
      std::cout<<"\n";
      std::cout<<"Or specify All to turn all algs off\n";
      throw art::Exception(art::errors::Configuration)<< "Invalid SkipAlgs specification";
    }
    // overwrite any settings above with special algs
    for(auto strng : specialAlgsVec) {
      bool gotit = false;
      for(unsigned short ib = 0; ib < AlgBitNames.size(); ++ib) {
        if(strng == AlgBitNames[ib]) {
          tcc.useAlg[ib] = true;
          gotit = true;
          break;
        }
      } // ib
      if(!gotit) {
        std::cout<<"******* Unknown SpecialAlgs input string '"<<strng<<"'\n";
        printHelp = true;
      }
    } // strng
    if(printHelp) {
      std::cout<<"Valid AlgNames:";
      for(auto strng : AlgBitNames) std::cout<<" "<<strng;
      std::cout<<"\n";
      std::cout<<"Or specify All to turn all algs off\n";
      throw art::Exception(art::errors::Configuration)<< "Invalid SkipAlgs specification";
    }
    
    // Configure the TMVA reader for the shower parent BDT
    if(fMVAShowerParentWeights != "NA" && tcc.showerTag[0] > 0) ConfigureMVA(tcc, fMVAShowerParentWeights);

    if(tcc.modes[kDebug]) {
      std::cout<<"Debug mode: using algs:";
      for(unsigned short ib = 0; ib < AlgBitNames.size(); ++ib) {
        if(ib % 15 == 0) std::cout<<"\n";
        if(tcc.useAlg[ib] && ib != kKilled) std::cout<<" "<<AlgBitNames[ib];
      }
      std::cout<<"\n";
      std::cout<<"Skipping algs:";
      for(unsigned short ib = 0; ib < AlgBitNames.size(); ++ib) if(!tcc.useAlg[ib] && ib != kKilled) std::cout<<" "<<AlgBitNames[ib];
      std::cout<<"\n";
      if(debug.Slice < 0) {
        std::cout<<"Debugging in all slices\n";
      } else {
        std::cout<<"Debug sub-slice index "<<debug.Slice<<"\n";
      }
      if(debug.WorkID < 0) std::cout<<"Debug WorkID "<<debug.WorkID<<"\n";
    } // debug mode
    
    evt.eventsProcessed = 0;
   
  } // reconfigure
  
  ////////////////////////////////////////////////
  bool TrajClusterAlg::SetInputHits(std::vector<recob::Hit> const& inputHits)
  {
    // defines the pointer to the input hit collection, analyzes them,
    // initializes global counters and refreshes service references
    ClearResults();
    evt.allHits = &inputHits;
    // refresh service references
    tcc.detprop = lar::providerFrom<detinfo::DetectorPropertiesService>();
    tcc.geom = lar::providerFrom<geo::Geometry>();
    evt.WorkID = 0;
    evt.globalTjID = 0;
    evt.globalVx2ID = 0;
    evt.globalVx3ID = 0;
    evt.globalPFPID = 0;
    evt.globalS2ID = 0;
    evt.globalS3ID = 0;
    // find the average hit RMS using the full hit collection and define the
    // configuration for the current TPC
    return AnalyzeHits();
  } // SetInputHits
  
  ////////////////////////////////////////////////
  void TrajClusterAlg::RunTrajClusterAlg(std::vector<unsigned int>& hitsInSlice, int sliceID)
  {
    // Reconstruct everything using the hits in a slice
    
    if(slices.empty()) ++evt.eventsProcessed;
    if(hitsInSlice.size() < 2) return;
    if(tcc.recoSlice > 0) {
      if(sliceID != tcc.recoSlice) return;
    }
    
    if(!CreateSlice(hitsInSlice)) {
      std::cout<<"CreateSlice failed\n";
      return;
    }
    // get a reference to the stored slice
    auto& slc = slices[slices.size() - 1];
    slc.ID = sliceID;
    if(evt.aveHitRMS.size() != slc.nPlanes) throw art::Exception(art::errors::Configuration)<<" AveHitRMS vector size != the number of planes ";
    // flag high-multiplicity hits
//    AnalyzeHits(slc);    
    if(tcc.recoSlice) std::cout<<"Reconstruct "<<hitsInSlice.size()<<" hits in Slice "<<sliceID<<" in TPC "<<slc.TPCID.TPC<<"\n";
    for(unsigned short plane = 0; plane < slc.nPlanes; ++plane) {
      CTP_t inCTP = EncodeCTP(slc.TPCID.Cryostat, slc.TPCID.TPC, plane);
      ReconstructAllTraj(slc, inCTP);
      if(!slc.isValid) return;
    } // plane

    // No sense taking muon direction if delta ray tagging is disabled
//    if(tcc.deltaRayTag[0] >= 0) TagMuonDirections(slc, debug.WorkID);
    Find3DVertices(slc);
    // Look for incomplete 3D vertices that won't be recovered because there are
    // missing trajectories in a plane
    FindMissedVxTjs(slc);
    ScoreVertices(slc);
    // Define the ParentID of trajectories using the vertex score
    // TODO: decide how to print debug output
    DefineTjParents(slc, false);
    for(unsigned short plane = 0; plane < slc.nPlanes; ++plane) {
      CTP_t inCTP = EncodeCTP(slc.TPCID.Cryostat, slc.TPCID.TPC, plane);
      if(!ChkVtxAssociations(slc, inCTP)) {
        std::cout<<"RTC: ChkVtxAssociations found an error\n";
      }
    } // plane
      if(tcc.match3DCuts[0] > 0) {
        FillmAllTraj(slc);
        FindPFParticles(slc);
        // TODO: decide how to print debug output
        DefinePFPParents(slc, false);
/*
        if(tcc.modes[kTagCosmics]) {
          for(auto& pfp : slc.pfps) {
            if(pfp.ID == 0) continue;
            SaveCRInfo(slc, pfp, prt, fIsRealData);
          } // pfp
        } // TagCosmics
 */
      } // 3D matching requested
      KillPoorVertices(slc);
      FindNeutralVertices(slc);
      // Use 3D matching information to find showers in 2D. FindShowers3D returns
      // true if the algorithm was successful indicating that the matching needs to be redone
      if(tcc.showerTag[0] == 2 || tcc.showerTag[0] == 4) {
        FindShowers3D(slc);
        if(tcc.modes[kSaveShowerTree]) {
          std::cout << "SHOWER TREE STAGE NUM SIZE: "  << stv.StageNum.size() << std::endl;
          showertree->Fill();
        }
      } // 3D shower code
//    } // tpcid

//    if(tcc.studyMode) tm.StudyShowerParents(hist);

    if(!slc.isValid) {
      mf::LogVerbatim("TC")<<"RunTrajCluster failed in MakeAllTrajClusters";
      return;
    }
    
    // dump a trajectory?
    if(tcc.modes[kDebug] && tcc.dbgDump) DumpTj();

//    if (tcc.modes[kSaveCRTree]) crtree->Fill();
    
/*
    // fill some basic histograms 
    if(tcc.modes[kStudy1]) {
      for(auto& vx2 : slc.vtxs) if(vx2.ID > 0 && vx2.Score > 0) hist.fVx2Score->Fill(vx2.Score);
      for(auto& vx3 : slc.vtx3s) if(vx3.ID > 0 && vx3.Score > 0) hist.fVx3Score->Fill(vx3.Score);
    }
*/
    Finish3DShowers(slc);
    
    // count algorithm usage
    for(auto& tj : slc.tjs) {
      for(unsigned short ib = 0; ib < AlgBitNames.size(); ++ib) if(tj.AlgMod[ib]) ++fAlgModCount[ib];
    } // tj
    
    // clear vectors that are not needed later
    slc.mallTraj.clear();
    slc.matchVec.clear();
    
  } // RunTrajClusterAlg

  ////////////////////////////////////////////////
  void TrajClusterAlg::ReconstructAllTraj(TCSlice& slc, CTP_t inCTP)
  {
    // Reconstruct clusters in inCTP and put them in allTraj

    unsigned short plane = DecodeCTP(inCTP).Plane;
    if(slc.firstWire[plane] > slc.nWires[plane]) return;
    unsigned int nwires = slc.lastWire[plane] - slc.firstWire[plane] - 1;
    if(nwires > slc.nWires[plane]) return;
    
    // Make several passes through the hits with user-specified cuts for each
    // pass. In general these are to not reconstruct large angle trajectories on
    // the first pass
    float maxHitsRMS = 4 * evt.aveHitRMS[plane];
    for(unsigned short pass = 0; pass < tcc.minPtsFit.size(); ++pass) {
      for(unsigned int ii = 0; ii < nwires; ++ii) {
        // decide which way to step given the sign of StepDir
        unsigned int iwire = 0;
        unsigned int jwire = 0;
        if(tcc.modes[kStepDir]) {
          // step DS
          iwire = slc.firstWire[plane] + ii;
          jwire = iwire + 1;
        } else {
          // step US
          iwire = slc.lastWire[plane] - ii - 1;
          jwire = iwire - 1;
        }
        if(iwire > slc.wireHitRange[plane].size() - 1) continue;
        if(jwire > slc.wireHitRange[plane].size() - 1) continue;
        // skip bad wires or no hits on the wire
        if(slc.wireHitRange[plane][iwire].first < 0) continue;
        if(slc.wireHitRange[plane][jwire].first < 0) continue;
        unsigned int ifirsthit = (unsigned int)slc.wireHitRange[plane][iwire].first;
        unsigned int ilasthit = (unsigned int)slc.wireHitRange[plane][iwire].second;
        unsigned int jfirsthit = (unsigned int)slc.wireHitRange[plane][jwire].first;
        unsigned int jlasthit = (unsigned int)slc.wireHitRange[plane][jwire].second;
        for(unsigned int iht = ifirsthit; iht < ilasthit; ++iht) {
          tcc.dbgStp = (tcc.modes[kDebug] && (slc.slHits[iht].allHitsIndex == debug.Hit));
          if(tcc.dbgStp)  {
            mf::LogVerbatim("TC")<<"+++++++ Pass "<<pass<<" Found debug hit "<<slices.size()-1<<":"<<PrintHit(slc.slHits[iht])<<" iht "<<iht;
          }
          // Only consider hits that are available
          if(slc.slHits[iht].InTraj != 0) continue;
          // We hope to make a trajectory point at the hit position of iht in WSE units
          // with a direction pointing to jht
          auto& iHit = (*evt.allHits)[slc.slHits[iht].allHitsIndex];
          if(tcc.useAlg[kNewStpCuts] && LongPulseHit(iHit)) continue;
          unsigned int fromWire = iHit.WireID().Wire;
          float fromTick = iHit.PeakTime();
          float iqtot = iHit.Integral();
          float hitsRMSTick = iHit.RMS();
          std::vector<unsigned int> iHitsInMultiplet;
          if(pass == 0) {
            // only use the hit on the first pass
            iHitsInMultiplet.resize(1);
            iHitsInMultiplet[0] = iht;
          } else {
            GetHitMultiplet(slc, iht, iHitsInMultiplet);
            // ignore very high multiplicities
            if(iHitsInMultiplet.size() > 4) continue;
          }
          if(iHitsInMultiplet.size() > 1) {
            fromTick = HitsPosTick(slc, iHitsInMultiplet, iqtot, kUnusedHits);
            hitsRMSTick = HitsRMSTick(slc, iHitsInMultiplet, kUnusedHits);
          }
          if(hitsRMSTick == 0) continue;
          bool fatIHit = (hitsRMSTick > maxHitsRMS);
          if(tcc.dbgStp) mf::LogVerbatim("TC")<<" hit RMS "<<iHit.RMS()<<" BB Multiplicity "<<iHitsInMultiplet.size()<<" AveHitRMS["<<plane<<"] "<<evt.aveHitRMS[plane]<<" HitsRMSTick "<<hitsRMSTick<<" fatIHit "<<fatIHit;
          for(unsigned int jht = jfirsthit; jht < jlasthit; ++jht) {
            // Only consider hits that are available
            if(slc.slHits[iht].InTraj != 0) break;
            if(slc.slHits[jht].InTraj != 0) continue;
            // clear out any leftover work inTraj's that weren't cleaned up properly
            for(auto& slHit : slc.slHits) {
              if(slHit.InTraj < 0) {
                std::cout<<"RAT: Dirty hit "<<PrintHit(slHit)<<" EventsProcessed "<<evt.eventsProcessed<<" WorkID "<<evt.WorkID<<"\n";
                slHit.InTraj = 0;
              }
            }
            unsigned int toWire = jwire;
            auto& jHit = (*evt.allHits)[slc.slHits[jht].allHitsIndex];
            if(tcc.useAlg[kNewStpCuts] && LongPulseHit(jHit)) continue;
            float toTick = jHit.PeakTime();
            float jqtot = jHit.Integral();
            std::vector<unsigned int> jHitsInMultiplet;
            if(pass == 0) {
              // only use the hit on the first pass
              jHitsInMultiplet.resize(1);
              jHitsInMultiplet[0] = jht;
            } else {
              GetHitMultiplet(slc, jht, jHitsInMultiplet);
              // ignore very high multiplicities
              if(jHitsInMultiplet.size() > 4) continue;
            }
            hitsRMSTick = HitsRMSTick(slc, jHitsInMultiplet, kUnusedHits);
            if(hitsRMSTick == 0) continue;
            bool fatJHit = (hitsRMSTick > maxHitsRMS);
            if(pass == 0) {
              // require both hits to be consistent
              if((fatIHit && !fatJHit) || (!fatIHit && fatJHit)) {
                continue;
              }
            } else {
              // pass > 0
              if(jHitsInMultiplet.size() > 1) toTick = HitsPosTick(slc, jHitsInMultiplet, jqtot, kUnusedHits);
            }
            bool hitsOK = TrajHitsOK(slc, iHitsInMultiplet, jHitsInMultiplet);
            // Ensure that the hits StartTick and EndTick have the proper overlap
            if(!hitsOK) continue;
            // start a trajectory with direction from iht -> jht
            Trajectory work;
            if(!StartTraj(slc, work, fromWire, fromTick, toWire, toTick, inCTP, pass)) continue;
            // check for a major failure
            if(!slc.isValid) return;
            if(work.Pts.empty()) {
              if(tcc.dbgStp) mf::LogVerbatim("TC")<<"ReconstructAllTraj: StartTraj failed";
              continue;
            }
            // check for a large angle crawl
            if(work.Pts[0].AngleCode > tcc.maxAngleCode[work.Pass]) {
              if(tcc.dbgStp) mf::LogVerbatim("TC")<<"ReconstructAllTraj: Wrong angle code "<<work.Pts[0].AngleCode<<" for this pass "<<work.Pass;
              ReleaseHits(slc, work);
              continue;
            }
            work.Pts[0].DeltaRMS = tcc.hitErrFac * tcc.unitsPerTick * hitsRMSTick;
            // don't include the charge of iht since it will be low if this
            // is a starting/ending track
            work.AveChg = jqtot;
            // try to add close hits
            bool sigOK;
            AddHits(slc, work, 0, sigOK);
            // check for a major failure
            if(!slc.isValid) return;
            if(!sigOK || work.Pts[0].Chg == 0) {
              if(tcc.dbgStp) mf::LogVerbatim("TC")<<" No hits at initial trajectory point ";
              ReleaseHits(slc, work);
              continue;
            }
            // move the TP position to the hit position but don't mess with the direction
            work.Pts[0].Pos = work.Pts[0].HitPos;
            // print the header and the first TP
            if(tcc.dbgStp) PrintTrajectory("RAT", slc, work, USHRT_MAX);
            // We can't update the trajectory yet because there is only one TP.
            work.EndPt[0] = 0;
            // now try stepping away
            StepAway(slc, work);
            // check for a major failure
            if(!slc.isValid) return;
            if(tcc.dbgStp) mf::LogVerbatim("TC")<<" After first StepAway. IsGood "<<work.IsGood;
/*
            if(!work.IsGood && fTryWithNextPass) {
              StepAway(slc, work);
              if(!work.IsGood || work.NeedsUpdate) {
                if(tcc.dbgStp) mf::LogVerbatim("TC")<<" xxxxxxx StepAway failed AGAIN ";
                ReleaseHits(slc, work);
                continue;
              } // Failed again
            }
*/
            // Check the quality of the work trajectory
            CheckTraj(slc, work);
            // check for a major failure
            if(!slc.isValid) return;
            if(tcc.dbgStp) mf::LogVerbatim("TC")<<"ReconstructAllTraj: After CheckTraj EndPt "<<work.EndPt[0]<<"-"<<work.EndPt[1]<<" IsGood "<<work.IsGood;
            if(tcc.dbgStp) mf::LogVerbatim("TC")<<"StepAway done: IsGood "<<work.IsGood<<" NumPtsWithCharge "<<NumPtsWithCharge(slc, work, true)<<" cut "<<tcc.minPts[work.Pass];
            // decide if the trajectory is long enough for this pass
            if(!work.IsGood || NumPtsWithCharge(slc, work, true) < tcc.minPts[work.Pass]) {
              if(tcc.dbgStp) mf::LogVerbatim("TC")<<" xxxxxxx Not enough points "<<NumPtsWithCharge(slc, work, true)<<" minimum "<<tcc.minPts[work.Pass]<<" or !IsGood";
              ReleaseHits(slc, work);
              continue;
            }
            if(tcc.dbgStp) mf::LogVerbatim("TC")<<"ReconstructAllTraj: calling StoreTraj with npts "<<work.EndPt[1];
            slc.isValid = StoreTraj(slc, work);
            // check for a major failure
            if(!slc.isValid) return;
            if(tcc.useAlg[kChkInTraj]) {
              slc.isValid = InTrajOK(slc, "RAT");
              if(!slc.isValid) {
                mf::LogVerbatim("TC")<<"InTrajOK failed in ReconstructAllTraj";
                return;
              }
            } // use ChkInTraj
            // See if it should be split
            CheckTrajBeginChg(slc, slc.tjs.size() - 1);
            break;
          } // jht
        } // iht
      } // iwire

/* This shouldn't be necessary
      // Ensure that all tjs are in the same order and do some clean up
      for(auto& tj : slc.tjs) {
        if(tj.AlgMod[kKilled]) continue;
        if(tj.CTP != inCTP) continue;
        if(tj.StepDir != tcc.stepDir && !tj.AlgMod[kSetDir]) ReverseTraj(slc, tj);
        // Feb 14
        CheckHiMultUnusedHits(slc, tj);
      } // tj
*/
      // Tag delta rays before merging and making vertices
      TagDeltaRays(slc, inCTP);
      // Try to merge trajectories before making vertices
      bool lastPass = (pass == tcc.minPtsFit.size() - 1);
      EndMerge(slc, inCTP, lastPass);
      if(!slc.isValid) return;

      // TY: Split high charge hits near the trajectory end
      ChkHiChgHits(slc, inCTP);

      Find2DVertices(slc, inCTP, pass);
      if(!slc.isValid) return;

    } // pass
    
    // Use unused hits in all trajectories
    // BB Nov 2018: This can be a bad idea if the fcl configuration allows widely separated hits to
    // be associated with TPs
//    UseUnusedHits(slc);
    
    // make junk trajectories using nearby un-assigned hits
    if(tcc.JTMaxHitSep2 > 0) {
      FindJunkTraj(slc, inCTP);
      if(!slc.isValid) return;
    }
    TagDeltaRays(slc, inCTP);
    
    // Tag ShowerLike Tjs
    if(tcc.showerTag[0] > 0) TagShowerLike("RAT", slc, inCTP);
    
    Find2DVertices(slc, inCTP);
    SplitTrajCrossingVertices(slc, inCTP);
    // Make vertices between long Tjs and junk Tjs
    MakeJunkVertices(slc, inCTP);
    // check for a major failure
    if(!slc.isValid) return;

    // last attempt to attach Tjs to vertices
    for(unsigned short ivx = 0; ivx < slc.vtxs.size(); ++ivx) {
      auto& vx2 = slc.vtxs[ivx];
      if(vx2.ID == 0) continue;
      if(vx2.CTP != inCTP) continue;
      AttachAnyTrajToVertex(slc, ivx, tcc.dbgStp || tcc.dbg2V);
      UpdateVxEnvironment("RAT", slc, vx2, false);
    } // ivx
    
    // Check the Tj <-> vtx associations and define the vertex quality
    if(!ChkVtxAssociations(slc, inCTP)) {
      std::cout<<"RAT: ChkVtxAssociations found an error. Events processed "<<evt.eventsProcessed<<" WorkID "<<evt.WorkID<<"\n";
    }

    // TY: Improve hit assignments near vertex 
    VtxHitsSwap(slc, inCTP);

    // Refine vertices, trajectories and nearby hits
//    Refine2DVertices();
    
  } // ReconstructAllTraj
  
  //////////////////////////////////////////
  void TrajClusterAlg::FindJunkTraj(TCSlice& slc, CTP_t inCTP)
  {
    // Makes junk trajectories using unassigned hits
    
    if(!tcc.useAlg[kJunkTj]) return;

    // shouldn't have to do this but...
    for(auto& slHit : slc.slHits) {
      if(slHit.InTraj < 0) {
        std::cout<<"FJT: dirty hit "<<PrintHit(slHit)<<"\n";
        slHit.InTraj = 0;
      }
    }
    unsigned short plane = DecodeCTP(inCTP).Plane;
    std::vector<unsigned int> tHits;
    // Stay well away from the last wire in the plane
    for(unsigned int iwire = slc.firstWire[plane]; iwire < slc.lastWire[plane] - 3; ++iwire) {
      // skip bad wires or no hits on the wire
      if(slc.wireHitRange[plane][iwire].first < 0) continue;
      unsigned int jwire = iwire + 1;
      if(slc.wireHitRange[plane][jwire].first < 0) continue;
      unsigned int ifirsthit = (unsigned int)slc.wireHitRange[plane][iwire].first;
      unsigned int ilasthit = (unsigned int)slc.wireHitRange[plane][iwire].second;
      unsigned int jfirsthit = (unsigned int)slc.wireHitRange[plane][jwire].first;
      unsigned int jlasthit = (unsigned int)slc.wireHitRange[plane][jwire].second;
      for(unsigned int iht = ifirsthit; iht < ilasthit; ++iht) {
        tcc.dbgStp = (tcc.modes[kDebug] && slc.slHits[iht].allHitsIndex == debug.Hit);
        auto& islHit = slc.slHits[iht];
        if(islHit.InTraj != 0) continue;
        bool prt = (tcc.dbgStp || tcc.dbgAlg[kJunkTj]);
        if(prt) {
          mf::LogVerbatim("TC")<<"FindJunkTraj: Found debug hit "<<PrintHit(islHit)<<" iht "<<iht<<" jfirsthit "<<jfirsthit<<" jlasthit "<<jlasthit;
        }
        std::vector<unsigned int> iHits;
        GetHitMultiplet(slc, iht, iHits);
        for(unsigned int jht = jfirsthit; jht < jlasthit; ++jht) {
          auto& jslHit = slc.slHits[jht];
          if(jslHit.InTraj != 0) continue;
          if(prt && HitSep2(slc, iht, jht) < 100) mf::LogVerbatim("TC")<<" use "<<PrintHit(jslHit);
          if(HitSep2(slc, iht, jht) > tcc.JTMaxHitSep2) continue;
          std::vector<unsigned int> jHits;
          GetHitMultiplet(slc, jht, jHits);
          // check for hit overlap consistency
          if(!TrajHitsOK(slc, iHits, jHits)) continue;
          tHits.clear();
          // add the available hits and flag them
          for(auto iht : iHits) if(slc.slHits[iht].InTraj == 0) tHits.push_back(iht);
          for(auto jht : jHits) if(slc.slHits[jht].InTraj == 0) tHits.push_back(jht);
          for(auto tht : tHits) slc.slHits[tht].InTraj = -4;
          unsigned int loWire;
          if(iwire != 0) { loWire = iwire - 1; } else { loWire = 0; }
          unsigned int hiWire = jwire + 1;
          if(hiWire > slc.nWires[plane]) break;
//          if(jwire < slc.nWires[plane] - 3) { hiWire = jwire + 2; } else { hiWire = slc.nWires[plane] - 1; }
          unsigned short nit = 0;
          while(nit < 100) {
            bool hitsAdded = false;
            for(unsigned int kwire = loWire; kwire <= hiWire; ++kwire) {
              if(slc.wireHitRange[plane][kwire].first < 0) continue;
              unsigned int kfirsthit = (unsigned int)slc.wireHitRange[plane][kwire].first;
              unsigned int klasthit = (unsigned int)slc.wireHitRange[plane][kwire].second;
              for(unsigned int kht = kfirsthit; kht < klasthit; ++kht) {
                if(slc.slHits[kht].InTraj != 0) continue;
                // this shouldn't be needed but do it anyway
                if(std::find(tHits.begin(), tHits.end(), kht) != tHits.end()) continue;
                // re-purpose jHits and check for consistency
                GetHitMultiplet(slc, kht, jHits);
                if(!TrajHitsOK(slc, tHits, jHits)) continue;
                // add them all and update the wire range
                for(auto jht : jHits)  {
                  if(slc.slHits[jht].InTraj != 0) continue;
                  tHits.push_back(jht);
                  slc.slHits[jht].InTraj = -4;
                  if(kwire > hiWire) hiWire = kwire;
                  if(kwire < loWire) loWire = kwire;
                  hitsAdded = true;
                } // jht
              } // kht
            } // kwire
            if(!hitsAdded) break;
            ++nit;
            ++hiWire;
            if(hiWire >= slc.nWires[plane]) break;
          } // nit < 100
          // clear InTraj
          for(auto iht : tHits) slc.slHits[iht].InTraj = 0;
          if(prt) {
            mf::LogVerbatim myprt("TC");
            myprt<<"FJT: tHits";
            for(auto tht : tHits) myprt<<" "<<PrintHit(slc.slHits[tht]);
            myprt<<"\n";
          } // prt
          // See if this is a ghost trajectory
          if(IsGhost(slc, tHits)) break;
          if(!MakeJunkTraj(slc, tHits)) {
            if(prt) mf::LogVerbatim()<<"FJT: MakeJunkTraj failed";
            break;
          }
          if(slc.slHits[jht].InTraj > 0) break;
        } // jht
      } // iht
    } // iwire
  } // FindJunkTraj
  

/*
  ////////////////////////////////////////////////
  void TrajClusterAlg::PrepareForNextPass(TCSlice& slc, Trajectory& tj)
  {
    // Any re-sizing should have been done by the calling routine. This code updates the Pass and adjusts the number of
    // fitted points to get FitCHi < 2
    
    fTryWithNextPass = false;

    // See if there is another pass available
    if(tj.Pass > tcc.minPtsFit.size()-2) return;
    ++tj.Pass;
    
    unsigned short lastPt = tj.Pts.size() - 1;
    // Return if the last fit chisq is OK
    if(tj.Pts[lastPt].FitChi < 1.5) {
      fTryWithNextPass = true;
      return;
    }
    TrajPoint& lastTP = tj.Pts[lastPt];
    unsigned short newNTPSFit = lastTP.NTPsFit;
    // only give it a few tries before giving up
    unsigned short nit = 0;

     while(lastTP.FitChi > 1.5 && lastTP.NTPsFit > 2) {
      if(lastTP.NTPsFit > 3) newNTPSFit -= 2;
      else if(lastTP.NTPsFit == 3) newNTPSFit = 2;
      lastTP.NTPsFit = newNTPSFit;
      FitTraj(slc, tj);
      if(tcc.dbgStp) mf::LogVerbatim("TC")<<"PrepareForNextPass: FitChi is > 1.5 "<<lastTP.FitChi<<" Reduced NTPsFit to "<<lastTP.NTPsFit<<" tj.Pass "<<tj.Pass;
      if(lastTP.NTPsFit <= tcc.minPtsFit[tj.Pass]) break;
      ++nit;
      if(nit == 3) break;
    }
    // decide if the next pass should indeed be attempted
    if(lastTP.FitChi > 2) return;
    fTryWithNextPass = true;
    
  } // PrepareForNextPass
*/
  ////////////////////////////////////////////////
  void TrajClusterAlg::ChkInTraj(std::string someText, TCSlice& slc)
  {
    // Check slc.tjs -> InTraj associations
    
    if(!tcc.useAlg[kChkInTraj]) return;
    
    ++fAlgModCount[kChkInTraj];
    
    int tID;
    unsigned int iht;
    unsigned short itj = 0;
    std::vector<unsigned int> tHits;
    std::vector<unsigned int> atHits;
    for(auto& tj : slc.tjs) {
      // ignore abandoned trajectories
      if(tj.AlgMod[kKilled]) continue;
      tID = tj.ID;
      for(auto& tp : tj.Pts) {
        if(tp.Hits.size() > 16) {
          tj.AlgMod[kKilled] = true;
          mf::LogWarning("TC")<<"ChkInTraj: More than 16 hits created a UseHit bitset overflow\n";
          slc.isValid = false;
          return;
        }
      } // tp
      if(tj.AlgMod[kKilled]) {
        std::cout<<someText<<" ChkInTraj hit size mis-match in tj ID "<<tj.ID<<" AlgBitNames";
        for(unsigned short ib = 0; ib < AlgBitNames.size(); ++ib) if(tj.AlgMod[ib]) std::cout<<" "<<AlgBitNames[ib];
        std::cout<<"\n";
        continue;
      }
      tHits = PutTrajHitsInVector(tj, kUsedHits);
      if(tHits.size() < 2) {
        mf::LogVerbatim("TC")<<someText<<" ChkInTraj: Insufficient hits in traj "<<tj.ID;
        PrintTrajectory("CIT", slc, tj, USHRT_MAX);
        tj.AlgMod[kKilled] = true;
        continue;
      }
      std::sort(tHits.begin(), tHits.end());
      atHits.clear();
      for(iht = 0; iht < slc.slHits.size(); ++iht) {
        if(slc.slHits[iht].InTraj == tID) atHits.push_back(iht);
      } // iht
      if(atHits.size() < 2) {
        mf::LogVerbatim("TC")<<someText<<" ChkInTraj: Insufficient hits in atHits in traj "<<tj.ID<<" Killing it";
        tj.AlgMod[kKilled] = true;
        continue;
      }
      if(!std::equal(tHits.begin(), tHits.end(), atHits.begin())) {
        mf::LogVerbatim myprt("TC");
        myprt<<someText<<" ChkInTraj failed: inTraj - UseHit mis-match for tj ID "<<tID<<" tj.WorkID "<<tj.WorkID<<" atHits size "<<atHits.size()<<" tHits size "<<tHits.size()<<" in CTP "<<tj.CTP<<"\n";
        myprt<<"AlgMods: ";
        for(unsigned short ib = 0; ib < AlgBitNames.size(); ++ib) if(tj.AlgMod[ib]) myprt<<" "<<AlgBitNames[ib];
        myprt<<"\n";
        myprt<<"index     inTraj     UseHit \n";
        for(iht = 0; iht < atHits.size(); ++iht) {
          myprt<<"iht "<<iht<<" "<<PrintHit(slc.slHits[atHits[iht]]);
          if(iht < tHits.size()) myprt<<" "<<PrintHit(slc.slHits[tHits[iht]]);
          if(atHits[iht] != tHits[iht]) myprt<<" <<< "<<atHits[iht]<<" != "<<tHits[iht];
          myprt<<"\n";
          slc.isValid = false;
        } // iht
        if(tHits.size() > atHits.size()) {
          for(iht = atHits.size(); iht < atHits.size(); ++iht) {
            myprt<<"atHits "<<iht<<" "<<PrintHit(slc.slHits[atHits[iht]])<<"\n";
          } // iht
          PrintTrajectory("CIT", slc, tj, USHRT_MAX);
        } // tHit.size > atHits.size()
      }
      // check the VtxID
      for(unsigned short end = 0; end < 2; ++end) {
        if(tj.VtxID[end] > slc.vtxs.size()) {
          mf::LogVerbatim("TC")<<someText<<" ChkInTraj: Bad VtxID "<<tj.ID;
          std::cout<<someText<<" ChkInTraj: Bad VtxID "<<tj.ID<<" vtx size "<<slc.vtxs.size()<<"\n";
          tj.AlgMod[kKilled] = true;
          PrintTrajectory("CIT", slc, tj, USHRT_MAX);
          slc.isValid = false;
          return;
        }
      } // end
      ++itj;
      if(!slc.isValid) return;
    } // tj
    
  } // ChkInTraj
  
  //////////////////////////////////////////
  void TrajClusterAlg::FindMissedVxTjs(TCSlice& slc)
  {
    // Find missing 2D vertices in a plane due to a mis-reconstructed Tj
    
    if(!tcc.useAlg[kMisdVxTj]) return;

    bool prt = (tcc.dbgStp || tcc.dbg3V || tcc.dbgAlg[kMisdVxTj]);

    float maxdoca = 6;
    for(unsigned short iv3 = 0; iv3 < slc.vtx3s.size(); ++iv3) {
      Vtx3Store& vx3 = slc.vtx3s[iv3];
      // ignore obsolete vertices
      if(vx3.ID == 0) continue;
      // check for a completed 3D vertex
      if(vx3.Wire < 0) continue;
      unsigned short mPlane = USHRT_MAX;
      unsigned short ntj_1stPlane = USHRT_MAX;
      unsigned short ntj_2ndPlane = USHRT_MAX;
      for(unsigned short plane = 0; plane < slc.nPlanes; ++plane) {
        if(vx3.Vx2ID[plane] > 0) {
          auto& vx2 = slc.vtxs[vx3.Vx2ID[plane] - 1];
          if(ntj_1stPlane == USHRT_MAX) {
            ntj_1stPlane = vx2.NTraj;
          } else {
            ntj_2ndPlane = vx2.NTraj;
          }
          continue;
        }
        mPlane = plane;
      } // plane
      if(mPlane == USHRT_MAX) continue;
      CTP_t mCTP = EncodeCTP(vx3.TPCID.Cryostat, vx3.TPCID.TPC, mPlane);
      // X position of the purported missing vertex
      // A TP for the missing 2D vertex
      TrajPoint tp;
      tp.Pos[0] = vx3.Wire;
      tp.Pos[1] = tcc.detprop->ConvertXToTicks(vx3.X, mPlane, vx3.TPCID.TPC, vx3.TPCID.Cryostat) * tcc.unitsPerTick;
      std::vector<int> tjIDs;
      std::vector<unsigned short> tj2Pts;
      for(unsigned short itj = 0; itj < slc.tjs.size(); ++itj) {
        auto& tj = slc.tjs[itj];
        if(tj.CTP != mCTP) continue;
        if(tj.AlgMod[kKilled]) continue;
        if(tj.Pts.size() < 6) continue;
        if(tj.AlgMod[kComp3DVx]) continue;
        float doca = maxdoca;
        // find the closest distance between the vertex and the trajectory
        unsigned short closePt = 0;
        TrajPointTrajDOCA(slc, tp, tj, closePt, doca);
        if(closePt > tj.EndPt[1]) continue;
        if(prt) mf::LogVerbatim("TC")<<"MisdVxTj: 3V"<<vx3.ID<<" candidate T"<<slc.tjs[itj].ID<<" closePT "<<closePt<<" doca "<<doca;
        tjIDs.push_back(tj.ID);
        tj2Pts.push_back(closePt);
      } // itj
      // handle the case where there are one or more TJs with TPs near the ends
      // that make a vertex (a failure by Find2DVertices)
      if(tjIDs.empty()) continue;
      if(prt) mf::LogVerbatim("TC")<<" 3V"<<vx3.ID<<" mPlane "<<mPlane<<" ntj_1stPlane "<<ntj_1stPlane<<" ntj_2ndPlane "<<ntj_2ndPlane; 
    } // iv3
  } // FindMissedVxTjs

  //////////////////////////////////////////
  void TrajClusterAlg::MergeTPHits(std::vector<unsigned int>& tpHits, std::vector<recob::Hit>& newHitCol, 
                                   std::vector<unsigned int>& newHitAssns)
  {
    // merge the hits indexed by tpHits into one or more hits with the requirement that the hits
    // are on different wires
    
    if(tpHits.empty()) return;
    
    // no merge required. Just put a close copy of the single hit in the output hit collection
    if(tpHits.size() == 1) {
      if(tpHits[0] > (*evt.allHits).size() - 1) {
        std::cout<<"MergeTPHits Bad input hit index "<<tpHits[0]<<" allHits size "<<(*evt.allHits).size()<<"\n";
        return;
      }
      newHitCol.push_back(MergeTPHitsOnWire(tpHits));
      newHitAssns[tpHits[0]] = newHitCol.size() - 1;
      return;
    } // tpHits.size() == 1
    
    // split the hit list into sub-lists of hits on a single wire
    std::vector<unsigned int> wires;
    std::vector<std::vector<unsigned int>> wireHits;
    auto& firstHit = (*evt.allHits)[tpHits[0]];
    wires.push_back(firstHit.WireID().Wire);
    std::vector<unsigned int> tmp(1, tpHits[0]);
    wireHits.push_back(tmp);
    for(unsigned short ii = 1; ii < tpHits.size(); ++ii) {
      auto& hit = (*evt.allHits)[tpHits[ii]];
      unsigned int wire = hit.WireID().Wire;
      unsigned short indx = 0;
      for(indx = 0; indx < wires.size(); ++indx) if(wires[indx] == wire) break;
      if(indx == wires.size()) {
        wires.push_back(wire);
        wireHits.resize(wireHits.size() + 1);
      }
      wireHits[indx].push_back(tpHits[ii]);
    } // ii
    
    // now merge hits in each sub-list. 
    for(unsigned short indx = 0; indx < wireHits.size(); ++indx) {
      auto& hitsOnWire = wireHits[indx];
      if(hitsOnWire.empty()) {
        std::cout<<"coding error\n";
        exit(1);
      }
      newHitCol.push_back(MergeTPHitsOnWire(hitsOnWire));
      for(unsigned short ii = 0; ii < hitsOnWire.size(); ++ii) {
        newHitAssns[hitsOnWire[ii]] = newHitCol.size() - 1;
      }
    } // hitsOnWire

    return;

  } // MergeTPHits

  //////////////////////////////////////////
  recob::Hit TrajClusterAlg::MergeTPHitsOnWire(std::vector<unsigned int>& tpHits)
  {
    // merge the hits indexed by tpHits into one hit
    
    if(tpHits.empty()) return recob::Hit();
    
    // no merge required. Just return a slightly modified copy of the single hit
    if(tpHits.size() == 1) {
      if(tpHits[0] > (*evt.allHits).size() - 1) {
        std::cout<<"MergeTPHits Bad input hit index "<<tpHits[0]<<" allHits size "<<(*evt.allHits).size()<<"\n";
        return recob::Hit();
      }
      auto& oldHit = (*evt.allHits)[tpHits[0]];
      raw::TDCtick_t startTick = oldHit.PeakTime() - 3 * oldHit.RMS();
      raw::TDCtick_t endTick = oldHit.PeakTime() + 3 * oldHit.RMS();
      
      return recob::Hit(oldHit.Channel(), 
                        startTick, endTick, 
                        oldHit.PeakTime(), oldHit.SigmaPeakTime(), 
                        oldHit.RMS(), 
                        oldHit.PeakAmplitude(), oldHit.SigmaPeakAmplitude(), 
                        oldHit.SummedADC(), oldHit.Integral(), oldHit.SigmaIntegral(), 
                        1, 0, // Multiplicity, LocalIndex
                        1, 0, // GoodnessOfFit, DOF
                        oldHit.View(), 
                        oldHit.SignalType(),
                        oldHit.WireID()
                        );
    } // tpHits.size() == 1

    double integral = 0;
    double sIntegral = 0;
    double peakTime = 0;
    double sPeakTime = 0;
    double peakAmp = 0;
    double sPeakAmp = 0;
    float sumADC = 0;
    raw::TDCtick_t startTick = INT_MAX;
    raw::TDCtick_t endTick = 0;
    for(auto allHitsIndex : tpHits) {
      if(allHitsIndex > (*evt.allHits).size() - 1) return recob::Hit();
      auto& hit = (*evt.allHits)[allHitsIndex];
      if(hit.StartTick() < startTick) startTick = hit.StartTick();
      if(hit.EndTick() > endTick) endTick = hit.EndTick();
      double intgrl = hit.Integral();
      sPeakTime += intgrl * hit.SigmaPeakTime();
      sPeakAmp += intgrl * hit.SigmaPeakAmplitude();
      sumADC += hit.SummedADC();
      integral += intgrl;
      sIntegral += intgrl * hit.SigmaIntegral();
      // Get the charge normalization from an input hit
    } // tpHit
    if(integral <= 0) {
      std::cout<<"MergeTPHits found bad hit integral "<<integral<<"\n";
      return recob::Hit();
    }
    
    // Create a signal shape vector to find the rms and peak tick
    std::vector<double> shape(endTick - startTick + 1, 0.);
    for(auto allHitsIndex : tpHits) {
      auto& hit = (*evt.allHits)[allHitsIndex];
      double peakTick = hit.PeakTime();
      double rms = hit.RMS();
      double peakAmp = hit.PeakAmplitude();
      for(raw::TDCtick_t tick = startTick; tick <= endTick; ++tick) {
        double arg = ((double)tick - peakTick) / rms;
        unsigned short indx = tick - startTick;
        shape[indx] +=  peakAmp * exp(-0.5 * arg * arg);
      } // tick
    } // allHitsIndex
    
    // find the peak tick
    double sigsum = 0;
    double sigsumt = 0;
    for(raw::TDCtick_t tick = startTick; tick <= endTick; ++tick) {
      unsigned short indx = tick - startTick;
      sigsum += shape[indx];
      sigsumt += shape[indx] * tick;
    } // tick
    
    peakTime = sigsumt / sigsum;
    // Use the sigma peak time calculated in the first loop
    sPeakTime /= integral;
    
    // find the RMS
    sigsumt = 0.;
    for(raw::TDCtick_t tick = startTick; tick <= endTick; ++tick) {
      double dTick = tick - peakTime;
      unsigned short indx = tick - startTick;
      sigsumt += shape[indx] * dTick * dTick;
    }
    double rms = std::sqrt(sigsumt / sigsum);
    // get a reference to the first hit to get the charge normalization, channel, view, etc
    auto& firstHit = (*evt.allHits)[tpHits[0]];
    double chgNorm = 2.507 * firstHit.PeakAmplitude() * firstHit.RMS() / firstHit.Integral();
    // find the amplitude from the integrated charge and the RMS
    peakAmp = (float)(integral * chgNorm / (2.507 * rms));
    // Use the sigma integral calculated in the first loop
    sPeakAmp /= integral;
    // reset the start and end tick
    startTick = peakTime - 3 * rms;
    endTick = peakTime + 3 * rms;
    
    // construct the hit
    return recob::Hit(firstHit.Channel(), 
                      startTick, endTick, 
                      peakTime, sPeakTime, 
                      rms, 
                      peakAmp, sPeakAmp, 
                      sumADC, integral, sIntegral, 
                      1, 0, // Multiplicity, LocalIndex
                      1, 0, // GoodnessOfFit, DOF
                      firstHit.View(), 
                      firstHit.SignalType(),
                      firstHit.WireID()
      );

  } // MergeTPHits

  /////////////////////////////////////////
  void TrajClusterAlg::DefineShTree(TTree* t) {
    showertree = t;

    showertree->Branch("run", &evt.run, "run/I");
    showertree->Branch("subrun", &evt.subRun, "subrun/I");
    showertree->Branch("event", &evt.event, "event/I");

    showertree->Branch("BeginWir", &stv.BeginWir);
    showertree->Branch("BeginTim", &stv.BeginTim);
    showertree->Branch("BeginAng", &stv.BeginAng);
    showertree->Branch("BeginChg", &stv.BeginChg);
    showertree->Branch("BeginVtx", &stv.BeginVtx);

    showertree->Branch("EndWir", &stv.EndWir);
    showertree->Branch("EndTim", &stv.EndTim);
    showertree->Branch("EndAng", &stv.EndAng);
    showertree->Branch("EndChg", &stv.EndChg);
    showertree->Branch("EndVtx", &stv.EndVtx);

    showertree->Branch("MCSMom", &stv.MCSMom);

    showertree->Branch("PlaneNum", &stv.PlaneNum);
    showertree->Branch("TjID", &stv.TjID);
    showertree->Branch("IsShowerTj", &stv.IsShowerTj);
    showertree->Branch("ShowerID", &stv.ShowerID);
    showertree->Branch("IsShowerParent", &stv.IsShowerParent);
    showertree->Branch("StageNum", &stv.StageNum);
    showertree->Branch("StageName", &stv.StageName);

    showertree->Branch("Envelope", &stv.Envelope);
    showertree->Branch("EnvPlane", &stv.EnvPlane);
    showertree->Branch("EnvStage", &stv.EnvStage);
    showertree->Branch("EnvShowerID", &stv.EnvShowerID);

    showertree->Branch("nStages", &stv.nStages);
    showertree->Branch("nPlanes", &stv.nPlanes);

  } // end DefineShTree
/*
  /////////////////////////////////////////
  void TrajClusterAlg::DefineCRTree(TTree *t){
    crtree = t;
    crtree->Branch("run", &evt.run, "run/I");
    crtree->Branch("subrun", &evt.subRun, "subrun/I");
    crtree->Branch("event", &evt.event, "event/I");
    crtree->Branch("cr_origin", &slc.crt.cr_origin);
    crtree->Branch("cr_pfpxmin", &slc.crt.cr_pfpxmin);
    crtree->Branch("cr_pfpxmax", &slc.crt.cr_pfpxmax);
    crtree->Branch("cr_pfpyzmindis", &slc.crt.cr_pfpyzmindis);
  }
*/
  /////////////////////////////////////////
  bool TrajClusterAlg::CreateSlice(std::vector<unsigned int>& hitsInSlice)
  {
    // Defines a TCSlice struct and pushes the slice onto slices. 
    // Sets the isValid flag true if successful.
    if((*evt.allHits).empty()) return false;
    if(hitsInSlice.size() < 2) return false;
    
    TCSlice slc;
    slc.slHits.resize(hitsInSlice.size());
    bool first = true;
    unsigned int cstat = 0;
    unsigned int tpc = 0;
    unsigned int cnt = 0;
    for(auto iht : hitsInSlice) {
      if(iht > (*evt.allHits).size() - 1) return false;
      auto& hit = (*evt.allHits)[iht];
      if(first) {
        cstat = hit.WireID().Cryostat;
        tpc = hit.WireID().TPC;
        slc.TPCID = geo::TPCID(cstat, tpc);
        first = false;
      }
      if(hit.WireID().Cryostat != cstat || hit.WireID().TPC != tpc) return false;
      slc.slHits[cnt].allHitsIndex = iht;
      ++cnt;
    } // iht
    // Define the wire hit range vectors, UnitsPerTick, etc
    if(!FillWireHitRange(slc)) return false;
    slc.isValid = true;
    slices.push_back(slc);
    if(tcc.modes[kDebug] && debug.Slice >= 0 && !tcc.dbgSlc) {
      tcc.dbgSlc = ((int)(slices.size() - 1) == debug.Slice);
      if(tcc.dbgSlc) std::cout<<"Enabled debugging in sub-slice "<<slices.size() - 1<<"\n";
      if(tcc.modes[kDebug] && (unsigned int)debug.Cryostat == cstat && (unsigned int)debug.TPC == tpc && debug.Plane >= 0) {
        debug.CTP = EncodeCTP((unsigned int)debug.Cryostat, (unsigned int)debug.TPC, (unsigned int)debug.Plane);
      }
    }
    return true;
  } // CreateSlice
  
  /////////////////////////////////////////
  void TrajClusterAlg::FinishEvent()
  {
    // final steps that involve correlations between slices
    // Stitch PFParticles between TPCs
    
    // define the PFP TjUIDs vector before calling StitchPFPs
    for(auto& slc : slices) {
      if(!slc.isValid) continue;
      for(auto& pfp : slc.pfps) {
        if(pfp.ID <= 0) continue;
        pfp.TjUIDs.resize(pfp.TjIDs.size());
        for(unsigned short ii = 0; ii < pfp.TjIDs.size(); ++ii) {
          // do a sanity check while we are here
          if(pfp.TjIDs[ii] <=0 || pfp.TjIDs[ii] > (int)slc.tjs.size()) {
            std::cout<<"FinishEvent found an invalid T"<<pfp.TjIDs[ii]<<" in P"<<pfp.UID<<"\n";
            slc.isValid = false;
            continue;
          } // sanity check
          auto& tj = slc.tjs[pfp.TjIDs[ii] - 1];
          pfp.TjUIDs[ii] = tj.UID;
        } // ii
      } // pfp
    } // slc

    StitchPFPs();
    // TODO: Try to make a neutrino PFParticle here
    // Ensure that all PFParticles have a start vertex
    for(auto& slc : slices) PFPVertexCheck(slc);
  } // FinishEvent

} // namespace cluster
