stereo_ktiti.cc{
  void LoadImages(const string &strPathToSequence, vector<string> &vstrImageLeft,
                vector<string> &vstrImageRight, vector<double> &vTimestamps);
  //*****************LoadImages:++Dection********************//
  // Create SLAM system.
  ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::STEREO,true);{
    System::System(const string &strVocFile, const string &strSettingsFile, 
		   const eSensor sensor,const bool bUseViewer):
		   mSensor(sensor), mpViewer(static_cast<Viewer*>(NULL)), 
		   mbReset(false),mbActivateLocalizationMode(false),
		   mbDeactivateLocalizationMode(false)//
    mp...= new ...()
  }
  // Main loop
  for(int ni=0; ni<nImages; ni++){
    // Pass the images to the SLAM system
    SLAM.TrackStereo(imLeft,imRight,tframe);{
      cv::Mat System::TrackStereo(const cv::Mat &imLeft, const cv::Mat &imRight, 
				  const double &timestamp)//System.cc116
      //GrabImage
      cv::Mat Tcw = mpTracker->GrabImageStereo(imLeft,imRight,timestamp);{
	cv::Mat Tracking::GrabImageStereo(const cv::Mat &imRectLeft, 
					  const cv::Mat &imRectRight, 
				   const double &timestamp)//Tracking.cc167
	 mCurrentFrame = Frame(mImGray,imGrayRight,timestamp,
			       mpORBextractorLeft,mpORBextractorRight,
			mpORBVocabulary,mThDepth);
	 //*****************Frame构造函数对比,双目和RGBD********************//
	 Frame::Frame(const cv::Mat &imLeft, const cv::Mat &imRight, 
		      const double &timeStamp, ORBextractor* extractorLeft, 
	       ORBextractor* extractorRight, ORBVocabulary* voc, cv::Mat &K, 
	       cv::Mat &distCoef, const float &bf, const float &thDepth):
	       mpORBvocabulary(voc),mpORBextractorLeft(extractorLeft),
	       mpORBextractorRight(extractorRight), mTimeStamp(timeStamp), 
	       mK(K.clone()),mDistCoef(distCoef.clone()), mbf(bf), 
	       mThDepth(thDepth),mpReferenceKF(static_cast<KeyFrame*>(NULL))
	 Track();{
	   void Tracking::Track()//Tracking.cc267
	   //初始化，双目和RGBD
	   StereoInitialization();{//Tracking.cc282
	     void Tracking::StereoInitialization()//Tracking.cc509
	     //第一帧的位姿设为单位矩阵
	     mCurrentFrame.SetPose(cv::Mat::eye(4,4,CV_32F));
	     // Create KeyFrame
	     KeyFrame* pKFini = new KeyFrame(mCurrentFrame,mpMap,mpKeyFrameDB);
	     // Create MapPoints and asscoiate to KeyFrame
	  }
	  CheckReplacedInLastFrame();
	  bOK = TrackReferenceKeyFrame();{
	    mCurrentFrame.ComputeBoW();
	    int nmatches = matcher.SearchByBoW(
	      mpReferenceKF,mCurrentFrame,vpMapPointMatches);
	    Optimizer::PoseOptimization(&mCurrentFrame);
	    // Discard outliers
	    return nmatchesMap>=10;
	  }
	  bOK = TrackWithMotionModel();{//Tracking.cc313
	    bool Tracking::TrackWithMotionModel()
	    UpdateLastFrame();{//801
	      
	    }
	    
	  }
	  bOK = Relocalization();{
	    
	  }
	  /*上面完成初始位姿的跟踪后，需要使用局部地图(参考帧的一二级共视帧组成)
	   * 来进行局部地图优化，来提高鲁棒性。
	局部地图中与当前帧有相同点的关键帧序列成为一级相关帧K1，
	而与一级相关帧K1有共视地图点的关键帧序列成为二级相关帧K2，
	把局部地图中的局部地图点，投影到当前帧上，如果在当前帧的视野内
	使用位姿优化 Optimizer::PoseOptimization(&mCurrentFrame)， 进行优化，
	更新地图点的信息(关键帧的观测关系)*/
	  bOK = TrackLocalMap();////Tracking.cc400
	}
	//Track end
      }
    }
  }
  SLAM.Shutdown();
  SLAM.SaveTrajectoryKITTI("CameraTrajectory.txt");
}