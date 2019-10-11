stereo_kitti.cc{
  LoadImages(string(argv[3]), vstrImageLeft, vstrImageRight, vTimestamps);
  // Initialize Mask R-CNN
  DynaSLAM::SegmentDynObject *MaskNet;
  ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::STEREO,true);
  // Main loop
  for(int ni=0; ni<nImages; ni++){
    // Segment out the images
    cv::Mat maskLeft = cv::Mat::ones(h,w,CV_8U);
    maskLeftRCNN = MaskNet->GetSegmentation();
    maskLeft = maskLeft - maskLeftRCNNdil;
    // Pass the images to the SLAM system
    SLAM.TrackStereo(imLeft, imRight, maskLeft, maskRight, tframe);{
      /*cv::Mat System::TrackStereo(const cv::Mat &imLeft, //system.cc102
      const cv::Mat &imRight, const cv::Mat &maskLeft, 
      const cv::Mat &maskRight,const double &timestamp)*/
      cv::Mat Tcw = mpTracker->GrabImageStereo(imLeft,imRight, 
					       maskLeft, maskRight, timestamp);
      /*cv::Mat Tracking::GrabImageStereo(//Tracking.cc156
       * const cv::Mat &imRectLeft, const cv::Mat &imRectRight, 
       * const cv::Mat &maskLeft, const cv::Mat &maskRight,const double &timestamp)*/
      mCurrentFrame = Frame(mImGray,imGrayRight,
			    imMaskLeft,imMaskRight,
			    timestamp,mpORBextractorLeft,mpORBextractorRight,
			    mpORBVocabulary,mK,mDistCoef,mbf,mThDepth);
      Track();
    }
  }
  SLAM.Shutdown();
  SLAM.SaveTrajectoryKITTI("CameraTrajectory.txt");
}
rgbd_tum.cc{
  LoadImages(strAssociationFilename, vstrImageFilenamesRGB, vstrImageFilenamesD, vTimestamps);
  // Initialize Mask R-CNN
  DynaSLAM::SegmentDynObject *MaskNet;
  ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::RGBD,true);
  // Main loop
  for(int ni=0; ni<nImages; ni++){
    maskRCNN = MaskNet->GetSegmentation(imRGB,string(argv[5]),
					vstrImageFilenamesRGB[ni].replace(0,4,""));
    cv::Mat maskRCNNdil = maskRCNN.clone();
    cv::dilate(maskRCNN,maskRCNNdil, kernel);
    mask = mask - maskRCNNdil;
    // Pass the image to the SLAM system
    if (argc == 7){SLAM.TrackRGBD(imRGB,imD,mask,tframe,imRGBOut,imDOut,maskOut);}
    else {SLAM.TrackRGBD(imRGB,imD,mask,tframe);}
    SLAM.TrackRGBD(imRGB,imD,mask,tframe,imRGBOut,imDOut,maskOut);{//system.cc153
      //cv::Mat System::TrackRGBD(const cv::Mat &im, const cv::Mat &depthmap, 
      //cv::Mat &mask,const double &timestamp, cv::Mat &imRGBOut, 
      //cv::Mat &imDOut, cv::Mat &maskOut)
      cv::Mat Tcw = mpTracker->GrabImageRGBD(im,depthmap,mask,timestamp, 
					     imRGBOut,imDOut,maskOut);{
	/*cv::Mat Tracking::GrabImageRGBD(const cv::Mat &imRGB,const cv::Mat &imD, 
	cv::Mat &mask, const double &timestamp, cv::Mat &imRGBOut,
                                cv::Mat &imDOut, cv::Mat &maskOut)*///Tracking.cc206
        mCurrentFrame = Frame(mImGray,imDepth,imMask,_imRGB,timestamp,mpORBextractorLeft,mpORBVocabulary,mK,mDistCoef,mbf,mThDepth);
	LightTrack();{
	  
	}
	imRGBOut = _imRGB;
	mGeometry.GeometricModelCorrection(mCurrentFrame,imDepth,imMask);{//Geometry.cc29
	  //void Geometry::GeometricModelCorrection(
	  //const ORB_SLAM2::Frame &currentFrame,cv::Mat &imDepth, cv::Mat &mask)
	  vector<ORB_SLAM2::Frame> vRefFrames = GetRefFrames(currentFrame);{
	    
	  }
	  vector<DynKeyPoint> vDynPoints = ExtractDynPoints(vRefFrames,currentFrame);{
	    //vector<Geometry::DynKeyPoint> Geometry::ExtractDynPoints(
	         //const vector<ORB_SLAM2::Frame> &vRefFrames,
	         //const ORB_SLAM2::Frame &currentFrame)//Geometry.cc29
	    for (int i(0); i < mnRefFrames; i++){//mnRefFrames = 5
	      
	    }
	  }
	  mask = DepthRegionGrowing(vDynPoints,imDepth);
	  CombineMasks(currentFrame,mask);
	}
	mCurrentFrame = Frame(mImGray,imDepth,imMask,imRGBOut,timestamp,mpORBextractorLeft,mpORBVocabulary,mK,mDistCoef,mbf,mThDepth);
	Track();{
	  
	}
      }
    }
  }
  SLAM.Shutdown();
  SLAM.SaveTrajectoryTUM("CameraTrajectory.txt");
  //!!!!!保存轨迹
  SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
}