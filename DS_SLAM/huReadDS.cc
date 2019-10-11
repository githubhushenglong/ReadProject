ros_tum_realtime.cc{
  LoadImages(strAssociationFilename, vstrImageFilenamesRGB, 
	     vstrImageFilenamesD, vTimestamps);
  ORB_SLAM2::System SLAM(argv[1],argv[2], argv[5],argv[6],argv[7],
			 ORB_SLAM2::System::RGBD, viewer);
  //main loop
  while(ros::ok()&&ni<nImages){
    Camera_Pose =  SLAM.TrackRGBD(imRGB,imD,tframe);{//System.cc227
      //cv::Mat System::TrackRGBD(const cv::Mat &im, const cv::Mat &depthmap, 
       // const double &timestamp)
      cv::Mat Tracking::GrabImageRGBD(//Tracking.cc211
	const cv::Mat &imRGB,const cv::Mat &imD, const double &timestamp){
	  mCurrentFrame = Frame(
	    mImGray,mImDepth,timestamp,mpORBextractorLeft,mpORBVocabulary,mThDepth);
	  // Remove dynamic points//Tracking.cc248
	  mCurrentFrame.CalculEverything(){
	    //(mImRGB,mImGray,mImDepth,mpSegment->mImgSegmentLatest);
	    //void Frame::CalculEverything( //Frame.cc226
	    //cv::Mat &imRGB, const cv::Mat &imGray,
	    //const cv::Mat &imDepth,const cv::Mat &imS)
	    //imS是分割后的图像
	    flag_mov = mpORBextractorLeft->CheckMovingKeyPoints(){
	      //实参：
	      //imGray,imS,mvKeysTemp,T_M;
	      //int ORBextractor::CheckMovingKeyPoints( 
	      //const cv::Mat &imGray, const cv::Mat &imS,
	      //std::vector<std::vector<cv::KeyPoint>>& mvKeysT,
	      //std::vector<cv::Point2f> T)
	                                  //函数定义：ORBextractor.cc1062
	      //const cv::Mat &imGray, const cv::Mat &imS,
	      //std::vector<std::vector<cv::KeyPoint>>& mvKeysT,
	      //std::vector<cv::Point2f> T)
	      
	    }
	  }
	  mImS = mpSegment->mImgSegmentLatest;
	  mImS_C = mpSegment->mImgSegment_color_final;
	  Track();
	}
    }
    segmentationTime=SLAM.mpSegment->mSegmentTime;
    Pub_CamPose(Camera_Pose); 
  }
}