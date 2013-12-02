#include <plugins/imagecapturer/CImageCapturer.h>


namespace beliefstate {
  CImageCapturer::CImageCapturer() {
    m_strImagesTopic = "";
  }

  CImageCapturer::~CImageCapturer() {
  }

  bool CImageCapturer::fileExists(string strFileName) {
    ifstream ifile(strFileName.c_str());
  
    if(ifile) {
      return true;
    }
  
    return false;
  }

  void CImageCapturer::freeFilename(string& strFileName, string strWorkingDirectory) {
    int nIndex = 0;
    string strBase = strWorkingDirectory + strFileName;
  
    while(this->fileExists(strBase)) {
      stringstream sts;
      sts << nIndex;
      sts << "_";
      sts << strFileName;
    
      nIndex++;
    
      strBase = strWorkingDirectory + sts.str();
    }
  
    strFileName = strBase;
  }

  bool CImageCapturer::captureFromTopic(string strTopicName, string& strFileName, string strWorkingDirectory, bool bUseFreeName) {
    int nTimeout = 1000;
    bool bReturnvalue = false;
    bool bGoon = true;
    m_bReceived = false;
  
    ros::NodeHandle nh;
    m_subImage = nh.subscribe(strTopicName, 1, &CImageCapturer::imageCallback, this);
  
    while(bGoon) {
      nTimeout--;
    
      ros::Duration(0.01).sleep();
      ros::spinOnce();
    
      if(nTimeout <= 0 || m_bReceived) {
	bGoon = false;
      }
    }
  
    m_subImage.shutdown();
  
    if(m_bReceived) {
      cv_bridge::CvImagePtr cv_ptr;
    
      try {
	cv_ptr = cv_bridge::toCvCopy(m_imgReceived, sensor_msgs::image_encodings::BGR8);
      
	cv::Mat imgMat = cv_ptr->image;
      
	if(bUseFreeName) {
	  this->freeFilename(strFileName, strWorkingDirectory);
	} else {
	  strFileName = strWorkingDirectory + strFileName;
	}
      
	cv::imwrite(strFileName, imgMat);
      
	cout << "Image from topic '" << strTopicName << "' file written to '" << strFileName << "'" << endl;
      
	if(m_strImagesTopic != "") {
	  m_pubImages.publish(m_imgReceived);
	
	  cout << "Image from topic '" << strTopicName << "' published to '" << m_strImagesTopic << "'" << endl;
	}
      
	bReturnvalue = true;
      } catch (cv_bridge::Exception& e) {
      }
    }
  
    return bReturnvalue;
  }

  void CImageCapturer::imageCallback(const sensor_msgs::Image &imgData) {
    if(!m_bReceived) {
      m_bReceived = true;
      m_imgReceived = imgData;
    }
  }

  void CImageCapturer::publishImages(string strImagesTopic) {
    m_strImagesTopic = strImagesTopic;
    ros::NodeHandle nh;
  
    m_pubImages = nh.advertise<sensor_msgs::Image>(strImagesTopic, 1);
  }
}