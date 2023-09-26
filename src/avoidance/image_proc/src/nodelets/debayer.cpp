/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
* 
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
* 
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/
#include <boost/make_shared.hpp>
#include <boost/version.hpp>
#if ((BOOST_VERSION / 100) % 1000) >= 53
#include <boost/thread/lock_guard.hpp>
#endif

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <dynamic_reconfigure/server.h>
#include <image_proc/DebayerConfig.h>

#include <opencv2/imgproc/imgproc.hpp>
// Until merged into OpenCV
#include "edge_aware.h"

#include <cv_bridge/cv_bridge.h>
#include <time.h>
#include <fstream>
#include <iostream>
#include <string>

namespace image_proc {

namespace enc = sensor_msgs::image_encodings;

class DebayerNodelet : public nodelet::Nodelet
{
  // ROS communication
  boost::shared_ptr<image_transport::ImageTransport> it_;
  image_transport::Subscriber sub_raw_;
  
  boost::mutex connect_mutex_;
  image_transport::Publisher pub_mono_;
  image_transport::Publisher pub_color_;
  image_transport::Publisher pub_noisy_;

  // Dynamic reconfigure
  boost::recursive_mutex config_mutex_;
  typedef image_proc::DebayerConfig Config;
  typedef dynamic_reconfigure::Server<Config> ReconfigureServer;
  boost::shared_ptr<ReconfigureServer> reconfigure_server_;
  Config config_;

  virtual void onInit();

  void connectCb();

  void imageCb(const sensor_msgs::ImageConstPtr& raw_msg);

  void configCb(Config &config, uint32_t level);
};

void DebayerNodelet::onInit()
{
  ros::NodeHandle &nh         = getNodeHandle();
  ros::NodeHandle &private_nh = getPrivateNodeHandle();
  it_.reset(new image_transport::ImageTransport(nh));

  // Set up dynamic reconfigure
  reconfigure_server_.reset(new ReconfigureServer(config_mutex_, private_nh));
  ReconfigureServer::CallbackType f = boost::bind(&DebayerNodelet::configCb, this, boost::placeholders::_1, boost::placeholders::_2);
  reconfigure_server_->setCallback(f);

  // Monitor whether anyone is subscribed to the output
  typedef image_transport::SubscriberStatusCallback ConnectCB;
  ConnectCB connect_cb = boost::bind(&DebayerNodelet::connectCb, this);
  // Make sure we don't enter connectCb() between advertising and assigning to pub_XXX
  boost::lock_guard<boost::mutex> lock(connect_mutex_);
  pub_mono_  = it_->advertise("image_mono",  1, connect_cb, connect_cb);
  pub_color_ = it_->advertise("image_color", 1, connect_cb, connect_cb);
  pub_noisy_ = it_-> advertise("noise_raw",1,connect_cb, connect_cb);
}

// Handles (un)subscribing when clients (un)subscribe
void DebayerNodelet::connectCb()
{
  boost::lock_guard<boost::mutex> lock(connect_mutex_);
  if (pub_mono_.getNumSubscribers() == 0 && pub_color_.getNumSubscribers() == 0)
    sub_raw_.shutdown();
  else if (!sub_raw_)
  {
    image_transport::TransportHints hints("raw", ros::TransportHints(), getPrivateNodeHandle());
    sub_raw_ = it_->subscribe("image_raw", 1, &DebayerNodelet::imageCb, this, hints);
  }
}

void DebayerNodelet::imageCb(const sensor_msgs::ImageConstPtr& raw_msg1)
{
  sensor_msgs::ImagePtr raw_msg;
  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(raw_msg1);
  time_t seconds;

  // std::ifstream ifs("profile.json");
  // Json::Reader reader;
  // Json::Value obj;

  // reader.parse(ifs,obj);

  // config_.gaussian_noise_flag = obj["gaussian_noise_flag"].asInt();


  std::string myText;
  std::ifstream readFile("/home/prasun/catkin_ws/src/avoidance/image_proc/src/nodelets/profile.txt");

  int gaussianNoiseFlag=0;
  int gaussianMean = 0;
  int gaussianStdDeviation = 0;
  std::string::size_type n;
  while (getline (readFile, myText)) {
  // Output the text from the file
    // gaussian_noise_flag = std::stoi(myText);
      n = myText.find(":");
      std::string tempKey = myText.substr(0, n);
      std::string tempValue = myText.substr(n+1);

      if(tempKey == "gaussianNoise"){
        if(tempValue == "True"){
          gaussianNoiseFlag=1;
        }
      }

      if(gaussianNoiseFlag==1){
        if(tempKey == "gaussianMean"){
          gaussianMean = std::stoi(tempValue);
        }
      }

      if(gaussianNoiseFlag==1){
        if(tempKey == "gaussianStdDeviation"){
          gaussianStdDeviation = std::stoi(tempValue);
        }
      }
  }
  readFile.close();
  

  seconds = time (NULL);
  // if (seconds%2 == 0){
  //     cv::Mat NoiseArr = cv_ptr->image.clone();
  //     cv::RNG rng;
  //     rng.fill(NoiseArr, cv::RNG::NORMAL, 20,20);
  //     add(cv_ptr->image, NoiseArr, cv_ptr->image);
  // }

  std::ofstream o;
  o.open("/home/prasun/catkin_ws/src/avoidance/image_proc/src/nodelets/test.txt", std::ios_base::app);
  o<<"New frame " << std::endl;
  o<<n<<std::endl;
  o<<myText<<std::endl;
  o<<gaussianNoiseFlag<<std::endl;
  o<<gaussianStdDeviation<<std::endl;
  o<<gaussianMean<<std::endl;
  o<<ros::this_node::getName()<<std::endl;
  o.close();
  
  if(gaussianNoiseFlag == 1){
    // seconds = time (NULL);
    
    cv::Mat NoiseArr = cv_ptr->image.clone();
    cv::RNG rng;
    rng.fill(NoiseArr, cv::RNG::NORMAL, gaussianMean,gaussianStdDeviation);
    add(cv_ptr->image, NoiseArr, cv_ptr->image);
    
  }

   
  raw_msg = cv_ptr->toImageMsg();
  pub_noisy_.publish(raw_msg);
  int bit_depth = enc::bitDepth(raw_msg->encoding);
  //@todo Fix as soon as bitDepth fixes it
  if (raw_msg->encoding == enc::YUV422)
    bit_depth = 8;

  // First publish to mono if needed
  if (pub_mono_.getNumSubscribers())
  {
    if (enc::isMono(raw_msg->encoding))
      pub_mono_.publish(raw_msg);
    else
    {
      if ((bit_depth != 8) && (bit_depth != 16))
      {
        NODELET_WARN_THROTTLE(30,
                            "Raw image data from topic '%s' has unsupported depth: %d",
                            sub_raw_.getTopic().c_str(), bit_depth);
      } else {
        // Use cv_bridge to convert to Mono. If a type is not supported,
        // it will error out there
        sensor_msgs::ImagePtr gray_msg;
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
          if (bit_depth == 8){
            cv_ptr = cv_bridge::toCvCopy(raw_msg, enc::MONO8);
            // cv::RNG rng; // rand number generate，

            // cv::Mat NoiseArr = cv_ptr->image.clone();
            // // cv::RNG rng;
            // rng.fill(NoiseArr, cv::RNG::NORMAL, 20,20);  

            // add(cv_ptr->image, NoiseArr, cv_ptr->image);  
            // int amount1=cv_ptr->image.rows*cv_ptr->image.cols*0.01;
            // int amount2=cv_ptr->image.rows*cv_ptr->image.cols*0.01;            
            // for(int counter=0; counter<amount1; ++counter)
            // {
              
            //   cv_ptr->image.at<uchar>(rng.uniform( 0,cv_ptr->image.rows), rng.uniform(0, cv_ptr->image.cols)) =0;

            // }
            // for (int counter=0; counter<amount2; ++counter)
            // {
            //   cv_ptr->image.at<uchar>(rng.uniform(0,cv_ptr->image.rows), rng.uniform(0,cv_ptr->image.cols)) = 255;
            // } 
            gray_msg = cv_ptr->toImageMsg();
            // gray_msg = cv_bridge::toCvCopy(raw_msg, enc::MONO8)->toImageMsg();
          }
          else{
            cv_ptr = cv_bridge::toCvCopy(raw_msg, enc::MONO16);
            // cv::RNG rng; // rand number generate，

            // cv::Mat NoiseArr = cv_ptr->image.clone();
            // // cv::RNG rng;
            // rng.fill(NoiseArr, cv::RNG::NORMAL, 20,20);  

            // add(cv_ptr->image, NoiseArr, cv_ptr->image); 
            // cv::RNG rng; // rand number generate，
            // int amount1=cv_ptr->image.rows*cv_ptr->image.cols*0.01;
            // int amount2=cv_ptr->image.rows*cv_ptr->image.cols*0.01;            
            // for(int counter=0; counter<amount1; ++counter)
            // {
              
            //   cv_ptr->image.at<uchar>(rng.uniform( 0,cv_ptr->image.rows), rng.uniform(0, cv_ptr->image.cols)) =0;

            // }
            // for (int counter=0; counter<amount2; ++counter)
            // {
            //   cv_ptr->image.at<uchar>(rng.uniform(0,cv_ptr->image.rows), rng.uniform(0,cv_ptr->image.cols)) = 255;
            // } 
            gray_msg = cv_ptr->toImageMsg();
            // gray_msg = cv_bridge::toCvCopy(raw_msg, enc::MONO16)->toImageMsg();
          }
            
          pub_mono_.publish(gray_msg);
        }
        catch (cv_bridge::Exception &e)
        {
          NODELET_WARN_THROTTLE(30, "cv_bridge conversion error: '%s'", e.what());
        }
      }
    }
  }

  // Next, publish to color
  if (!pub_color_.getNumSubscribers())
    return;

  if (enc::isMono(raw_msg->encoding))
  {
    // For monochrome, no processing needed!
    pub_color_.publish(raw_msg);

    // Warn if the user asked for color
    NODELET_WARN_THROTTLE(30,
                            "Color topic '%s' requested, but raw image data from topic '%s' is grayscale",
                            pub_color_.getTopic().c_str(), sub_raw_.getTopic().c_str());
  }
  else if (enc::isColor(raw_msg->encoding))
  {
    sensor_msgs::ImagePtr publish_color_msg;
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(raw_msg);
    // cv::RNG rng; // rand number generate，
    // int amount1=cv_ptr->image.rows*cv_ptr->image.cols*0.01;
    // int amount2=cv_ptr->image.rows*cv_ptr->image.cols*0.01;            
    // for(int counter=0; counter<amount1; ++counter)
    // {
      
    //   cv_ptr->image.at<uchar>(rng.uniform( 0,cv_ptr->image.rows), rng.uniform(0, cv_ptr->image.cols)) =0;

    // }
    // for (int counter=0; counter<amount2; ++counter)
    // {
    //   cv_ptr->image.at<uchar>(rng.uniform(0,cv_ptr->image.rows), rng.uniform(0,cv_ptr->image.cols)) = 255;
    // } 

    // cv::Mat NoiseArr = cv_ptr->image.clone();
    // cv::RNG rng;
    // rng.fill(NoiseArr, cv::RNG::NORMAL, 50,50);  

    // add(cv_ptr->image, NoiseArr, cv_ptr->image);   
    publish_color_msg = cv_ptr->toImageMsg();
    pub_color_.publish(publish_color_msg);
  }
  else if (enc::isBayer(raw_msg->encoding)) {
    int type = bit_depth == 8 ? CV_8U : CV_16U;
    const cv::Mat bayer(raw_msg->height, raw_msg->width, CV_MAKETYPE(type, 1),
                        const_cast<uint8_t*>(&raw_msg->data[0]), raw_msg->step);

      sensor_msgs::ImagePtr color_msg = boost::make_shared<sensor_msgs::Image>();
      color_msg->header   = raw_msg->header;
      color_msg->height   = raw_msg->height;
      color_msg->width    = raw_msg->width;
      color_msg->encoding = bit_depth == 8? enc::BGR8 : enc::BGR16;
      color_msg->step     = color_msg->width * 3 * (bit_depth / 8);
      color_msg->data.resize(color_msg->height * color_msg->step);

      cv::Mat color(color_msg->height, color_msg->width, CV_MAKETYPE(type, 3),
                    &color_msg->data[0], color_msg->step);

      int algorithm;
      {
        boost::lock_guard<boost::recursive_mutex> lock(config_mutex_);
        algorithm = config_.debayer;
      }
      
      if (algorithm == Debayer_EdgeAware ||
          algorithm == Debayer_EdgeAwareWeighted)
      {
        // These algorithms are not in OpenCV yet
        if (raw_msg->encoding != enc::BAYER_GRBG8)
        {
          NODELET_WARN_THROTTLE(30, "Edge aware algorithms currently only support GRBG8 Bayer. "
                                "Falling back to bilinear interpolation.");
          algorithm = Debayer_Bilinear;
        }
        else
        {
          if (algorithm == Debayer_EdgeAware)
            debayerEdgeAware(bayer, color);
          else
            debayerEdgeAwareWeighted(bayer, color);
        }
      }
      if (algorithm == Debayer_Bilinear ||
          algorithm == Debayer_VNG)
      {
        int code = -1;
        if (raw_msg->encoding == enc::BAYER_RGGB8 ||
            raw_msg->encoding == enc::BAYER_RGGB16)
          code = cv::COLOR_BayerBG2BGR;
        else if (raw_msg->encoding == enc::BAYER_BGGR8 ||
                 raw_msg->encoding == enc::BAYER_BGGR16)
          code = cv::COLOR_BayerRG2BGR;
        else if (raw_msg->encoding == enc::BAYER_GBRG8 ||
                 raw_msg->encoding == enc::BAYER_GBRG16)
          code = cv::COLOR_BayerGR2BGR;
        else if (raw_msg->encoding == enc::BAYER_GRBG8 ||
                 raw_msg->encoding == enc::BAYER_GRBG16)
          code = cv::COLOR_BayerGB2BGR;

        if (algorithm == Debayer_VNG)
          code += cv::COLOR_BayerBG2BGR_VNG - cv::COLOR_BayerBG2BGR;

        try
        {
          cv::cvtColor(bayer, color, code);
        }
        catch (cv::Exception &e)
        {
          NODELET_WARN_THROTTLE(30, "cvtColor error: '%s', bayer code: %d, width %d, height %d",
                       e.what(), code, bayer.cols, bayer.rows);
          return;
        }
      }

      sensor_msgs::ImagePtr publish_color_msg;
      cv_bridge::CvImagePtr cv_ptr;
      cv_ptr = cv_bridge::toCvCopy(raw_msg);
      // cv::RNG rng; // rand number generate，
      // int amount1=cv_ptr->image.rows*cv_ptr->image.cols*0.01;
      // int amount2=cv_ptr->image.rows*cv_ptr->image.cols*0.01;            
      // for(int counter=0; counter<amount1; ++counter)
      // {
        
      //   cv_ptr->image.at<uchar>(rng.uniform( 0,cv_ptr->image.rows), rng.uniform(0, cv_ptr->image.cols)) =0;

      // }
      // for (int counter=0; counter<amount2; ++counter)
      // {
      //   cv_ptr->image.at<uchar>(rng.uniform(0,cv_ptr->image.rows), rng.uniform(0,cv_ptr->image.cols)) = 255;
      // } 

      
      // cv::Mat NoiseArr = cv_ptr->image.clone();
      // cv::RNG rng;
      // rng.fill(NoiseArr, cv::RNG::NORMAL, 50,50);  

      // add(cv_ptr->image, NoiseArr, cv_ptr->image);   
      
      // publish_color_msg = cv_ptr->toImageMsg();
      
      pub_color_.publish(publish_color_msg);
  }
  else if (raw_msg->encoding == enc::YUV422)
  {
    // Use cv_bridge to convert to BGR8
    sensor_msgs::ImagePtr color_msg;
    try
    {
      sensor_msgs::ImagePtr color_msg;
      cv_bridge::CvImagePtr cv_ptr;
      cv_ptr = cv_bridge::toCvCopy(raw_msg, enc::BGR8);
      cv::RNG rng; // rand number generate，
      int amount1=cv_ptr->image.rows*cv_ptr->image.cols*0.01;
      int amount2=cv_ptr->image.rows*cv_ptr->image.cols*0.01;            
      for(int counter=0; counter<amount1; ++counter)
      {
        
        cv_ptr->image.at<uchar>(rng.uniform( 0,cv_ptr->image.rows), rng.uniform(0, cv_ptr->image.cols)) =0;

      }
      for (int counter=0; counter<amount2; ++counter)
      {
        cv_ptr->image.at<uchar>(rng.uniform(0,cv_ptr->image.rows), rng.uniform(0,cv_ptr->image.cols)) = 255;
      } 
      color_msg = cv_ptr->toImageMsg();
      pub_color_.publish(color_msg);
    }
    catch (cv_bridge::Exception &e)
    {
      NODELET_WARN_THROTTLE(30, "cv_bridge conversion error: '%s'", e.what());
    }
  }
  else if (raw_msg->encoding == enc::TYPE_8UC3)
  {
    // 8UC3 does not specify a color encoding. Is it BGR, RGB, HSV, XYZ, LUV...?
    NODELET_ERROR_THROTTLE(10,
                           "Raw image topic '%s' has ambiguous encoding '8UC3'. The "
                           "source should set the encoding to 'bgr8' or 'rgb8'.",
                           sub_raw_.getTopic().c_str());
  }
  else
  {
    NODELET_ERROR_THROTTLE(10, "Raw image topic '%s' has unsupported encoding '%s'",
                           sub_raw_.getTopic().c_str(), raw_msg->encoding.c_str());
  }
}

void DebayerNodelet::configCb(Config &config, uint32_t level)
{
  config_ = config;
}

} // namespace image_proc

// Register nodelet
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS( image_proc::DebayerNodelet, nodelet::Nodelet)