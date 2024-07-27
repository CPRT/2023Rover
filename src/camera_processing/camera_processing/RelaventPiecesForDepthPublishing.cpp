


NOTES from zed_wrapper_nodelet.cpp

Need to produce a sensor_msgs Image

Relavent bits from zed_wrapper_nodelet.cpp:

sl::Mat mat_depth;

    if (!mOpenniDepthMode)
    {
      mZed.retrieveMeasure(mat_depth, sl::MEASURE::DEPTH, sl::MEM::CPU, mMatResol);
    }
    else
    {
      mZed.retrieveMeasure(mat_depth, sl::MEASURE::DEPTH_U16_MM, sl::MEM::CPU, mMatResol);
    }


    ros::Time stamp = sl_tools::slTime2Ros(grab_ts);
  if (mSvoMode)
  {
    stamp = ros::Time::now();
  }


    // Publish the depth image if someone has subscribed to
  if (depthSubnumber > 0)
  {
    sensor_msgs::ImagePtr depthImgMsg = boost::make_shared<sensor_msgs::Image>();
    publishDepth(depthImgMsg, mat_depth, stamp);
  }



  void ZEDWrapperNodelet::publishDepth(sensor_msgs::ImagePtr imgMsgPtr, sl::Mat depth, ros::Time t)
{
  mDepthCamInfoMsg->header.stamp = t;

  // NODELET_DEBUG_STREAM("mOpenniDepthMode: " << mOpenniDepthMode);

  if (!mOpenniDepthMode)
  {
    // NODELET_INFO("Using float32");
    sl_tools::imageToROSmsg(imgMsgPtr, depth, mDepthOptFrameId, t);
    mPubDepth.publish(imgMsgPtr, mDepthCamInfoMsg);

    return;
  }

  // NODELET_INFO("Using depth16");
  sl_tools::imageToROSmsg(imgMsgPtr, depth, mDepthOptFrameId, t);
  mPubDepth.publish(imgMsgPtr, mDepthCamInfoMsg);
}



ros::Time slTime2Ros(sl::Timestamp t)
{
  uint32_t sec = static_cast<uint32_t>(t.getNanoseconds() / 1000000000);
  uint32_t nsec = static_cast<uint32_t>(t.getNanoseconds() % 1000000000);
  return ros::Time(sec, nsec);
}
