
 // -------------------------------------------------------------------------------- 
 // Copyright (c) 2017 CNR-ITIA <iras@itia.cnr.it>
 // All rights reserved.
 //
 // Redistribution and use in source and binary forms, with or without
 // modification, are permitted provided that the following conditions are met:
 //
 // 1. Redistributions of source code must retain the above copyright notice,
 // this list of conditions and the following disclaimer.
 // 2. Redistributions in binary form must reproduce the above copyright
 // notice, this list of conditions and the following disclaimer in the
 // documentation and/or other materials provided with the distribution.
 // 3. Neither the name of mosquitto nor the names of its
 // contributors may be used to endorse or promote products derived from
 // this software without specific prior written permission.
 //
 //
 // THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 // AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 // IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 // ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 // LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 // CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 // SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 // INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 // CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 // ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 // POSSIBILITY OF SUCH DAMAGE.
 // -------------------------------------------------------------------------------- 

#ifndef __ITIA_MSG_CONVERSIONS__
#define __ITIA_MSG_CONVERSIONS__

#include <itia_msgs/MotionStamped.h>
#include <itia_gutils/itia_gutils.h>
#include <kdl_conversions/kdl_msg.h>
#include <kdl/frameacc.hpp>
#include <eigen_conversions/eigen_msg.h>
#include <Eigen/Geometry>

namespace itia
{
namespace msgs
{

  
  inline void motionMsgToKdl(const itia_msgs::MotionStamped& msg, KDL::Frame* frame)
  {
    tf::poseMsgToKDL(msg.pose,(*frame));
    frame->M = itia::gutils::orthonorm(frame->M);
  }
  
  inline void motionMsgToKdl(const itia_msgs::MotionStamped& msg, KDL::FrameAcc* frame_acc)
  {
    KDL::Frame frame;
    tf::poseMsgToKDL(msg.pose,frame);
    frame_acc->M.R=itia::gutils::orthonorm(frame.M);
    frame_acc->p.p=frame.p;
  
    KDL::Vector lin,rot;
    if (msg.lin.size()>0)
      tf::vectorMsgToKDL(msg.lin.at(0),lin);
    else
      lin=KDL::Vector(0,0,0);
  
    
    if (msg.lin.size()>0)
      tf::vectorMsgToKDL(msg.ang.at(0),rot);
    else
      rot = KDL::Vector(0,0,0);
  
    frame_acc->M.w=rot;
    frame_acc->p.v=lin;
  
    if (msg.lin.size()>1)
      tf::vectorMsgToKDL(msg.lin.at(1),lin);
    else
      lin = KDL::Vector(0,0,0);
    if (msg.lin.size()>1)
      tf::vectorMsgToKDL(msg.ang.at(1),rot);
    else
      rot = KDL::Vector(0,0,0);
  
    frame_acc->M.dw=rot;
    frame_acc->p.dv=lin;
  }


  void motionKdlToMsg(const KDL::FrameAcc& frame_acc, itia_msgs::MotionStamped* motion);
  void motionKdlToMsg(const KDL::FrameAcc& frame_acc, ros::Time& time, itia_msgs::MotionStamped* motion);
  
  inline void motionKdlToMsg(const KDL::FrameAcc& frame_acc, ros::Time& time, itia_msgs::MotionStamped* motion)
  {
    if (motion->lin.size()<2)
      motion->lin.resize(2);
    if (motion->ang.size()<2)
      motion->ang.resize(2);
    
    KDL::Frame frame = frame_acc.GetFrame();
    frame.M = itia::gutils::orthonorm(frame.M);
    tf::poseKDLToMsg (   frame, motion->pose );
    tf::vectorKDLToMsg ( frame_acc.p.v,motion->lin.at(0) );
    tf::vectorKDLToMsg ( frame_acc.M.w,motion->ang.at(0) );
    tf::vectorKDLToMsg ( frame_acc.p.dv,motion->lin.at(1) );
    tf::vectorKDLToMsg ( frame_acc.M.dw,motion->ang.at(1) );
    motion->header.stamp=time;
  };
  
  inline void motionKdlToMsg(const KDL::FrameAcc& frame_acc, itia_msgs::MotionStamped* motion)
  {
    ros::Time time=ros::Time::now();
    motionKdlToMsg(frame_acc,time,motion);
  }
  
  //void motionEigenToMsg(const Eigen::Affine3d& T, const Eigen::MatrixXd& twists, itia_msgs::MotionStamped* motion)
  inline void motionEigenToMsg(const Eigen::Affine3d& T, const Eigen::MatrixXd& twists, itia_msgs::MotionStamped* motion)
  {
    
    tf::poseEigenToMsg(T, motion->pose);
    if (twists.rows() != 6)
      throw("twists have wrong dimensions");
      
    int nder = twists.cols();
    motion->lin.resize(nder);
    motion->ang.resize(nder);
    for (int idx = 0;idx<nder;idx++)
    {
      tf::vectorEigenToMsg(twists.block(0, idx, 3, 1), motion->lin.at(idx));
      tf::vectorEigenToMsg(twists.block(3, idx, 3, 1), motion->ang.at(idx));
    }
  }
  
}
}

#endif
