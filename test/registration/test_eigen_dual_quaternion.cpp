/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2014-, Open Perception, Inc.
 *
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
 *   * Neither the name of the copyright holder(s) nor the names of its
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
 *
 * $Id$
 *
 */

#include <gtest/gtest.h>
#include <pcl/point_types.h>
#include <pcl/registration/eigen.h>
#include <pcl/registration/eigen_dual_quaternion.h>

using namespace std;

float frand ()
{
  return ((float)rand () / (float)RAND_MAX);
}

double drand ()
{
  return ((double)rand () / (double)RAND_MAX);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, DualQuaternionDefaultInitf)
{
  Eigen::DualQuaternion<float> dq;

  EXPECT_FLOAT_EQ(dq.real ().w (), 1);
  EXPECT_FLOAT_EQ(dq.real ().x (), 0);
  EXPECT_FLOAT_EQ(dq.real ().y (), 0);
  EXPECT_FLOAT_EQ(dq.real ().z (), 0);

  EXPECT_FLOAT_EQ(dq.dual ().w (), 0);
  EXPECT_FLOAT_EQ(dq.dual ().x (), 0);
  EXPECT_FLOAT_EQ(dq.dual ().y (), 0);
  EXPECT_FLOAT_EQ(dq.dual ().z (), 0);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, DualQuaternionDefaultInitd)
{
  Eigen::DualQuaternion<double> dq;

  EXPECT_DOUBLE_EQ(dq.real ().w (), 1);
  EXPECT_DOUBLE_EQ(dq.real ().x (), 0);
  EXPECT_DOUBLE_EQ(dq.real ().y (), 0);
  EXPECT_DOUBLE_EQ(dq.real ().z (), 0);

  EXPECT_DOUBLE_EQ(dq.dual ().w (), 0);
  EXPECT_DOUBLE_EQ(dq.dual ().x (), 0);
  EXPECT_DOUBLE_EQ(dq.dual ().y (), 0);
  EXPECT_DOUBLE_EQ(dq.dual ().z (), 0);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, DualQuaternionQuaternionVectorInitf)
{
  Eigen::Quaternionf  q (frand (), frand (), frand (), frand ());
  Eigen::Vector3f v (frand (), frand (), frand ());
  Eigen::Quaternionf vq (0, v (0), v (1), v (2));
  Eigen::Quaternionf qd = vq * q;

  Eigen::DualQuaternion<float> dq (q, v);

  EXPECT_FLOAT_EQ(dq.real ().w (), q.w ());
  EXPECT_FLOAT_EQ(dq.real ().x (), q.x ());
  EXPECT_FLOAT_EQ(dq.real ().y (), q.y ());
  EXPECT_FLOAT_EQ(dq.real ().z (), q.z ());

  EXPECT_FLOAT_EQ(dq.dual ().w (), qd.w () * 0.5);
  EXPECT_FLOAT_EQ(dq.dual ().x (), qd.x () * 0.5);
  EXPECT_FLOAT_EQ(dq.dual ().y (), qd.y () * 0.5);
  EXPECT_FLOAT_EQ(dq.dual ().z (), qd.z () * 0.5);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, DualQuaternionQuaternionVectorInitd)
{
  Eigen::Quaterniond  q (drand (), drand (), drand (), drand ());
  Eigen::Vector3d v (drand (), drand (), drand ());
  Eigen::Quaterniond vq (0, v (0), v (1), v (2));
  Eigen::Quaterniond qd = vq * q;

  Eigen::DualQuaternion<double> dq (q, v);

  EXPECT_DOUBLE_EQ(dq.real ().w (), q.w ());
  EXPECT_DOUBLE_EQ(dq.real ().x (), q.x ());
  EXPECT_DOUBLE_EQ(dq.real ().y (), q.y ());
  EXPECT_DOUBLE_EQ(dq.real ().z (), q.z ());

  EXPECT_DOUBLE_EQ(dq.dual ().w (), qd.w () * 0.5);
  EXPECT_DOUBLE_EQ(dq.dual ().x (), qd.x () * 0.5);
  EXPECT_DOUBLE_EQ(dq.dual ().y (), qd.y () * 0.5);
  EXPECT_DOUBLE_EQ(dq.dual ().z (), qd.z () * 0.5);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, DualQuaternionTwoQuaternionInitf)
{
  Eigen::Quaternionf  qr (frand (), frand (), frand (), frand ());
  Eigen::Quaternionf  qd (frand (), frand (), frand (), frand ());

  Eigen::DualQuaternion<float> dq (qr, qd);

  EXPECT_FLOAT_EQ(dq.real ().w (), qr.w ());
  EXPECT_FLOAT_EQ(dq.real ().x (), qr.x ());
  EXPECT_FLOAT_EQ(dq.real ().y (), qr.y ());
  EXPECT_FLOAT_EQ(dq.real ().z (), qr.z ());

  EXPECT_FLOAT_EQ(dq.dual ().w (), qd.w ());
  EXPECT_FLOAT_EQ(dq.dual ().x (), qd.x ());
  EXPECT_FLOAT_EQ(dq.dual ().y (), qd.y ());
  EXPECT_FLOAT_EQ(dq.dual ().z (), qd.z ());
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, DualQuaternionTwoQuaternionInitd)
{
  Eigen::Quaterniond  qr (drand (), drand (), drand (), drand ());
  Eigen::Quaterniond  qd (drand (), drand (), drand (), drand ());

  Eigen::DualQuaternion<double> dq (qr, qd);

  EXPECT_DOUBLE_EQ(dq.real ().w (), qr.w ());
  EXPECT_DOUBLE_EQ(dq.real ().x (), qr.x ());
  EXPECT_DOUBLE_EQ(dq.real ().y (), qr.y ());
  EXPECT_DOUBLE_EQ(dq.real ().z (), qr.z ());

  EXPECT_DOUBLE_EQ(dq.dual ().w (), qd.w ());
  EXPECT_DOUBLE_EQ(dq.dual ().x (), qd.x ());
  EXPECT_DOUBLE_EQ(dq.dual ().y (), qd.y ());
  EXPECT_DOUBLE_EQ(dq.dual ().z (), qd.z ());
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, DualQuaternionMatrixInitf)
{
  // TODO
  Eigen::Matrix4f m = Eigen::Matrix4f::Identity();

  Eigen::DualQuaternion<float> dq (m);

  /*
  EXPECT_FLOAT_EQ(dq.real ().w (), qr.w ());
  EXPECT_FLOAT_EQ(dq.real ().x (), qr.x ());
  EXPECT_FLOAT_EQ(dq.real ().y (), qr.y ());
  EXPECT_FLOAT_EQ(dq.real ().z (), qr.z ());

  EXPECT_FLOAT_EQ(dq.dual ().w (), qd.w ());
  EXPECT_FLOAT_EQ(dq.dual ().x (), qd.x ());
  EXPECT_FLOAT_EQ(dq.dual ().y (), qd.y ());
  EXPECT_FLOAT_EQ(dq.dual ().z (), qd.z ());
  */
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, DualQuaternionMatrixInitd)
{
  // TODO
  Eigen::Matrix4d m = Eigen::Matrix4d::Identity();

  Eigen::DualQuaternion<double> dq (m);

  /*
  EXPECT_DOUBLE_EQ(dq.real ().w (), qr.w ());
  EXPECT_DOUBLE_EQ(dq.real ().x (), qr.x ());
  EXPECT_DOUBLE_EQ(dq.real ().y (), qr.y ());
  EXPECT_DOUBLE_EQ(dq.real ().z (), qr.z ());

  EXPECT_DOUBLE_EQ(dq.dual ().w (), qd.w ());
  EXPECT_DOUBLE_EQ(dq.dual ().x (), qd.x ());
  EXPECT_DOUBLE_EQ(dq.dual ().y (), qd.y ());
  EXPECT_DOUBLE_EQ(dq.dual ().z (), qd.z ());
  */
}

/* ---[ */
int
main (int argc, char** argv)
{
  testing::InitGoogleTest (&argc, argv);
  return (RUN_ALL_TESTS ());
}
/* ]--- */
