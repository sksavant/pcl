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

  EXPECT_FLOAT_EQ (dq.real ().w (), 1);
  EXPECT_FLOAT_EQ (dq.real ().x (), 0);
  EXPECT_FLOAT_EQ (dq.real ().y (), 0);
  EXPECT_FLOAT_EQ (dq.real ().z (), 0);

  EXPECT_FLOAT_EQ (dq.dual ().w (), 0);
  EXPECT_FLOAT_EQ (dq.dual ().x (), 0);
  EXPECT_FLOAT_EQ (dq.dual ().y (), 0);
  EXPECT_FLOAT_EQ (dq.dual ().z (), 0);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, DualQuaternionDefaultInitd)
{
  Eigen::DualQuaternion<double> dq;

  EXPECT_DOUBLE_EQ (dq.real ().w (), 1);
  EXPECT_DOUBLE_EQ (dq.real ().x (), 0);
  EXPECT_DOUBLE_EQ (dq.real ().y (), 0);
  EXPECT_DOUBLE_EQ (dq.real ().z (), 0);

  EXPECT_DOUBLE_EQ (dq.dual ().w (), 0);
  EXPECT_DOUBLE_EQ (dq.dual ().x (), 0);
  EXPECT_DOUBLE_EQ (dq.dual ().y (), 0);
  EXPECT_DOUBLE_EQ (dq.dual ().z (), 0);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, DualQuaternionQuaternionVectorInitf)
{
  Eigen::Quaternionf q (frand (), frand (), frand (), frand ());
  Eigen::Vector3f v (frand (), frand (), frand ());
  Eigen::Quaternionf vq (0, v (0), v (1), v (2));
  Eigen::Quaternionf qd = vq * q;

  Eigen::DualQuaternion<float> dq (q, v);

  EXPECT_FLOAT_EQ (dq.real ().w (), q.w ());
  EXPECT_FLOAT_EQ (dq.real ().x (), q.x ());
  EXPECT_FLOAT_EQ (dq.real ().y (), q.y ());
  EXPECT_FLOAT_EQ (dq.real ().z (), q.z ());

  EXPECT_FLOAT_EQ (dq.dual ().w (), qd.w () * 0.5);
  EXPECT_FLOAT_EQ (dq.dual ().x (), qd.x () * 0.5);
  EXPECT_FLOAT_EQ (dq.dual ().y (), qd.y () * 0.5);
  EXPECT_FLOAT_EQ (dq.dual ().z (), qd.z () * 0.5);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, DualQuaternionQuaternionVectorInitd)
{
  Eigen::Quaterniond q (drand (), drand (), drand (), drand ());
  Eigen::Vector3d v (drand (), drand (), drand ());
  Eigen::Quaterniond vq (0, v (0), v (1), v (2));
  Eigen::Quaterniond qd = vq * q;

  Eigen::DualQuaternion<double> dq (q, v);

  EXPECT_DOUBLE_EQ (dq.real ().w (), q.w ());
  EXPECT_DOUBLE_EQ (dq.real ().x (), q.x ());
  EXPECT_DOUBLE_EQ (dq.real ().y (), q.y ());
  EXPECT_DOUBLE_EQ (dq.real ().z (), q.z ());

  EXPECT_DOUBLE_EQ (dq.dual ().w (), qd.w () * 0.5);
  EXPECT_DOUBLE_EQ (dq.dual ().x (), qd.x () * 0.5);
  EXPECT_DOUBLE_EQ (dq.dual ().y (), qd.y () * 0.5);
  EXPECT_DOUBLE_EQ (dq.dual ().z (), qd.z () * 0.5);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, DualQuaternionTwoQuaternionInitf)
{
  Eigen::Quaternionf qr (frand (), frand (), frand (), frand ());
  Eigen::Quaternionf qd (frand (), frand (), frand (), frand ());

  Eigen::DualQuaternion<float> dq (qr, qd);

  EXPECT_FLOAT_EQ (dq.real ().w (), qr.w ());
  EXPECT_FLOAT_EQ (dq.real ().x (), qr.x ());
  EXPECT_FLOAT_EQ (dq.real ().y (), qr.y ());
  EXPECT_FLOAT_EQ (dq.real ().z (), qr.z ());

  EXPECT_FLOAT_EQ (dq.dual ().w (), qd.w ());
  EXPECT_FLOAT_EQ (dq.dual ().x (), qd.x ());
  EXPECT_FLOAT_EQ (dq.dual ().y (), qd.y ());
  EXPECT_FLOAT_EQ (dq.dual ().z (), qd.z ());
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, DualQuaternionTwoQuaternionInitd)
{
  Eigen::Quaterniond qr (drand (), drand (), drand (), drand ());
  Eigen::Quaterniond qd (drand (), drand (), drand (), drand ());

  Eigen::DualQuaternion<double> dq (qr, qd);

  EXPECT_DOUBLE_EQ (dq.real ().w (), qr.w ());
  EXPECT_DOUBLE_EQ (dq.real ().x (), qr.x ());
  EXPECT_DOUBLE_EQ (dq.real ().y (), qr.y ());
  EXPECT_DOUBLE_EQ (dq.real ().z (), qr.z ());

  EXPECT_DOUBLE_EQ (dq.dual ().w (), qd.w ());
  EXPECT_DOUBLE_EQ (dq.dual ().x (), qd.x ());
  EXPECT_DOUBLE_EQ (dq.dual ().y (), qd.y ());
  EXPECT_DOUBLE_EQ (dq.dual ().z (), qd.z ());
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, DualQuaternionMatrixInitf)
{
  // TODO

  const Eigen::Quaternionf qr = Eigen::Quaternionf (.9f, .1f, -.25f, .15f).normalized ();
  const Eigen::Translation3f t = Eigen::Translation3f (.5f, -2.f, 1.f);
  const Eigen::Matrix4f m = (Eigen::Affine3f (t * qr)).matrix ();

  Eigen::Quaternionf tq (0, t.x (), t.y (), t.z ());
  Eigen::Quaternionf qd = tq * qr;

  Eigen::DualQuaternion<float> dq (m);

  EXPECT_FLOAT_EQ (dq.real ().w (), qr.w ());
  EXPECT_FLOAT_EQ (dq.real ().x (), qr.x ());
  EXPECT_FLOAT_EQ (dq.real ().y (), qr.y ());
  EXPECT_FLOAT_EQ (dq.real ().z (), qr.z ());

  EXPECT_FLOAT_EQ (dq.dual ().w (), qd.w () * 0.5);
  EXPECT_FLOAT_EQ (dq.dual ().x (), qd.x () * 0.5);
  EXPECT_FLOAT_EQ (dq.dual ().y (), qd.y () * 0.5);
  EXPECT_FLOAT_EQ (dq.dual ().z (), qd.z () * 0.5);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, DualQuaternionMatrixInitd)
{
  const Eigen::Quaterniond qr = Eigen::Quaterniond (.9, .1, -.25, .15).normalized ();
  const Eigen::Translation3d t = Eigen::Translation3d (.5, -2, 1);
  const Eigen::Matrix4d m = (Eigen::Affine3d (t * qr)).matrix ();

  Eigen::Quaterniond tq (0, t.x (), t.y (), t.z ());
  Eigen::Quaterniond qd = tq * qr;

  Eigen::DualQuaternion<double> dq (m);

  EXPECT_DOUBLE_EQ (dq.real ().w (), qr.w ());
  EXPECT_DOUBLE_EQ (dq.real ().x (), qr.x ());
  EXPECT_DOUBLE_EQ (dq.real ().y (), qr.y ());
  EXPECT_DOUBLE_EQ (dq.real ().z (), qr.z ());

  EXPECT_DOUBLE_EQ (dq.dual ().w (), qd.w () * 0.5);
  EXPECT_DOUBLE_EQ (dq.dual ().x (), qd.x () * 0.5);
  EXPECT_DOUBLE_EQ (dq.dual ().y (), qd.y () * 0.5);
  EXPECT_DOUBLE_EQ (dq.dual ().z (), qd.z () * 0.5);
}

TEST (PCL, DualQuaternionNormalizef)
{
  Eigen::Quaternionf qr (frand (), frand (), frand (), frand ());
  Eigen::Quaternionf qd (frand (), frand (), frand (), frand ());

  Eigen::DualQuaternion<float> dq (qr, qd);

  dq.normalize ();

  EXPECT_FLOAT_EQ (dq.real ().norm (), 1.0f);
  //EXPECT_FLOAT_EQ (dq.real ().dot (dq.dual ()), 0); // FIXME Numerical unstability or normal float equality isues?
  EXPECT_NEAR (dq.real ().dot (dq.dual ()), 0, 1e-5); // FIXME randomly chose 1e-5, is there a theoretical bound? To compute

}

TEST (PCL, DualQuaternionNormalized)
{
  Eigen::Quaterniond  qr (drand (), drand (), drand (), drand ());
  Eigen::Quaterniond  qd (drand (), drand (), drand (), drand ());

  Eigen::DualQuaternion<double> dq (qr, qd);

  dq.normalize ();

  EXPECT_DOUBLE_EQ (dq.real ().norm (), 1.0);
  //EXPECT_DOUBLE_EQ (dq.real ().dot (dq.dual ()), 0); // FIXME Numerical unstability ?
  EXPECT_NEAR (dq.real ().dot (dq.dual ()), 0, 1e-10); // FIXME randomly chose 1e-10, is there a theoretical bound? To compute

}

TEST (PCL, DualQuaternionGetTranslationf)
{
  Eigen::Quaternionf  q (frand (), frand (), frand (), frand ());
  //q.normalize ();
  Eigen::Vector3f v (frand (), frand (), frand ());

  Eigen::DualQuaternion<float> dq (q, v);

  Eigen::Vector3f v_res = dq.getTranslation ();

  // FIXME : expect_float_eq sometimes fails use expect_near maybe?
  EXPECT_FLOAT_EQ (v_res (0), v (0));
  EXPECT_FLOAT_EQ (v_res (1), v (1));
  EXPECT_FLOAT_EQ (v_res (2), v (2));

}

TEST (PCL, DualQuaternionGetTranslationd)
{
  Eigen::Quaterniond  q (drand (), drand (), drand (), drand ());
  //q.normalize ();
  Eigen::Vector3d v (drand (), drand (), drand ());

  Eigen::DualQuaternion<double> dq (q, v);

  Eigen::Vector3d v_res = dq.getTranslation ();

  // FIXME : expect_double_eq sometimes fails use expect_near maybe?
  EXPECT_DOUBLE_EQ (v_res (0), v (0));
  EXPECT_DOUBLE_EQ (v_res (1), v (1));
  EXPECT_DOUBLE_EQ (v_res (2), v (2));

}

TEST (PCL, DualQuaternionDotf)
{
  Eigen::Quaternionf  qr1 (frand (), frand (), frand (), frand ());
  Eigen::Quaternionf  qd1 (frand (), frand (), frand (), frand ());

  Eigen::Quaternionf  qr2 (frand (), frand (), frand (), frand ());
  Eigen::Quaternionf  qd2 (frand (), frand (), frand (), frand ());

  Eigen::DualQuaternion<float> dq1 (qr1, qd1);
  Eigen::DualQuaternion<float> dq2 (qr2, qd2);

  float dot_res_12 = qr1.dot (qr2);
  float dot_res_21 = qr2.dot (qr1);

  EXPECT_FLOAT_EQ (dot_res_12, dot_res_21);

  // FIXME Maybe split into two tests?
  Eigen::DualQuaternion<float> dq_init;

  float dot_init = dq_init.dot (dq1);

  EXPECT_FLOAT_EQ (dq1.real ().w (), dot_init);
}

TEST (PCL, DualQuaternionDotd)
{
  Eigen::Quaterniond  qr1 (drand (), drand (), drand (), drand ());
  Eigen::Quaterniond  qd1 (drand (), drand (), drand (), drand ());

  Eigen::Quaterniond  qr2 (drand (), drand (), drand (), drand ());
  Eigen::Quaterniond  qd2 (drand (), drand (), drand (), drand ());

  Eigen::DualQuaternion<double> dq1 (qr1, qd1);
  Eigen::DualQuaternion<double> dq2 (qr2, qd2);

  double dot_res_12 = qr1.dot (qr2);
  double dot_res_21 = qr2.dot (qr1);

  EXPECT_DOUBLE_EQ (dot_res_12, dot_res_21);

  // FIXME Maybe split into two tests?
  Eigen::DualQuaternion<double> dq_init;

  double dot_init = dq_init.dot (dq1);

  EXPECT_DOUBLE_EQ (dq1.real ().w (), dot_init);
}

TEST (PCL, DualQuaternionMultiplyf)
{
  Eigen::Quaternionf  qr1 (frand (), frand (), frand (), frand ());
  Eigen::Quaternionf  qd1 (frand (), frand (), frand (), frand ());

  Eigen::Quaternionf  qr2 (frand (), frand (), frand (), frand ());
  Eigen::Quaternionf  qd2 (frand (), frand (), frand (), frand ());

  Eigen::DualQuaternion<float> dq1 (qr1, qd1);
  Eigen::DualQuaternion<float> dq2 (qr2, qd2);

  Eigen::DualQuaternion<float> dq_res = dq1 * dq2;

  Eigen::Quaternionf real_res = qr1 * qr2;
  Eigen::Quaternionf dual_res1 = qr1 * qd2;
  Eigen::Quaternionf dual_res2 = qd1 * qr2;

  EXPECT_FLOAT_EQ (dq_res.real ().w (), real_res.w ());
  EXPECT_FLOAT_EQ (dq_res.real ().x (), real_res.x ());
  EXPECT_FLOAT_EQ (dq_res.real ().y (), real_res.y ());
  EXPECT_FLOAT_EQ (dq_res.real ().z (), real_res.z ());

  EXPECT_FLOAT_EQ (dq_res.dual ().w (), dual_res1.w () + dual_res2.w ());
  EXPECT_FLOAT_EQ (dq_res.dual ().x (), dual_res1.x () + dual_res2.x ());
  EXPECT_FLOAT_EQ (dq_res.dual ().y (), dual_res1.y () + dual_res2.y ());
  EXPECT_FLOAT_EQ (dq_res.dual ().z (), dual_res1.z () + dual_res2.z ());
}

TEST (PCL, DualQuaternionMultiplyd)
{
  Eigen::Quaterniond  qr1 (drand (), drand (), drand (), drand ());
  Eigen::Quaterniond  qd1 (drand (), drand (), drand (), drand ());

  Eigen::Quaterniond  qr2 (drand (), drand (), drand (), drand ());
  Eigen::Quaterniond  qd2 (drand (), drand (), drand (), drand ());

  Eigen::DualQuaternion<double> dq1 (qr1, qd1);
  Eigen::DualQuaternion<double> dq2 (qr2, qd2);

  Eigen::DualQuaternion<double> dq_res = dq1 * dq2;

  Eigen::Quaterniond real_res = qr1 * qr2;
  Eigen::Quaterniond dual_res1 = qr1 * qd2;
  Eigen::Quaterniond dual_res2 = qd1 * qr2;

  EXPECT_DOUBLE_EQ (dq_res.real ().w (), real_res.w ());
  EXPECT_DOUBLE_EQ (dq_res.real ().x (), real_res.x ());
  EXPECT_DOUBLE_EQ (dq_res.real ().y (), real_res.y ());
  EXPECT_DOUBLE_EQ (dq_res.real ().z (), real_res.z ());

  EXPECT_DOUBLE_EQ (dq_res.dual ().w (), dual_res1.w () + dual_res2.w ());
  EXPECT_DOUBLE_EQ (dq_res.dual ().x (), dual_res1.x () + dual_res2.x ());
  EXPECT_DOUBLE_EQ (dq_res.dual ().y (), dual_res1.y () + dual_res2.y ());
  EXPECT_DOUBLE_EQ (dq_res.dual ().z (), dual_res1.z () + dual_res2.z ());
}

TEST (PCL, DualQuaternionScalarMultiplyf)
{
  const Eigen::Quaternionf qr = Eigen::Quaternionf (.9f, .1f, -.25f, .15f).normalized ();
  const Eigen::Quaternionf qd = Eigen::Quaternionf (0, .5f, -2.f, 1.f);

  float fac = 0.75;

  Eigen::DualQuaternion<float> dq (qr, qd);

  Eigen::DualQuaternion<float> dq_res = dq * fac;

  EXPECT_FLOAT_EQ (0.70954424f, dq_res.real ().w ());
  EXPECT_FLOAT_EQ (0.07883824f, dq_res.real ().x ());
  EXPECT_FLOAT_EQ (-0.19709562f, dq_res.real ().y ());
  EXPECT_FLOAT_EQ (0.11825737f, dq_res.real ().z ());

  EXPECT_FLOAT_EQ (0.0f, dq_res.dual ().w ());
  EXPECT_FLOAT_EQ (0.375f, dq_res.dual ().x ());
  EXPECT_FLOAT_EQ (-1.5f, dq_res.dual ().y ());
  EXPECT_FLOAT_EQ (0.75f, dq_res.dual ().z ());

}

TEST (PCL, DualQuaternionScalarMultiplyd)
{
  const Eigen::Quaterniond qr = Eigen::Quaterniond (.9, .1, -.25, .15).normalized ();
  const Eigen::Quaterniond qd = Eigen::Quaterniond (0, .5, -2, 1);

  double fac = 0.75;

  Eigen::DualQuaternion<double> dq (qr, qd);

  Eigen::DualQuaternion<double> dq_res = dq * fac;

  EXPECT_NEAR (0.70954424, dq_res.real ().w (), 1e-7);
  EXPECT_NEAR (0.07883824, dq_res.real ().x (), 1e-7);
  EXPECT_NEAR (-0.19709562, dq_res.real ().y (), 1e-7);
  EXPECT_NEAR (0.11825737, dq_res.real ().z (), 1e-7);

  EXPECT_NEAR (0.0, dq_res.dual ().w (), 1e-7);
  EXPECT_NEAR (0.375, dq_res.dual ().x (), 1e-7);
  EXPECT_NEAR (-1.5, dq_res.dual ().y (), 1e-7);
  EXPECT_NEAR (0.75, dq_res.dual ().z (), 1e-7);

}

TEST (PCL, DualQuaternionAddf)
{
  const Eigen::Quaternionf qr1 = Eigen::Quaternionf (.9f, .1f, -.25f, .15f);
  const Eigen::Quaternionf qd1 = Eigen::Quaternionf (0, .5f, -2.f, 1.f);

  Eigen::DualQuaternion<float> dq1 (qr1, qd1);

  const Eigen::Quaternionf qr2 = Eigen::Quaternionf (.67f, .86f, .25f, -.95f);
  const Eigen::Quaternionf qd2 = Eigen::Quaternionf (0, .6f, -8.f, .7f);

  Eigen::DualQuaternion<float> dq2 (qr2, qd2);

  Eigen::DualQuaternion<float> dq_res = dq1 + dq2;

  EXPECT_NEAR (1.57f, dq_res.real ().w (), 1e-5);
  EXPECT_NEAR (0.96f, dq_res.real ().x (), 1e-5);
  EXPECT_NEAR (0.0f, dq_res.real ().y (), 1e-5);
  EXPECT_NEAR (-0.8f, dq_res.real ().z (), 1e-5);

  EXPECT_NEAR (0.0f, dq_res.dual ().w (), 1e-5);
  EXPECT_NEAR (1.1f, dq_res.dual ().x (), 1e-5);
  EXPECT_NEAR (-10.0f, dq_res.dual ().y (), 1e-5);
  EXPECT_NEAR (1.7f, dq_res.dual ().z (), 1e-5);

}

/* ---[ */
int
main (int argc, char** argv)
{
  srand (time(NULL));
  testing::InitGoogleTest (&argc, argv);
  return (RUN_ALL_TESTS ());
}
/* ]--- */
