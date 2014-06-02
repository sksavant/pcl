/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2012-, Open Perception, Inc.
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

#ifndef PCL_REGISTRATION_EIGEN_DUAL_QUATERNION_HPP_
#define PCL_REGISTRATION_EIGEN_DUAL_QUATERNION_HPP_

template<typename Scalar>
Eigen::DualQuaternion<Scalar>::DualQuaternion (const Scalar *data)
{

}

template<typename Scalar>
Eigen::DualQuaternion<Scalar>::DualQuaternion (const QuaternionS &r, const Vector3S &t)
{

}

template<typename Scalar>
Eigen::DualQuaternion<Scalar>::DualQuaternion (const QuaternionS &qr, const QuaternionS &qd)
{

}

template<typename Scalar>
Eigen::DualQuaternion<Scalar>::DualQuaternion (const Matrix4S &tm)
{

}

template<typename Scalar> void
Eigen::DualQuaternion<Scalar>::normalize ()
{
  

}

template<typename Scalar> inline typename Eigen::DualQuaternion<Scalar>::Vector3S
Eigen::DualQuaternion<Scalar>::getTranslation ()
{
  Eigen::DualQuaternion<Scalar>::Vector3S vec;

  return vec;

}

template<typename Scalar> inline typename Eigen::DualQuaternion<Scalar>
Eigen::DualQuaternion<Scalar>::operator+ (const DualQuaternion<Scalar> &a)
{
  Eigen::DualQuaternion<Scalar> res;

  return res;

}

template<typename Scalar> inline typename Eigen::DualQuaternion<Scalar>
Eigen::DualQuaternion<Scalar>::operator* (const DualQuaternion<Scalar> &a)
{
  Eigen::DualQuaternion<Scalar> res;

  return res;

}

template<typename Scalar> inline typename Eigen::DualQuaternion<Scalar>
Eigen::DualQuaternion<Scalar>::operator* (const Scalar &a)
{
  Eigen::DualQuaternion<Scalar> res(this->real(), this->dual());

  return res;

}

template<typename Scalar> inline typename Eigen::DualQuaternion<Scalar>
Eigen::DualQuaternion<Scalar>::operator~ ()
{
  Eigen::DualQuaternion<Scalar> res(this->qr, this->qd);

  return res;

}

template<typename Scalar> inline typename Eigen::DualQuaternion<Scalar>
Eigen::DualQuaternion<Scalar>::operator! ()
{
  Eigen::DualQuaternion<Scalar> res;

  return res;

}

template<typename Scalar> inline typename Eigen::DualQuaternion<Scalar>
Eigen::DualQuaternion<Scalar>::log ()
{
  Eigen::DualQuaternion<Scalar> res;

  return res;
}

template<typename Scalar> inline typename Eigen::DualQuaternion<Scalar>
Eigen::DualQuaternion<Scalar>::exp ()
{
  Eigen::DualQuaternion<Scalar> res;

  return res;
}

template<typename Scalar> inline typename Eigen::DualQuaternion<Scalar>::Scalar
Eigen::DualQuaternion<Scalar>::dot (const DualQuaternion<Scalar> &a)
{
  Scalar res;

  return res;
}

template<typename Scalar> inline typename Eigen::DualQuaternion<Scalar>::QuaternionS
Eigen::DualQuaternion<Scalar>::real () const
{
  return this->qr;
}

template<typename Scalar> inline typename Eigen::DualQuaternion<Scalar>::QuaternionS&
Eigen::DualQuaternion<Scalar>::real ()
{
  return this->qr;
}

template<typename Scalar> inline typename Eigen::DualQuaternion<Scalar>::QuaternionS
Eigen::DualQuaternion<Scalar>::dual () const
{
  return this->qd;
}

template<typename Scalar> inline typename Eigen::DualQuaternion<Scalar>::QuaternionS&
Eigen::DualQuaternion<Scalar>::dual ()
{
  return this->qd;
}

#endif
