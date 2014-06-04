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
  qr = r;
  qd = QuaternionS (0, t (0), t (1), t (2)) * r * 0.5;
}

template<typename Scalar>
Eigen::DualQuaternion<Scalar>::DualQuaternion (const QuaternionS &qr, const QuaternionS &qd)
{
  qr = qr;
  qd = qd;
}

template<typename Scalar>
Eigen::DualQuaternion<Scalar>::DualQuaternion (const Matrix4S &tm)
{
  qr = QuaternionS (tm);
  qd = QuaternionS (0, tm (0,3), tm (1,3), tm (2,3)) * qr * 0.5;
}

template<typename Scalar> void
Eigen::DualQuaternion<Scalar>::normalize ()
{
  Scalar sign;
  if (qr.w () < 0)
  {
    sign = -1;
  }
  else
  {
    sign = 1;
  }
  Scalar norm = qr.norm () * sign;
  // TODO : Check norm==0?
  qr = qr * (1.0 / norm);
  qd = qd * (1.0 / norm);
  qd = qd + qr * (-1 * qr.dot (qd));
}

template<typename Scalar> inline typename Eigen::DualQuaternion<Scalar>::Vector3S
Eigen::DualQuaternion<Scalar>::getTranslation ()
{
  QuaternionS t = qd * qr.conjugate () * 2;
  return Vector3S(t.x(), t.y(), t.z());
}

template<typename Scalar> inline typename Eigen::DualQuaternion<Scalar>
Eigen::DualQuaternion<Scalar>::operator+ (const DualQuaternion<Scalar> &a)
{
  Eigen::DualQuaternion<Scalar> res;

  res.real ().w () = qr.w () + a.real ().w ();
  res.real ().x () = qr.x () + a.real ().x ();
  res.real ().y () = qr.y () + a.real ().y ();
  res.real ().z () = qr.z () + a.real ().z ();

  res.dual ().w () = qr.w () + a.real ().w();
  res.dual ().x () = qr.x () + a.real ().x();
  res.dual ().y () = qr.y () + a.real ().y();
  res.dual ().z () = qr.z () + a.real ().z();

  return (res);
}

template<typename Scalar> inline typename Eigen::DualQuaternion<Scalar>
Eigen::DualQuaternion<Scalar>::operator* (const DualQuaternion<Scalar> &a)
{
  // this * a
  Eigen::DualQuaternion<Scalar> res;
  res.real() = qr * a.real();
  res.dual() = qr * a.dual() + qd * a.real();

  return (res);
}

template<typename Scalar> inline typename Eigen::DualQuaternion<Scalar>
Eigen::DualQuaternion<Scalar>::operator* (const Scalar &a)
{
  Eigen::DualQuaternion<Scalar> res;
  res.real() = qr * a;
  res.dual() = qd * a;

  return (res);
}

template<typename Scalar> inline typename Eigen::DualQuaternion<Scalar>
Eigen::DualQuaternion<Scalar>::operator~ ()
{
  Eigen::DualQuaternion<Scalar> res (qr.conjugate (), qd.conjugate () * -1);
  return res;
}

template<typename Scalar> inline typename Eigen::DualQuaternion<Scalar>
Eigen::DualQuaternion<Scalar>::operator! ()
{
  Eigen::DualQuaternion<Scalar> res (qr.conjugate (), qd.conjugate ());
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

template<typename Scalar> inline Scalar
Eigen::DualQuaternion<Scalar>::dot (const DualQuaternion<Scalar> &a)
{
  Scalar res = qr.dot (a.real ()) + qd.dot (a.dual ());
  return res;
}

template<typename Scalar> inline typename Eigen::DualQuaternion<Scalar>::QuaternionS
Eigen::DualQuaternion<Scalar>::real () const
{
  return qr;
}

template<typename Scalar> inline typename Eigen::DualQuaternion<Scalar>::QuaternionS&
Eigen::DualQuaternion<Scalar>::real ()
{
  return qr;
}

template<typename Scalar> inline typename Eigen::DualQuaternion<Scalar>::QuaternionS
Eigen::DualQuaternion<Scalar>::dual () const
{
  return qd;
}

template<typename Scalar> inline typename Eigen::DualQuaternion<Scalar>::QuaternionS&
Eigen::DualQuaternion<Scalar>::dual ()
{
  return qd;
}

#endif
