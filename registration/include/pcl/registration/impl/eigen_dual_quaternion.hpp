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
  qd = QuaternionS (0, t (0), t (1), t (2)) * r;
  // FIXME : Can we use qd * Eigen::UniformScaling(0.5)?
  qd = QuaternionS (qd.w () * 0.5, qd.x () * 0.5, qd.y () * 0.5, qd.z () * 0.5);



}

template<typename Scalar>
Eigen::DualQuaternion<Scalar>::DualQuaternion (const QuaternionS &qr, const QuaternionS &qd)
{
  this->qr = qr;
  this->qd = qd;
}

template<typename Scalar>
Eigen::DualQuaternion<Scalar>::DualQuaternion (const Matrix4S &tm)
{
  Eigen::Matrix<Scalar, 3, 3> rot = Eigen::Matrix<Scalar, 3, 3>::Zero();
  for (int i=0; i<3; ++i)
  {
    for (int j=0; j<3; ++j)
    {
      rot (i,j) = tm (i,j);
    }
  }
  qr = rot;
  qd = QuaternionS (0, tm (0,3), tm (1,3), tm (2,3)) * qr;
  // FIXME : Can we use qd * Eigen::UniformScaling(0.5)?
  qd = QuaternionS (qd.w () * 0.5, qd.x () * 0.5, qd.y () * 0.5, qd.z () * 0.5);
}

template<typename Scalar> void
Eigen::DualQuaternion<Scalar>::normalize ()
{
  Scalar sign = 1;
  qr.normalize ();
  if (qr.w () < 0)
  {
    qr = QuaternionS (-qr.w (), -qr.x (), -qr.y (), -qr.z ());
  }
  Scalar norm = qr.norm () * sign;
  // TODO : Check norm==0?

  qd.w () = qd.w () / norm;
  qd.x () = qd.x () / norm;
  qd.y () = qd.y () / norm;
  qd.z () = qd.z () / norm;

  Scalar dot_rd = qr.dot (qd);

  qd.w () = qd.w () + qr.w () * (-dot_rd);
  qd.x () = qd.x () + qr.x () * (-dot_rd);
  qd.y () = qd.y () + qr.y () * (-dot_rd);
  qd.z () = qd.z () + qr.z () * (-dot_rd);
}

template<typename Scalar> inline typename Eigen::DualQuaternion<Scalar>::Matrix4S
Eigen::DualQuaternion<Scalar>::getMatrix ()
{
  Matrix4S transform_matrix;
  //Eigen::Transform<Scalar, 3, Eigen::Affine> transform;
  //transform.translate (getTranslation ());
  //transform.rotate (qr);
  Eigen::Matrix<Scalar, 3, 3> rot_matrix = qr.matrix ();
  Vector3S translation = getTranslation ();
  for (int i = 0; i < 3; ++i)
  {
    for (int j = 0; j < 3; ++j)
    {
      transform_matrix (i, j) = rot_matrix (i, j);
    }
    transform_matrix (3, i) = 0;
    transform_matrix (i, 3) = translation (i);
  }
  transform_matrix (3, 3) = 0;
  return (transform_matrix) ;
}

template<typename Scalar> inline typename Eigen::DualQuaternion<Scalar>::Vector3S
Eigen::DualQuaternion<Scalar>::getTranslation ()
{
  QuaternionS t = qd * qr.conjugate ();
  Scalar qr_norm = qr.squaredNorm ();
  return (Vector3S (t.x () * 2 / qr_norm, t.y () * 2 / qr_norm, t.z () * 2 / qr_norm));
}

template<typename Scalar> inline typename Eigen::DualQuaternion<Scalar>
Eigen::DualQuaternion<Scalar>::operator+ (const DualQuaternion<Scalar> &a)
{
  Eigen::DualQuaternion<Scalar> res;

  res.real ().w () = qr.w () + a.real ().w ();
  res.real ().x () = qr.x () + a.real ().x ();
  res.real ().y () = qr.y () + a.real ().y ();
  res.real ().z () = qr.z () + a.real ().z ();

  res.dual ().w () = qd.w () + a.dual ().w ();
  res.dual ().x () = qd.x () + a.dual ().x ();
  res.dual ().y () = qd.y () + a.dual ().y ();
  res.dual ().z () = qd.z () + a.dual ().z ();

  return (res);
}

template<typename Scalar> inline typename Eigen::DualQuaternion<Scalar>
Eigen::DualQuaternion<Scalar>::operator* (const DualQuaternion<Scalar> &a)
{
  // this * a
  Eigen::DualQuaternion<Scalar> res;
  res.real() = qr * a.real();
  QuaternionS dual_1 = qr * a.dual ();
  QuaternionS dual_2 = qd * a.real ();

  res.dual().w () = dual_1.w () + dual_2.w ();
  res.dual().x () = dual_1.x () + dual_2.x ();
  res.dual().y () = dual_1.y () + dual_2.y ();
  res.dual().z () = dual_1.z () + dual_2.z ();

  return (res);
}

template<typename Scalar> inline typename Eigen::DualQuaternion<Scalar>
Eigen::DualQuaternion<Scalar>::operator* (const Scalar &a)
{
  Eigen::DualQuaternion<Scalar> res;

  res.real() = QuaternionS (qr.w () * a, qr.x () * a, qr.y () * a, qr.z () * a);
  res.dual() = QuaternionS (qd.w () * a, qd.x () * a, qd.y () * a, qd.z () * a);

  return (res);
}

template<typename Scalar> inline typename Eigen::DualQuaternion<Scalar>
Eigen::DualQuaternion<Scalar>::operator~ ()
{
  Eigen::DualQuaternion<Scalar> res (qr.conjugate (), qd.conjugate () * -1);
  return res;
}

template<typename Scalar> inline typename Eigen::DualQuaternion<Scalar>
Eigen::DualQuaternion<Scalar>::conjugate ()
{
  Eigen::DualQuaternion<Scalar> res (qr.conjugate (), qd.conjugate ());
  return res;
}

template<typename Scalar> inline typename Eigen::DualQuaternion<Scalar>
Eigen::DualQuaternion<Scalar>::log ()
{
  const Scalar dq_epsilon = 1e-8;
  // Small angle assumption TODO
  // Unitary?
  Eigen::DualQuaternion<Scalar> res;
  res.real () = qr;
  res.dual () = qd;

  const Scalar h0 = std::acos (res.real ().w ());
  std::cerr << "h0 " << h0 << "\n";

  if (h0*h0  < dq_epsilon)
  {
    res.real ().w () = 0.0;
    res.real ().x () = res.real ().x () * 0.5;
    res.real ().y () = res.real ().y () * 0.5;
    res.real ().z () = res.real ().z () * 0.5;

    res.dual ().w () = 0.0;
    res.dual ().x () = res.dual ().x () * 0.5;
    res.dual ().y () = res.dual ().y () * 0.5;
    res.dual ().z () = res.dual ().z () * 0.5;

    return (res);
  }

  res.real ().w () = 0.0;
  const Scalar ish0 = 1.0 / res.real ().norm ();
  res.real ().normalize ();

  const Scalar he = - res.dual ().w () * ish0;
  res.dual ().w () = 0.0;

  QuaternionS rp(res.real ());
  Scalar factor = - res.real ().dot (res.dual ()) / res.real ().dot (res.real ());
  rp = QuaternionS (rp.w () * factor, rp.x () * factor, rp.y () * factor, rp.z () * factor);
  res.dual ().w () = (res.dual ().w () + rp.w ()) * ish0 * h0;
  res.dual ().x () = (res.dual ().x () + rp.x ()) * ish0 * h0;
  res.dual ().y () = (res.dual ().y () + rp.y ()) * ish0 * h0;
  res.dual ().z () = (res.dual ().z () + rp.z ()) * ish0 * h0;

  rp = res.real ();
  rp = QuaternionS (rp.w () * he, rp.x () * he, rp.y () * he, rp.z () * he);

  res.dual ().w () = (res.dual ().w () + rp.w ()) * 0.5;
  res.dual ().x () = (res.dual ().x () + rp.x ()) * 0.5;
  res.dual ().y () = (res.dual ().y () + rp.y ()) * 0.5;
  res.dual ().z () = (res.dual ().z () + rp.z ()) * 0.5;

  res.real ().w () = res.real ().w () * h0 * 0.5;
  res.real ().x () = res.real ().x () * h0 * 0.5;
  res.real ().y () = res.real ().y () * h0 * 0.5;
  res.real ().z () = res.real ().z () * h0 * 0.5;

  return res;
}

template<typename Scalar> inline typename Eigen::DualQuaternion<Scalar>
Eigen::DualQuaternion<Scalar>::exp ()
{
  Eigen::DualQuaternion<Scalar> res;

  res.real () = qr;
  res.dual () = qd;
  const Scalar h0 = 2.0 * res.real ().norm ();

  const Scalar he = 4.0 * res.real ().dot (res.dual ()) / h0;
  const Scalar sh0 = sin (h0);
  const Scalar ch0 = cos (h0);

  QuaternionS rp (res.real ());
  Scalar factor = - res.real ().dot (res.dual ()) / res.real ().dot (res.real ());
  rp = QuaternionS (rp.w () * factor, rp.x () * factor, rp.y () * factor, rp.z () * factor);

  res.dual ().w () = (res.dual ().w () + rp.w ()) * 2.0 / h0 * sh0;
  res.dual ().x () = (res.dual ().x () + rp.x ()) * 2.0 / h0 * sh0;
  res.dual ().y () = (res.dual ().y () + rp.y ()) * 2.0 / h0 * sh0;
  res.dual ().z () = (res.dual ().z () + rp.z ()) * 2.0 / h0 * sh0;

  rp = res.real ();
  factor = he * ch0 * 2.0 / h0;
  rp = QuaternionS (rp.w () * factor, rp.x () * factor, rp.y () * factor, rp.z () * factor);

  res.dual ().w () = (res.dual ().w () + rp.w ()) * -he * sh0;
  res.dual ().x () = (res.dual ().x () + rp.x ()) * -he * sh0;
  res.dual ().y () = (res.dual ().y () + rp.y ()) * -he * sh0;
  res.dual ().z () = (res.dual ().z () + rp.z ()) * -he * sh0;

  //res.real ().w () = res.real ().w () * sh0 * 2.0 / h0;
  res.real ().w () = ch0;
  res.real ().x () = res.real ().x () * sh0 * 2.0 / h0;
  res.real ().y () = res.real ().y () * sh0 * 2.0 / h0;
  res.real ().z () = res.real ().z () * sh0 * 2.0 / h0;

  res.normalize ();
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
