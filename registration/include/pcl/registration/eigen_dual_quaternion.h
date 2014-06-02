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
 * $Id: lmeds.h 1370 2011-06-19 01:06:01Z jspricke $
 *
 */

#ifndef PCL_REGISTRATION_EIGEN_DUAL_QUATERNION_H_
#define PCL_REGISTRATION_EIGEN_DUAL_QUATERNION_H_

#if defined __GNUC__
#  pragma GCC system_header 
#endif

#include <Eigen/Dense>

namespace Eigen{

  template<typename Scalar = float>
  class DualQuaternion
  {
    public:
      typedef Eigen::Quaternion<Scalar> QuaternionS;
      typedef Eigen::Matrix<Scalar, 3, 1> Vector3S;
      typedef Eigen::Matrix<Scalar, 4, 4> Matrix4S;

      EIGEN_MAKE_ALIGNED_OPERATOR_NEW

      DualQuaternion ()
        : qr(QuaternionS(1,0,0,0))
        , qd(QuaternionS(0,0,0,0))
      {
      }

      DualQuaternion (const Scalar *data);
      DualQuaternion (const QuaternionS r, const Vector3S t);
      DualQuaternion (const Matrix4S tm);

      inline void
      normalize ();

      inline Vector3S
      getTranslation ();


      inline DualQuaternion<Scalar>
      operator+ (const DualQuaternion<Scalar> a);

      inline DualQuaternion<Scalar>
      operator* (const DualQuaternion<Scalar> a);

      inline DualQuaternion<Scalar>
      operator~ ();

      inline DualQuaternion<Scalar>
      operator! ();

      inline DualQuaternion<Scalar>
      log ();

      inline DualQuaternion<Scalar>
      exp ();

      inline typename Eigen::DualQuaternion<Scalar>::Scalar
      dot (const DualQuaternion<Scalar> a);

    protected:

    private:
      QuaternionS qr; //Real part
      QuaternionS qd; //Dual part
  };

} // Eigen namespace

#include <pcl/registration/impl/eigen_dual_quaternion.hpp>

#endif    // PCL_REGISTRATION_EIGEN_H_
