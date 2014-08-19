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
 * $Id: multiview_automatic_registration.hpp$
 *
 *
 */

#ifndef PCL_REGISTATION_IMPL_MULTIVIEW_AUTOMATIC_REGISTATION_HPP_
#define PCL_REGISTATION_IMPL_MULTIVIEW_AUTOMATIC_REGISTATION_HPP_

#define MultiviewRegistrationT  pcl::registration::MultiviewRegistration<PointT, LocalRegistration, GraphRegistration, Scalar>

template<typename PointT, typename LocalRegistration, typename GraphRegistration, typename Scalar> inline typename MultiviewRegistrationT::Vertex
MultiviewRegistrationT::addPointCloud (const PointCloudPtr &cloud, const Vector6 &pose)
{
  Vertex v = add_vertex (*local_reg_graph_);
  (*local_reg_graph_)[v].cloud_ = cloud;
  (*local_reg_graph_)[v].pose_ = pose;
  (*local_reg_graph_)[v].index_ = (int) v;
  return (v);
}

template<typename PointT, typename LocalRegistration, typename GraphRegistration, typename Scalar> inline void
MultiviewRegistrationT::compute ()
{
  localRegistration ();
  buildLRGraph ();
  globalRegistration ();
}

template<typename PointT, typename LocalRegistration, typename GraphRegistration, typename Scalar> inline typename MultiviewRegistrationT::PointCloudPtr
MultiviewRegistrationT::getTransformedCloud (const Vertex &vertex) const
{
  PointCloudPtr pc (new PointCloud);

  return pc;
}

template<typename PointT, typename LocalRegistration, typename GraphRegistration, typename Scalar> inline typename MultiviewRegistrationT::PointCloudPtr
MultiviewRegistrationT::getConcatenatedCloud () const
{
  PointCloudPtr pc (new PointCloud);

  return pc;
}

template<typename PointT, typename LocalRegistration, typename GraphRegistration, typename Scalar> inline void
MultiviewRegistrationT::localRegistration ()
{
  int num_v = num_vertices (*local_reg_graph_);
  for (int source = 0; source < num_v; ++source)
  {
    Vertex source_v = source;
    PointCloud source_cloud = (*local_reg_graph_)[source_v].cloud_;
    for (int target = source + 1; target < num_v; ++ target)
    {
      Vertex target_v = target;
      PointCloud target_cloud = (*local_reg_graph_)[target_v].cloud_;
      LocalRegistration reg;
      reg.setInputCloud (source_cloud);
      reg.setInputTarget (target_cloud);
      PointCloud temp_final;
      reg.align (temp_final);
      reg.getFitnessScore ();
    }
  }
}

template<typename PointT, typename LocalRegistration, typename GraphRegistration, typename Scalar> inline void
MultiviewRegistrationT::buildLRGraph ()
{

}

template<typename PointT, typename LocalRegistration, typename GraphRegistration, typename Scalar> inline void
MultiviewRegistrationT::globalRegistration ()
{

}

//TODO
//#define PCL_INSTANTIATE_MULTIVIEW_REGISTRATION(T) template class PCL_EXPORTS pcl::registration::MultiviewRegistration<T>;

#endif //PCL_REGISTATION_IMPL_MULTIVIEW_AUTOMATIC_REGISTATION_HPP_
