/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2012, Willow Garage, Inc.
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

#ifndef PCL_REGISTRATION_IMPL_DQ_DIFFUSION_HPP_
#define PCL_REGISTRATION_IMPL_DQ_DIFFUSION_HPP_

template<typename PointT> inline typename pcl::registration::DQDiffusion<PointT>::ViewGraphPtr
pcl::registration::DQDiffusion<PointT>::getViewGraph () const
{
  return (view_graph_);
}

template<typename PointT> typename pcl::registration::DQDiffusion<PointT>::ViewGraph::vertices_size_type
pcl::registration::DQDiffusion<PointT>::getNumVertices () const
{
  return (num_vertices (*view_graph_));
}

template<typename PointT> typename pcl::registration::DQDiffusion<PointT>::Vertex
pcl::registration::DQDiffusion<PointT>::addPointCloud (const PointCloudPtr &cloud, const Eigen::Vector6f &pose)
{
  Vertex v = add_vertex(*view_graph_);
  (*view_graph_)[v].cloud_ = cloud;
  if (v == 0 && pose != Eigen::Vector6f::Zero ())
  {
    PCL_WARN("[pcl::registration::DQDiffusion::addPointCloud] The pose estimate is ignored for the first cloud in the graph since that will become the reference pose.\n");
    (*view_graph_)[v].pose_ = Eigen::Vector6f::Zero ();
    return (v);
  }
  (*view_graph_)[v].pose_ = pose;
  return (v);
}

template<typename PointT> inline void
pcl::registration::DQDiffusion<PointT>::setPointCloud (const Vertex &vertex, const PointCloudPtr &cloud)
{
  if (vertex >= getNumVertices())
  {
    PCL_ERROR("[pcl::registration::DQDiffusion::setPointCloud] You are attempting to set a point cloud to a non-existing graph vertex.\n");
    return;
  }
  (*view_graph_)[vertex].cloud_ = cloud;
}

template<typename PointT> inline typename pcl::registration::DQDiffusion<PointT>::PointCloudPtr
pcl::registration::DQDiffusion<PointT>::getPointCloud (const Vertex &vertex) const
{
  if (vertex >= getNumVertices ())
  {
    PCL_ERROR("[pcl::registration::DQDiffusion::getPointCloud] You are attempting to get a point cloud from a non-existing graph vertex.\n");
    return (PointCloudPtr ());
  }
  return ((*view_graph_)[vertex].cloud_);
}

template<typename PointT> inline void
pcl::registration::DQDiffusion<PointT>::setPose (const Vertex &vertex, const Eigen::Vector6f &pose)
{
  if (vertex >= getNumVertices ())
  {
    PCL_ERROR("[pcl::registration::DQDiffusion::setPose] You are attempting to set a pose estimate to a non-existing graph vertex.\n");
    return;
  }
  if (vertex == 0)
  {
    PCL_ERROR("[pcl::registration::DQDiffusion::setPose] The pose estimate is ignored for the first cloud in the graph since that will become the reference pose.\n");
    return;
  }
  (*view_graph_)[vertex].pose_ = pose;
}

template<typename PointT> inline Eigen::Vector6f
pcl::registration::DQDiffusion<PointT>::getPose (const Vertex &vertex) const
{
  if (vertex >= getNumVertices ())
  {
    PCL_ERROR("[pcl::registration::LUM::getPose] You are attempting to get a pose estimate from a non-existing graph vertex.\n");
    return (Eigen::Vector6f::Zero ());
  }
  return ((*view_graph_)[vertex].pose_);
}

template<typename PointT> inline Eigen::Affine3f
pcl::registration::DQDiffusion<PointT>::getTransformation (const Vertex &vertex) const
{
  Eigen::Vector6f pose = getPose (vertex);
  return (pcl::getTransformation (pose (0), pose (1), pose (2), pose (3), pose (4), pose (5)));
}

template<typename PointT> void
compute ()
{
  // TODO
  return;
}

template<typename PointT> typename pcl::registration::DQDiffusion<PointT>::PointCloudPtr
pcl::registration::DQDiffusion<PointT>::getTransformedCloud (const Vertex &vertex) const
{
  PointCloudPtr out (new PointCloud);
  // TODO
  //pcl::transformPointCloud (*getPointCloud (vertex), *out, getTransformation (vertex));
  return (out);
}

template<typename PointT> typename pcl::registration::DQDiffusion<PointT>::PointCloudPtr
pcl::registration::DQDiffusion<PointT>::getConcatenatedCloud () const
{
  PointCloudPtr out (new PointCloud);
  // TODO
  //
  return (out);
}

#define PCL_INSTANTIATE_DQ_DIFFUSION(T) template class PCL_EXPORTS pcl::registration::DQDiffusion<T>;

#endif // PCL_REGISTRATION_IMPL_DQ_DIFFUSION_HPP_
