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

#ifndef PCL_REGISTRATION_IMPL_DQ_DIFFUSION_HPP_
#define PCL_REGISTRATION_IMPL_DQ_DIFFUSION_HPP_

template<typename PointT, typename Scalar> inline typename pcl::registration::DQDiffusion<PointT, Scalar>::ViewGraphPtr
pcl::registration::DQDiffusion<PointT, Scalar>::getViewGraph () const
{
  return (view_graph_);
}

template<typename PointT, typename Scalar> typename pcl::registration::DQDiffusion<PointT, Scalar>::ViewGraph::vertices_size_type
pcl::registration::DQDiffusion<PointT, Scalar>::getNumVertices () const
{
  return (num_vertices (*view_graph_));
}

template<typename PointT, typename Scalar> void
pcl::registration::DQDiffusion<PointT, Scalar>::setDiffusionIterations (int diffusion_iterations)
{
  diffusion_iterations_ = diffusion_iterations;
}

template<typename PointT, typename Scalar> inline int
pcl::registration::DQDiffusion<PointT, Scalar>::getDiffusionIterations () const
{
  return (diffusion_iterations_);
}

template<typename PointT, typename Scalar> typename pcl::registration::DQDiffusion<PointT, Scalar>::Vertex
pcl::registration::DQDiffusion<PointT, Scalar>::addPointCloud (const PointCloudPtr &cloud, const Vector6 &pose)
{
  Vertex v = add_vertex(*view_graph_);
  (*view_graph_)[v].cloud_ = cloud;
  if (v == 0 && pose != Vector6::Zero ())
  {
    PCL_WARN("[pcl::registration::DQDiffusion::addPointCloud] The pose estimate is ignored for the first cloud in the graph since that will become the reference pose.\n");
    (*view_graph_)[v].pose_ = Vector6::Zero ();
    return (v);
  }
  (*view_graph_)[v].pose_ = pose;
  return (v);
}

template<typename PointT, typename Scalar> inline void
pcl::registration::DQDiffusion<PointT, Scalar>::setPointCloud (const Vertex &vertex, const PointCloudPtr &cloud)
{
  if (vertex >= getNumVertices())
  {
    PCL_ERROR("[pcl::registration::DQDiffusion::setPointCloud] You are attempting to set a point cloud to a non-existing graph vertex.\n");
    return;
  }
  (*view_graph_)[vertex].cloud_ = cloud;
}

template<typename PointT, typename Scalar> inline typename pcl::registration::DQDiffusion<PointT, Scalar>::PointCloudPtr
pcl::registration::DQDiffusion<PointT, Scalar>::getPointCloud (const Vertex &vertex) const
{
  if (vertex >= getNumVertices ())
  {
    PCL_ERROR("[pcl::registration::DQDiffusion::getPointCloud] You are attempting to get a point cloud from a non-existing graph vertex.\n");
    return (PointCloudPtr ());
  }
  return ((*view_graph_)[vertex].cloud_);
}

template<typename PointT, typename Scalar> inline void
pcl::registration::DQDiffusion<PointT, Scalar>::addPairwiseTransformation (const Vertex &from_vertex, const Vertex &to_vertex, Matrix4 &transformation, Scalar weight)
{
  Edge e;
  bool present;
  Vertex source_vertex, target_vertex;
  Matrix4 edge_transformation;
  if (from_vertex < to_vertex)
  {
    source_vertex = from_vertex;
    target_vertex = to_vertex;
    edge_transformation = transformation;
  }
  else
  {
    source_vertex = to_vertex;
    target_vertex = from_vertex;
    edge_transformation = (Affine3 (transformation)).inverse (Eigen::Affine).matrix ();
  }
  boost::tuples::tie (e, present) = edge (source_vertex, target_vertex, *view_graph_);
  if (!present)
  {
    boost::tuples::tie (e, present) = add_edge (source_vertex, target_vertex, *view_graph_);
  }
  (*view_graph_)[e].pairwise_transform_ = Eigen::DualQuaternion<Scalar> (edge_transformation);
  (*view_graph_)[e].weight_ = weight;
}

template<typename PointT, typename Scalar> inline void
pcl::registration::DQDiffusion<PointT, Scalar>::setLinearApproximation (bool linear_approximation)
{
  linear_approximation_ = linear_approximation;
}

template<typename PointT, typename Scalar> inline void
pcl::registration::DQDiffusion<PointT, Scalar>::setPose (const Vertex &vertex, const Vector6 &pose)
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

template<typename PointT, typename Scalar> inline typename pcl::registration::DQDiffusion<PointT, Scalar>::Vector6
pcl::registration::DQDiffusion<PointT, Scalar>::getPose (const Vertex &vertex) const
{
  if (vertex >= getNumVertices ())
  {
    PCL_ERROR("[pcl::registration::DQDiffusion::getPose] You are attempting to get a pose estimate from a non-existing graph vertex.\n");
    return (Vector6::Zero ());
  }
  return ((*view_graph_)[vertex].pose_);
}

template<typename PointT, typename Scalar> inline typename pcl::registration::DQDiffusion<PointT, Scalar>::Matrix4
pcl::registration::DQDiffusion<PointT, Scalar>::getTransformation (const Vertex &vertex) const
{
  Vector6 pose = getPose (vertex);
  return (pcl::getTransformation (pose (0), pose (1), pose (2), pose (3), pose (4), pose (5)).matrix());
}

template<typename PointT, typename Scalar> void
pcl::registration::DQDiffusion<PointT, Scalar>::linearDiffusion ()
{
  for (int rep = 0; rep != diffusion_iterations_; ++rep)
  {
    typename ViewGraph::vertex_iterator v, v_end;
    typename ViewGraph::out_edge_iterator e, e_end;
    for (boost::tuples::tie (v, v_end) = vertices (*view_graph_); v != v_end; ++v)
    {
      Eigen::DualQuaternion<Scalar> q2; //vector of Q of each pose?
      for (boost::tuples::tie (e, e_end) = out_edges (*v, *view_graph_); e != e_end; ++e)
      {
        Eigen::DualQuaternion<Scalar> qi = (*view_graph_)[(*e)].diffused_transform_ * (*view_graph_)[(*e)].pairwise_transform_;
        const Scalar w = copysign(1.0, (*view_graph_)[(*e)].diffused_transform_.real ().dot (qi.real ()))*(*view_graph_)[(*e)].weight_/(*view_graph_)[(*v)].weight_sum_;
        q2 = q2 + qi * w;
        std::cerr << "Vertex " << *v << " : Edge " << *e << "\n";
      }
      q2.normalize ();
    }
    //Stuff TODO
  }
}

template<typename PointT, typename Scalar> void
pcl::registration::DQDiffusion<PointT, Scalar>::manifoldDiffusion ()
{

}

template<typename PointT, typename Scalar> void
pcl::registration::DQDiffusion<PointT, Scalar>::compute ()
{
  // Sanity checks

  // BFS index list starting from 0
  // Use transformations from 0 to estimate the pose.
  // Question : Use intermediate pose estimates?
  // Issue : More than one connected components : No transformation from 0 to some vetices : Handle it?
  // TODO=

  boost::bfs_order_visitor vis;
  boost::breadth_first_search(*view_graph_, vertex(0, *view_graph_), boost::visitor(vis));

  int num_vertices = getNumVertices();
  Edge e;
  bool present;
  for (int v = 1; v < num_vertices - 1; ++v)
  {
    Vertex target = v;
    if ((*view_graph_)[target].pose_ != Vector6::Zero()){
      continue;
    }
    for (int u = 0; u < v; ++u)
    {
      Vertex source = u;
      boost::tuples::tie (e, present) = edge (source, target, *view_graph_);
      if (present)
      {
        Affine3 new_transform = Affine3 (getTransformation (source) * (*view_graph_)[e].pairwise_transform_.getMatrix ());
        Vector6 p = Vector6::Zero ();
        pcl::getTranslationAndEulerAngles (new_transform, p (0), p (1), p (2), p (3), p (4), p (5));
        (*view_graph_)[target].pose_ = p;
      }
    }
  }

  // Apply dual quaternion average (DLB/DIB) on the graph pose estimates
  // Iterate for num_iterations time
  if (linear_approximation_)
  {
    linearDiffusion ();
  }
  else
  {
    manifoldDiffusion ();
  }
}

template<typename PointT, typename Scalar> typename pcl::registration::DQDiffusion<PointT, Scalar>::PointCloudPtr
pcl::registration::DQDiffusion<PointT, Scalar>::getTransformedCloud (const Vertex &vertex) const
{
  PointCloudPtr out (new PointCloud);
  pcl::transformPointCloud (*getPointCloud (vertex), *out, getTransformation (vertex));
  return (out);
}

template<typename PointT, typename Scalar> typename pcl::registration::DQDiffusion<PointT, Scalar>::PointCloudPtr
pcl::registration::DQDiffusion<PointT, Scalar>::getConcatenatedCloud () const
{
  PointCloudPtr out (new PointCloud);
  typename ViewGraph::vertex_iterator v, v_end;
  for (boost::tuples::tie (v, v_end) = vertices (*view_graph_); v != v_end; ++v)
  {
    PointCloud temp;
    pcl::transformPointCloud (*getPointCloud (*v), temp, getTransformation (*v));
    *out += temp;
  }
  return (out);
}

template<typename PointT, typename Scalar> inline Scalar
pcl::registration::DQDiffusion<PointT, Scalar>::getFitnessScore ()
{
  // TODO RMS error same as rmste() in demo code
  double ste = 0.0;
  typename ViewGraph::vertex_iterator v, v_end;
  typename ViewGraph::out_edge_iterator e, e_end;
  for (boost::tuples::tie (v, v_end) = vertices (*view_graph_); v != v_end; ++v)
  {
    for (boost::tuples::tie (e, e_end) = out_edges (*v, *view_graph_); e != e_end; ++e)
    {
      Eigen::DualQuaternion<Scalar> q_e = (*view_graph_)[(*e)].pairwise_transform_;
      //Eigen::DualQuaternion<Scalar> q = ((!q_e)*(*view_graph_)[(*e)].target ()*!(*view_graph_)[(*v)].pose_).normalize();
      Eigen::DualQuaternion<Scalar> q = ((q_e.conjugate ())*(*view_graph_)[(*e)].diffused_transform_*(*view_graph_)[(*e)].diffused_transform_.conjugate ());
      q.normalize ();
      q = q.log();
      ste += q.dot(q)*((*view_graph_)[(*e)].weight_);
      std::cerr << "Vertex " << *v << " : Edge " << *e << " : STE " << ste << "\n";
    }
  }
  return std::sqrt(ste);
}

#define PCL_INSTANTIATE_DQ_DIFFUSION(T) template class PCL_EXPORTS pcl::registration::DQDiffusion<T>;

#endif // PCL_REGISTRATION_IMPL_DQ_DIFFUSION_HPP_
