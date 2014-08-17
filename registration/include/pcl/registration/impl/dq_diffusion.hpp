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
  (*view_graph_)[v].weight_sum_ = 0.0;
  if (v == 0 && pose != Vector6::Zero ())
  {
    PCL_WARN("[pcl::registration::DQDiffusion::addPointCloud] The pose estimate is ignored for the first cloud in the graph since that will become the reference pose.\n");
    (*view_graph_)[v].pose_ = Eigen::DualQuaternion<Scalar> ();//Vector6::Zero ();
    return (v);
  }
  Vector6 p = pose;
  std::cerr << "addPointCloud " << v << " " << p(0) << " " << p(1) << " " << p(2) << " " << p(3) << " " << p(4) << " " << p(5) << "\n";
  (*view_graph_)[v].pose_ = Eigen::DualQuaternion<Scalar> (pcl::getTransformation (pose (0), pose (1), pose (2), pose (3), pose (4), pose (5)).matrix());

  Affine3 new_transform = Affine3 ((*view_graph_)[v].pose_.getMatrix ());
  p = Vector6::Zero ();
  pcl::getTranslationAndEulerAngles (new_transform, p (0), p (1), p (2), p (3), p (4), p (5));
  std::cerr << "addPointCloud " << v << " " << p(0) << " " << p(1) << " " << p(2) << " " << p(3) << " " << p(4) << " " << p(5) << "\n";
  std::cerr << "\n";
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
  Eigen::DualQuaternion<Scalar> edge_transformation;
  if (from_vertex < to_vertex)
  {
    source_vertex = from_vertex;
    target_vertex = to_vertex;
    edge_transformation = Eigen::DualQuaternion<Scalar> (transformation);
  }
  else
  {
    source_vertex = to_vertex;
    target_vertex = from_vertex;
    edge_transformation = Eigen::DualQuaternion<Scalar> (transformation).conjugate ();
    //edge_transformation = (Affine3 (transformation)).inverse (Eigen::Affine).matrix ();
  }
  boost::tuples::tie (e, present) = edge (source_vertex, target_vertex, *view_graph_);
  if (!present)
  {
    boost::tuples::tie (e, present) = add_edge (source_vertex, target_vertex, *view_graph_);
  }
  else
  {
    Scalar current_weight = (*view_graph_)[e].weight_;
    (*view_graph_)[source_vertex].weight_sum_ -= current_weight;
    (*view_graph_)[target_vertex].weight_sum_ -= current_weight;
  }
  (*view_graph_)[e].pairwise_transform_ = edge_transformation;

  // DEBUG
  Affine3 new_transform = Affine3 ((*view_graph_)[e].pairwise_transform_.getMatrix ());
  Vector6 p = Vector6::Zero ();
  pcl::getTranslationAndEulerAngles (new_transform, p (0), p (1), p (2), p (3), p (4), p (5));
  std::cerr << "addPairwiseTransformation " << e << " " << p(0) << " " << p(1) << " " << p(2) << " " << p(3) << " " << p(4) << " " << p(5) << "\n";

  (*view_graph_)[e].weight_ = weight;
  (*view_graph_)[source_vertex].weight_sum_ += weight;
  (*view_graph_)[target_vertex].weight_sum_ += weight;
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
  (*view_graph_)[vertex].pose_ = Eigen::DualQuaternion<Scalar> (pcl::getTransformation (pose (0), pose (1), pose (2), pose (3), pose (4), pose (5)).matrix());
}

template<typename PointT, typename Scalar> inline typename pcl::registration::DQDiffusion<PointT, Scalar>::Vector6
pcl::registration::DQDiffusion<PointT, Scalar>::getPose (const Vertex &vertex) const
{
  if (vertex >= getNumVertices ())
  {
    PCL_ERROR("[pcl::registration::DQDiffusion::getPose] You are attempting to get a pose estimate from a non-existing graph vertex.\n");
    return (Vector6::Zero ());
  }
  Affine3 new_transform = Affine3 (getTransformation (vertex));
  Vector6 p = Vector6::Zero ();
  pcl::getTranslationAndEulerAngles (new_transform, p (0), p (1), p (2), p (3), p (4), p (5));
  return (p);
}

template<typename PointT, typename Scalar> inline typename pcl::registration::DQDiffusion<PointT, Scalar>::Matrix4
pcl::registration::DQDiffusion<PointT, Scalar>::getTransformation (const Vertex &vertex) const
{
  if (vertex >= getNumVertices ())
  {
    PCL_ERROR("[pcl::registration::DQDiffusion::getPose] You are attempting to get a pose estimate from a non-existing graph vertex.\n");
    return (Matrix4::Zero ());
  }
  return ((*view_graph_)[vertex].pose_. getMatrix ());
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
        //Eigen::DualQuaternion<Scalar> qi = (*view_graph_)[(*e)].diffused_transform_ * (*view_graph_)[(*e)].pairwise_transform_;
        //const Scalar w = copysign(1.0, (*view_graph_)[(*e)].diffused_transform_.real ().dot (qi.real ()))*(*view_graph_)[(*e)].weight_/(*view_graph_)[(*v)].weight_sum_;
        //q2 = q2 + qi * w;
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


template<typename PointT, typename Scalar> inline Eigen::DualQuaternion<Scalar>
pcl::registration::DQDiffusion<PointT, Scalar>::getPairwiseTransformation (Edge &e, Vertex& source, Vertex& target)
{
  if (target > source)
  {
    return (*view_graph_)[e].pairwise_transform_;
  }
  else
  {
    return (*view_graph_)[e].pairwise_transform_.conjugate ();
    //return Eigen::DualQuaternion<Scalar> ((Affine3 ((*view_graph_)[e].pairwise_transform_.getMatrix ())).inverse (Eigen::Affine).matrix ());

  }

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

  std::vector<int> order;
  boost::bfs_order_visitor vis  = boost::bfs_order_visitor(order);
  boost::breadth_first_search(*view_graph_, vertex(0, *view_graph_), boost::visitor(vis));
  //boost::undirected_dfs(*view_graph_, vertex(0, *view_graph_), vis);

  // order.size() should be same as num_vertices
  // int num_vertices = getNumVertices();

  Edge e;
  bool present;

  for (int u = 0; u < getNumVertices(); ++u)
  {
      Vertex target = u;
      Vector6 p = getPose (target);
      std::cerr << target << " ";
      std::cerr << p(0) << " " << p(1) << " " << p(2) << " " << p(3) << " " << p(4) << " " << p(5) << "\n";
  }

  for (int v = 1; v < order.size (); ++v)
  {
    Vertex target = order[v];
    if (getTransformation (target) != Eigen::DualQuaternion<Scalar> ().getMatrix ())
    {
      std::cerr << "Continuing " << target << "\n";
      //continue;
    }

    for (int u = 0; u < v; ++u)
    {
      Vertex source = order[u];
      boost::tuples::tie (e, present) = edge (source, target, *view_graph_);
      if (present)
      {
        Affine3 new_transform = Affine3 (getTransformation (source) * getPairwiseTransformation (e, source, target).getMatrix ());
        (*view_graph_)[target].pose_ = Eigen::DualQuaternion<Scalar> (new_transform.matrix ());
        Vector6 p = getPose (target);
        std::cerr << source << " " << target << "\n";
        std::cerr << p(0) << " " << p(1) << " " << p(2) << " " << p(3) << " " << p(4) << " " << p(5) << "\n";
        break;
      }
    }
  }

  std::cerr << getFitnessScore () << "\n";

  // Apply dual quaternion average (DLB/DIB) on the graph pose estimates
  // Iterate for diffusion_iterations_ times
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
  Scalar ste = 0.0;
  typename ViewGraph::vertex_iterator v, v_end;
  typename ViewGraph::out_edge_iterator e, e_end;
  for (boost::tuples::tie (v, v_end) = vertices (*view_graph_); v != v_end; ++v)
  {
    for (boost::tuples::tie (e, e_end) = out_edges (*v, *view_graph_); e != e_end; ++e)
    {
      Edge e_edge = (*e);
      Vertex source_v = (*v);
      Vertex target_v = boost::target ((*e), (*view_graph_));
      Eigen::DualQuaternion<Scalar> q_e = getPairwiseTransformation (e_edge, source_v, target_v);
      Eigen::DualQuaternion<Scalar> q = (q_e.conjugate () * (*view_graph_)[target_v].pose_) * (*view_graph_)[source_v].pose_.conjugate ();

      q.normalize ();

      q = q.log ();
      // DEBUG
  Affine3 new_transform = Affine3 (q.getMatrix ());
  Vector6 p = Vector6::Zero ();
  pcl::getTranslationAndEulerAngles (new_transform, p (0), p (1), p (2), p (3), p (4), p (5));
  std::cerr << "getFitnessScore " << e_edge << " " << p(0) << " " << p(1) << " " << p(2) << " " << p(3) << " " << p(4) << " " << p(5) << "\n";
      // DEBUG

      Scalar edge_ste = q.dot(q)*((*view_graph_)[(*e)].weight_);

      ste += edge_ste;
      //Eigen::DualQuaternion<Scalar> q = ((q_e.conjuate ())*(*view_graph_)[(*e)].target ()*!(*view_graph_)[(*v)].pose_).normalize();
      //Eigen::DualQuaternion<Scalar> q = ((q_e.conjugate ())*(*view_graph_)[(*e)].diffused_transform_*(*view_graph_)[(*e)].diffused_transform_.conjugate ());
      std::cerr << "Vertex 0x10 " << *v << " : Edge " << *e << " : CST " << edge_ste << " : STE " << ste << "\n";

    }
  }
  std::cerr << "\n";
  return std::sqrt(ste);
}

#define PCL_INSTANTIATE_DQ_DIFFUSION(T) template class PCL_EXPORTS pcl::registration::DQDiffusion<T>;

#endif // PCL_REGISTRATION_IMPL_DQ_DIFFUSION_HPP_
