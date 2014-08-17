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

#include <boost/lexical_cast.hpp>

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

template<typename PointT, typename Scalar> void
pcl::registration::DQDiffusion<PointT, Scalar>::setAverageIterations (int average_iterations)
{
  average_iterations_ = average_iterations;
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

// DEBUG
template<typename Scalar>
void printDQ (Eigen::DualQuaternion<Scalar> q, std::string id = "")
{
  Eigen::Quaternion<Scalar> t = q.real ();
  std::cerr << id << ": " << t.w () << " " << t.x () << " " << t.y () << " " << t.z () << " ";
  t = q.dual ();
  std::cerr << t.w () << " " << t.x () << " " << t.y () << " " << t.z () << "\n";
}

template<typename PointT, typename Scalar> void
pcl::registration::DQDiffusion<PointT, Scalar>::linearDiffusion ()
{
  // TODO remove
  //diffusion_iterations_ = 1;
  for (int rep = 0; rep != diffusion_iterations_; ++rep)
  {
    std::cerr << rep << " " << getFitnessScore () << "\n";
    typename ViewGraph::vertex_iterator v, v_end;
    typename ViewGraph::out_edge_iterator e, e_end;
    std::vector<Eigen::DualQuaternion<Scalar> > q2 (getNumVertices (), Eigen::DualQuaternion<Scalar> (0.0)); //vector of Q of each pose?

    /*
    // Debug
    for (int i=0; i<getNumVertices (); ++i)
    {
      printDQ<Scalar> (q2[i], "");
    }
    // Debug
    */

    for (boost::tuples::tie (v, v_end) = vertices (*view_graph_); v != v_end; ++v)
    {
      Vertex source_v = (*v);
      for (boost::tuples::tie (e, e_end) = out_edges (*v, *view_graph_); e != e_end; ++e)
      {
        Edge e_edge = (*e);
        Vertex target_v = boost::target ((*e), (*view_graph_));
        Eigen::DualQuaternion<Scalar> q_e = getPairwiseTransformation (e_edge, source_v, target_v);

        Eigen::DualQuaternion<Scalar> qi = q_e.conjugate () * (*view_graph_)[target_v].pose_;
        const Scalar w = copysign(1.0, (*view_graph_)[source_v].pose_.real ().dot (qi.real ())) * (*view_graph_)[e_edge].weight_ / (*view_graph_)[source_v].weight_sum_;

        q2[source_v] = q2[source_v] + (qi * w);
        //q2 = q2 + qi * w;
        //std::cerr << "Vertex " << *v << " : Edge " << *e << "\n";
      }
      q2[source_v].normalize ();
    }
    for (boost::tuples::tie (v, v_end) = vertices (*view_graph_); v != v_end; ++v)
    {
      Vertex source_v = (*v);
      (*view_graph_)[source_v].pose_ = q2[source_v];

      /*
      //Debug
  Eigen::DualQuaternion<Scalar> q = (*view_graph_)[source_v].pose_;
  Affine3 new_transform = Affine3 (q.getMatrix ());
  Eigen::Quaternion<Scalar> t = q.real ();
  Vector6 p = Vector6::Zero ();
  pcl::getTranslationAndEulerAngles (new_transform, p (0), p (1), p (2), p (3), p (4), p (5));
  //std::cerr << "linearDiffusion: q " << source_v << " " << p(0) << " " << p(1) << " " << p(2) << " " << p(3) << " " << p(4) << " " << p(5) << "\n";
  std::cerr << "linearDiffusion: q.real() " << source_v << ": " << t.w () << " " << t.x () << " " << t.y () << " " << t.z () << "\n";
  t = q.dual ();
  std::cerr << "linearDiffusion: q.dual() " << source_v << ": " << t.w () << " " << t.x () << " " << t.y () << " " << t.z () << "\n";
      // Debug
      */

    }
  }
  Vertex start = 0;
  Eigen::DualQuaternion<Scalar> st = (*view_graph_)[start].pose_.conjugate ();
  typename ViewGraph::vertex_iterator v, v_end;
  for (boost::tuples::tie (v, v_end) = vertices (*view_graph_); v != v_end; ++v)
  {
    Vertex v_iter = (*v);
    (*view_graph_)[v_iter].pose_ = ((*view_graph_)[v_iter].pose_ * st);
    (*view_graph_)[v_iter].pose_.normalize ();
  }

}

template<typename PointT, typename Scalar> void
pcl::registration::DQDiffusion<PointT, Scalar>::manifoldDiffusion ()
{
  //diffusion_iterations_ = 1;
  for (int rep = 0; rep != diffusion_iterations_; ++rep)
  {
    std::cerr << rep << " " << getFitnessScore () << "\n";
    typename ViewGraph::vertex_iterator v, v_end;
    typename ViewGraph::out_edge_iterator e, e_end;
    std::vector<Eigen::DualQuaternion<Scalar> > q2 (getNumVertices (), Eigen::DualQuaternion<Scalar> (0.0)); //vector of Q of each pose?

    for (boost::tuples::tie (v, v_end) = vertices (*view_graph_); v != v_end; ++v)
    {
      Vertex source_v = (*v);
      for (boost::tuples::tie (e, e_end) = out_edges (*v, *view_graph_); e != e_end; ++e)
      {
        Edge e_edge = (*e);
        Vertex target_v = boost::target ((*e), (*view_graph_));
        Eigen::DualQuaternion<Scalar> q_e = getPairwiseTransformation (e_edge, source_v, target_v);

        Eigen::DualQuaternion<Scalar> qi = q_e.conjugate () * (*view_graph_)[target_v].pose_;
        const Scalar w = boost::math::sign((*view_graph_)[source_v].pose_.real ().dot (qi.real ())) * (*view_graph_)[e_edge].weight_ / (*view_graph_)[source_v].weight_sum_;

        q2[source_v] = q2[source_v] + (qi * w);
        //std::cerr << "Vertex " << *v << " : Edge " << *e << "\n";
      }
      q2[source_v].normalize ();
      //printDQ<Scalar> (q2[source_v], boost::lexical_cast<std::string>(source_v)+" : Bexp");

      ///*
      for(int avg_rep = 0; avg_rep != average_iterations_; ++avg_rep)
      {
        Eigen::DualQuaternion<Scalar> log_mean = Eigen::DualQuaternion<Scalar> (0.0);
        for (boost::tuples::tie (e, e_end) = out_edges (*v, *view_graph_); e != e_end; ++e)
        {
          Edge e_edge = (*e);
          Vertex target_v = boost::target ((*e), (*view_graph_));
          Eigen::DualQuaternion<Scalar> q_e = getPairwiseTransformation (e_edge, source_v, target_v);

          Eigen::DualQuaternion<Scalar> qi = q_e.conjugate () * (*view_graph_)[target_v].pose_ * q2[source_v].conjugate ();
          qi.normalize ();
          //std::cerr << "qirw :" <<  qi.real ().w () << "\n";
          qi = qi * boost::math::sign(qi.real ().w ());
          qi = qi.log ();
          qi = qi * (*view_graph_)[e_edge].weight_;
          log_mean = log_mean + qi;
        }
        log_mean = log_mean * (1.0/ (*view_graph_)[source_v].weight_sum_);
        q2[source_v] = ((log_mean.exp ()) * q2[source_v]);
        q2[source_v].normalize ();
        //printDQ<Scalar> (q2[source_v], boost::lexical_cast<std::string>(source_v)+" : Aexp");
      }
      //*/
    }

    for (boost::tuples::tie (v, v_end) = vertices (*view_graph_); v != v_end; ++v)
    {
      Vertex source_v = (*v);
      (*view_graph_)[source_v].pose_ = q2[source_v];
    }
  }

  Vertex start = 0;
  Eigen::DualQuaternion<Scalar> st = (*view_graph_)[start].pose_.conjugate ();
  typename ViewGraph::vertex_iterator v, v_end;
  for (boost::tuples::tie (v, v_end) = vertices (*view_graph_); v != v_end; ++v)
  {
    Vertex v_iter = (*v);
    (*view_graph_)[v_iter].pose_ = ((*view_graph_)[v_iter].pose_ * st);
    (*view_graph_)[v_iter].pose_.normalize ();
  }

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

  /* TODO : use bfs
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
  */

  int n = getNumVertices ();
  std::vector<int> view_stack (1,0);
  std::vector<char> view_visited(n,0);

  view_visited[0]=1;
  //std::vector<dual_quaternion> Q(n);
  int stack_pos=0;
  while (stack_pos != (int)view_stack.size())
  {
    Vertex cur_view = view_stack[stack_pos];
    typename ViewGraph::out_edge_iterator e, e_end;
    for (boost::tuples::tie (e, e_end) = out_edges (cur_view, *view_graph_); e != e_end; ++e)
    {
      Edge e_edge = (*e);
      Vertex target_v = boost::target (e_edge, *view_graph_);
      if (!view_visited[target_v])
      {
        view_stack.push_back(target_v);
        view_visited[target_v]=1;
        //std::cerr << cur_view << " -> " << target_v << " <=> " << std::endl;// << Q[cur_view] << " * " << iter->second << " = " << iter->second * Q[cur_view] << std::endl;
        //Q[iter->first] = ( iter->second * Q[cur_view] ).normalize();
        (*view_graph_)[target_v].pose_ = getPairwiseTransformation (e_edge, cur_view, target_v) * (*view_graph_)[cur_view].pose_;
        (*view_graph_)[target_v].pose_.normalize ();
      }
    }
    ++stack_pos;
  }
  if (stack_pos!=n)
  {
    std::cerr << "view graph not connected\n";
    throw std::runtime_error("view graph not connected in void pcl::registration::DQDiffusion::compute()");
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

      /*// DEBUG
  Affine3 new_transform = Affine3 (q_e.getMatrix ());
  Eigen::Quaternion<Scalar> t = q_e.real ();
  Vector6 p = Vector6::Zero ();
  pcl::getTranslationAndEulerAngles (new_transform, p (0), p (1), p (2), p (3), p (4), p (5));
  std::cerr << "getFitnessScore: q_e " << e_edge << " " << p(0) << " " << p(1) << " " << p(2) << " " << p(3) << " " << p(4) << " " << p(5) << "\n";
  std::cerr << "getFitnessScore: q_e.real() " << e_edge << " " << t.w () << " " << t.x () << " " << t.y () << " " << t.z () << "\n";
  t = q_e.dual ();
  std::cerr << "getFitnessScore: q_e.dual() " << e_edge << " " << t.w () << " " << t.x () << " " << t.y () << " " << t.z () << "\n";

  new_transform = Affine3 (q.getMatrix ());
  pcl::getTranslationAndEulerAngles (new_transform, p (0), p (1), p (2), p (3), p (4), p (5));
  std::cerr << "getFitnessScore: qbl " << e_edge << " " << p(0) << " " << p(1) << " " << p(2) << " " << p(3) << " " << p(4) << " " << p(5) << "\n";
  t = q.real ();
  std::cerr << "getFitnessScore: qbl.real() " << e_edge << " " << t.w () << " " << t.x () << " " << t.y () << " " << t.z () << "\n";
  t = q.dual ();
  std::cerr << "getFitnessScore: qbl.dual() " << e_edge << " " << t.w () << " " << t.x () << " " << t.y () << " " << t.z () << "\n";
      // DEBUG
      // */

      q = q.log ();

      /*
      // DEBUG
  new_transform = Affine3 (q.getMatrix ());
  pcl::getTranslationAndEulerAngles (new_transform, p (0), p (1), p (2), p (3), p (4), p (5));
  std::cerr << "getFitnessScore: qal " << e_edge << " " << p(0) << " " << p(1) << " " << p(2) << " " << p(3) << " " << p(4) << " " << p(5) << "\n";
  t = q.real ();
  std::cerr << "getFitnessScore: qal.real() " << e_edge << " " << t.w () << " " << t.x () << " " << t.y () << " " << t.z () << "\n";
  t = q.dual ();
  std::cerr << "getFitnessScore: qal.dual() " << e_edge << " " << t.w () << " " << t.x () << " " << t.y () << " " << t.z () << "\n";
      // DEBUG
      */

      Scalar edge_ste = q.dot(q)*((*view_graph_)[(*e)].weight_);

      ste += edge_ste;

      //std::cerr << "Vertex 0x10 " << *v << " : Edge " << *e << " : CST " << edge_ste << " : STE " << ste << "\n";

    }
  }
  return std::sqrt(ste);
}

#define PCL_INSTANTIATE_DQ_DIFFUSION(T) template class PCL_EXPORTS pcl::registration::DQDiffusion<T>;

#endif // PCL_REGISTRATION_IMPL_DQ_DIFFUSION_HPP_
