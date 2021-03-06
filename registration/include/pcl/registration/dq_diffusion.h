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

#ifndef PCL_REGISTRATION_DQ_DIFFUSUION_H_
#define PCL_REGISTRATION_DQ_DIFFUSUION_H_

#include <pcl/pcl_base.h> //Base?
#include <pcl/registration/eigen.h> //Eigen includes
#include <pcl/registration/eigen_dual_quaternion.h>
#include <pcl/registration/boost.h> //Boost includes
#include <pcl/common/transforms.h> // For transforming clouds
#include <pcl/registration/boost_graph.h> //graph includes and definitions


namespace pcl
{
  namespace registration
  {

    /**
     * \brief Multiview Registration via Graph diffusion of Dual Quaternions
     * \details
     *
     * \author Savant Krishna
     * \ingroup registration
     */
    template <typename PointT, typename Scalar = float>
    class DQDiffusion
    {
      public:
        typedef boost::shared_ptr<DQDiffusion<PointT, Scalar> > Ptr;
        typedef boost::shared_ptr<const DQDiffusion<PointT, Scalar> > ConstPtr;

        typedef pcl::PointCloud<PointT> PointCloud;
        typedef typename PointCloud::Ptr PointCloudPtr;
        typedef typename PointCloud::ConstPtr PointCloudConstPtr;

        typedef Eigen::Matrix<Scalar, 4, 4> Matrix4;
        typedef Eigen::Matrix<Scalar, 6, 1> Vector6;
        typedef Eigen::Transform<Scalar, 3, Eigen::Affine> Affine3;

        struct VertexProperties
        {
          PointCloudPtr cloud_;
          //Vector6 pose_; // initial estimate input
          Eigen::DualQuaternion<Scalar> pose_;
          Scalar weight_sum_;
          EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        };

        struct EdgeProperties
        {
          //Matrix4 transformation_; // TODO change to DQ
          Eigen::DualQuaternion<Scalar> pairwise_transform_;
          //Eigen::DualQuaternion<Scalar> diffused_transform_;
          Scalar weight_;
          EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        };

        typedef boost::adjacency_list<boost::eigen_vecS, boost::eigen_vecS, boost::undirectedS, VertexProperties, EdgeProperties, boost::no_property, boost::eigen_listS> ViewGraph;

        typedef boost::shared_ptr<ViewGraph> ViewGraphPtr;
        typedef typename ViewGraph::vertex_descriptor Vertex;
        typedef typename ViewGraph::edge_descriptor Edge;

        DQDiffusion ()
          : view_graph_ (new ViewGraph)
          , linear_approximation_ (false)
          , diffusion_iterations_(1000)
          , average_iterations_(4)
        {
        }

        inline ViewGraphPtr
        getViewGraph () const;

        typename ViewGraph::vertices_size_type
        getNumVertices () const;

        void
        setDiffusionIterations (int diffusion_iterations);

        void
        setAverageIterations (int average_iterations);

        inline int
        getDiffusionIterations () const;

        Vertex
        addPointCloud (const PointCloudPtr &cloud, const Vector6 &pose = Vector6::Zero ());

        inline void
        setPointCloud (const Vertex &vertex, const PointCloudPtr &cloud);

        inline PointCloudPtr
        getPointCloud (const Vertex &vertex) const;

        inline void
        addPairwiseTransformation (const Vertex &from_vertex, const Vertex &to_vertex, Matrix4 &transformation, Scalar weight = 1.0);

        inline void
        setLinearApproximation (bool linear_approximation);

        inline void
        setPose (const Vertex &vertex, const Vector6 &pose); //TODO dq pose input?

        inline Vector6
        getPose (const Vertex &Vertex) const;

        inline Matrix4
        getTransformation (const Vertex &vertex) const;

        void
        compute ();

        PointCloudPtr
        getTransformedCloud (const Vertex &vertex) const;

        PointCloudPtr
        getConcatenatedCloud () const;

        Scalar
        getFitnessScore ();

      protected:
        void
        linearDiffusion ();

        void
        manifoldDiffusion ();

        inline Eigen::DualQuaternion<Scalar>
        getPairwiseTransformation (Edge &e, Vertex& source, Vertex& target);

      private:
        /** \brief The internal view graph structure. */
        ViewGraphPtr view_graph_;

        bool linear_approximation_;
        int diffusion_iterations_;
        int average_iterations_;
    };
  }
}

#ifdef PCL_NO_PRECOMPILE
#include <pcl/registration/impl/dq_diffusion.hpp>
#endif

#endif // PCL_REGISTRATION_DQ_DIFFUSUION_H_
