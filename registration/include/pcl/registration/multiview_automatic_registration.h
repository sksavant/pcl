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
 * $Id: multiview_automatic_registration.h$
 *
 *
 */

#ifndef PCL_REGISTATION_MULTIVIEW_AUTOMATIC_REGISTATION_H_
#define PCL_REGISTATION_MULTIVIEW_AUTOMATIC_REGISTATION_H_

#include <pcl/pcl_base.h>
#include <pcl/registration/eigen.h>
#include <pcl/registration/boost.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/boost_graph.h>

namespace Eigen
{
  typedef Eigen::Matrix<float, 6, 1> Vector6f;
  typedef Eigen::Matrix<float, 6, 6> Matrix6f;
}

namespace pcl
{
  namespace registration
  {

    template<typename PointT, typename Scalar = float>
    class MultiviewRegistration
    {
      public:
        typedef boost::shared_ptr<MultiviewRegistration<PointT, Scalar> > Ptr;
        typedef boost::shared_ptr<const MultiviewRegistration<PointT, Scalar> > ConstPtr;

        typedef pcl::PointCloud<PointT> PointCloud;
        typedef typename PointCloud::Ptr PointCloudPtr;
        typedef typename PointCloud::ConstPtr PointCloudConstPtr;

        struct VertexProperties
        {
          // TODO
          PointCloudPtr cloud_;
          Eigen::Vector6f pose_;
          EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        };

        struct EdgeProperties
        {
          // TODO
          Eigen::Matrix6f cinv_;
          Eigen::Vector6f cinvd_;
          EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        };

        typedef boost::adjacency_list<boost::eigen_vecS, boost::eigen_vecS, boost::bidirectionalS, VertexProperties, EdgeProperties, boost::no_property, boost::eigen_listS> ModelGraph;
        typedef boost::shared_ptr<ModelGraph> ModelGraphPtr;
        typedef typename ModelGraph::vertex_descriptor Vertex;
        typedef typename ModelGraph::edge_descriptor Edge;

        MultiviewRegistration ()
          : model_graph_ (new ModelGraph)
        {
        }

      private:
        ModelGraphPtr model_graph_;

    };
  }
}

#ifdef PCL_NO_PRECOMPILE
#include <pcl/registration/impl/multiview_automatic_registration.hpp>
#endif

#endif
