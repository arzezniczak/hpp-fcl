/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011-2014, Willow Garage, Inc.
 *  Copyright (c) 2014-2015, Open Source Robotics Foundation
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
 *   * Neither the name of Open Source Robotics Foundation nor the names of its
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
 */

/** \author Jia Pan */

#include <hpp/fcl/ccd/conservative_advancement.h>
#include <hpp/fcl/ccd/motion.h>
#include <hpp/fcl/collision_node.h>
#include <hpp/fcl/traversal/traversal_node_bvhs.h>
#include <hpp/fcl/traversal/traversal_node_setup.h>
#include <hpp/fcl/traversal/traversal_recurse.h>
#include <hpp/fcl/collision.h>


namespace fcl
{




template<typename BV>
bool conservativeAdvancement(const BVHModel<BV>& o1,
                             const MotionBase* motion1,
                             const BVHModel<BV>& o2,
                             const MotionBase* motion2,
                             const CollisionRequest& request,
                             CollisionResult& result,
                             FCL_REAL& toc)
{
  return true;
}

namespace details
{

template<typename BV, typename ConservativeAdvancementOrientedNode>
bool conservativeAdvancementMeshOriented(const BVHModel<BV>& o1,
                                         const MotionBase* motion1,
                                         const BVHModel<BV>& o2,
                                         const MotionBase* motion2,
                                         const CollisionRequest& request,
                                         CollisionResult& result,
                                         FCL_REAL& toc)
{
  return false;
}


}

template<>
bool conservativeAdvancement(const BVHModel<RSS>& o1,
                             const MotionBase* motion1,
                             const BVHModel<RSS>& o2,
                             const MotionBase* motion2,
                             const CollisionRequest& request,
                             CollisionResult& result,
                             FCL_REAL& toc);


template<>
bool conservativeAdvancement(const BVHModel<OBBRSS>& o1,
                             const MotionBase* motion1,
                             const BVHModel<OBBRSS>& o2,
                             const MotionBase* motion2,
                             const CollisionRequest& request,
                             CollisionResult& result,
                             FCL_REAL& toc);

template<typename S1, typename S2, typename NarrowPhaseSolver>
bool conservativeAdvancement(const S1& o1,
                             const MotionBase* motion1,
                             const S2& o2,
                             const MotionBase* motion2,
                             const NarrowPhaseSolver* solver,
                             const CollisionRequest& request,
                             CollisionResult& result,
                             FCL_REAL& toc)
{
  return false;
}

template<typename BV, typename S, typename NarrowPhaseSolver>
bool conservativeAdvancement(const BVHModel<BV>& o1,
                             const MotionBase* motion1,
                             const S& o2,
                             const MotionBase* motion2,
                             const NarrowPhaseSolver* nsolver,
                             const CollisionRequest& request,
                             CollisionResult& result,
                             FCL_REAL& toc)
{
  return false;
}

namespace details
{

template<typename BV, typename S, typename NarrowPhaseSolver, typename ConservativeAdvancementOrientedNode>
bool conservativeAdvancementMeshShapeOriented(const BVHModel<BV>& o1,
                                              const MotionBase* motion1,
                                              const S& o2,
                                              const MotionBase* motion2,
                                              const NarrowPhaseSolver* nsolver,
                                              const CollisionRequest& request,
                                              CollisionResult& result,
                                              FCL_REAL& toc)
{
  return false;
}

}


template<typename S, typename NarrowPhaseSolver>
bool conservativeAdvancement(const BVHModel<RSS>& o1,
                             const MotionBase* motion1,
                             const S& o2,
                             const MotionBase* motion2,
                             const NarrowPhaseSolver* nsolver,
                             const CollisionRequest& request,
                             CollisionResult& result,
                             FCL_REAL& toc)
{
  return false;
}

template<typename S, typename NarrowPhaseSolver>
bool conservativeAdvancement(const BVHModel<OBBRSS>& o1,
                             const MotionBase* motion1,
                             const S& o2,
                             const MotionBase* motion2,
                             const NarrowPhaseSolver* nsolver,
                             const CollisionRequest& request,
                             CollisionResult& result,
                             FCL_REAL& toc)
{
  return false;
}
                            
template<typename S, typename BV, typename NarrowPhaseSolver>
bool conservativeAdvancement(const S& o1,
                             const MotionBase* motion1,
                             const BVHModel<BV>& o2,
                             const MotionBase* motion2,
                             const NarrowPhaseSolver* nsolver,
                             const CollisionRequest& request,
                             CollisionResult& result,
                             FCL_REAL& toc)
{
  return false;  
}

namespace details
{

template<typename S, typename BV, typename NarrowPhaseSolver, typename ConservativeAdvancementOrientedNode>
bool conservativeAdvancementShapeMeshOriented(const S& o1,
                                              const MotionBase* motion1,
                                              const BVHModel<BV>& o2,
                                              const MotionBase* motion2,
                                              const NarrowPhaseSolver* nsolver,
                                              const CollisionRequest& request,
                                              CollisionResult& result,
                                              FCL_REAL& toc)
{
  return false;
}

}

template<typename S, typename NarrowPhaseSolver>
bool conservativeAdvancement(const S& o1,
                             const MotionBase* motion1,
                             const BVHModel<RSS>& o2,
                             const MotionBase* motion2,
                             const NarrowPhaseSolver* nsolver,
                             const CollisionRequest& request,
                             CollisionResult& result,
                             FCL_REAL& toc)
{
  return false;
}


template<typename S, typename NarrowPhaseSolver>
bool conservativeAdvancement(const S& o1,
                             const MotionBase* motion1,
                             const BVHModel<OBBRSS>& o2,
                             const MotionBase* motion2,
                             const NarrowPhaseSolver* nsolver,
                             const CollisionRequest& request,
                             CollisionResult& result,
                             FCL_REAL& toc)
{
  return false;
}



template<>
bool conservativeAdvancement(const BVHModel<RSS>& o1,
                             const MotionBase* motion1,
                             const BVHModel<RSS>& o2,
                             const MotionBase* motion2,
                             const CollisionRequest& request,
                             CollisionResult& result,
                             FCL_REAL& toc)
{
  return false;
}

template<>
bool conservativeAdvancement(const BVHModel<OBBRSS>& o1,
                             const MotionBase* motion1,
                             const BVHModel<OBBRSS>& o2,
                             const MotionBase* motion2,
                             const CollisionRequest& request,
                             CollisionResult& result,
                             FCL_REAL& toc)
{
  return false;
}


template<typename BV, typename NarrowPhaseSolver>
FCL_REAL BVHConservativeAdvancement(const CollisionGeometry* o1, const MotionBase* motion1, const CollisionGeometry* o2, const MotionBase* motion2, const NarrowPhaseSolver* nsolver, const ContinuousCollisionRequest& request, ContinuousCollisionResult& result)
{
  return 0;
}

template<typename S1, typename S2, typename NarrowPhaseSolver>
FCL_REAL ShapeConservativeAdvancement(const CollisionGeometry* o1, const MotionBase* motion1, const CollisionGeometry* o2, const MotionBase* motion2, const NarrowPhaseSolver* nsolver, const ContinuousCollisionRequest& request, ContinuousCollisionResult& result)
{
  return 0;
}

template<typename S, typename BV, typename NarrowPhaseSolver>
FCL_REAL ShapeBVHConservativeAdvancement(const CollisionGeometry* o1, const MotionBase* motion1, const CollisionGeometry* o2, const MotionBase* motion2, const NarrowPhaseSolver* nsolver, const ContinuousCollisionRequest& request, ContinuousCollisionResult& result)
{
  return 0;
}

template<typename BV, typename S, typename NarrowPhaseSolver>
FCL_REAL BVHShapeConservativeAdvancement(const CollisionGeometry* o1, const MotionBase* motion1, const CollisionGeometry* o2, const MotionBase* motion2, const NarrowPhaseSolver* nsolver, const ContinuousCollisionRequest& request, ContinuousCollisionResult& result)
{
  return 0;
}

template<typename NarrowPhaseSolver>
ConservativeAdvancementFunctionMatrix<NarrowPhaseSolver>::ConservativeAdvancementFunctionMatrix()
{
}
}


