/*********************************************************************
*
* Software License Agreement (GPLv3)
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>.
*
*********************************************************************/

#ifndef __MONOCYCLE_FLATOUTPUT_HPP__
#define __MONOCYCLE_FLATOUTPUT_HPP__
#pragma once

#include "codrha_local_planner/common.h"
#include <Eigen/Dense>

namespace codrha_local_planner {

class MonocycleFlatoutput {

public:

        static const unsigned flatDim = 2;        // 2D curve
        static const unsigned poseDim = 3;        // unicycle configuration dimension
        static const unsigned positionDim = 2;         // position in the 2D plane dimension
        static const unsigned oriDim = 1;         // position in the 2D plane dimension
        static const unsigned veloDim = 2;        // velocity dimension in 2D plane
        static const unsigned accelDim = 2;       // acceleration dimension in 2D plane

        // static const unsigned derivOrdForPosition = 0;
        // static const unsigned derivOrdForPose = 1;
        // static const unsigned derivOrdForVelo = 2;
        // static const unsigned derivOrdForAccel = 3;

        static const unsigned derivOrdForPosition = 0;
        static const unsigned derivOrdForPose = 1;
        static const unsigned derivOrdForVelo = 2;
        static const unsigned derivOrdForAccel = 3;

        // Indexes
        static const unsigned linSpeedIdx = 0;
        static const unsigned angSpeedIdx = 1;
        static const unsigned linAccelIdx = 0;
        static const unsigned angAccelIdx = 1;
        static const unsigned positionIdx = 0;
        static const unsigned oriIdx = 2;

        // template<typename Derived>
        // Eigen::Block<Derived, 2, 2>
        // topLeft2x2Corner(DenseBase<Derived>& m)
        // {
        //   return Eigen::Block<Derived, 2, 2>(m.derived(), 0, 0);
        // }
        // template<typename Derived>
        // const Eigen::Block<const Derived, 2, 2>
        // topLeft2x2Corner(const DenseBase<Derived>& m)
        // {
        //   return Eigen::Block<const Derived, 2, 2>(m.derived(), 0, 0);
        // }

        template <class Derived>
        static Eigen::Block<Derived, positionDim, 1>
        positionInPose(Eigen::DenseBase<Derived>& pose)
        {
                return Eigen::Block<Derived, positionDim, 1>(pose.derived(), positionIdx, 0);
        }

        template <class Derived>
        static const Eigen::Block<const Derived, positionDim, 1>
        positionInPose(const Eigen::DenseBase<Derived>& pose)
        {
                return Eigen::Block<const Derived, positionDim, 1>(pose.derived(), positionIdx, 0);
        }

        template <class Derived>
        static Eigen::Block<Derived, oriDim, 1>
        yawInPose(Eigen::DenseBase<Derived>& pose)
        {
                return Eigen::Block<Derived, oriDim, 1>(pose.derived(), oriIdx, 0);
        }

        template <class Derived>
        static const Eigen::Block<const Derived, oriDim, 1>
        yawInPose(const Eigen::DenseBase<Derived>& pose)
        {
                return Eigen::Block<const Derived, oriDim, 1>(pose.derived(), oriIdx, 0);
        }

        // NOT CONST IS FORBIDEN
        // template <class Derived>
        // static Eigen::Block<Derived, flatDim, 1>
        // poseToFlat(Eigen::DenseBase<Derived>& pose)
        // {
        //         return Eigen::Block<Derived, flatDim, 1>(pose.derived(), 0, 0);
        // }

        template <class Derived>
        static const Eigen::Block<const Derived, flatDim, 1>
        poseToFlat(const Eigen::DenseBase<Derived>& pose)
        {
                return Eigen::Block<const Derived, flatDim, 1>(pose.derived(), 0, 0);
        }

        template <class Derived>
        static const Eigen::Matrix<typename Derived::Scalar, poseDim, 1>
        flatToPose(const Eigen::DenseBase<Derived>& dFlat)
        {
                if (dFlat.cols() <= derivOrdForPose)
                {
                        std::stringstream ss;
                        ss << "MonocycleFlatoutput::flatToPose: number of columns too "
                                "small. Provided " << dFlat.cols() << ", but " <<
                                derivOrdForPose+1 << " is expected" << std::endl;
                        throw common::MyException(ss.str());
                }
                double vx = dFlat(0, 1);

                return (Eigen::Matrix<typename Derived::Scalar, poseDim, 1>() << dFlat.col(0),
                        atan2(dFlat(1, 1), vx))
                       .finished();
        }

        template <class Derived>
        static const Eigen::Block<const Derived, positionDim, 1>
        flatToPosition(const Eigen::DenseBase<Derived>& dFlat)
        {
                if (dFlat.cols() <= derivOrdForPosition)
                {
                        std::stringstream ss;
                        ss << "MonocycleFlatoutput::flatToPosition: number of columns too "
                                "small. Provided " << dFlat.cols() << ", but " <<
                                derivOrdForPosition+1 << " is expected" << std::endl;
                        throw common::MyException(ss.str());
                }
                // return dFlat.block(positionIdx, 0, positionDim, 1);
                // return dFlat.block<positionDim, 1>(positionIdx, 0);
                return Eigen::Block<const Derived, positionDim, 1>(dFlat.derived(), positionIdx, 0);
        }

        template <class Derived>
        static const Eigen::Matrix<typename Derived::Scalar, veloDim, 1>
        flatToVelocity(const Eigen::DenseBase<Derived>& dFlat)
        {
                if (dFlat.cols() <= derivOrdForVelo)
                {
                        std::stringstream ss;
                        ss << "MonocycleFlatoutput::flatToVelocity: number of columns too "
                                "small. Provided " << dFlat.cols() << ", but " <<
                                derivOrdForVelo+1 << " is expected" << std::endl;
                        throw common::MyException(ss.str());
                }
                double vx = dFlat(0, 1);

                typename Derived::Scalar den = vx * vx + dFlat(1, 1) * dFlat(1, 1); //+eps so no /0 +
                // static_cast<typename Derived::Scalar>(std::numeric_limits<
                // float >::epsilon())
                // den = den < static_cast<typename Derived::Scalar>(std::numeric_limits<double>::epsilon())
                den = den < std::numeric_limits<typename Derived::Scalar>::epsilon()
                      ? std::numeric_limits<typename Derived::Scalar>::epsilon()
                      : den;

                return (Eigen::Matrix<typename Derived::Scalar, veloDim, 1>()
                        // << (dFlat.block<flatDim, 1>(0, 1)).norm(), // can use template block, don't know why
                        // << (dFlat.block(0, 1, flatDim, 1)).norm(),
                        << Eigen::Block<const Derived, flatDim, 1>(dFlat.derived(), 0, 1).norm(),
                        // << (dFlat.block(0, 1, flatDim, 1)).norm(),
                        (vx * dFlat(1, 2) - dFlat(1, 1) * dFlat(0, 2)) / den)
                       .finished();
        }

        //             eps = np.finfo(float).eps
        //             pquartereps = eps**(.25)
        //             # Prevent division by zero
        //             dz_norm = LA.norm(zl[:, 1])
        //             # min_den_norm = np.finfo(float).eps**(-4)
        //             # den = dz_norm if dz_norm >= min_den_norm else min_den_norm
        //             den = dz_norm if abs(dz_norm) > pquartereps else pquartereps

        template <class Derived>
        static const Eigen::Matrix<typename Derived::Scalar, accelDim, 1>
        flatToAcceleration(const Eigen::DenseBase<Derived>& dFlat)
        {
                if (dFlat.cols() <= derivOrdForAccel)
                {
                        std::stringstream ss;
                        ss << "MonocycleFlatoutput::flatToAcceleration: number of columns too "
                                "small. Provided " << dFlat.cols() << ", but " <<
                                derivOrdForAccel+1 << " is expected" << std::endl;
                        throw common::MyException(ss.str());
                }
                // double vx = dFlat(0, 1) < 0.0 ? 0.0 : dFlat(0, 1);
                double vx = dFlat(0, 1);

                typedef typename Derived::Scalar Scalar;

                Scalar dflat_norm = Eigen::Block<const Derived, flatDim, 1>(dFlat.derived(), 0, 1).norm();
                // Scalar dflat_norm = (dFlat.block<flatDim, 1>(0, 1)).norm();
                // Scalar flat_norm = (dFlat.block(0, 1, flatDim, 1)).norm();

                Scalar min_den_norm_dv = static_cast<Scalar>(std::numeric_limits<double>::epsilon());
                Scalar min_den_norm_dw =
                        pow(static_cast<Scalar>(std::numeric_limits<double>::epsilon()), 0.25);

                Scalar dflat_norm_den1 =
                        dflat_norm < min_den_norm_dv ? min_den_norm_dv : dflat_norm;
                Scalar dflat_norm_den2 =
                        dflat_norm < min_den_norm_dw ? min_den_norm_dw : dflat_norm;

                Scalar dv = (vx * dFlat(0, 2) + dFlat(1, 1) * dFlat(1, 2)) / dflat_norm_den1;
                Scalar dw =
                        ((dFlat(0, 2) * dFlat(1, 2) + dFlat(1, 3) * vx -
                          (dFlat(1, 2) * dFlat(0, 2) + dFlat(0, 3) * dFlat(1, 1))) *
                         (pow(dflat_norm, 2)) -
                         (vx * dFlat(1, 2) - dFlat(1, 1) * dFlat(0, 2)) * 2 * dflat_norm * dv) /
                        pow(dflat_norm_den2, 4);

                return (Eigen::Matrix<Scalar, accelDim, 1>() << dv, dw).finished();
        }

        typedef Eigen::Matrix<double, flatDim, derivOrdForPosition+1>
        MatForPositionD;
        // MatrixFdPositiond;

        typedef Eigen::Matrix<double, flatDim, derivOrdForPose+1>
        MatForPoseD;
        // MatrixFdPosed;

        typedef Eigen::Matrix<double, flatDim, derivOrdForVelo+1>
        MatForVeloD;
        // MatrixFdVelod;

        typedef Eigen::Matrix<double, flatDim, derivOrdForAccel+1>
        MatForAccelD;
        // MatrixFdAcceld;

        typedef Eigen::Matrix<double, poseDim, 1> PoseVectorD;
        typedef Eigen::Matrix<double, positionDim, 1> PositionVectorD;
        typedef Eigen::Matrix<double, flatDim, 1> FlatVectorD;
        typedef Eigen::Matrix<double, veloDim, 1> VeloVectorD;
        typedef Eigen::Matrix<double, accelDim, 1> AccelVectorD;

};

typedef MonocycleFlatoutput MF;

};
#endif // __MONOCYCLE_FLATOUTPUT_HPP__
