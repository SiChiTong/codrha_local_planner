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

#ifndef __TRAJECTORY_HPP__
#define __TRAJECTORY_HPP__
#pragma once

#include "codrha_local_planner/common.h"
#include <iostream>
#include <unsupported/Eigen/Splines>
#include <boost/current_function.hpp>

template <class T, const unsigned t_dim, const unsigned t_degree>
class Trajectory
{

public:

        typedef Eigen::Spline<T, t_dim, t_degree> TrajectorySpline;

        template <class T2>
        Eigen::Matrix<T, t_dim, Eigen::Dynamic> cArray2CtrlPtsMat(T2 ctrlpts);

        /**
         * Interpolates a given sequence of points considered to be equally
         * spaced wrt the parametric variable (namely the time).
         * @param points         sequence of points
         * @param parVarInterval final value of the parametric variable
         */
        void interpolate(const Eigen::Matrix<T, t_dim, Eigen::Dynamic> &points,
                        const double parVarInterval);

        /**
         * Reconfigure trajectory using new sequence of control points. Other
         * parameters are kept unchanged.
         * @param ctrlpts spline control points
         */
        template <class T2> void update(const T2 *ctrlpts);

        /**
         * See update"("const T2 *ctrlpts")"
         */
        // template <class T2> void update(volatile T2 *ctrlpts);

        /**
         * Reconfigure trajectory using new sequence of control points and new final
         * parametric value. Other parameters are kept unchanged.
         * @param ctrlpts        spline control points
         * @param parVarInterval final value of the parametric variable
         */
        template <class T2> void update(const T2 *ctrlpts, const double parVarInterval);
        // template <class T2>
        // void update(volatile T2 *ctrlpts, volatile double parVarInterval);
        void update(const Eigen::Matrix<T, t_dim, Eigen::Dynamic> &ctrlpts);
        void update(const Eigen::Matrix<T, t_dim, Eigen::Dynamic> &ctrlpts,
                    const double parVarInterval);

        // void updateFromUniform(const double *ctrlpts);
        // void updateFromUniform(const double *ctrlpts, const double parVarInterval);
        // void updateFromUniform(const Eigen::Matrix<double, t_dim, Eigen::Dynamic>&
        // ctrlpts);
        // void updateFromUniform(const Eigen::Matrix<double, t_dim, Eigen::Dynamic>&
        // ctrlpts, const double parVarInterval);

        Eigen::Matrix<T, t_dim, 1> operator()(const double evalTime) const;
        Eigen::Matrix<T, t_dim, Eigen::Dynamic>
        operator()(const double evalTime, const unsigned deriv) const;

        void getParameters(double *params) const;

        Eigen::Matrix<T, t_dim, Eigen::Dynamic> getCtrlPts() const
        {
                return _trajecSpl.ctrls();
        };


        Eigen::Matrix<double, 1, Eigen::Dynamic> getKnots() const
        {
                return _trajecSpl.knots();
        };

        double getParVarInterval() const
        {
                return  _trajecSpl.knots().rightCols(1) (0,0);
        }

        // int getSpan(const double t) const;

        int nCtrlPts() const
        {
                return _nCtrlPts;
        };

        int nParam() const
        {
                return _nCtrlPts * t_dim;
        };

        int splDegree() const
        {
                return t_degree;
        };

        // template <class T, const unsigned t_dim, const unsigned t_degree>
        Trajectory();

        Trajectory(unsigned nCtrlPts);

        Trajectory(const Eigen::Matrix<T, t_dim, Eigen::Dynamic> &ctrlPts,
                   const double parVarInterval);

        ~Trajectory(){};

private:

        unsigned _nCtrlPts;
        unsigned _nIntervNonNull;
        TrajectorySpline _trajecSpl;
        typename TrajectorySpline::KnotVectorType _knots;
        double _getParVarInterval() const
        {
                return _trajecSpl.knots().rightCols(1)(0, 0);
        };
        Eigen::Array<T, 1, Eigen::Dynamic>
        _genKnots(const double initT, const double finalT, const bool nonUniform,
                  const unsigned nIntervNonNull) const;
};


template <class T, const unsigned t_dim, const unsigned t_degree>
Trajectory<T, t_dim, t_degree>::Trajectory(unsigned nCtrlPts)
        : _nCtrlPts(nCtrlPts)
{
        _nIntervNonNull = _nCtrlPts - t_degree;
        Eigen::Matrix<T, t_dim, Eigen::Dynamic> ctrlPts(t_dim, _nCtrlPts);
        ctrlPts = Eigen::Matrix<T, t_dim, Eigen::Dynamic>::Zero(t_dim, _nCtrlPts);
        _knots = _genKnots(0.0, 0.0, true, _nIntervNonNull);
        // new (&_trajecSpl)Trajectory<T, t_dim, t_degree>::TrajectorySpline(_knots,
        // ctrlPts);
        _trajecSpl.~TrajectorySpline();
        new (&_trajecSpl)
        Trajectory<T, t_dim, t_degree>::TrajectorySpline(_knots, ctrlPts);
        // _trajecSpl = Trajectory<T, t_dim, t_degree>::TrajectorySpline(_knots,
        // ctrlPts);
}

template <class T, const unsigned t_dim, const unsigned t_degree>
Trajectory<T, t_dim, t_degree>::Trajectory()
        : Trajectory<T, t_dim, t_degree>::Trajectory(8){}

template <class T, const unsigned t_dim, const unsigned t_degree>
Trajectory<T, t_dim, t_degree>::Trajectory(
        const Eigen::Matrix<T, t_dim, Eigen::Dynamic> &ctrlPts,
        const double parVarInterval)
{
        _nCtrlPts = ctrlPts.cols();
        _nIntervNonNull = _nCtrlPts - t_degree;
        _knots = _genKnots(0.0, parVarInterval, true, _nIntervNonNull);
        // new (&_trajecSpl)TrajectorySpline(_knots, ctrlPts);
        _trajecSpl.~TrajectorySpline();
        new (&_trajecSpl)
        Trajectory<T, t_dim, t_degree>::TrajectorySpline(_knots, ctrlPts);
        // _trajecSpl = Trajectory<T, t_dim, t_degree>::TrajectorySpline(_knots,
        // ctrlPts);
}

// Trajectory<T, t_dim, t_degree>::Trajectory(Trajectory<T, t_dim,
// t_degree>::Trajectory traj):
// {
//         // TODO
// };

template <class T, const unsigned t_dim, const unsigned t_degree>
template <class T2>
Eigen::Matrix<T, t_dim, Eigen::Dynamic>
Trajectory<T, t_dim, t_degree>::cArray2CtrlPtsMat(T2 ctrlpts)
{
        Eigen::Matrix<T, t_dim, Eigen::Dynamic> ctrlPts(t_dim, _nCtrlPts);

        // Feed ctrlPts rows with values from the primal variables x
        for (auto i = 0; i < int(_nCtrlPts * t_dim); ++i)
        {
                ctrlPts(i % t_dim, i / t_dim) = ctrlpts[i];
        }
        return ctrlPts;
}

// template <class T, const unsigned t_dim, const unsigned t_degree>
// void Trajectory<T, t_dim, t_degree>::interpolate(
//         const Eigen::Matrix<T, t_dim, Eigen::Dynamic> &points,
//         const double parVarInterval)
// {
//         Eigen::RowVectorXd parVariable =
//                 Eigen::RowVectorXd::LinSpaced(_nCtrlPts, 0.0, parVarInterval);
//         typename TrajectorySpline::KnotVectorType chordLengths;
//         Eigen::ChordLengths(parVariable, chordLengths);
//         _trajecSpl = Eigen::SplineFitting<TrajectorySpline>::Interpolate(
//                 points, t_degree, chordLengths);
//
//         typename TrajectorySpline::ControlPointVectorType ctrlPts(t_dim, _nCtrlPts);
//         ctrlPts = _trajecSpl.ctrls();
//
//         _knots = _genKnots(0.0, parVarInterval, true, _nIntervNonNull);
//         _trajecSpl.~TrajectorySpline();
//         new (&_trajecSpl)
//         Trajectory<T, t_dim, t_degree>::TrajectorySpline(_knots, ctrlPts);
//         // _trajecSpl = Trajectory<T, t_dim, t_degree>::TrajectorySpline(_knots,
//         // ctrlPts);
// }

template <class T, const unsigned t_dim, const unsigned t_degree>
void Trajectory<T, t_dim, t_degree>::interpolate(
        const Eigen::Matrix<T, t_dim, Eigen::Dynamic> &points,
        const double parVarInterval)
{
        _nCtrlPts = points.cols();
        _nIntervNonNull = _nCtrlPts - t_degree;

        Eigen::RowVectorXd parVariable =
                Eigen::RowVectorXd::LinSpaced(_nCtrlPts, 0.0, parVarInterval);
        typename TrajectorySpline::KnotVectorType chordLengths;
        Eigen::ChordLengths(parVariable, chordLengths);

        _trajecSpl = Eigen::SplineFitting<TrajectorySpline>::Interpolate(
                points, t_degree, chordLengths);

        typename TrajectorySpline::ControlPointVectorType ctrlPts(t_dim, _nCtrlPts);
        ctrlPts = _trajecSpl.ctrls();

        _knots = _genKnots(0.0, parVarInterval, true, _nIntervNonNull);
        _trajecSpl.~TrajectorySpline();
        new (&_trajecSpl)
        Trajectory<T, t_dim, t_degree>::TrajectorySpline(_knots, ctrlPts);
}

template <class T, const unsigned t_dim, const unsigned t_degree>
template <class T2>
void Trajectory<T, t_dim, t_degree>::update(const T2 *ctrlpts)
{
        _knots = _trajecSpl.knots();
        _trajecSpl.~TrajectorySpline();
        new (&_trajecSpl)TrajectorySpline(_knots, cArray2CtrlPtsMat(ctrlpts));
}

// template <class T, const unsigned t_dim, const unsigned t_degree>
// template <class T2>
// void Trajectory<T, t_dim, t_degree>::update(volatile T2 *ctrlpts)
// {
//         _knots = _trajecSpl.knots();
//         _trajecSpl.~TrajectorySpline();
//         new (&_trajecSpl)TrajectorySpline(_knots, cArray2CtrlPtsMat(ctrlpts));
// }

template <class T, const unsigned t_dim, const unsigned t_degree>
void Trajectory<T, t_dim, t_degree>::update(
        const Eigen::Matrix<T, t_dim, Eigen::Dynamic> &ctrlpts)
{
        _knots = _trajecSpl.knots();
        _trajecSpl.~TrajectorySpline();
        new (&_trajecSpl)TrajectorySpline(_knots, ctrlpts);
}

template <class T, const unsigned t_dim, const unsigned t_degree>
template <class T2>
void Trajectory<T, t_dim, t_degree>::update(const T2 *ctrlpts,
                                              const double parVarInterval)
{
        _knots = _genKnots(0.0, parVarInterval, true, _nIntervNonNull);
        _trajecSpl.~TrajectorySpline();
        new (&_trajecSpl)TrajectorySpline(_knots, cArray2CtrlPtsMat(ctrlpts));
}

// template <class T, const unsigned t_dim, const unsigned t_degree>
// template <class T2>
// void Trajectory<T, t_dim, t_degree>::update(volatile T2 *ctrlpts,
//                                               volatile double parVarInterval)
// {
//         _knots = _genKnots(0.0, parVarInterval, true, _nIntervNonNull);
//         _trajecSpl.~TrajectorySpline();
//         new (&_trajecSpl)TrajectorySpline(_knots, cArray2CtrlPtsMat(ctrlpts));
// }

template <class T, const unsigned t_dim, const unsigned t_degree>
void Trajectory<T, t_dim, t_degree>::update(
        const Eigen::Matrix<T, t_dim, Eigen::Dynamic> &ctrlpts,
        const double parVarInterval)
{
        _knots = _genKnots(0.0, parVarInterval, true, _nIntervNonNull);
        _trajecSpl.~TrajectorySpline();
        new (&_trajecSpl)TrajectorySpline(_knots, ctrlpts);
}

// void Trajectory<T, t_dim, t_degree>::updateFromUniform(const double
// *ctrlpts)
// {
//         double parVarInterval = _trajecSpl.knots().tail(1) (0,0);
//
//         TrajectorySpline auxSpline(_genKnots(0.0, parVarInterval, false,
//         _nIntervNonNull), cArray2CtrlPtsMat(ctrlpts));
//
//         Eigen::RowVectorXd parVariable =
//         Eigen::RowVectorXd::LinSpaced(_nCtrlPts, 0.0, parVarInterval);
//
//         Eigen::Matrix<double, t_dim, Eigen::Dynamic> points(dim, _nCtrlPts);
//
//         for (int i=0; i <int(_nCtrlPts); ++i)
//         {
//                 points.col(i) = auxSpline(parVariable(i));
//         }
//
//         interpolate(points, parVarInterval);
// }
//
//
// void Trajectory<T, t_dim, t_degree>::updateFromUniform(const double
// *ctrlpts, const double parVarInterval)
// {
//         TrajectorySpline auxSpline(_genKnots(0.0, parVarInterval, false,
//         _nIntervNonNull), cArray2CtrlPtsMat(ctrlpts));
//
//         Eigen::RowVectorXd parVariable =
//         Eigen::RowVectorXd::LinSpaced(_nCtrlPts, 0.0, parVarInterval);
//
//         Eigen::Matrix<double, t_dim, Eigen::Dynamic> points(dim, _nCtrlPts);
//
//         for (int i=0; i <int(_nCtrlPts); ++i)
//         {
//                 points.col(i) = auxSpline(parVariable(i));
//         }
//
//         interpolate(points, parVarInterval);
// }
//
//
// void Trajectory<T, t_dim, t_degree>::updateFromUniform(const
// Eigen::Matrix<double, t_dim, Eigen::Dynamic>& ctrlpts)
// {
//         double parVarInterval = _trajecSpl.knots().tail(1) (0,0);
//
//         TrajectorySpline auxSpline(_genKnots(0.0, parVarInterval, false,
//         _nIntervNonNull), ctrlpts);
//
//         Eigen::RowVectorXd parVariable =
//         Eigen::RowVectorXd::LinSpaced(_nCtrlPts, 0.0, parVarInterval);
//
//         Eigen::Matrix<double, t_dim, Eigen::Dynamic> points(dim, _nCtrlPts);
//
//         for (int i=0; i <int(_nCtrlPts); ++i)
//         {
//                 points.col(i) = auxSpline(parVariable(i));
//         }
//
//         interpolate(points, parVarInterval);
// }
//
//
// void Trajectory<T, t_dim, t_degree>::updateFromUniform( const
// Eigen::Matrix<double, t_dim, Eigen::Dynamic>& ctrlpts, const double
// parVarInterval)
// {
//         TrajectorySpline auxSpline(_genKnots(0.0, parVarInterval, false,
//         _nIntervNonNull), ctrlpts);
//
//         Eigen::RowVectorXd parVariable =
//         Eigen::RowVectorXd::LinSpaced(_nCtrlPts, 0.0, parVarInterval);
//
//         Eigen::Matrix<double, t_dim, Eigen::Dynamic> points(dim, _nCtrlPts);
//
//         for (int i=0; i <int(_nCtrlPts); ++i)
//         {
//                 points.col(i) = auxSpline(parVariable(i));
//         }
//
//         interpolate(points, parVarInterval);
// }

template <class T, const unsigned t_dim, const unsigned t_degree>
Eigen::Matrix<T, t_dim, 1> Trajectory<T, t_dim, t_degree>::
operator()(const double evalTime) const
{
        // if (evalTime > _getParVarInterval())
        // {
        //         std::stringstream ss;
        //         ss << "In " << BOOST_CURRENT_FUNCTION << ": invalid evaluation "
        //         // ss << "In " << "" << ": invalid evaluation "
        //                 "time " << evalTime << " should be lower than " << _getParVarInterval();
        //         throw common::MyException(ss.str());
        // }
        double et = 0.0;
        if (evalTime > _getParVarInterval())
        {
                et = _getParVarInterval();
        }
        else if (evalTime < 0.0)
        {
                et = 0.0;
        }
        return _trajecSpl(et);
}

template <class T, const unsigned t_dim, const unsigned t_degree>
Eigen::Matrix<T, t_dim, Eigen::Dynamic> Trajectory<T, t_dim, t_degree>::
operator()(const double evalTime, const unsigned deriv) const
{
        // if (evalTime > _getParVarInterval())
        // {
        //         std::stringstream ss;
        //         ss << "In " << BOOST_CURRENT_FUNCTION << ": invalid evaluation "
        //                 "time " << evalTime << " should be lower than " << _getParVarInterval();
        //         throw common::MyException(ss.str());
        // }
        if (deriv <= 0)
                return _trajecSpl(evalTime);
        else
                return _trajecSpl.derivatives(evalTime, deriv);
}

template <class T, const unsigned t_dim, const unsigned t_degree>
void Trajectory<T, t_dim, t_degree>::getParameters(double *params) const
{
        if (params == NULL)
        {
                std::stringstream ss;
                ss << "Trajectory<T, t_dim, t_degree>::getParameters: invalid C-like "
                        "array (points to NULL). Was memory allocated?";
                throw common::MyException(ss.str());
        }
        for (auto i = 0; i < int(_nCtrlPts * t_dim); ++i)
        {
                params[i] = _trajecSpl.ctrls()(i % t_dim, i / t_dim);
        }
}

template <class T, const unsigned t_dim, const unsigned t_degree>
Eigen::Array<T, 1, Eigen::Dynamic> Trajectory<T, t_dim, t_degree>::_genKnots(
        const double initT, const double finalT, const bool nonUniform,
        const unsigned nIntervNonNull) const
{
        if (nIntervNonNull < 2)
        {
                std::stringstream ss;
                ss << "Trajectory<T, t_dim, t_degree>::_genKnots: number of non null "
                        "intervals is too low [ "
                   << nIntervNonNull << " ].";
                throw common::MyException(ss.str());
        }

        double d = (finalT - initT) / (4 + (nIntervNonNull - 2));
        // d is the nonuniform interval base value (spacing produce intervals like
        // this: 2*d, d,... , d, 2*d)

        Eigen::Array<double, 1, Eigen::Dynamic> knots(t_degree * 2 +
                                                      nIntervNonNull + 1);

        // first and last knots
        knots.head(t_degree) =
                Eigen::Array<double, 1, Eigen::Dynamic>::Constant(t_degree, initT);
        knots.tail(t_degree) =
                Eigen::Array<double, 1, Eigen::Dynamic>::Constant(t_degree, finalT);

        // intermediaries knots
        if (nonUniform)
        {
                knots(t_degree) = initT;
                knots(t_degree + 1) = initT + 2 * d;

                unsigned i = 0;
                for (i = 0; i < nIntervNonNull - 2; ++i)
                {
                        knots(t_degree + i + 2) = knots(t_degree + i + 1) + d;
                }

                knots(t_degree + 2 + i) = finalT; // = knots(t_degree+2+i-1) + 2*d
        } else                          // uniform
        {
                knots.segment(t_degree, nIntervNonNull + 1) =
                        Eigen::Array<double, 1, Eigen::Dynamic>::LinSpaced(nIntervNonNull + 1,
                                                                           initT, finalT);
        }
        return knots;
}

// int Trajectory<T, t_dim, t_degree>::getSpan(const double t) const
// {
//         int ret = 0;
//         while (ret <= int(_nCtrlPts) - 1 &&  t>= _knots(0,ret))
//                 ret++;
//         return ret - 1 - (_nIntervNonNull-1);
// }

#endif // __TRAJECTORY_HPP__

// cmake:sourcegroup=MotionPlanner
