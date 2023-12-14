#ifndef HLPR_JACO_TRAJECTORY_ACTION_SERVER_UTIL_H_
#define HLPR_JACO_TRAJECTORY_ACTION_SERVER_UTIL_H_

#include <cmath>
#include <string>
#include <utility>
#include <vector>
#include <ecl/geometry.hpp>
#include <trajectory_msgs/JointTrajectory.h>

/** Adjust angle to equivalent angle on [-pi, pi]
 *  @param angle the angle to be simplified (-inf, inf)
 *  @return the simplified angle on [-pi, pi]
 */
inline double simplify_angle(double angle)
{
  double previous_rev = floor(angle / (2.0 * M_PI)) * 2.0 * M_PI;
  double next_rev = ceil(angle / (2.0 * M_PI)) * 2.0 * M_PI;
  double current_rev;
  if (fabs(angle - previous_rev) < fabs(angle - next_rev))
    return angle - previous_rev;
  return angle - next_rev;
}

/** Calculates nearest desired angle to the current angle
 *  @param desired desired joint angle [-pi, pi]
 *  @param current current angle (-inf, inf)
 *  @return the closest equivalent angle (-inf, inf)
 *
 *  also known as "smallest delta" or "shortest way around the circle"
 */
inline double nearest_equivalent(double desired, double current)
{
  //calculate current number of revolutions
  double previous_rev = floor(current / (2 * M_PI));
  double next_rev = ceil(current / (2 * M_PI));
  double current_rev;
  if (fabs(current - previous_rev * 2 * M_PI) < fabs(current - next_rev * 2 * M_PI))
    current_rev = previous_rev;
  else
    current_rev = next_rev;

  //determine closest angle
  double lowVal = (current_rev - 1) * 2 * M_PI + desired;
  double medVal = current_rev * 2 * M_PI + desired;
  double highVal = (current_rev + 1) * 2 * M_PI + desired;
  if (fabs(current - lowVal) <= fabs(current - medVal) && fabs(current - lowVal) <= fabs(current - highVal))
    return lowVal;
  if (fabs(current - medVal) <= fabs(current - lowVal) && fabs(current - medVal) <= fabs(current - highVal))
    return medVal;
  return highVal;
}

struct Spline {
    virtual double operator()(const double &x) const = 0;
    inline double at(const double &x) const {
        return (*this)(x);
    }
    virtual double derivative(const double &x) const = 0;
    virtual double dderivative(const double &x) const = 0;

    static std::unique_ptr<Spline> construct(ecl::Array<double> const & time, ecl::Array<double> const & points, const double maxCurvature = 0);
};

template <class SplineClass>
struct SplineImpl : public Spline {
    SplineImpl(SplineClass && spline) :
        Spline(), spline(spline) {};

    virtual inline double operator()(const double &x) const {
        return this->spline(x);
    }
    virtual inline double derivative(const double &x) const {
        return this->spline.derivative(x);
    }
    virtual double dderivative(const double &x) const {
        return this->spline.dderivative(x);
    }
    private:
        SplineClass spline; 
};




std::pair<
    ecl::Array<double>,
    std::vector< std::unique_ptr<Spline> > >
    toSplines(trajectory_msgs::JointTrajectory const & trajectory, std::vector<std::string> const & jointNames, double const maxCurvature);


#endif // HLPR_JACO_TRAJECTORY_ACTION_SERVER_UTIL_H_
