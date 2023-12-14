#include <hlpr_jaco_trajectory_action_server/util.h>


std::unique_ptr<Spline> Spline::construct(ecl::Array<double> const & time, ecl::Array<double> const & points, const double maxCurvature) {
    std::unique_ptr<Spline> spline;
    try {
        spline.reset(new SplineImpl<ecl::SmoothLinearSpline>(ecl::SmoothLinearSpline(time, points, maxCurvature)));
    } catch (ecl::DataException<int> &) {
        spline.reset(new SplineImpl<ecl::CubicSpline>(ecl::CubicSpline::Natural(time, points)));
    }
    return spline;
}


std::pair<
    ecl::Array<double>,
    std::vector< std::unique_ptr<Spline> > >
    toSplines(trajectory_msgs::JointTrajectory const & trajectory, std::vector<std::string> const & jointNames, double const maxCurvature) {

        // initialize sizes of joint-specific trajectories
        const std::size_t numPoints = trajectory.points.size();
        ecl::Array<double> times;
        times.resize(numPoints);

        std::vector<ecl::Array<double> > jointValues;
        std::for_each(jointValues.begin(), jointValues.end(), [numPoints] (ecl::Array<double> & vec) {
            vec.resize(numPoints);
        });

        // look up the corresponding index of each joint name that we want. Throw std::runtime_error if we can't find
        // a requested name inside the trajectory message.
        std::vector<std::size_t> jointIndices(jointNames.size());
        std::transform(jointNames.begin(), jointNames.end(), jointIndices.begin(), [&trajectory] (std::string const & name) {
            auto it = std::find(trajectory.joint_names.begin(), trajectory.joint_names.end(), name);
            if (it == trajectory.joint_names.end()) {
                throw std::runtime_error("Joint [" + name + "] not specified in trajectory");
            }
            return std::distance(trajectory.joint_names.begin(), it);
        });

        // Populate the time and joint values from the trajectory message. (Basically a transpose).
        for (unsigned int j=0; j < numPoints; ++j) {
            times[j] = trajectory.points[j].time_from_start.toSec();
            for (unsigned int k=0; k < jointIndices.size(); ++k) {
                jointValues[k][j] = trajectory.points[j].positions[jointIndices[k]];
            }
        }

        // Spline-ify each joint using Spline::construct
        std::vector<std::unique_ptr<Spline> > splines(jointNames.size());
        std::transform(jointValues.begin(), jointValues.end(), splines.begin(), [&times, maxCurvature] (ecl::Array<double> const & values) {
            return Spline::construct(times, values, maxCurvature);
        });

        // Return a tuple of times and splines
        return { std::move(times), std::move(splines) };
}
