#include <boost/python.hpp>
#include "paths/path.h"
#include "paths/path_reeds_shepp.h"
#include "paths/path_constant_steer.h"

#include "geometry/2dplane.h"
#include "models/kinematic_bicycle_model.h"

using namespace boost::python;
using namespace Planner;

BOOST_PYTHON_MODULE(paths)
{
	enum_<Steer>("Steer")
		.value("LEFT", Steer::Left)
		.value("STRAIGHT", Steer::Straight)
		.value("RIGHT", Steer::Right);

	enum_<Direction>("Direction")
		.value("FORWARD", Direction::Forward)
		.value("BACKWARD", Direction::Backward)
		.value("NO_MOTION", Direction::NoMotion);

	struct PlanarPathWrapper : PlanarPath, wrapper<PlanarPath> {
		virtual Pose2d Interpolate(double) const override {
			return this->get_override("interpolate")();
		}
		virtual void Truncate(double) override {
			this->get_override("truncate")();
		}
		virtual double ComputeCost(double, double, double) const override {
			return this->get_override("compute_cost")();
		}
	};

	class_<PlanarPathWrapper, boost::noncopyable>("PlanarPath")
		.def("get_initial_state", &PlanarPath::GetInitialState, return_value_policy<copy_const_reference>())
		.def("get_final_state", &PlanarPath::GetFinalState, return_value_policy<copy_const_reference>())
		.def("interpolate", pure_virtual(&PlanarPath::Interpolate))
		.def("truncate", pure_virtual(&PlanarPath::Truncate))
		.def("get_length", &PlanarPath::GetLength)
		.def("compute_cost", pure_virtual(&PlanarPath::ComputeCost));

	class_<PathReedsShepp, bases<PlanarPath>>("PathReedsShepp", init<Pose2d, ReedsShepp::PathSegment, double>());
	
	class_<PathConstantSteer, bases<PlanarPath>>("PathConstantSteer", init<KinematicBicycleModel*, Pose2d, double, double, Direction>());
}
