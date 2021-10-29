#include <boost/python.hpp>

#include "core/base.h"
#include "algo/path_planner.h"
#include "algo/hybrid_a_star.h"

#include "models/kinematic_bicycle_model.h"

#include "paths/path.h"
#include "paths/path_reeds_shepp.h"
#include "paths/path_constant_steer.h"

#include "geometry/2dplane.h"
#include "state_validator/state_validator_occupancy_map.h"

using namespace boost::python;
using namespace Planner;

BOOST_PYTHON_MODULE(pyplanning)
{
	//             _                  _ _   _               
	//       /\   | |                (_) | | |              
	//      /  \  | | __ _  ___  _ __ _| |_| |__  _ __ ___  
	//     / /\ \ | |/ _` |/ _ \| '__| | __| '_ \| '_ ` _ \
	//    / ____ \| | (_| | (_) | |  | | |_| | | | | | | | |
	//   /_/    \_\_|\__, |\___/|_|  |_|\__|_| |_|_| |_| |_|
	//                __/ |                                 
	//               |___/                                  

	enum_<Status>("Status")
		.value("SUCCESS", Status::Success)
		.value("FAILURE", Status::Failure);

	struct PlanarPathPlannerWrapper : PlanarPathPlanner, wrapper<PlanarPathPlanner> {
		virtual Status SearchPath() override
		{
			return this->get_override("search_path")();
		}
		virtual std::vector<Pose2d> GetPath() override
		{
			return this->get_override("get_path")();
		}
	};

	class_<PlanarPathPlannerWrapper, boost::noncopyable>("PlanarPathPlanner")
		.def("search_path", pure_virtual(&PlanarPathPlanner::SearchPath))
		.def("get_path", pure_virtual(&PlanarPathPlanner::GetPath))
		.def("set_init_state", &PlanarPathPlanner::SetInitState)
		.def("set_goal_state", &PlanarPathPlanner::SetGoalState);

	class_<HybridAStar, boost::noncopyable, bases<PlanarPathPlanner>>("HybridAStar", init<Ref<StateValidatorOccupancyMap>>())
		.def(init<Ref<StateValidatorOccupancyMap>, HybridAStar::SearchParameters>())
		.def("initialize", &HybridAStar::Initialize)
		.def_readwrite("path_interpolation", &HybridAStar::pathInterpolation);

	//     _____                           _              
	//    / ____|                         | |             
	//   | |  __  ___  ___  _ __ ___   ___| |_ _ __ _   _ 
	//   | | |_ |/ _ \/ _ \| '_ ` _ \ / _ \ __| '__| | | |
	//   | |__| |  __/ (_) | | | | | |  __/ |_| |  | |_| |
	//    \_____|\___|\___/|_| |_| |_|\___|\__|_|   \__, |
	//                                               __/ |
	//                                              |___/ 
	class_<Point2d>("Point2d")
		.def(init<double, double>())
		.def("x", static_cast<double& (Point2d::*)()>(&Point2d::x), return_value_policy<copy_non_const_reference>())
		.def("y", static_cast<double& (Point2d::*)()>(&Point2d::y), return_value_policy<copy_non_const_reference>())
		.def(self + self)
		.def(self - self)
		.def(self == self)
		.def(self != self);

	class_<Pose2d>("Pose2d")
		.def(init<Point2d, double>())
		.def(init<double, double, double>())
		.def_readwrite("position", &Pose2d::position)
		.def_readwrite("theta", &Pose2d::theta)
		.def("x", static_cast<double& (Pose2d::*)()>(&Pose2d::x), return_value_policy<copy_non_const_reference>())
		.def("y", static_cast<double& (Pose2d::*)()>(&Pose2d::y), return_value_policy<copy_non_const_reference>())
		.def(self + self)
		.def(self - self)
		.def(self == self)
		.def(self != self);
	
	//    _____      _   _         
	//   |  __ \    | | | |        
	//   | |__) |_ _| |_| |__  ___ 
	//   |  ___/ _` | __| '_ \/ __|
	//   | |  | (_| | |_| | | \__ \
	//   |_|   \__,_|\__|_| |_|___/
	//                             
	enum_<Steer>("Steer")
		.value("LEFT", Steer::Left)
		.value("STRAIGHT", Steer::Straight)
		.value("RIGHT", Steer::Right);

	enum_<Direction>("Direction")
		.value("FORWARD", Direction::Forward)
		.value("BACKWARD", Direction::Backward)
		.value("NO_MOTION", Direction::NoMotion);

	struct PlanarPathWrapper : PlanarPath, wrapper<PlanarPath> {
		virtual Pose2d Interpolate(double) const override
		{
			return this->get_override("interpolate")();
		}
		virtual void Truncate(double) override
		{
			this->get_override("truncate")();
		}
		virtual Direction GetDirection(double) const override
		{
			return this->get_override("get_direction")();
		}
		virtual double ComputeCost(double, double, double) const override
		{
			return this->get_override("compute_cost")();
		}
	};

	class_<PlanarPathWrapper, boost::noncopyable>("PlanarPath")
		.def("get_initial_state", &PlanarPath::GetInitialState, return_value_policy<copy_const_reference>())
		.def("get_final_state", &PlanarPath::GetFinalState, return_value_policy<copy_const_reference>())
		.def("interpolate", pure_virtual(&PlanarPath::Interpolate))
		.def("truncate", pure_virtual(&PlanarPath::Truncate))
		.def("get_length", &PlanarPath::GetLength)
		.def("get_direction", pure_virtual(&PlanarPath::GetDirection))
		.def("compute_cost", pure_virtual(&PlanarPath::ComputeCost));

// 	class_<PathReedsShepp, bases<PlanarPath>>("PathReedsShepp", init<Pose2d, ReedsShepp::PathSegment, double>());
// 
// 	class_<PathConstantSteer, bases<PlanarPath>>("PathConstantSteer", init<KinematicBicycleModel*, Pose2d, double, double, Direction>());
}
