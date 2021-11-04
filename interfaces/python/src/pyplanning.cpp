#include <boost/python.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>

#include "core/base.h"
#include "algo/path_planner.h"
#include "algo/hybrid_a_star.h"

#include "geometry/2dplane.h"

#include "models/kinematic_bicycle_model.h"

#include "paths/path.h"
#include "paths/path_reeds_shepp.h"
#include "paths/path_constant_steer.h"

#include "state_space/state_space.h"
#include "state_space/state_space_reeds_shepp.h"
#include "state_space/state_space_se2.h"

#include "state_validator/occupancy_map.h"
#include "state_validator/obstacle_list_occupancy_map.h"
#include "state_validator/obstacle.h"
#include "state_validator/state_validator.h"
#include "state_validator/state_validator_free.h"
#include "state_validator/state_validator_occupancy_map.h"
#include "state_validator/gvd.h"

using namespace boost::python;
using namespace Planner;

#define INSTANTIATE_STD_VECTOR_TO_PYTHON_LIST(Type)                       \
	class_<std::vector<Type>>(("StdVector" + std::string(#Type)).c_str()) \
		.def(vector_indexing_suite<std::vector<Type>>());

BOOST_PYTHON_MODULE(pyplanning)
{

	PP_FOR_EACH(INSTANTIATE_STD_VECTOR_TO_PYTHON_LIST, Pose2d, Point2d);

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
		.def<double& (Point2d::*)()>("x", &Point2d::x, return_value_policy<copy_non_const_reference>())
		.def<double& (Point2d::*)()>("y", &Point2d::y, return_value_policy<copy_non_const_reference>())
		.def(self + self)
		.def(self - self)
		.def(self == self)
		.def(self != self);

	class_<Pose2d>("Pose2d")
		.def(init<Point2d, double>())
		.def(init<double, double, double>())
		.def_readwrite("position", &Pose2d::position)
		.def_readwrite("theta", &Pose2d::theta)
		.def<double& (Pose2d::*)()>("x", &Pose2d::x, return_value_policy<copy_non_const_reference>())
		.def<double& (Pose2d::*)()>("y", &Pose2d::y, return_value_policy<copy_non_const_reference>())
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

	class_<PathReedsShepp, bases<PlanarPath>>("PathReedsShepp", init<Pose2d, ReedsShepp::PathSegment, double>());

	class_<PathConstantSteer, bases<PlanarPath>>("PathConstantSteer", init<KinematicBicycleModel*, Pose2d, double, double, Direction>());

	//     _____ _        _        _____
	//    / ____| |      | |      / ____|
	//   | (___ | |_ __ _| |_ ___| (___  _ __   __ _  ___ ___
	//    \___ \| __/ _` | __/ _ \\___ \| '_ \ / _` |/ __/ _ \
	//    ____) | || (_| | ||  __/____) | |_) | (_| | (_|  __/
	//   |_____/ \__\__,_|\__\___|_____/| .__/ \__,_|\___\___|
	//                                  | |
	//                                  |_|

	struct PlanarStateSpaceWrapper : PlanarStateSpace, wrapper<PlanarStateSpace> {
		PlanarStateSpaceWrapper(const std::array<Pose2d, 2>& bounds) :
			PlanarStateSpace(bounds) { }
		PlanarStateSpaceWrapper(const Pose2d& lb, const Pose2d& ub) :
			PlanarStateSpace(lb, ub) { }
		virtual double ComputeDistance(const Pose2d&, const Pose2d&) override
		{
			return this->get_override("compute_distance")();
		}
	};

	class_<PlanarStateSpaceWrapper, Ref<PlanarStateSpaceWrapper>, boost::noncopyable>("PlanarStateSpace", init<const std::array<Pose2d, 2>&>())
		.def(init<const Pose2d&, const Pose2d&>())
		.def("compute_distance)", pure_virtual(&PlanarStateSpace::ComputeDistance))
		.def("enforce_bounds", &PlanarStateSpace::EnforceBounds)
		.def("validate_bounds", &PlanarStateSpace::ValidateBounds)
		.def("sample_uniform", &PlanarStateSpace::SampleUniform)
		.def("sample_gaussian", &PlanarStateSpace::SampleGaussian);

	class_<StateSpaceReedsShepp, Ref<StateSpaceReedsShepp>, bases<PlanarStateSpace>>("StateSpaceReedsShepp", init<const std::array<Pose2d, 2>&, double>())
		.def(init<const Pose2d&, const Pose2d&>())
		.def_readonly("min_turning_radius", &StateSpaceReedsShepp::minTurningRadius);

	class_<StateSpaceSE2, Ref<StateSpaceSE2>, bases<PlanarStateSpace>>("StateSpaceSE2", init<const std::array<Pose2d, 2>&>())
		.def(init<const Pose2d&, const Pose2d&>());

	//   __      __   _ _     _       _
	//   \ \    / /  | (_)   | |     | |
	//    \ \  / /_ _| |_  __| | __ _| |_ ___  _ __
	//     \ \/ / _` | | |/ _` |/ _` | __/ _ \| '__|
	//      \  / (_| | | | (_| | (_| | || (_) | |
	//       \/ \__,_|_|_|\__,_|\__,_|\__\___/|_|
	//

	struct OccupancyMapWrapper : OccupancyMap, wrapper<OccupancyMap> {
		OccupancyMapWrapper(float resolution) :
			OccupancyMap(resolution) { }

		virtual void Update() override
		{
			if (override f = this->get_override("update"))
				f();
			else
				OccupancyMap::Update();
		}
		void default_Update() { this->OccupancyMap::Update(); }

		virtual bool IsOccupied(const GridCellPosition&) override
		{
			return this->get_override("is_occupied")();
		}
	};

	class_<OccupancyMapWrapper, Ref<OccupancyMapWrapper>, boost::noncopyable>("OccupancyMap", init<float>())
		.def("rows", &OccupancyMap::Rows)
		.def("columns", &OccupancyMap::Columns)
		.def("update", &OccupancyMap::Update, &OccupancyMapWrapper::default_Update)
		.def("set_position", &OccupancyMap::SetPosition)
		.def("get_position", &OccupancyMap::GetPosition, return_value_policy<copy_const_reference>())
		.def("is_occupied)", pure_virtual(&OccupancyMap::IsOccupied))
		.def<int (OccupancyMap::*)(const GridCellPosition&)>("get_occupancy_value", &OccupancyMap::GetOccupancyValue)
		.def<int (OccupancyMap::*)(int, int)>("get_occupancy_value", &OccupancyMap::GetOccupancyValue)
		.def<Ref<GVD::ObstacleDistanceMap>& (OccupancyMap::*)()>("get_obstacle_map", &OccupancyMap::GetObstacleDistanceMap, return_value_policy<copy_non_const_reference>())
		.def<Point2d (OccupancyMap::*)(const GridCellPosition&) const>("grid_cell_to_local_position", &OccupancyMap::GridCellToLocalPosition)
		.def<Point2d (OccupancyMap::*)(const GridCellPosition&) const>("grid_cell_to_world_position", &OccupancyMap::GridCellToWorldPosition)
		.def<GridCellPosition (OccupancyMap::*)(const Point2d&, bool) const>("local_position_to_grid_cell", &OccupancyMap::LocalPositionToGridCell)
		.def<Point2d (OccupancyMap::*)(const Point2d&) const>("local_position_to_world_position", &OccupancyMap::LocalPositionToWorldPosition)
		.def<GridCellPosition (OccupancyMap::*)(const Point2d&, bool) const>("world_position_to_grid_cell", &OccupancyMap::WorldPositionToGridCell)
		.def<Point2d (OccupancyMap::*)(const Point2d&) const>("world_position_to_local_position", &OccupancyMap::WorldPositionToLocalPosition)
		.def<bool (OccupancyMap::*)(const Point2d&) const>("is_inside_map", &OccupancyMap::IsInsideMap)
		.def<bool (OccupancyMap::*)(const GridCellPosition&) const>("is_inside_map", &OccupancyMap::IsInsideMap);

	class_<ObstacleListOccupancyMap, Ref<ObstacleListOccupancyMap>, bases<OccupancyMap>>("ObstacleListOccupancyMap", init<float>())
		.def("add_obstacle", &ObstacleListOccupancyMap::AddObstacle)
		.def("remove_obstacle", &ObstacleListOccupancyMap::RemoveObstacle)
		.def("get_num_obstacles", &ObstacleListOccupancyMap::GetNumObstacles);

	class_<Obstacle, Ref<Obstacle>>("Obstacle")
		.def("set_shape", &Obstacle::SetShape)
		.def("set_pose", &Obstacle::SetPose);

	struct ShapeWrapper : Shape, wrapper<Shape> {
		virtual void GetGridCellPositions(const OccupancyMap&, const Pose2d, std::vector<GridCellPosition>&) override
		{
		}
	};
	class_<ShapeWrapper, Ref<ShapeWrapper>, boost::noncopyable>("Shape");
	class_<CompositeShape, Ref<CompositeShape>, bases<Shape>>("CompositeShape")
		.def("add", &CompositeShape::Add);
	class_<PolygonShape, Ref<PolygonShape>, bases<Shape>>("PolygonShape", init<const std::vector<Point2d>&>());
	class_<RegularPolygonShape, Ref<RegularPolygonShape>, bases<Shape>>("RegularPolygonShape", init<double, int>());
	class_<RectangleShape, Ref<RectangleShape>, bases<Shape>>("RectangleShape", init<double, double>());
	class_<CircleShape, Ref<CircleShape>, bases<Shape>>("CircleShape", init<double, int>());

	struct PlanarStateValidatorWrapper : PlanarStateValidator, wrapper<PlanarStateValidator> {
		PlanarStateValidatorWrapper(const Ref<PlanarStateSpace>& stateSpace) :
			PlanarStateValidator(stateSpace) { }
		virtual bool IsStateValid(const Pose2d&) override
		{
			return this->get_override("is_state_valid")();
		}
		virtual bool IsPathValid(const Path<Pose2d>&, float*) override
		{
			return this->get_override("is_path_valid")();
		}
	};

	class_<PlanarStateValidatorWrapper, boost::noncopyable>("PlanarStateValidator", init<const Ref<PlanarStateSpace>&>())
		.def("is_state_valid", pure_virtual(&PlanarStateValidator::IsStateValid))
		.def("is_path_valid", pure_virtual<bool (PlanarStateValidator::*)(const PlanarPath&, float*)>(&PlanarStateValidator::IsPathValid))
		.add_property("state_space", make_function(&PlanarStateValidator::GetStateSpace, return_value_policy<copy_non_const_reference>()));

	class_<PlanarStateValidatorFree, bases<PlanarStateValidator>>("PlanarStateValidatorFree", init<const Ref<PlanarStateSpace>&>());

	class_<StateValidatorOccupancyMap, bases<PlanarStateValidator>>("StateValidatorOccupancyMap", init<const Ref<PlanarStateSpace>&, const Ref<OccupancyMap>&>())
		.def("get_occupancy_map", &StateValidatorOccupancyMap::GetOccupancyMap, return_value_policy<copy_non_const_reference>())
		.def_readwrite("min_path_interpolation_distance", &StateValidatorOccupancyMap::minPathInterpolationDistance)
		.def_readwrite("min_safe_radius", &StateValidatorOccupancyMap::minSafeRadius);

	class_<GVD, boost::noncopyable>("GVD", init<const Ref<OccupancyMap>&>())
		.def("update", &GVD::Update)
		.def<float (GVD::*)(int, int) const>("get_distance_to_nearest_obstacle", &GVD::GetDistanceToNearestObstacle)
		.def<float (GVD::*)(const GridCellPosition&) const>("get_distance_to_nearest_obstacle", &GVD::GetDistanceToNearestObstacle)
		.def<bool (GVD::*)(const Point2d&, float&) const>("get_distance_to_nearest_obstacle", &GVD::GetDistanceToNearestObstacle)
		.def<float (GVD::*)(int, int) const>("get_distance_to_nearest_voronoi_edge", &GVD::GetDistanceToNearestVoronoiEdge)
		.def<float (GVD::*)(const GridCellPosition&) const>("get_distance_to_nearest_voronoi_edge", &GVD::GetDistanceToNearestVoronoiEdge)
		.def<bool (GVD::*)(const Point2d&, float&) const>("get_distance_to_nearest_voronoi_edge", &GVD::GetDistanceToNearestVoronoiEdge)
		.def<float (GVD::*)(int, int) const>("get_path_cost", &GVD::GetPathCost)
		.def<float (GVD::*)(const GridCellPosition&) const>("get_path_cost", &GVD::GetPathCost)
		.def<bool (GVD::*)(const Point2d&, float&) const>("get_path_cost", &GVD::GetPathCost)
		.def("visualize", &GVD::Visualize);
}
