#include <pybind11/pybind11.h>
#include <pybind11/operators.h>
#include <pybind11/stl.h>

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

using namespace pybind11;
using namespace Planner;

void Initialize()
{
	PP_INIT_LOGGER;
}

PYBIND11_MODULE(pyplanning, m)
{

	m.def("initialize", &Initialize);

	//             _                  _ _   _
	//       /\   | |                (_) | | |
	//      /  \  | | __ _  ___  _ __ _| |_| |__  _ __ ___
	//     / /\ \ | |/ _` |/ _ \| '__| | __| '_ \| '_ ` _ \
	//    / ____ \| | (_| | (_) | |  | | |_| | | | | | | | |
	//   /_/    \_\_|\__, |\___/|_|  |_|\__|_| |_|_| |_| |_|
	//                __/ |
	//               |___/

	enum_<Status>(m, "Status")
		.value("SUCCESS", Status::Success)
		.value("FAILURE", Status::Failure);

	struct PlanarPathPlannerWrapper : PlanarPathPlanner {
		using PlanarPathPlanner::PlanarPathPlanner;
		virtual Status SearchPath() override { PYBIND11_OVERRIDE_PURE(Status, PlanarPathPlanner, SearchPath); }
		virtual std::vector<Pose2d> GetPath() override { PYBIND11_OVERRIDE_PURE(std::vector<Pose2d>, PlanarPathPlanner, GetPath); }
	};

	class_<PlanarPathPlanner, PlanarPathPlannerWrapper>(m, "PlanarPathPlanner")
        .def(init<>())
		.def("search_path", &PlanarPathPlanner::SearchPath)
		.def("get_path", &PlanarPathPlanner::GetPath)
		.def("set_init_state", &PlanarPathPlanner::SetInitState)
		.def("set_goal_state", &PlanarPathPlanner::SetGoalState);

	class_<HybridAStar, PlanarPathPlanner>(m, "HybridAStar")
        .def(init<Ref<StateValidatorOccupancyMap>>())
		.def(init<Ref<StateValidatorOccupancyMap>, HybridAStar::SearchParameters>())
		.def("initialize", &HybridAStar::Initialize)
		.def_readwrite("path_interpolation", &HybridAStar::pathInterpolation)
		.def("get_explored_paths", &HybridAStar::GetExploredPaths)
		.def("visualize_obstacle_heuristic", &HybridAStar::VisualizeObstacleHeuristic);

	//     _____                           _
	//    / ____|                         | |
	//   | |  __  ___  ___  _ __ ___   ___| |_ _ __ _   _
	//   | | |_ |/ _ \/ _ \| '_ ` _ \ / _ \ __| '__| | | |
	//   | |__| |  __/ (_) | | | | | |  __/ |_| |  | |_| |
	//    \_____|\___|\___/|_| |_| |_|\___|\__|_|   \__, |
	//                                               __/ |
	//                                              |___/

	class_<Point2d>(m, "Point2d")
		.def(init<double, double>())
		.def<double& (Point2d::*)()>("x", &Point2d::x)
		.def<double& (Point2d::*)()>("y", &Point2d::y)
		.def(self + self)
		.def(self - self)
		.def(self == self)
		.def(self != self);

	class_<Pose2d>(m, "Pose2d")
		.def(init<Point2d, double>())
		.def(init<double, double, double>())
		.def_readwrite("position", &Pose2d::position)
		.def_readwrite("theta", &Pose2d::theta)
		.def<double& (Pose2d::*)()>("x", &Pose2d::x)
		.def<double& (Pose2d::*)()>("y", &Pose2d::y)
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

	enum_<Steer>(m, "Steer")
		.value("LEFT", Steer::Left)
		.value("STRAIGHT", Steer::Straight)
		.value("RIGHT", Steer::Right);

	enum_<Direction>(m, "Direction")
		.value("FORWARD", Direction::Forward)
		.value("BACKWARD", Direction::Backward)
		.value("NO_MOTION", Direction::NoMotion);

	struct PlanarPathWrapper : PlanarPath {
		using PlanarPath::PlanarPath;
		virtual Pose2d Interpolate(double ratio) const override { PYBIND11_OVERRIDE_PURE(Pose2d, PlanarPath, Interpolate, ratio); }
		virtual void Truncate(double ratio) override { PYBIND11_OVERRIDE_PURE(void, PlanarPath, Truncate, ratio); }
		virtual Direction GetDirection(double ratio) const override { PYBIND11_OVERRIDE_PURE(Direction, PlanarPath, GetDirection, ratio); }
		virtual double ComputeCost(double a, double b, double c) const override { PYBIND11_OVERRIDE_PURE(double, PlanarPath, ComputeCost, a, b, c); }
	};

	class_<PlanarPath, PlanarPathWrapper>(m, "PlanarPath")
		.def(init<>())
		.def("get_initial_state", &PlanarPath::GetInitialState)
		.def("get_final_state", &PlanarPath::GetFinalState)
		.def("interpolate", &PlanarPath::Interpolate)
		.def("truncate", &PlanarPath::Truncate)
		.def("get_length", &PlanarPath::GetLength)
		.def("get_direction", &PlanarPath::GetDirection)
		.def("compute_cost", &PlanarPath::ComputeCost);

	class_<PathReedsShepp, PlanarPath>(m, "PathReedsShepp")
		.def(init<Pose2d, ReedsShepp::PathSegment, double>());

	class_<PathConstantSteer, PlanarPath>(m, "PathConstantSteer")
		.def(init<KinematicBicycleModel*, Pose2d, double, double, Direction>());

	//     _____ _        _        _____
	//    / ____| |      | |      / ____|
	//   | (___ | |_ __ _| |_ ___| (___  _ __   __ _  ___ ___
	//    \___ \| __/ _` | __/ _ \\___ \| '_ \ / _` |/ __/ _ \
	//    ____) | || (_| | ||  __/____) | |_) | (_| | (_|  __/
	//   |_____/ \__\__,_|\__\___|_____/| .__/ \__,_|\___\___|
	//                                  | |
	//                                  |_|

	struct PlanarStateSpaceWrapper : PlanarStateSpace {
		using PlanarStateSpace::PlanarStateSpace;
		virtual double ComputeDistance(const Pose2d& a, const Pose2d& b) override { PYBIND11_OVERRIDE_PURE(double, PlanarStateSpace, ComputeDistance, a, b); }
	};

	class_<PlanarStateSpace, Ref<PlanarStateSpace>, PlanarStateSpaceWrapper>(m, "PlanarStateSpace")
		.def(init<const std::array<Pose2d, 2>&>())
		.def(init<const Pose2d&, const Pose2d&>())
		.def("compute_distance)", &PlanarStateSpace::ComputeDistance)
		.def("enforce_bounds", &PlanarStateSpace::EnforceBounds)
		.def("validate_bounds", &PlanarStateSpace::ValidateBounds)
		.def("sample_uniform", &PlanarStateSpace::SampleUniform)
		.def("sample_gaussian", &PlanarStateSpace::SampleGaussian);

	class_<StateSpaceReedsShepp, Ref<StateSpaceReedsShepp>, PlanarStateSpace>(m, "StateSpaceReedsShepp")
		.def(init<const std::array<Pose2d, 2>&, double>())
		.def(init<const Pose2d&, const Pose2d&, double>())
		.def_readonly("min_turning_radius", &StateSpaceReedsShepp::minTurningRadius);

	class_<StateSpaceSE2, Ref<StateSpaceSE2>, PlanarStateSpace>(m, "StateSpaceSE2")
		.def(init<const std::array<Pose2d, 2>&>())
		.def(init<const Pose2d&, const Pose2d&>());

	//   __      __   _ _     _       _
	//   \ \    / /  | (_)   | |     | |
	//    \ \  / /_ _| |_  __| | __ _| |_ ___  _ __
	//     \ \/ / _` | | |/ _` |/ _` | __/ _ \| '__|
	//      \  / (_| | | | (_| | (_| | || (_) | |
	//       \/ \__,_|_|_|\__,_|\__,_|\__\___/|_|
	//

	struct OccupancyMapWrapper : OccupancyMap {
		using OccupancyMap::OccupancyMap;
		virtual void Update() override { PYBIND11_OVERRIDE(void, OccupancyMap, Update); }
		virtual bool IsOccupied(const GridCellPosition& a) override { PYBIND11_OVERRIDE_PURE(bool, OccupancyMap, IsOccupied, a); }
	};

	class_<OccupancyMap, Ref<OccupancyMap>, OccupancyMapWrapper>(m, "OccupancyMap")
		.def(init<float>())
		.def("rows", &OccupancyMap::Rows)
		.def("columns", &OccupancyMap::Columns)
		.def("update", &OccupancyMap::Update)
		.def("set_position", &OccupancyMap::SetPosition)
		.def("get_position", &OccupancyMap::GetPosition)
		.def("is_occupied)", &OccupancyMap::IsOccupied)
		.def<int (OccupancyMap::*)(const GridCellPosition&)>("get_occupancy_value", &OccupancyMap::GetOccupancyValue)
		.def<int (OccupancyMap::*)(int, int)>("get_occupancy_value", &OccupancyMap::GetOccupancyValue)
		.def<Ref<GVD::ObstacleDistanceMap>& (OccupancyMap::*)()>("get_obstacle_map", &OccupancyMap::GetObstacleDistanceMap)
		.def<Point2d (OccupancyMap::*)(const GridCellPosition&) const>("grid_cell_to_local_position", &OccupancyMap::GridCellToLocalPosition)
		.def<Point2d (OccupancyMap::*)(const GridCellPosition&) const>("grid_cell_to_world_position", &OccupancyMap::GridCellToWorldPosition)
		.def<GridCellPosition (OccupancyMap::*)(const Point2d&, bool) const>("local_position_to_grid_cell", &OccupancyMap::LocalPositionToGridCell)
		.def<Point2d (OccupancyMap::*)(const Point2d&) const>("local_position_to_world_position", &OccupancyMap::LocalPositionToWorldPosition)
		.def<GridCellPosition (OccupancyMap::*)(const Point2d&, bool) const>("world_position_to_grid_cell", &OccupancyMap::WorldPositionToGridCell)
		.def<Point2d (OccupancyMap::*)(const Point2d&) const>("world_position_to_local_position", &OccupancyMap::WorldPositionToLocalPosition)
		.def<bool (OccupancyMap::*)(const Point2d&) const>("is_inside_map", &OccupancyMap::IsInsideMap)
		.def<bool (OccupancyMap::*)(const GridCellPosition&) const>("is_inside_map", &OccupancyMap::IsInsideMap);

	class_<ObstacleListOccupancyMap, Ref<ObstacleListOccupancyMap>, OccupancyMap>(m, "ObstacleListOccupancyMap")
		.def(init<float>())
		.def("add_obstacle", &ObstacleListOccupancyMap::AddObstacle)
		.def("remove_obstacle", &ObstacleListOccupancyMap::RemoveObstacle)
		.def("get_num_obstacles", &ObstacleListOccupancyMap::GetNumObstacles);

	class_<Obstacle, Ref<Obstacle>>(m, "Obstacle")
		.def(init<>())
		.def("set_shape", &Obstacle::SetShape)
		.def("set_pose", &Obstacle::SetPose);

	struct ShapeWrapper : Shape {
		using Shape::Shape;
		virtual void GetGridCellPositions(const OccupancyMap&a , const Pose2d b , std::vector<GridCellPosition>&c) override
		{
			PYBIND11_OVERRIDE_PURE(void, Shape, GetGridCellPositions, a, b, c);
		}
	};
	class_<Shape, Ref<Shape>, ShapeWrapper>(m, "Shape")
		.def(init<>());
	class_<CompositeShape, Ref<CompositeShape>, Shape>(m, "CompositeShape")
		.def(init<>())
		.def("add", &CompositeShape::Add);
	class_<PolygonShape, Ref<PolygonShape>, Shape>(m, "PolygonShape")
		.def(init<const std::vector<Point2d>&>());
	class_<RegularPolygonShape, Ref<RegularPolygonShape>, Shape>(m, "RegularPolygonShape")
		.def(init<double, int>());
	class_<RectangleShape, Ref<RectangleShape>, Shape>(m, "RectangleShape")
		.def(init<double, double>());
	class_<CircleShape, Ref<CircleShape>, Shape>(m, "CircleShape")
		.def(init<double, int>());

	struct PlanarStateValidatorWrapper : PlanarStateValidator {
		using PlanarStateValidator::PlanarStateValidator;
		virtual bool IsStateValid(const Pose2d& a) override { PYBIND11_OVERRIDE_PURE(bool, PlanarStateValidator, IsStateValid, a); }
		virtual bool IsPathValid(const Path<Pose2d>& a, float* b) override { PYBIND11_OVERRIDE_PURE(bool, PlanarStateValidator, IsStateValid, a, b); }
	};

	class_<PlanarStateValidator, Ref<PlanarStateValidator>, PlanarStateValidatorWrapper>(m, "PlanarStateValidator")
		.def(init<const Ref<PlanarStateSpace>&>())
		.def("is_state_valid", &PlanarStateValidator::IsStateValid)
		.def<bool (PlanarStateValidator::*)(const PlanarPath&, float*)>("is_path_valid", &PlanarStateValidator::IsPathValid)
		.def_property("state_space", &PlanarStateValidator::GetStateSpace, nullptr);

	class_<PlanarStateValidatorFree, Ref<PlanarStateValidatorFree>, PlanarStateValidator>(m, "PlanarStateValidatorFree")
		.def(init<const Ref<PlanarStateSpace>&>());

	class_<StateValidatorOccupancyMap, Ref<StateValidatorOccupancyMap>, PlanarStateValidator>(m, "StateValidatorOccupancyMap")
		.def(init<const Ref<PlanarStateSpace>&, const Ref<OccupancyMap>&>())
		.def("get_occupancy_map", &StateValidatorOccupancyMap::GetOccupancyMap)
		.def_readwrite("min_path_interpolation_distance", &StateValidatorOccupancyMap::minPathInterpolationDistance)
		.def_readwrite("min_safe_radius", &StateValidatorOccupancyMap::minSafeRadius);

	class_<GVD>(m, "GVD")
		.def(init<const Ref<OccupancyMap>&>())
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