#include <pybind11/pybind11.h>
#include <pybind11/operators.h>
#include <pybind11/stl.h>
#include <pybind11/functional.h>

#include "core/base.h"

#include "algo/path_planner.h"
#include "algo/hybrid_a_star.h"
#include "algo/a_star_n2.h"

#include "geometry/2dplane.h"

#include "models/kinematic_bicycle_model.h"

#include "paths/path.h"
#include "paths/path_se2.h"
#include "paths/path_reeds_shepp.h"
#include "paths/path_constant_steer.h"

#include "state_space/state_space.h"
#include "state_space/state_space_se2.h"

#include "state_validator/occupancy_map.h"
#include "state_validator/obstacle_list_occupancy_map.h"
#include "state_validator/obstacle.h"
#include "state_validator/state_validator.h"
#include "state_validator/state_validator_free.h"
#include "state_validator/state_validator_occupancy_map.h"
#include "state_validator/gvd.h"

#include "utils/grid.h"

using namespace pybind11;
using namespace Planner;

void Initialize()
{
	PP_INIT;
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

	struct PathPlannerSE2BaseWrapper : PathPlannerSE2Base {
		using PathPlannerSE2Base::PathPlannerSE2Base;
		virtual Status SearchPath() override { PYBIND11_OVERRIDE_PURE(Status, PathPlannerSE2Base, SearchPath); }
		virtual std::vector<Pose2d> GetPath() const override { PYBIND11_OVERRIDE_PURE(std::vector<Pose2d>, PathPlannerSE2Base, GetPath); }
	};

	class_<PathPlannerSE2Base, PathPlannerSE2BaseWrapper>(m, "PathPlannerSE2Base")
		.def(init<>())
		.def("search_path", &PathPlannerSE2Base::SearchPath)
		.def("get_path", &PathPlannerSE2Base::GetPath)
		.def("set_init_state", &PathPlannerSE2Base::SetInitState)
		.def("set_goal_state", &PathPlannerSE2Base::SetGoalState);

	class_<HybridAStar::SearchParameters>(m, "HybridAStarSearchParameters")
		.def(init<>())
		.def(init<double, double, double, double, double, unsigned int, double, double>())
		.def_readonly("wheelbase", &HybridAStar::SearchParameters::wheelbase)
		.def_readonly("min_turning_radius", &HybridAStar::SearchParameters::minTurningRadius)
		.def_readonly("direction_switching_cost", &HybridAStar::SearchParameters::directionSwitchingCost)
		.def_readonly("reverse_cost_multiplier", &HybridAStar::SearchParameters::reverseCostMultiplier)
		.def_readonly("forward_cost_multiplier", &HybridAStar::SearchParameters::forwardCostMultiplier)
		.def_readonly("voronoi_cost_multiplier", &HybridAStar::SearchParameters::voronoiCostMultiplier)
		.def_readonly("num_generated_motion", &HybridAStar::SearchParameters::numGeneratedMotion)
		.def_readonly("spatial_resolution", &HybridAStar::SearchParameters::spatialResolution)
		.def_readonly("angular_resolution", &HybridAStar::SearchParameters::angularResolution);

	class_<Smoother::Parameters>(m, "HybridAStarSmootherParameters")
		.def(init<float>())
		.def_readwrite("step_tolerance", &Smoother::Parameters::stepTolerance)
		.def_readwrite("max_iterations", &Smoother::Parameters::maxIterations)
		.def_readwrite("learning_rate", &Smoother::Parameters::learningRate)
		.def_readwrite("path_weight", &Smoother::Parameters::pathWeight)
		.def_readwrite("smooth_weight", &Smoother::Parameters::smoothWeight)
		.def_readwrite("voronoi_weight", &Smoother::Parameters::voronoiWeight)
		.def_readwrite("collision_weight", &Smoother::Parameters::collisionWeight)
		.def_readwrite("curvature_weight", &Smoother::Parameters::curvatureWeight)
		.def_readwrite("collision_ratio", &Smoother::Parameters::collisionRatio)
		.def_readonly("max_curvature", &Smoother::Parameters::maxCurvature);

	class_<HybridAStar, PathPlannerSE2Base>(m, "HybridAStar")
		.def(init<>())
		.def(init<const HybridAStar::SearchParameters&>())
		.def("initialize", &HybridAStar::Initialize)
		.def_readwrite("path_interpolation", &HybridAStar::pathInterpolation)
		.def("get_graph_search_explored_set", &HybridAStar::GetGraphSearchExploredSet)
		.def("get_graph_search_path", &HybridAStar::GetGraphSearchPath)
		.def("get_graph_search_optimal_cost", &HybridAStar::GetGraphSearchOptimalCost)
		.def("get_search_parameters", &HybridAStar::GetSearchParameters)
		.def_property("smoother_parameters", &HybridAStar::GetSmootherParameters, &HybridAStar::SetSmootherParameters)
		.def("visualize_obstacle_heuristic", &HybridAStar::VisualizeObstacleHeuristic);

	struct PathPlannerN2BaseWrapper : PathPlannerN2Base {
		using PathPlannerN2Base::PathPlannerN2Base;
		virtual Status SearchPath() override { PYBIND11_OVERRIDE_PURE(Status, PathPlannerN2Base, SearchPath); }
		virtual std::vector<GridCellPosition> GetPath() const override { PYBIND11_OVERRIDE_PURE(std::vector<GridCellPosition>, PathPlannerN2Base, GetPath); }
	};

	class_<PathPlannerN2Base, PathPlannerN2BaseWrapper>(m, "PathPlannerN2Base")
		.def(init<>())
		.def("search_path", &PathPlannerN2Base::SearchPath)
		.def("get_path", &PathPlannerN2Base::GetPath)
		.def("set_init_state", &PathPlannerN2Base::SetInitState)
		.def("set_goal_state", &PathPlannerN2Base::SetGoalState);

	// Create GetExploredStates to return a std::unordered_set
	struct PyAStarN2 : public AStarN2 {
		using AStarN2::AStarN2;
		const std::unordered_set<GridCellPosition, std::hash<GridCellPosition>, std::equal_to<GridCellPosition>>& GetExploredStates() const { return m_explored; }
	};

	class_<PyAStarN2, PathPlannerN2Base>(m, "AStarN2")
		.def(init<>())
		.def<bool (AStarN2::*)(
			const Ref<OccupancyMap>&,
			const std::function<double(const GridCellPosition&, const GridCellPosition&)>&,
			const std::function<double(const GridCellPosition&, const GridCellPosition&)>&)>("initialize", &AStarN2::Initialize)
		.def("get_explored_states", &PyAStarN2::GetExploredStates)
		.def("get_optimal_cost", &AStarN2::GetOptimalCost);

	// Create GetExploredStates to return a std::unordered_set
	struct PyBidirectionalAStarN2 : public BidirectionalAStarN2 {
		using BidirectionalAStarN2::BidirectionalAStarN2;
		std::tuple<std::unordered_set<GridCellPosition, std::hash<GridCellPosition>, std::equal_to<GridCellPosition>>,
			std::unordered_set<GridCellPosition, std::hash<GridCellPosition>, std::equal_to<GridCellPosition>>>
		GetExploredStates()
		{
			std::unordered_set<GridCellPosition, std::hash<GridCellPosition>, std::equal_to<GridCellPosition>> fExplored, rExplored;
			auto [fExploredMap, rExploredMap] = BidirectionalAStarN2::GetExploredStates();
			for (auto kv : fExploredMap)
				fExplored.insert(kv.first);
			for (auto kv : rExploredMap)
				rExplored.insert(kv.first);
			return std::make_tuple(fExplored, rExplored);
		}
	};

	class_<PyBidirectionalAStarN2, PathPlannerN2Base>(m, "BidirectionalAStarN2")
		.def(init<>())
		.def<bool (BidirectionalAStarN2::*)(
			const Ref<OccupancyMap>&,
			const std::function<double(const GridCellPosition&, const GridCellPosition&)>&,
			const std::function<double(const GridCellPosition&, const GridCellPosition&)>&)>("initialize", &BidirectionalAStarN2::Initialize)
		.def("get_explored_states", &PyBidirectionalAStarN2::GetExploredStates)
		.def("get_optimal_cost", &BidirectionalAStarN2::GetOptimalCost);

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
		.def(init<const Point2d&, double>())
		.def(init<double, double, double>())
		.def_readwrite("position", &Pose2d::position)
		.def_readwrite("theta", &Pose2d::theta)
		.def<double& (Pose2d::*)()>("x", &Pose2d::x)
		.def<double& (Pose2d::*)()>("y", &Pose2d::y)
		.def(self + self)
		.def(self - self)
		.def(self == self)
		.def(self != self);

	class_<GridCellPosition>(m, "GridCellPosition")
		.def(init<>())
		.def(init<int, int>())
		.def_readwrite("row", &GridCellPosition::row)
		.def_readwrite("col", &GridCellPosition::col)
		.def("__repr__",
			[](const GridCellPosition& cell) {
				return "<GridCellPosition: row " + std::to_string(cell.row) + ", col: " + std::to_string(cell.col) + ">";
			}
		);

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

	struct PathSE2BaseWrapper : PathSE2Base {
		using PathSE2Base::PathSE2Base;
		virtual Pose2d Interpolate(double ratio) const override { PYBIND11_OVERRIDE_PURE(Pose2d, PathSE2Base, Interpolate, ratio); }
		virtual void Truncate(double ratio) override { PYBIND11_OVERRIDE_PURE(void, PathSE2Base, Truncate, ratio); }
	};

	class_<PathSE2Base, Ref<PathSE2Base>, PathSE2BaseWrapper>(m, "PathSE2Base")
		.def(init<>())
		.def("get_initial_state", &PathSE2Base::GetInitialState)
		.def("get_final_state", &PathSE2Base::GetFinalState)
		.def<Pose2d (PathSE2Base::*)(double) const>("interpolate", &PathSE2Base::Interpolate)
		.def<std::vector<Pose2d> (PathSE2Base::*)(const std::vector<double>&) const>("interpolate", &PathSE2Base::Interpolate)
		.def("truncate", &PathSE2Base::Truncate)
		.def("get_length", &PathSE2Base::GetLength);

	class_<PathSE2, Ref<PathSE2>, PathSE2Base>(m, "PathSE2")
		.def(init<const Pose2d&, const Pose2d&>());

	struct PathNonHolonomicSE2BaseWrapper : PathSE2BaseWrapper, virtual PathNonHolonomicSE2Base {
		using PathNonHolonomicSE2Base::PathNonHolonomic;
		virtual Direction GetDirection(double ratio) const override { PYBIND11_OVERRIDE_PURE(Direction, PathNonHolonomicSE2Base, GetDirection, ratio); }
		virtual std::set<double> GetCuspPointRatios() const override { PYBIND11_OVERRIDE(std::set<double>, PathNonHolonomicSE2Base, GetCuspPointRatios); }
	};

	class_<PathNonHolonomicSE2Base, Ref<PathNonHolonomicSE2Base>, PathSE2Base, PathNonHolonomicSE2BaseWrapper>(m, "PathNonHolonomicSE2Base")
		.def("get_direction", &PathNonHolonomicSE2Base::GetDirection)
		.def("get_cusp_point_ratios", &PathNonHolonomicSE2Base::GetCuspPointRatios);

	class_<PathReedsShepp, Ref<PathReedsShepp>, PathNonHolonomicSE2Base>(m, "PathReedsShepp")
		.def(init<const Pose2d&, ReedsShepp::PathSegment, double>())
		.def_property("min_turning_radius", &PathReedsShepp::GetMinTurningRadius, nullptr);

	class_<PathConstantSteer, Ref<PathConstantSteer>, PathNonHolonomicSE2Base>(m, "PathConstantSteer")
		.def(init<const Ref<KinematicBicycleModel>&, const Pose2d&, double, double, Direction>())
		.def_property("steering", &PathConstantSteer::GetSteeringAngle, nullptr);

	class_<KinematicBicycleModel, Ref<KinematicBicycleModel>>(m, "KinematicBicycleModel")
		.def(init<double, double>());

	struct PathConnectionSE2BaseWrapper : PathConnectionSE2Base {
		using PathConnectionSE2Base::PathConnectionSE2Base;
		virtual Ref<Path<Pose2d>> Connect(const Pose2d& from, const Pose2d& to) override { PYBIND11_OVERRIDE_PURE(Ref<Path<Pose2d>>, PathConnectionSE2Base, Connect, from, to); }
	};

	class_<PathConnectionSE2Base, Ref<PathConnectionSE2Base>, PathConnectionSE2BaseWrapper>(m, "PathConnectionSE2Base")
		.def(init<>())
		.def("connect", &PathConnectionSE2Base::Connect);

	class_<PathConnectionSE2, Ref<PathConnectionSE2>, PathConnectionSE2Base>(m, "PathConnectionSE2")
		.def(init<>());

	class_<PathConnectionReedsShepp, Ref<PathConnectionReedsShepp>, PathConnectionSE2Base>(m, "PathConnectionReedsShepp")
		.def(init<double, double, double, double>());

	//     _____ _        _        _____
	//    / ____| |      | |      / ____|
	//   | (___ | |_ __ _| |_ ___| (___  _ __   __ _  ___ ___
	//    \___ \| __/ _` | __/ _ \\___ \| '_ \ / _` |/ __/ _ \
	//    ____) | || (_| | ||  __/____) | |_) | (_| | (_|  __/
	//   |_____/ \__\__,_|\__\___|_____/| .__/ \__,_|\___\___|
	//                                  | |
	//                                  |_|

	class_<StateSpaceSE2, Ref<StateSpaceSE2>>(m, "StateSpaceSE2")
		.def(init<const std::array<Pose2d, 2>&>())
		.def(init<const Pose2d&, const Pose2d&>())
		.def("enforce_bounds", &StateSpaceSE2::EnforceBounds)
		.def("validate_bounds", &StateSpaceSE2::ValidateBounds)
		.def("sample_uniform", &StateSpaceSE2::SampleUniform)
		.def("sample_gaussian", &StateSpaceSE2::SampleGaussian)
		.def_readonly("bounds", &StateSpaceSE2::bounds);

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
		.def("initialize_size", &OccupancyMap::InitializeSize)
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
		.def("set_pose", &Obstacle::SetPose)
		.def("get_boundary_grid_cell_position", &Obstacle::GetBoundaryGridCellPosition)
		.def("get_boundary_world_position", &Obstacle::GetBoundaryWorldPosition);

	struct ShapeWrapper : Shape {
		using Shape::Shape;
		virtual void GetGridCellsPosition(const OccupancyMap& a, const Pose2d& b, std::vector<GridCellPosition>& c) override
		{
			PYBIND11_OVERRIDE_PURE(void, Shape, GetGridCellsPosition, a, b, c);
		}
		virtual void GetVerticesPosition(const Pose2d& a, std::vector<Point2d>& b) override
		{
			PYBIND11_OVERRIDE_PURE(void, Shape, GetVerticesPosition, a, b);
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

	struct StateValidatorSE2BaseWrapper : StateValidatorSE2Base {
		using StateValidatorSE2Base::StateValidatorSE2Base;
		virtual bool IsStateValid(const Pose2d& a) override { PYBIND11_OVERRIDE_PURE(bool, StateValidatorSE2Base, IsStateValid, a); }
		virtual bool IsPathValid(const Path<Pose2d>& a, float* b) override { PYBIND11_OVERRIDE_PURE(bool, StateValidatorSE2Base, IsStateValid, a, b); }
	};

	class_<StateValidatorSE2Base, Ref<StateValidatorSE2Base>, StateValidatorSE2BaseWrapper>(m, "StateValidatorSE2Base")
		.def(init<const Ref<StateSpaceSE2>&>())
		.def("is_state_valid", &StateValidatorSE2Base::IsStateValid)
		.def<bool (StateValidatorSE2Base::*)(const PathSE2Base&, float*)>("is_path_valid", &StateValidatorSE2Base::IsPathValid)
		.def_property("state_space", &StateValidatorSE2Base::GetStateSpace, nullptr);

	class_<StateValidatorSE2Free, Ref<StateValidatorSE2Free>, StateValidatorSE2Base>(m, "StateValidatorSE2Free")
		.def(init<const Ref<StateSpaceSE2>&>());

	class_<StateValidatorOccupancyMap, Ref<StateValidatorOccupancyMap>, StateValidatorSE2Base>(m, "StateValidatorOccupancyMap")
		.def(init<const Ref<StateSpaceSE2>&, const Ref<OccupancyMap>&>())
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
