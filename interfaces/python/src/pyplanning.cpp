#include <pybind11/pybind11.h>
#include <pybind11/operators.h>
#include <pybind11/stl.h>
#include <pybind11/functional.h>

#include "core/base.h"
#include "algo/path_planner.h"
#include "algo/hybrid_a_star.h"
#include "algo/planar_a_star_grid.h"

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

#include "utils/grid.h"

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
		virtual std::vector<Pose2d> GetPath() const override { PYBIND11_OVERRIDE_PURE(std::vector<Pose2d>, PlanarPathPlanner, GetPath); }
	};

	class_<PlanarPathPlanner, PlanarPathPlannerWrapper>(m, "PlanarPathPlanner")
		.def(init<>())
		.def("search_path", &PlanarPathPlanner::SearchPath)
		.def("get_path", &PlanarPathPlanner::GetPath)
		.def("set_init_state", &PlanarPathPlanner::SetInitState)
		.def("set_goal_state", &PlanarPathPlanner::SetGoalState);

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

	class_<HybridAStar, PlanarPathPlanner>(m, "HybridAStar")
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

	struct GridPathPlannerWrapper : GridPathPlanner {
		using GridPathPlanner::GridPathPlanner;
		virtual Status SearchPath() override { PYBIND11_OVERRIDE_PURE(Status, GridPathPlanner, SearchPath); }
		virtual std::vector<GridCellPosition> GetPath() const override { PYBIND11_OVERRIDE_PURE(std::vector<GridCellPosition>, GridPathPlanner, GetPath); }
	};

	class_<GridPathPlanner, GridPathPlannerWrapper>(m, "GridPathPlanner")
		.def(init<>())
		.def("search_path", &GridPathPlanner::SearchPath)
		.def("get_path", &GridPathPlanner::GetPath)
		.def("set_init_state", &GridPathPlanner::SetInitState)
		.def("set_goal_state", &GridPathPlanner::SetGoalState);

	struct PyPlanarAStarGrid : public PlanarAStarGrid {
		using PlanarAStarGrid::PlanarAStarGrid;
		const std::unordered_set<GridCellPosition, std::hash<GridCellPosition>, std::equal_to<GridCellPosition>>& GetExploredStates() const { return m_explored; }
	};

	class_<PyPlanarAStarGrid, GridPathPlanner>(m, "PlanarAStarGrid")
		.def(init<>())
		.def<bool (PlanarAStarGrid::*)(
			const Ref<OccupancyMap>&,
			const std::function<double(const GridCellPosition&, const GridCellPosition&)>&,
			const std::function<double(const GridCellPosition&, const GridCellPosition&)>&)>("initialize", &PlanarAStarGrid::Initialize)
		.def("get_explored_states", &PyPlanarAStarGrid::GetExploredStates)
		.def("get_optimal_cost", &PlanarAStarGrid::GetOptimalCost);

	struct PyPlanarBidirectionalAStarGrid : public PlanarBidirectionalAStarGrid {
		using PlanarBidirectionalAStarGrid::PlanarBidirectionalAStarGrid;
		std::tuple<std::unordered_set<GridCellPosition, std::hash<GridCellPosition>, std::equal_to<GridCellPosition>>,
			std::unordered_set<GridCellPosition, std::hash<GridCellPosition>, std::equal_to<GridCellPosition>>>
		GetExploredStates()
		{
			std::unordered_set<GridCellPosition, std::hash<GridCellPosition>, std::equal_to<GridCellPosition>> fExplored, rExplored;
			auto [fExploredMap, rExploredMap] = PlanarBidirectionalAStarGrid::GetExploredStates();
			for (auto kv : fExploredMap)
				fExplored.insert(kv.first);
			for (auto kv : rExploredMap)
				rExplored.insert(kv.first);
			return std::make_tuple(fExplored, rExplored);
		}
	};

	class_<PyPlanarBidirectionalAStarGrid, GridPathPlanner>(m, "PlanarBidirectionalAStarGrid")
		.def(init<>())
		.def<bool (PlanarBidirectionalAStarGrid::*)(
			const Ref<OccupancyMap>&,
			const std::function<double(const GridCellPosition&, const GridCellPosition&)>&,
			const std::function<double(const GridCellPosition&, const GridCellPosition&)>&)>("initialize", &PlanarBidirectionalAStarGrid::Initialize)
		.def("get_explored_states", &PyPlanarBidirectionalAStarGrid::GetExploredStates)
		.def("get_optimal_cost", &PlanarBidirectionalAStarGrid::GetOptimalCost);

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
		.def_readwrite("col", &GridCellPosition::col);

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
	};

	class_<PlanarPath, Ref<PlanarPath>, PlanarPathWrapper>(m, "PlanarPath")
		.def(init<>())
		.def("get_initial_state", &PlanarPath::GetInitialState)
		.def("get_final_state", &PlanarPath::GetFinalState)
		.def<Pose2d (PlanarPath::*)(double) const>("interpolate", &PlanarPath::Interpolate)
		.def<std::vector<Pose2d> (PlanarPath::*)(const std::vector<double>&) const>("interpolate", &PlanarPath::Interpolate)
		.def("truncate", &PlanarPath::Truncate)
		.def("get_length", &PlanarPath::GetLength);

	struct PlanarNonHolonomicPathWrapper : PlanarPathWrapper, virtual PlanarNonHolonomicPath {
		using PlanarNonHolonomicPath::PlanarNonHolonomicPath;
		virtual Direction GetDirection(double ratio) const override { PYBIND11_OVERRIDE_PURE(Direction, PlanarNonHolonomicPath, GetDirection, ratio); }
		virtual std::set<double> GetCuspPointRatios() const override { PYBIND11_OVERRIDE(std::set<double>, PlanarNonHolonomicPath, GetCuspPointRatios); }
	};

	class_<PlanarNonHolonomicPath, Ref<PlanarNonHolonomicPath>, PlanarPath, PlanarNonHolonomicPathWrapper>(m, "PlanarNonHolonomicPath")
		.def("get_direction", &PlanarNonHolonomicPath::GetDirection)
		.def("get_cusp_point_ratios", &PlanarNonHolonomicPath::GetCuspPointRatios);

	class_<PathReedsShepp, Ref<PathReedsShepp>, PlanarNonHolonomicPath>(m, "PathReedsShepp")
		.def(init<const Pose2d&, ReedsShepp::PathSegment, double>())
		.def_property("min_turning_radius", &PathReedsShepp::GetMinTurningRadius, nullptr);

	class_<PathConstantSteer, Ref<PathConstantSteer>, PlanarNonHolonomicPath>(m, "PathConstantSteer")
		.def(init<const Ref<KinematicBicycleModel>&, const Pose2d&, double, double, Direction>())
		.def_property("steering", &PathConstantSteer::GetSteeringAngle, nullptr);

	class_<KinematicBicycleModel, Ref<KinematicBicycleModel>>(m, "KinematicBicycleModel")
		.def(init<double, double>());

	struct PathConnectionWrapper : PlanarPathConnection {
		using PlanarPathConnection::PlanarPathConnection;
		virtual Ref<Path<Pose2d>> Connect(const Pose2d& from, const Pose2d& to) override { PYBIND11_OVERRIDE_PURE(Ref<Path<Pose2d>>, PlanarPathConnection, Connect, from, to); }
	};

	class_<PlanarPathConnection, Ref<PlanarPathConnection>, PathConnectionWrapper>(m, "PlanarPathConnection")
		.def(init<>())
		.def("connect", &PlanarPathConnection::Connect);

	class_<ReedsSheppConnection, Ref<ReedsSheppConnection>, PlanarPathConnection>(m, "ReedsSheppConnection")
		.def(init<>());

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
		.def("sample_gaussian", &PlanarStateSpace::SampleGaussian)
		.def_readonly("bounds", &PlanarStateSpace::bounds);

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
