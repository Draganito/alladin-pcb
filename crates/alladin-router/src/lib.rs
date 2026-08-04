//! Alladin router: the algorithmic layer that turns a blocked straight
//! line into a routed path. See `walkaround` module docs for the mapping
//! back to KiCad's `PNS::WALKAROUND` concept.

pub mod astar;
pub mod capsule_walkaround;
pub mod failure;
pub mod grid_astar;
pub mod grid_obstacle;
pub mod optimizer;
mod quadtree_candidates;
pub mod shove;
pub mod walkaround;

pub use astar::{diagnose_failure, find_path_astar};
pub use capsule_walkaround::walkaround_capsule;
pub use failure::{Endpoint, FailureReason};
pub use grid_astar::{find_path_grid, smooth_path};
pub use grid_obstacle::{GridObstacleMap, DEFAULT_GRID_STEP};
pub use optimizer::optimize_path;
pub use shove::try_shove_blockers;
pub use walkaround::{route_single_net, tangent_points, walkaround_single_obstacle};
