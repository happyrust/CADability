//! Parametric-domain exact boolean operations.
//!
//! Replaces mesh-based intersection with direct parametric surface-surface
//! intersection using iso-curve scanning + Newton refinement.

pub mod surface_surface_intersection;
pub mod edge_face_intersection;
pub mod face_splitter;

pub use surface_surface_intersection::{
    parametric_intersection_curves, LinkedIntersectionPoint, SSIConfig,
};
pub use edge_face_intersection::{edge_face_intersections, EdgeFaceIntersection};
pub use face_splitter::{split_face_by_curves, signed_area, point_in_polygon};
