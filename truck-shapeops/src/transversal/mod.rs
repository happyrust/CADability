mod divide_face;
mod faces_classification;
mod integrate;
mod intersection_curve;
mod loops_store;
/// Parametric-domain exact boolean operations module.
pub mod parametric;
mod polyline_construction;
pub use integrate::{and, or, subtract, ShapeOpsCurve, ShapeOpsSurface};
