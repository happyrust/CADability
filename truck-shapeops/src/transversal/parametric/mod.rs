//! Parametric-domain exact boolean operations.
//!
//! Replaces mesh-based intersection with direct parametric surface-surface
//! intersection. Uses analytic solutions for plane-plane and plane-curved
//! cases, and Newton-based iso-curve scanning for general curved pairs.

pub mod analytic_intersection;
pub mod surface_surface_intersection;
pub mod edge_face_intersection;
pub mod face_splitter;
pub mod integrate_parametric;

pub use analytic_intersection::{
    is_planar, plane_plane_intersection, plane_surface_intersection,
};
pub use surface_surface_intersection::{
    parametric_intersection_curves, LinkedIntersectionPoint, SSIConfig,
};
pub use edge_face_intersection::{edge_face_intersections, EdgeFaceIntersection};
pub use face_splitter::{split_face_by_curves, signed_area, point_in_polygon};
pub use integrate_parametric::{and_parametric, or_parametric, subtract_parametric};

use truck_base::cgmath64::*;
use truck_geometry::prelude::*;
use truck_meshalgo::prelude::PolylineCurve;

use super::intersection_curve::IntersectionCurveWithParameters;

/// Unified SSI dispatcher: picks the best method based on surface types.
///
/// 1. Plane-Plane → analytic line intersection
/// 2. Plane-Curved → signed-distance bisection on iso-curves
/// 3. Curved-Curved → Newton-based iso-curve scanning
pub fn intersect_surfaces<S0, S1>(
    s0: S0,
    bounds0: ((f64, f64), (f64, f64)),
    s1: S1,
    bounds1: ((f64, f64), (f64, f64)),
    cfg: &SSIConfig,
) -> Option<Vec<(PolylineCurve<Point3>, IntersectionCurveWithParameters<S0, S1>)>>
where
    S0: ParametricSurface3D + SearchNearestParameter<D2, Point = Point3> + SearchParameter<D2, Point = Point3> + Clone,
    S1: ParametricSurface3D + SearchNearestParameter<D2, Point = Point3> + SearchParameter<D2, Point = Point3> + Clone,
{
    let planar0 = is_planar(&s0, bounds0);
    let planar1 = is_planar(&s1, bounds1);

    match (planar0, planar1) {
        (true, true) => {
            // Plane-Plane: analytic
            plane_plane_intersection(&s0, bounds0, &s1, bounds1)
        }
        (true, false) => {
            // Plane-Curved: signed distance bisection
            plane_surface_intersection(&s0, bounds0, &s1, bounds1, cfg.tol, cfg.min_divisions)
        }
        (false, true) => {
            // Curved-Plane: swap and bisect
            let result = plane_surface_intersection(&s1, bounds1, &s0, bounds0, cfg.tol, cfg.min_divisions)?;
            // Need to swap the intersection curves (surface0 <-> surface1)
            let swapped = result.into_iter().map(|(poly, _ic)| {
                let ic2 = IntersectionCurveWithParameters::try_new(s0.clone(), s1.clone(), poly.clone());
                ic2.map(|ic| (poly, ic))
            }).flatten().collect();
            Some(swapped)
        }
        (false, false) => {
            // General: Newton iso-curve scanning
            parametric_intersection_curves(s0, bounds0, s1, bounds1, cfg)
        }
    }
}
