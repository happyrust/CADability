//! Analytic intersection for special surface pairs (plane-plane, plane-curved).
//!
//! The general parametric Newton SSI fails for plane-plane (rank-deficient Jacobian).
//! This module provides direct analytic solutions for these cases.

use truck_base::cgmath64::*;
use truck_base::tolerance::TOLERANCE;
use truck_geometry::prelude::*;
use truck_meshalgo::prelude::PolylineCurve;

use super::super::intersection_curve::IntersectionCurveWithParameters;

/// Try to compute intersection curve between two planes analytically.
///
/// Two non-parallel planes intersect in a line. Returns the portion of
/// the intersection line that lies within both parameter-domain bounds.
pub fn plane_plane_intersection<S0, S1>(
    surface0: &S0,
    bounds0: ((f64, f64), (f64, f64)),
    surface1: &S1,
    bounds1: ((f64, f64), (f64, f64)),
) -> Option<Vec<(PolylineCurve<Point3>, IntersectionCurveWithParameters<S0, S1>)>>
where
    S0: ParametricSurface3D + SearchNearestParameter<D2, Point = Point3> + SearchParameter<D2, Point = Point3> + Clone,
    S1: ParametricSurface3D + SearchNearestParameter<D2, Point = Point3> + SearchParameter<D2, Point = Point3> + Clone,
{
    let n0 = surface0.normal(0.5, 0.5);
    let n1 = surface1.normal(0.5, 0.5);

    // Check if normals are constant (planar surfaces)
    let n0b = surface0.normal(0.0, 0.0);
    let n1b = surface1.normal(0.0, 0.0);
    if (n0 - n0b).magnitude() > TOLERANCE * 100.0 || (n1 - n1b).magnitude() > TOLERANCE * 100.0 {
        return None; // not planar
    }

    // Direction of intersection line
    let dir = n0.cross(n1);
    if dir.magnitude() < TOLERANCE * 10.0 {
        return Some(Vec::new()); // parallel planes: no intersection (or coincident)
    }
    let dir = dir.normalize();

    // Find a point on the intersection line by solving the system:
    //   n0 · (P - p0) = 0
    //   n1 · (P - p1) = 0
    //   dir · P = 0  (arbitrary constraint to fix the free parameter)
    let p0 = surface0.subs(
        (bounds0.0 .0 + bounds0.0 .1) / 2.0,
        (bounds0.1 .0 + bounds0.1 .1) / 2.0,
    );
    let p1 = surface1.subs(
        (bounds1.0 .0 + bounds1.0 .1) / 2.0,
        (bounds1.1 .0 + bounds1.1 .1) / 2.0,
    );

    let d0 = n0.dot(p0.to_vec());
    let d1 = n1.dot(p1.to_vec());

    // Solve: n0·P = d0, n1·P = d1, dir·P = dir·midpoint
    // where midpoint is the average of the two surface centers (an approximate point near the intersection)
    let midpoint = Point3::from_vec((p0.to_vec() + p1.to_vec()) / 2.0);
    let d2 = dir.dot(midpoint.to_vec());

    // Build matrix with rows [n0, n1, dir]
    // cgmath Matrix3::new is column-major, so we need columns = transposed rows
    let mat = Matrix3::new(
        n0.x, n1.x, dir.x,
        n0.y, n1.y, dir.y,
        n0.z, n1.z, dir.z,
    );
    let rhs = Vector3::new(d0, d1, d2);

    let Some(inv) = mat.invert() else {
        return Some(Vec::new());
    };
    let base_point = Point3::from_vec(inv * rhs);

    // Find the range of the parameter t (along dir) where the line lies within both surfaces
    let mut t_min = f64::MIN;
    let mut t_max = f64::MAX;

    // Sample the surface corner points and project onto the line to get t-range
    for &u in &[bounds0.0 .0, bounds0.0 .1] {
        for &v in &[bounds0.1 .0, bounds0.1 .1] {
            let pt = surface0.subs(u, v);
            let t = (pt - base_point).dot(dir);
            // This gives one extreme, but we need the actual bounds on the plane
        }
    }

    // Better: project the line onto each surface's parameter domain to find valid range
    let t_samples = sample_line_on_surface(surface0, bounds0, base_point, dir, 20);
    let t_samples2 = sample_line_on_surface(surface1, bounds1, base_point, dir, 20);

    if t_samples.is_empty() || t_samples2.is_empty() {
        return Some(Vec::new());
    }

    let t_lo = t_samples
        .first()
        .unwrap()
        .max(*t_samples2.first().unwrap());
    let t_hi = t_samples
        .last()
        .unwrap()
        .min(*t_samples2.last().unwrap());

    if t_hi - t_lo < TOLERANCE {
        return Some(Vec::new());
    }

    // Build polyline along the intersection line
    let n_pts = 10.max(((t_hi - t_lo) / TOLERANCE).min(50.0) as usize);
    let pts: Vec<Point3> = (0..=n_pts)
        .map(|i| {
            let t = t_lo + (t_hi - t_lo) * i as f64 / n_pts as f64;
            base_point + t * dir
        })
        .collect();

    let poly = PolylineCurve(pts.clone());
    if let Some(ic) =
        IntersectionCurveWithParameters::try_new(surface0.clone(), surface1.clone(), poly.clone())
    {
        Some(vec![(PolylineCurve(pts), ic)])
    } else {
        Some(Vec::new())
    }
}

/// Find the range of t where (base_point + t * dir) lies within the surface's bounds.
fn sample_line_on_surface<S>(
    surface: &S,
    bounds: ((f64, f64), (f64, f64)),
    base_point: Point3,
    dir: Vector3,
    n_samples: usize,
) -> Vec<f64>
where
    S: ParametricSurface3D + SearchNearestParameter<D2, Point = Point3> + SearchParameter<D2, Point = Point3>,
{
    // Sample a wide range and find where projections land within bounds
    // First estimate the scale from the surface bounds
    let corner_pts = [
        surface.subs(bounds.0 .0, bounds.1 .0),
        surface.subs(bounds.0 .1, bounds.1 .0),
        surface.subs(bounds.0 .0, bounds.1 .1),
        surface.subs(bounds.0 .1, bounds.1 .1),
    ];
    // Project corners onto the line to estimate the t-range
    let mut t_values: Vec<f64> = corner_pts.iter().map(|pt| (pt - base_point).dot(dir)).collect();
    t_values.sort_by(|a, b| a.partial_cmp(b).unwrap());
    let t_lo = t_values[0] - 0.5;
    let t_hi = t_values[t_values.len() - 1] + 0.5;
    let range = (t_hi - t_lo).max(0.1);

    let mut valid_t = Vec::new();
    let tol = TOLERANCE * 100.0;
    for i in 0..=n_samples {
        let t = t_lo + range * i as f64 / n_samples as f64;
        let pt = base_point + t * dir;
        let uv = surface.search_parameter(pt, None, 10)
            .or_else(|| surface.search_nearest_parameter(pt, None, 10));
        if let Some((u, v)) = uv {
            let on_surface = surface.subs(u, v).distance(pt) < tol * 10.0;
            let in_bounds = u >= bounds.0 .0 - tol && u <= bounds.0 .1 + tol
                         && v >= bounds.1 .0 - tol && v <= bounds.1 .1 + tol;
            if on_surface && in_bounds {
                valid_t.push(t);
            }
        }
    }
    valid_t.sort_by(|a, b| a.partial_cmp(b).unwrap());
    valid_t
}

/// Try to intersect a planar surface with a general parametric surface.
///
/// Projects the general surface's iso-curves onto the plane and finds crossings.
/// This handles the plane-cylinder, plane-sphere, plane-bspline cases.
pub fn plane_surface_intersection<S0, S1>(
    plane_surface: &S0,
    plane_bounds: ((f64, f64), (f64, f64)),
    other_surface: &S1,
    other_bounds: ((f64, f64), (f64, f64)),
    tol: f64,
    n_div: usize,
) -> Option<Vec<(PolylineCurve<Point3>, IntersectionCurveWithParameters<S0, S1>)>>
where
    S0: ParametricSurface3D + SearchNearestParameter<D2, Point = Point3> + SearchParameter<D2, Point = Point3> + Clone,
    S1: ParametricSurface3D + SearchNearestParameter<D2, Point = Point3> + SearchParameter<D2, Point = Point3> + Clone,
{
    // Get the plane's constant normal and a point on it
    let n = plane_surface.normal(0.5, 0.5);
    let p0 = plane_surface.subs(
        (plane_bounds.0 .0 + plane_bounds.0 .1) / 2.0,
        (plane_bounds.1 .0 + plane_bounds.1 .1) / 2.0,
    );
    let d = n.dot(p0.to_vec());

    // Sample points on the other surface and check signed distance to the plane
    let mut intersection_pts = Vec::new();

    let u_step = (other_bounds.0 .1 - other_bounds.0 .0) / n_div as f64;
    let v_step = (other_bounds.1 .1 - other_bounds.1 .0) / n_div as f64;

    // Scan along u-iso-curves
    for ui in 0..=n_div {
        let u = other_bounds.0 .0 + u_step * ui as f64;
        let mut prev_sign = 0.0f64;
        for vi in 0..=n_div {
            let v = other_bounds.1 .0 + v_step * vi as f64;
            let pt = other_surface.subs(u, v);
            let signed_dist = n.dot(pt.to_vec()) - d;

            if vi > 0 && prev_sign * signed_dist < 0.0 {
                // Sign change: intersection in this segment
                let v_prev = other_bounds.1 .0 + v_step * (vi - 1) as f64;
                if let Some(ipt) = bisect_plane_crossing(
                    other_surface, u, v_prev, v, true, &n, d, tol,
                ) {
                    // Check if it's within the plane's bounds
                    if let Some((pu, pv)) = plane_surface.search_parameter(ipt, None, 50) {
                        let pt = plane_bounds;
                        if pu >= pt.0 .0 - tol && pu <= pt.0 .1 + tol
                            && pv >= pt.1 .0 - tol && pv <= pt.1 .1 + tol
                        {
                            if !intersection_pts.iter().any(|p: &Point3| p.distance(ipt) < tol * 10.0) {
                                intersection_pts.push(ipt);
                            }
                        }
                    }
                }
            }
            if signed_dist.abs() < tol {
                if let Some((pu, pv)) = plane_surface.search_parameter(pt, None, 50) {
                    let pb = plane_bounds;
                    if pu >= pb.0 .0 - tol && pu <= pb.0 .1 + tol
                        && pv >= pb.1 .0 - tol && pv <= pb.1 .1 + tol
                    {
                        if !intersection_pts.iter().any(|p: &Point3| p.distance(pt) < tol * 10.0) {
                            intersection_pts.push(pt);
                        }
                    }
                }
            }
            prev_sign = signed_dist;
        }
    }

    // Scan along v-iso-curves
    for vi in 0..=n_div {
        let v = other_bounds.1 .0 + v_step * vi as f64;
        let mut prev_sign = 0.0f64;
        for ui in 0..=n_div {
            let u = other_bounds.0 .0 + u_step * ui as f64;
            let pt = other_surface.subs(u, v);
            let signed_dist = n.dot(pt.to_vec()) - d;

            if ui > 0 && prev_sign * signed_dist < 0.0 {
                let u_prev = other_bounds.0 .0 + u_step * (ui - 1) as f64;
                if let Some(ipt) = bisect_plane_crossing(
                    other_surface, u_prev, v, u, false, &n, d, tol,
                ) {
                    if let Some((pu, pv)) = plane_surface.search_parameter(ipt, None, 50) {
                        let pb = plane_bounds;
                        if pu >= pb.0 .0 - tol && pu <= pb.0 .1 + tol
                            && pv >= pb.1 .0 - tol && pv <= pb.1 .1 + tol
                        {
                            if !intersection_pts.iter().any(|p: &Point3| p.distance(ipt) < tol * 10.0) {
                                intersection_pts.push(ipt);
                            }
                        }
                    }
                }
            }
            prev_sign = signed_dist;
        }
    }

    if intersection_pts.len() < 2 {
        return Some(Vec::new());
    }

    // Sort points to form a polyline (simple greedy nearest-neighbor)
    let mut ordered = vec![intersection_pts.remove(0)];
    while !intersection_pts.is_empty() {
        let last = ordered.last().unwrap();
        let (idx, _) = intersection_pts
            .iter()
            .enumerate()
            .min_by(|(_, a), (_, b)| {
                a.distance(*last)
                    .partial_cmp(&b.distance(*last))
                    .unwrap()
            })
            .unwrap();
        ordered.push(intersection_pts.remove(idx));
    }

    let poly = PolylineCurve(ordered.clone());
    if let Some(ic) = IntersectionCurveWithParameters::try_new(
        plane_surface.clone(),
        other_surface.clone(),
        poly.clone(),
    ) {
        Some(vec![(PolylineCurve(ordered), ic)])
    } else {
        Some(Vec::new())
    }
}

/// Bisect to find where an iso-curve of a surface crosses a plane.
fn bisect_plane_crossing<S: ParametricSurface3D>(
    surface: &S,
    fixed: f64,
    lo: f64,
    hi: f64,
    fixed_is_u: bool,
    normal: &Vector3,
    plane_d: f64,
    tol: f64,
) -> Option<Point3> {
    let eval = |t: f64| -> f64 {
        let pt = if fixed_is_u {
            surface.subs(fixed, t)
        } else {
            surface.subs(t, fixed)
        };
        normal.dot(pt.to_vec()) - plane_d
    };

    let mut a = lo;
    let mut b = hi;
    let mut fa = eval(a);

    for _ in 0..40 {
        let mid = (a + b) / 2.0;
        let fm = eval(mid);
        if fm.abs() < tol {
            return Some(if fixed_is_u {
                surface.subs(fixed, mid)
            } else {
                surface.subs(mid, fixed)
            });
        }
        if fa * fm <= 0.0 {
            b = mid;
        } else {
            a = mid;
            fa = fm;
        }
    }

    let mid = (a + b) / 2.0;
    Some(if fixed_is_u {
        surface.subs(fixed, mid)
    } else {
        surface.subs(mid, fixed)
    })
}

/// Check if a surface has constant normal (i.e., is planar).
pub fn is_planar<S: ParametricSurface3D>(
    surface: &S,
    bounds: ((f64, f64), (f64, f64)),
) -> bool {
    let n_ref = surface.normal(
        (bounds.0 .0 + bounds.0 .1) / 2.0,
        (bounds.1 .0 + bounds.1 .1) / 2.0,
    );
    let corners = [
        (bounds.0 .0, bounds.1 .0),
        (bounds.0 .1, bounds.1 .0),
        (bounds.0 .0, bounds.1 .1),
        (bounds.0 .1, bounds.1 .1),
    ];
    corners
        .iter()
        .all(|&(u, v)| (surface.normal(u, v) - n_ref).magnitude() < TOLERANCE * 100.0)
}

#[cfg(test)]
mod tests {
    use super::*;
    use truck_geometry::specifieds::Plane;

    #[test]
    fn test_is_planar() {
        let p = Plane::new(
            Point3::origin(),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
        );
        assert!(is_planar(&p, ((0.0, 1.0), (0.0, 1.0))));
    }

    #[test]
    fn test_perpendicular_planes() {
        let p0 = Plane::new(
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
        );
        let p1 = Plane::new(
            Point3::new(0.5, 0.0, 0.0),
            Point3::new(0.5, 1.0, 0.0),
            Point3::new(0.5, 0.0, 1.0),
        );
        let bounds0 = ((0.0, 1.0), (0.0, 1.0));
        let bounds1 = ((0.0, 1.0), (0.0, 1.0));
        // Debug: check normals
        let n0 = p0.normal(); let n1 = p1.normal();
        eprintln!("n0={n0:?} n1={n1:?} cross={:?} mag={}", n0.cross(n1), n0.cross(n1).magnitude());
        // Debug: check sample_line_on_surface
        let dir = n0.cross(n1).normalize();
        let c0 = p0.subs(0.5, 0.5); let c1 = p1.subs(0.5, 0.5);
        let d0 = n0.dot(c0.to_vec()); let d1 = n1.dot(c1.to_vec());
        eprintln!("c0={c0:?} c1={c1:?} d0={d0} d1={d1}");
        let mat = Matrix3::from_cols(
            Vector3::new(n0.x, n1.x, dir.x), Vector3::new(n0.y, n1.y, dir.y), Vector3::new(n0.z, n1.z, dir.z),
        ).transpose();
        let base = Point3::from_vec(mat.invert().unwrap() * Vector3::new(d0, d1, 0.0));
        eprintln!("base_point={base:?} dir={dir:?}");
        let t0 = sample_line_on_surface(&p0, bounds0, base, dir, 20);
        let t1 = sample_line_on_surface(&p1, bounds1, base, dir, 20);
        eprintln!("t_samples_p0 ({})={:?}", t0.len(), t0);
        eprintln!("t_samples_p1 ({})={:?}", t1.len(), t1);

        let result = plane_plane_intersection(&p0, bounds0, &p1, bounds1);
        assert!(result.is_some());
        let curves = result.unwrap();
        eprintln!("num_curves={}", curves.len());
        assert!(!curves.is_empty(), "Perpendicular planes should intersect");
        for (poly, _) in &curves {
            for pt in poly.iter() {
                assert!((pt.x - 0.5).abs() < 0.01, "x should be 0.5, got {:?}", pt);
            }
        }
    }

    #[test]
    fn test_parallel_planes_no_intersection() {
        let p0 = Plane::new(
            Point3::origin(),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
        );
        let p1 = Plane::new(
            Point3::new(0.0, 0.0, 1.0),
            Point3::new(1.0, 0.0, 1.0),
            Point3::new(0.0, 1.0, 1.0),
        );
        let result = plane_plane_intersection(
            &p0, ((0.0, 1.0), (0.0, 1.0)),
            &p1, ((0.0, 1.0), (0.0, 1.0)),
        );
        assert!(result.is_some());
        assert!(result.unwrap().is_empty(), "Parallel planes should not intersect");
    }
}
