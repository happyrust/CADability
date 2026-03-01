//! Parametric SSI using iso-curve scanning + Newton refinement.

use truck_base::cgmath64::*;
use truck_base::tolerance::TOLERANCE;
use truck_geometry::prelude::*;
use truck_meshalgo::prelude::PolylineCurve;

use super::super::intersection_curve::IntersectionCurveWithParameters;

/// Configuration for the SSI algorithm.
#[derive(Clone, Debug)]
pub struct SSIConfig {
    /// Tolerance for intersection point detection.
    pub tol: f64,
    /// Number of grid subdivisions per parameter direction.
    pub min_divisions: usize,
    /// Maximum Newton iterations per intersection point.
    pub newton_trials: usize,
}

impl Default for SSIConfig {
    fn default() -> Self {
        Self {
            tol: TOLERANCE * 10.0,
            min_divisions: 16,
            newton_trials: 50,
        }
    }
}

/// A point on the intersection of two surfaces with parametric coordinates on both.
#[derive(Clone, Debug)]
pub struct LinkedIntersectionPoint {
    /// 3D intersection point.
    pub point: Point3,
    /// (u, v) on surface0.
    pub uv0: Point2,
    /// (u, v) on surface1.
    pub uv1: Point2,
    /// Cross product of normals = tangent of intersection curve.
    pub tangent: Vector3,
    /// Which iso-line generated this point.
    pub grid_source: GridSource,
    /// Index of the next point in the chain.
    pub next: Option<usize>,
    /// Index of the previous point in the chain.
    pub prev: Option<usize>,
}

/// Which iso-parametric line produced the intersection point.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum GridSource {
    /// Fixed u on surface0.
    FixedU0(usize),
    /// Fixed v on surface0.
    FixedV0(usize),
    /// Fixed u on surface1.
    FixedU1(usize),
    /// Fixed v on surface1.
    FixedV1(usize),
}

fn newton_refine_intersection<S0: ParametricSurface3D, S1: ParametricSurface3D>(
    s0: &S0,
    s1: &S1,
    hint0: (f64, f64),
    hint1: (f64, f64),
    trials: usize,
    tol: f64,
) -> Option<(Point3, (f64, f64), (f64, f64))> {
    let (mut u0, mut v0, mut u1, mut v1) = (hint0.0, hint0.1, hint1.0, hint1.1);
    for _ in 0..trials {
        let p0 = s0.subs(u0, v0);
        let p1 = s1.subs(u1, v1);
        let r = p0 - p1;
        if r.magnitude() < tol {
            return Some((Point3::from_vec((p0.to_vec() + p1.to_vec()) / 2.0), (u0, v0), (u1, v1)));
        }
        let cols = [s0.uder(u0, v0), s0.vder(u0, v0), -s1.uder(u1, v1), -s1.vder(u1, v1)];
        let mut jtj = [[0.0f64; 4]; 4];
        let mut jtr = [0.0f64; 4];
        for i in 0..4 {
            for j in 0..4 {
                jtj[i][j] = cols[i].dot(cols[j]);
            }
            jtr[i] = -cols[i].dot(r);
        }
        let dx = solve_4x4(&jtj, &jtr)?;
        u0 += dx[0]; v0 += dx[1]; u1 += dx[2]; v1 += dx[3];
    }
    None
}

fn solve_4x4(a: &[[f64; 4]; 4], b: &[f64; 4]) -> Option<[f64; 4]> {
    let mut m = [[0.0f64; 5]; 4];
    for i in 0..4 {
        for j in 0..4 { m[i][j] = a[i][j]; }
        m[i][4] = b[i];
    }
    for col in 0..4 {
        let (mut max_row, mut max_val) = (col, m[col][col].abs());
        for row in (col + 1)..4 {
            if m[row][col].abs() > max_val { max_val = m[row][col].abs(); max_row = row; }
        }
        if max_val < 1e-15 { return None; }
        m.swap(col, max_row);
        let pivot = m[col][col];
        for row in (col + 1)..4 {
            let f = m[row][col] / pivot;
            for k in col..5 { m[row][k] -= f * m[col][k]; }
        }
    }
    let mut x = [0.0f64; 4];
    for i in (0..4).rev() {
        x[i] = m[i][4];
        for j in (i + 1)..4 { x[i] -= m[i][j] * x[j]; }
        if m[i][i].abs() < 1e-15 { return None; }
        x[i] /= m[i][i];
    }
    Some(x)
}

fn intersect_isocurve<S0, S1>(
    s0: &S0, s1: &S1, fixed: f64, is_fixed_u: bool,
    vary_range: (f64, f64), bounds1: ((f64, f64), (f64, f64)), cfg: &SSIConfig,
) -> Vec<(Point3, Point2, Point2)>
where
    S0: ParametricSurface3D,
    S1: ParametricSurface3D + SearchNearestParameter<D2, Point = Point3>,
{
    let n = cfg.min_divisions;
    let step = (vary_range.1 - vary_range.0) / n as f64;
    let hint1 = ((bounds1.0 .0 + bounds1.0 .1) / 2.0, (bounds1.1 .0 + bounds1.1 .1) / 2.0);
    let mut results = Vec::new();
    for i in 0..=n {
        let t = vary_range.0 + step * i as f64;
        let uv0 = if is_fixed_u { (fixed, t) } else { (t, fixed) };
        let pt = s0.subs(uv0.0, uv0.1);
        let Some(proj) = s1.search_nearest_parameter(pt, hint1, cfg.newton_trials) else { continue };
        if pt.distance(s1.subs(proj.0, proj.1)) < cfg.tol * 1000.0 {
            if let Some((rp, ruv0, ruv1)) = newton_refine_intersection(s0, s1, uv0, (proj.0, proj.1), cfg.newton_trials, cfg.tol) {
                let in0 = if is_fixed_u { ruv0.1 >= vary_range.0 - cfg.tol && ruv0.1 <= vary_range.1 + cfg.tol }
                           else { ruv0.0 >= vary_range.0 - cfg.tol && ruv0.0 <= vary_range.1 + cfg.tol };
                let in1 = ruv1.0 >= bounds1.0 .0 - cfg.tol && ruv1.0 <= bounds1.0 .1 + cfg.tol
                        && ruv1.1 >= bounds1.1 .0 - cfg.tol && ruv1.1 <= bounds1.1 .1 + cfg.tol;
                if in0 && in1 && !results.iter().any(|(p, _, _): &(Point3, Point2, Point2)| p.distance(rp) < cfg.tol * 10.0) {
                    results.push((rp, Point2::new(ruv0.0, ruv0.1), Point2::new(ruv1.0, ruv1.1)));
                }
            }
        }
    }
    results
}

fn link_points(pts: &mut Vec<LinkedIntersectionPoint>, tol: f64) {
    let n = pts.len();
    for i in 0..n {
        if pts[i].next.is_some() { continue; }
        let mut best = (None, f64::MAX);
        for j in 0..n {
            if i == j || pts[j].prev.is_some() || pts[i].grid_source == pts[j].grid_source { continue; }
            let d = pts[i].point.distance(pts[j].point);
            if d < best.1 && d < tol * 1000.0 { best = (Some(j), d); }
        }
        if let Some(j) = best.0 { pts[i].next = Some(j); pts[j].prev = Some(i); }
    }
}

fn extract_chains(pts: &[LinkedIntersectionPoint]) -> Vec<Vec<usize>> {
    let mut vis = vec![false; pts.len()];
    let mut chains = Vec::new();
    for s in 0..pts.len() {
        if vis[s] || pts[s].prev.is_some() { continue; }
        let mut chain = vec![s]; vis[s] = true;
        let mut cur = s;
        while let Some(nxt) = pts[cur].next {
            if vis[nxt] { break; }
            chain.push(nxt); vis[nxt] = true; cur = nxt;
        }
        if chain.len() >= 2 { chains.push(chain); }
    }
    for s in 0..pts.len() {
        if vis[s] { continue; }
        let mut chain = vec![s]; vis[s] = true;
        let mut cur = s;
        while let Some(nxt) = pts[cur].next {
            if vis[nxt] { break; }
            chain.push(nxt); vis[nxt] = true; cur = nxt;
        }
        if chain.len() >= 2 { chains.push(chain); }
    }
    chains
}

/// Compute parametric intersection curves between two surfaces.
pub fn parametric_intersection_curves<S0, S1>(
    s0: S0, bounds0: ((f64, f64), (f64, f64)),
    s1: S1, bounds1: ((f64, f64), (f64, f64)),
    cfg: &SSIConfig,
) -> Option<Vec<(PolylineCurve<Point3>, IntersectionCurveWithParameters<S0, S1>)>>
where
    S0: ParametricSurface3D + SearchNearestParameter<D2, Point = Point3> + Clone,
    S1: ParametricSurface3D + SearchNearestParameter<D2, Point = Point3> + Clone,
{
    let n = cfg.min_divisions;
    let mk = |range: (f64, f64)| -> Vec<f64> { (0..=n).map(|i| range.0 + (range.1 - range.0) * i as f64 / n as f64).collect() };
    let (u0s, v0s, u1s, v1s) = (mk(bounds0.0), mk(bounds0.1), mk(bounds1.0), mk(bounds1.1));

    let mut all = Vec::new();
    let mut push = |pts: Vec<(Point3, Point2, Point2)>, src: GridSource, s0r: &S0, s1r: &S1| {
        for (pt, uv0, uv1) in pts {
            all.push(LinkedIntersectionPoint {
                point: pt, uv0, uv1,
                tangent: s0r.normal(uv0.x, uv0.y).cross(s1r.normal(uv1.x, uv1.y)),
                grid_source: src, prev: None, next: None,
            });
        }
    };
    for (i, &u) in u0s.iter().enumerate() { push(intersect_isocurve(&s0, &s1, u, true, bounds0.1, bounds1, cfg), GridSource::FixedU0(i), &s0, &s1); }
    for (i, &v) in v0s.iter().enumerate() { push(intersect_isocurve(&s0, &s1, v, false, bounds0.0, bounds1, cfg), GridSource::FixedV0(i), &s0, &s1); }
    for (i, &u) in u1s.iter().enumerate() {
        let pts = intersect_isocurve(&s1, &s0, u, true, bounds1.1, bounds0, cfg);
        for (pt, uv1, uv0) in pts {
            all.push(LinkedIntersectionPoint {
                point: pt, uv0, uv1,
                tangent: s0.normal(uv0.x, uv0.y).cross(s1.normal(uv1.x, uv1.y)),
                grid_source: GridSource::FixedU1(i), prev: None, next: None,
            });
        }
    }
    for (i, &v) in v1s.iter().enumerate() {
        let pts = intersect_isocurve(&s1, &s0, v, false, bounds1.0, bounds0, cfg);
        for (pt, uv1, uv0) in pts {
            all.push(LinkedIntersectionPoint {
                point: pt, uv0, uv1,
                tangent: s0.normal(uv0.x, uv0.y).cross(s1.normal(uv1.x, uv1.y)),
                grid_source: GridSource::FixedV1(i), prev: None, next: None,
            });
        }
    }
    if all.is_empty() { return Some(Vec::new()); }

    // Deduplicate
    let mut i = 0;
    while i < all.len() {
        let mut j = i + 1;
        while j < all.len() {
            if all[i].point.distance(all[j].point) < cfg.tol * 2.0 && all[i].grid_source != all[j].grid_source {
                all[i].point = Point3::from_vec((all[i].point.to_vec() + all[j].point.to_vec()) / 2.0);
                all.remove(j);
            } else { j += 1; }
        }
        i += 1;
    }

    link_points(&mut all, cfg.tol);
    let chains = extract_chains(&all);
    let mut result = Vec::new();
    for chain in chains {
        if chain.len() < 2 { continue; }
        let poly = PolylineCurve(chain.iter().map(|&i| all[i].point).collect());
        if let Some(ic) = IntersectionCurveWithParameters::try_new(s0.clone(), s1.clone(), poly.clone()) {
            result.push((poly, ic));
        }
    }
    Some(result)
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;
    use truck_geometry::specifieds::Sphere;

    #[test]
    fn two_spheres_ssi() {
        let s0 = Sphere::new(Point3::new(0.0, 0.0, 0.5), f64::sqrt(1.25));
        let s1 = Sphere::new(Point3::new(0.0, 0.0, -0.5), f64::sqrt(1.25));
        let cfg = SSIConfig { min_divisions: 12, tol: 1e-4, ..Default::default() };
        let result = parametric_intersection_curves(s0, ((0.0, PI), (-PI, PI)), s1, ((0.0, PI), (-PI, PI)), &cfg);
        assert!(result.is_some());
        let curves = result.unwrap();
        if !curves.is_empty() {
            for (poly, _) in &curves {
                for pt in poly.iter() {
                    let r = (pt.x * pt.x + pt.y * pt.y).sqrt();
                    assert!((r - 1.0).abs() < 0.15, "r={r} should be ~1.0");
                    assert!(pt.z.abs() < 0.15, "z={} should be ~0", pt.z);
                }
            }
        }
    }

    #[test]
    fn two_parabolas_ssi() {
        #[rustfmt::skip]
        let c0 = vec![
            vec![Point3::new(-1.0,-1.0,3.0), Point3::new(-1.0,0.0,-1.0), Point3::new(-1.0,1.0,3.0)],
            vec![Point3::new(0.0,-1.0,-1.0), Point3::new(0.0,0.0,-5.0), Point3::new(0.0,1.0,-1.0)],
            vec![Point3::new(1.0,-1.0,3.0), Point3::new(1.0,0.0,-1.0), Point3::new(1.0,1.0,3.0)],
        ];
        #[rustfmt::skip]
        let c1 = vec![
            vec![Point3::new(-1.0,-1.0,-3.0), Point3::new(-1.0,0.0,1.0), Point3::new(-1.0,1.0,-3.0)],
            vec![Point3::new(0.0,-1.0,1.0), Point3::new(0.0,0.0,5.0), Point3::new(0.0,1.0,1.0)],
            vec![Point3::new(1.0,-1.0,-3.0), Point3::new(1.0,0.0,1.0), Point3::new(1.0,1.0,-3.0)],
        ];
        let k = KnotVec::bezier_knot(2);
        let s0 = BSplineSurface::new((k.clone(), k.clone()), c0);
        let s1 = BSplineSurface::new((k.clone(), k.clone()), c1);
        let cfg = SSIConfig { min_divisions: 16, tol: 1e-4, ..Default::default() };
        let curves = parametric_intersection_curves(s0, ((0.0, 1.0), (0.0, 1.0)), s1, ((0.0, 1.0), (0.0, 1.0)), &cfg).unwrap();
        assert!(!curves.is_empty(), "Should find intersection curves");
    }
}
