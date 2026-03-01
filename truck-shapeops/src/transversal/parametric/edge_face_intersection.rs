//! Parametric edge-face intersection detection using Newton's method.

use truck_base::cgmath64::*;
use truck_geometry::prelude::*;
use truck_topology::*;

/// An intersection point between an edge and a face.
#[derive(Clone, Debug)]
pub struct EdgeFaceIntersection {
    /// 3D intersection point.
    pub point: Point3,
    /// Parameter on the edge curve.
    pub edge_param: f64,
    /// (u, v) parameters on the face surface.
    pub face_uv: Point2,
    /// Whether the edge is from shell0 (true) or shell1 (false).
    pub edge_from_shell0: bool,
    /// Index of the edge in its shell.
    pub edge_index: usize,
    /// Index of the face in the opposite shell.
    pub face_index: usize,
}

/// Find all intersection points between edges of one shell and faces of another.
pub fn edge_face_intersections<C, S>(
    shell0: &Shell<Point3, C, S>,
    shell1: &Shell<Point3, C, S>,
    tol: f64,
) -> Vec<EdgeFaceIntersection>
where
    C: ParametricCurve3D + BoundedCurve + ParameterDivision1D<Point = Point3> + Clone,
    S: ParametricSurface3D
        + SearchParameter<D2, Point = Point3>
        + SearchNearestParameter<D2, Point = Point3>
        + Clone,
{
    let mut results = Vec::new();
    let edges0 = unique_edges(shell0);
    let edges1 = unique_edges(shell1);
    for (ei, edge) in edges0.iter().enumerate() {
        for (fi, face) in shell1.face_iter().enumerate() {
            find_crossings(edge, &face, ei, fi, true, tol, &mut results);
        }
    }
    for (ei, edge) in edges1.iter().enumerate() {
        for (fi, face) in shell0.face_iter().enumerate() {
            find_crossings(edge, &face, ei, fi, false, tol, &mut results);
        }
    }
    results
}

fn unique_edges<C, S>(shell: &Shell<Point3, C, S>) -> Vec<Edge<Point3, C>> {
    let mut edges = Vec::new();
    let mut seen = std::collections::HashSet::new();
    for face in shell.face_iter() {
        for wire in face.boundaries() {
            for edge in wire.edge_iter() {
                if seen.insert(edge.id()) { edges.push(edge.clone()); }
            }
        }
    }
    edges
}

fn find_crossings<C, S>(
    edge: &Edge<Point3, C>, face: &Face<Point3, C, S>,
    edge_index: usize, face_index: usize, edge_from_shell0: bool,
    tol: f64, results: &mut Vec<EdgeFaceIntersection>,
) where
    C: ParametricCurve3D + BoundedCurve + ParameterDivision1D<Point = Point3> + Clone,
    S: ParametricSurface3D + SearchParameter<D2, Point = Point3>
        + SearchNearestParameter<D2, Point = Point3> + Clone,
{
    let curve = edge.curve();
    let surface = face.surface();
    let (t0, t1) = curve.range_tuple();
    let n = 16;
    let step = (t1 - t0) / n as f64;
    for i in 0..n {
        let t_mid = t0 + step * (i as f64 + 0.5);
        let pt = curve.subs(t_mid);
        let Some((u_hint, v_hint)) = surface.search_nearest_parameter(pt, None, 50) else { continue };
        let Some(((u, v), t)) = truck_geotrait::algo::surface::search_intersection_parameter(
            &surface, (u_hint, v_hint), &curve, t_mid, 50,
        ) else { continue };
        if t < t0 - tol || t > t1 + tol { continue; }
        let ipt = curve.subs(t);
        if !results.iter().any(|r| r.point.distance(ipt) < tol * 10.0) {
            results.push(EdgeFaceIntersection {
                point: ipt, edge_param: t, face_uv: Point2::new(u, v),
                edge_from_shell0, edge_index, face_index,
            });
        }
    }
}
