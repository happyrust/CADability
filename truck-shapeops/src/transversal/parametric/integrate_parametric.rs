//! Full parametric boolean pipeline: replaces mesh-based `process_one_pair_of_shells`.
//!
//! This module provides `and_parametric`, `or_parametric`, `subtract_parametric`
//! that use parametric SSI instead of mesh collision for intersection detection.

use crate::alternative::Alternative;
use super::super::divide_face;
use super::super::faces_classification::FacesClassification;
use super::super::integrate::{ShapeOpsCurve, ShapeOpsSurface};
use super::super::loops_store::{self, BoundaryWire, LoopsStore, LoopsStoreQuadruple, ShapesOpStatus};
use super::surface_surface_intersection::SSIConfig;
use super::intersect_surfaces;

use rustc_hash::FxHashMap as HashMap;
use truck_base::cgmath64::*;
use truck_geometry::prelude::*;
use truck_meshalgo::prelude::*;
use truck_topology::*;

type PolylineCurve = truck_meshalgo::prelude::PolylineCurve<Point3>;
type AltCurve<C, S> = Alternative<C, IntersectionCurve<PolylineCurve, S, S>>;
type AltShell<C, S> = Shell<Point3, AltCurve<C, S>, S>;

fn altshell_to_shell<C: ShapeOpsCurve<S>, S: ShapeOpsSurface>(
    altshell: &AltShell<C, S>,
    tol: f64,
) -> Option<Shell<Point3, C, S>> {
    altshell.try_mapped(
        |p| Some(*p),
        |c| match c {
            Alternative::FirstType(c) => Some(c.clone()),
            Alternative::SecondType(ic) => {
                let bsp = BSplineCurve::quadratic_approximation(ic, ic.range_tuple(), tol, 100)?;
                Some(IntersectionCurve::new(ic.surface0().clone(), ic.surface1().clone(), bsp).into())
            }
        },
        |s| Some(s.clone()),
    )
}

/// Estimate parameter bounds for a surface from a face's wire vertices.
fn estimate_surface_bounds<C, S>(face: &Face<Point3, C, S>) -> ((f64, f64), (f64, f64))
where
    C: BoundedCurve<Point = Point3>,
    S: ParametricSurface3D + SearchNearestParameter<D2, Point = Point3>,
{
    let surface = face.surface();
    let mut umin = f64::MAX;
    let mut umax = f64::MIN;
    let mut vmin = f64::MAX;
    let mut vmax = f64::MIN;
    for wire in face.boundaries() {
        for vertex in wire.vertex_iter() {
            if let Some((u, v)) = surface.search_nearest_parameter(vertex.point(), None, 50) {
                umin = umin.min(u);
                umax = umax.max(u);
                vmin = vmin.min(v);
                vmax = vmax.max(v);
            }
        }
    }
    let du = (umax - umin).max(1e-6) * 0.1;
    let dv = (vmax - vmin).max(1e-6) * 0.1;
    ((umin - du, umax + du), (vmin - dv, vmax + dv))
}

/// Create loops stores using parametric SSI instead of mesh collision.
fn create_loops_stores_parametric<C, S>(
    geom_shell0: &Shell<Point3, C, S>,
    poly_shell0: &Shell<Point3, PolylineCurve, Option<PolygonMesh>>,
    geom_shell1: &Shell<Point3, C, S>,
    poly_shell1: &Shell<Point3, PolylineCurve, Option<PolygonMesh>>,
    ssi_config: &SSIConfig,
) -> Option<LoopsStoreQuadruple<C>>
where
    C: BoundedCurve<Point = Point3>
        + SearchNearestParameter<D1, Point = Point3>
        + SearchParameter<D1, Point = Point3>
        + Cut<Point = Point3, Vector = Vector3>
        + From<IntersectionCurve<PolylineCurve, S, S>>,
    S: ParametricSurface3D
        + SearchNearestParameter<D2, Point = Point3>
        + SearchParameter<D2, Point = Point3>
        + Clone,
{
    let mut geom_loops_store0: LoopsStore<_, _> = geom_shell0.face_iter().collect();
    let mut poly_loops_store0: LoopsStore<_, _> = poly_shell0.face_iter().collect();
    let mut geom_loops_store1: LoopsStore<_, _> = geom_shell1.face_iter().collect();
    let mut poly_loops_store1: LoopsStore<_, _> = poly_shell1.face_iter().collect();

    let store0_len = geom_loops_store0.len();
    let store1_len = geom_loops_store1.len();

    (0..store0_len)
        .flat_map(move |i| (0..store1_len).map(move |j| (i, j)))
        .try_for_each(|(face_index0, face_index1)| {
            let ori0 = geom_shell0[face_index0].orientation();
            let ori1 = geom_shell1[face_index1].orientation();
            let surface0 = geom_shell0[face_index0].surface();
            let surface1 = geom_shell1[face_index1].surface();

            // Get parameter bounds from face vertices
            let bounds0 = estimate_surface_bounds(&geom_shell0[face_index0]);
            let bounds1 = estimate_surface_bounds(&geom_shell1[face_index1]);

            // PARAMETRIC SSI: dispatches to analytic (plane-plane),
            // signed-distance (plane-curved), or Newton (curved-curved)
            let curves = intersect_surfaces(
                surface0.clone(),
                bounds0,
                surface1.clone(),
                bounds1,
                ssi_config,
            )?;

            for (polyline, intersection_curve) in curves {
                let mut intersection_curve: IntersectionCurve<PolylineCurve, S, S> =
                    intersection_curve.into();
                let status = ShapesOpStatus::from_is_curve(&intersection_curve)?;
                let (status0, status1) = match (ori0, ori1) {
                    (true, true) => (status, status.not()),
                    (true, false) => (status.not(), status.not()),
                    (false, true) => (status, status),
                    (false, false) => (status.not(), status),
                };

                if polyline.front().near(&polyline.back()) {
                    // Closed loop
                    let poly_wire = create_independent_loop(polyline);
                    poly_loops_store0[face_index0]
                        .add_independent_loop(BoundaryWire::new(poly_wire.clone(), status0));
                    poly_loops_store1[face_index1]
                        .add_independent_loop(BoundaryWire::new(poly_wire, status1));
                    let geom_wire = create_independent_loop(intersection_curve);
                    geom_loops_store0[face_index0]
                        .add_independent_loop(BoundaryWire::new(geom_wire.clone(), status0));
                    geom_loops_store1[face_index1]
                        .add_independent_loop(BoundaryWire::new(geom_wire, status1));
                } else {
                    // Open curve: split edges at endpoints
                    let pv0 = Vertex::new(polyline.front());
                    let pv1 = Vertex::new(polyline.back());
                    let gv0 = Vertex::new(polyline.front());
                    let gv1 = Vertex::new(polyline.back());
                    let mut pemap0 = HashMap::default();
                    let mut pemap1 = HashMap::default();
                    let mut gemap0 = HashMap::default();
                    let mut gemap1 = HashMap::default();

                    let idx00 = poly_loops_store0.add_polygon_vertex(face_index0, &pv0, &mut pemap0);
                    if let Some((wi, ei, kind)) = idx00 {
                        geom_loops_store0.add_geom_vertex(
                            (face_index0, wi, ei), &gv0, kind, &surface1, &mut gemap0,
                        )?;
                        *intersection_curve.leader_mut().first_mut().unwrap() = gv0.point();
                    }
                    let idx01 = poly_loops_store0.add_polygon_vertex(face_index0, &pv1, &mut pemap1);
                    if let Some((wi, ei, kind)) = idx01 {
                        geom_loops_store0.add_geom_vertex(
                            (face_index0, wi, ei), &gv1, kind, &surface1, &mut gemap1,
                        )?;
                        *intersection_curve.leader_mut().last_mut().unwrap() = gv1.point();
                    }
                    let idx10 = poly_loops_store1.add_polygon_vertex(face_index1, &pv0, &mut pemap0);
                    if let Some((wi, ei, kind)) = idx10 {
                        geom_loops_store1.add_geom_vertex(
                            (face_index1, wi, ei), &gv0, kind, &surface0, &mut gemap0,
                        )?;
                        *intersection_curve.leader_mut().first_mut().unwrap() = gv0.point();
                    }
                    let idx11 = poly_loops_store1.add_polygon_vertex(face_index1, &pv1, &mut pemap1);
                    if let Some((wi, ei, kind)) = idx11 {
                        geom_loops_store1.add_geom_vertex(
                            (face_index1, wi, ei), &gv1, kind, &surface0, &mut gemap1,
                        )?;
                        *intersection_curve.leader_mut().last_mut().unwrap() = gv1.point();
                    }

                    let pedge = Edge::new(&pv0, &pv1, polyline);
                    let gedge = Edge::new(&gv0, &gv1, intersection_curve.into());
                    poly_loops_store0[face_index0].add_edge(pedge.clone(), status0);
                    geom_loops_store0[face_index0].add_edge(gedge.clone(), status0);
                    poly_loops_store1[face_index1].add_edge(pedge, status1);
                    geom_loops_store1[face_index1].add_edge(gedge, status1);
                }
            }
            Some(())
        })?;

    Some(LoopsStoreQuadruple {
        geom_loops_store0,
        poly_loops_store0,
        geom_loops_store1,
        poly_loops_store1,
    })
}

fn create_independent_loop<P, C, D>(mut poly_curve: C) -> Wire<P, D>
where
    C: Cut<Point = P>,
    D: From<C>,
{
    let (t0, t1) = poly_curve.range_tuple();
    let t = (t0 + t1) / 2.0;
    let poly_curve1 = poly_curve.cut(t);
    let v0 = Vertex::new(poly_curve.front());
    let v1 = Vertex::new(poly_curve1.front());
    let edge0 = Edge::new(&v0, &v1, poly_curve.into());
    let edge1 = Edge::new(&v1, &v0, poly_curve1.into());
    wire![edge0, edge1]
}

/// Process one pair of shells using parametric SSI.
fn process_one_pair_parametric<C: ShapeOpsCurve<S> + BoundedCurve, S: ShapeOpsSurface + SearchParameter<D2, Point = Point3>>(
    shell0: &Shell<Point3, C, S>,
    shell1: &Shell<Point3, C, S>,
    tol: f64,
    ssi_config: &SSIConfig,
) -> Option<[Shell<Point3, C, S>; 2]> {
    nonpositive_tolerance!(tol);

    // Still need polygon shells for "unknown" face classification via ray casting
    let poly_shell0 = shell0.triangulation(tol);
    let poly_shell1 = shell1.triangulation(tol);

    let altshell0: AltShell<C, S> =
        shell0.mapped(|x| *x, |c| Alternative::FirstType(c.clone()), Clone::clone);
    let altshell1: AltShell<C, S> =
        shell1.mapped(|x| *x, |c| Alternative::FirstType(c.clone()), Clone::clone);

    // Use parametric SSI
    let loops_store::LoopsStoreQuadruple {
        geom_loops_store0: loops_store0,
        geom_loops_store1: loops_store1,
        ..
    } = create_loops_stores_parametric(
        &altshell0, &poly_shell0, &altshell1, &poly_shell1, ssi_config,
    )?;

    let mut cls0 = divide_face::divide_faces(&altshell0, &loops_store0, tol)?;
    cls0.integrate_by_component();
    let mut cls1 = divide_face::divide_faces(&altshell1, &loops_store1, tol)?;
    cls1.integrate_by_component();

    let [mut and0, mut or0, unknown0] = cls0.and_or_unknown();
    unknown0.into_iter().try_for_each(|face| {
        let pt = face.boundaries()[0].vertex_iter().next().unwrap().point();
        let dir = hash::take_one_unit(pt);
        let count = poly_shell1.iter().try_fold(0, |count, face| {
            let poly = face.surface()?;
            Some(count + poly.signed_crossing_faces(pt, dir))
        })?;
        if count >= 1 { and0.push(face); } else { or0.push(face); }
        Some(())
    })?;

    let [mut and1, mut or1, unknown1] = cls1.and_or_unknown();
    unknown1.into_iter().try_for_each(|face| {
        let pt = face.boundaries()[0].vertex_iter().next().unwrap().point();
        let dir = hash::take_one_unit(pt);
        let count = poly_shell0.iter().try_fold(0, |count, face| {
            let poly = face.surface()?;
            Some(count + poly.signed_crossing_faces(pt, dir))
        })?;
        if count >= 1 { and1.push(face); } else { or1.push(face); }
        Some(())
    })?;

    and0.append(&mut and1);
    or0.append(&mut or1);
    Some([
        altshell_to_shell(&and0, tol)?,
        altshell_to_shell(&or0, tol)?,
    ])
}

/// AND operation using parametric SSI.
pub fn and_parametric<C: ShapeOpsCurve<S> + BoundedCurve, S: ShapeOpsSurface + SearchParameter<D2, Point = Point3>>(
    solid0: &Solid<Point3, C, S>,
    solid1: &Solid<Point3, C, S>,
    tol: f64,
) -> Option<Solid<Point3, C, S>> {
    let cfg = SSIConfig { tol, ..Default::default() };
    let shell0 = solid0.boundaries().first().unwrap();
    let shell1 = solid1.boundaries().first().unwrap();
    let [mut and_shell, _] = process_one_pair_parametric(shell0, shell1, tol, &cfg)?;
    for shell in solid0.boundaries().iter().skip(1) {
        let [res, _] = process_one_pair_parametric(&and_shell, shell, tol, &cfg)?;
        and_shell = res;
    }
    for shell in solid1.boundaries().iter().skip(1) {
        let [res, _] = process_one_pair_parametric(&and_shell, shell, tol, &cfg)?;
        and_shell = res;
    }
    let components = and_shell.connected_components();
    Solid::try_new(components).ok()
}

/// OR operation using parametric SSI.
pub fn or_parametric<C: ShapeOpsCurve<S> + BoundedCurve, S: ShapeOpsSurface + SearchParameter<D2, Point = Point3>>(
    solid0: &Solid<Point3, C, S>,
    solid1: &Solid<Point3, C, S>,
    tol: f64,
) -> Option<Solid<Point3, C, S>> {
    let cfg = SSIConfig { tol, ..Default::default() };
    let shell0 = solid0.boundaries().first().unwrap();
    let shell1 = solid1.boundaries().first().unwrap();
    let [_, mut or_shell] = process_one_pair_parametric(shell0, shell1, tol, &cfg)?;
    for shell in solid0.boundaries().iter().skip(1) {
        let [_, res] = process_one_pair_parametric(&or_shell, shell, tol, &cfg)?;
        or_shell = res;
    }
    for shell in solid1.boundaries().iter().skip(1) {
        let [_, res] = process_one_pair_parametric(&or_shell, shell, tol, &cfg)?;
        or_shell = res;
    }
    let components = or_shell.connected_components();
    Solid::try_new(components).ok()
}

/// SUBTRACT operation using parametric SSI: solid0 - solid1.
pub fn subtract_parametric<C: ShapeOpsCurve<S> + BoundedCurve, S: ShapeOpsSurface + SearchParameter<D2, Point = Point3>>(
    solid0: &Solid<Point3, C, S>,
    solid1: &Solid<Point3, C, S>,
    tol: f64,
) -> Option<Solid<Point3, C, S>> {
    let mut neg = solid1.clone();
    neg.not();
    and_parametric(solid0, &neg, tol)
}
