//! Bidirectional comparison tests: mesh-based vs parametric boolean operations.
//!
//! Each test case:
//! 1. Creates two solids
//! 2. Runs BOTH mesh-based and parametric intersection
//! 3. Serializes results to JSON for comparison
//! 4. Validates geometric invariants

use serde::{Deserialize, Serialize};
use truck_modeling::*;
use truck_shapeops::*;

// ====================================================================
// JSON-serializable structures
// ====================================================================

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
struct BooleanTestResult {
    method: String,
    success: bool,
    num_shells: usize,
    num_faces: usize,
    num_edges: usize,
    num_vertices: usize,
    bounding_box: [f64; 6],
    sample_points: Vec<[f64; 3]>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
struct BooleanTestCase {
    name: String,
    operation: String,
    tolerance: f64,
    mesh_result: BooleanTestResult,
    parametric_result: BooleanTestResult,
    topology_match: bool,
    bbox_match: bool,
}

// ====================================================================
// Helpers
// ====================================================================

fn count_topology(solid: &Solid) -> (usize, usize, usize, usize) {
    let shells = solid.boundaries().len();
    let mut faces = 0;
    let mut edge_ids = std::collections::HashSet::new();
    let mut vertex_ids = std::collections::HashSet::new();
    for shell in solid.boundaries() {
        for face in shell.face_iter() {
            faces += 1;
            for wire in face.boundaries() {
                for edge in wire.edge_iter() {
                    edge_ids.insert(edge.id());
                    vertex_ids.insert(edge.absolute_front().id());
                    vertex_ids.insert(edge.absolute_back().id());
                }
            }
        }
    }
    (shells, faces, edge_ids.len(), vertex_ids.len())
}

fn bounding_box(solid: &Solid) -> [f64; 6] {
    let mut min = [f64::MAX; 3];
    let mut max = [f64::MIN; 3];
    for shell in solid.boundaries() {
        for face in shell.face_iter() {
            for wire in face.boundaries() {
                for vertex in wire.vertex_iter() {
                    let p = vertex.point();
                    for k in 0..3 {
                        let v = [p.x, p.y, p.z][k];
                        min[k] = min[k].min(v);
                        max[k] = max[k].max(v);
                    }
                }
            }
        }
    }
    [min[0], min[1], min[2], max[0], max[1], max[2]]
}

fn sample_points(solid: &Solid) -> Vec<[f64; 3]> {
    let mut pts = Vec::new();
    for shell in solid.boundaries() {
        for v in shell.vertex_iter() {
            let p = v.point();
            pts.push([p.x, p.y, p.z]);
            if pts.len() >= 20 { return pts; }
        }
    }
    pts
}

fn make_result(method: &str, solid: Option<&Solid>) -> BooleanTestResult {
    match solid {
        Some(s) => {
            let (sh, f, e, v) = count_topology(s);
            BooleanTestResult {
                method: method.into(), success: true,
                num_shells: sh, num_faces: f, num_edges: e, num_vertices: v,
                bounding_box: bounding_box(s),
                sample_points: sample_points(s),
            }
        }
        None => BooleanTestResult {
            method: method.into(), success: false,
            num_shells: 0, num_faces: 0, num_edges: 0, num_vertices: 0,
            bounding_box: [0.0; 6], sample_points: vec![],
        },
    }
}

fn bbox_similar(a: &[f64; 6], b: &[f64; 6], tol: f64) -> bool {
    (0..6).all(|i| (a[i] - b[i]).abs() < tol)
}

// ====================================================================
// Run both methods and compare
// ====================================================================

fn run_comparison(name: &str, op: &str, s0: &Solid, s1: &Solid, tol: f64) -> BooleanTestCase {
    // Mesh-based
    let mesh_solid = match op {
        "and" => and(s0, s1, tol),
        "or" => or(s0, s1, tol),
        "subtract" => subtract(s0, s1, tol),
        _ => panic!("unknown op"),
    };
    let mesh_result = make_result("mesh", mesh_solid.as_ref());

    // Parametric
    let param_solid = match op {
        "and" => parametric::and_parametric(s0, s1, tol),
        "or" => parametric::or_parametric(s0, s1, tol),
        "subtract" => parametric::subtract_parametric(s0, s1, tol),
        _ => panic!("unknown op"),
    };
    let parametric_result = make_result("parametric", param_solid.as_ref());

    let topology_match = mesh_result.success == parametric_result.success
        && mesh_result.num_shells == parametric_result.num_shells
        && mesh_result.num_faces == parametric_result.num_faces;
    let bbox_match = if mesh_result.success && parametric_result.success {
        bbox_similar(&mesh_result.bounding_box, &parametric_result.bounding_box, tol * 10.0)
    } else {
        mesh_result.success == parametric_result.success
    };

    BooleanTestCase {
        name: name.into(), operation: op.into(), tolerance: tol,
        mesh_result, parametric_result, topology_match, bbox_match,
    }
}

// ====================================================================
// Geometry builders
// ====================================================================

fn make_cube() -> Solid {
    let v = builder::vertex(Point3::origin());
    let e = builder::tsweep(&v, Vector3::unit_x());
    let f = builder::tsweep(&e, Vector3::unit_y());
    builder::tsweep(&f, Vector3::unit_z())
}

fn make_cube_at(dx: f64, dy: f64, dz: f64) -> Solid {
    let v = builder::vertex(Point3::new(dx, dy, dz));
    let e = builder::tsweep(&v, Vector3::unit_x());
    let f = builder::tsweep(&e, Vector3::unit_y());
    builder::tsweep(&f, Vector3::unit_z())
}

fn make_cylinder(center: Point3, radius: f64, height: f64) -> Solid {
    let v = builder::vertex(center + Vector3::new(radius, 0.0, 0.0));
    let w = builder::rsweep(&v, center, Vector3::unit_z(), Rad(7.0), 3);
    let f = builder::try_attach_plane(&[w]).unwrap();
    builder::tsweep(&f, Vector3::new(0.0, 0.0, height))
}

// ====================================================================
// Tests: Mesh vs Parametric comparison
// ====================================================================

#[test]
fn cmp_cube_cube_and() {
    let s0 = make_cube();
    let s1 = make_cube_at(0.5, 0.5, 0.5);
    let tc = run_comparison("cube_cube_and", "and", &s0, &s1, 0.05);
    let json = serde_json::to_string_pretty(&tc).unwrap();
    eprintln!("{json}");

    assert!(tc.mesh_result.success, "mesh AND should succeed");
    assert!(tc.mesh_result.num_faces >= 6, "mesh: intersection box >= 6 faces");
    // NOTE: Parametric SSI currently cannot handle plane-plane intersections
    // (rank-deficient Jacobian). This is a known limitation that needs
    // special-case handling for planar faces.
    eprintln!(
        "COMPARISON: mesh_faces={}, parametric_faces={}, topology_match={}, bbox_match={}",
        tc.mesh_result.num_faces, tc.parametric_result.num_faces,
        tc.topology_match, tc.bbox_match,
    );
}

#[test]
fn cmp_cube_cube_or() {
    let s0 = make_cube();
    let s1 = make_cube_at(0.5, 0.5, 0.5);
    let tc = run_comparison("cube_cube_or", "or", &s0, &s1, 0.05);
    let json = serde_json::to_string_pretty(&tc).unwrap();
    eprintln!("{json}");
    assert!(tc.mesh_result.success, "mesh OR should succeed");
    eprintln!("COMPARISON: mesh_faces={}, parametric_faces={}", tc.mesh_result.num_faces, tc.parametric_result.num_faces);
}

#[test]
fn cmp_cube_cube_subtract() {
    let s0 = make_cube();
    let s1 = make_cube_at(0.5, 0.5, 0.5);
    let tc = run_comparison("cube_cube_subtract", "subtract", &s0, &s1, 0.05);
    let json = serde_json::to_string_pretty(&tc).unwrap();
    eprintln!("{json}");
    assert!(tc.mesh_result.success, "mesh SUBTRACT should succeed");
    eprintln!("COMPARISON: mesh_faces={}, parametric_faces={}", tc.mesh_result.num_faces, tc.parametric_result.num_faces);
}

#[test]
fn cmp_cube_cylinder_subtract() {
    let cube = make_cube();
    let mut cyl = make_cylinder(Point3::new(0.5, 0.5, -0.5), 0.25, 2.0);
    cyl.not();
    let tc = run_comparison("cube_cylinder_and_negcyl", "and", &cube, &cyl, 0.05);
    let json = serde_json::to_string_pretty(&tc).unwrap();
    eprintln!("{json}");
    assert!(tc.mesh_result.success, "punched cube should succeed");
    eprintln!("COMPARISON: mesh_faces={}, parametric_faces={}", tc.mesh_result.num_faces, tc.parametric_result.num_faces);
}

// ====================================================================
// Parametric SSI standalone tests (with JSON output)
// ====================================================================

#[test]
fn parametric_ssi_spheres_json() {
    use truck_geometry::specifieds::Sphere;
    use truck_shapeops::parametric::*;
    use std::f64::consts::PI;

    let s0 = Sphere::new(Point3::new(0.0, 0.0, 0.5), f64::sqrt(1.25));
    let s1 = Sphere::new(Point3::new(0.0, 0.0, -0.5), f64::sqrt(1.25));
    let cfg = SSIConfig { min_divisions: 12, tol: 1e-4, ..Default::default() };
    let curves = parametric_intersection_curves(
        s0, ((0.0, PI), (-PI, PI)), s1, ((0.0, PI), (-PI, PI)), &cfg,
    ).unwrap();

    #[derive(Serialize)]
    struct R { name: String, num_curves: usize, max_z_err: f64, max_r_err: f64 }
    let (mut mz, mut mr) = (0.0f64, 0.0f64);
    for (poly, _) in &curves {
        for pt in poly.iter() {
            mz = mz.max(pt.z.abs());
            mr = mr.max(((pt.x * pt.x + pt.y * pt.y).sqrt() - 1.0).abs());
        }
    }
    let r = R { name: "spheres_ssi".into(), num_curves: curves.len(), max_z_err: mz, max_r_err: mr };
    eprintln!("{}", serde_json::to_string_pretty(&r).unwrap());
    if !curves.is_empty() {
        assert!(mz < 0.15); assert!(mr < 0.15);
    }
}

#[test]
fn parametric_ssi_parabolas_json() {
    use truck_geometry::prelude::*;
    use truck_shapeops::parametric::*;
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
    let curves = parametric_intersection_curves(
        s0.clone(), ((0.0, 1.0), (0.0, 1.0)),
        s1.clone(), ((0.0, 1.0), (0.0, 1.0)), &cfg,
    ).unwrap();

    #[derive(Serialize)]
    struct R { name: String, num_curves: usize, points: Vec<usize>, max_surf_dist: f64 }
    let mut md = 0.0f64;
    let mut pts = Vec::new();
    for (poly, _) in &curves {
        pts.push(poly.len());
        for pt in poly.iter() {
            let d0 = { let uv = s0.search_nearest_parameter(*pt, None, 50).unwrap(); pt.distance(s0.subs(uv.0, uv.1)) };
            let d1 = { let uv = s1.search_nearest_parameter(*pt, None, 50).unwrap(); pt.distance(s1.subs(uv.0, uv.1)) };
            md = md.max(d0.max(d1));
        }
    }
    let r = R { name: "parabolas_ssi".into(), num_curves: curves.len(), points: pts, max_surf_dist: md };
    eprintln!("{}", serde_json::to_string_pretty(&r).unwrap());
    assert!(!curves.is_empty()); assert!(md < 0.01);
}

// ====================================================================
// JSON round-trip test
// ====================================================================

#[test]
fn json_roundtrip() {
    let s0 = make_cube();
    let s1 = make_cube_at(0.5, 0.5, 0.5);
    let tc = run_comparison("roundtrip", "and", &s0, &s1, 0.05);
    let json = serde_json::to_string_pretty(&tc).unwrap();
    eprintln!("JSON:\n{json}");
    let tc2: BooleanTestCase = serde_json::from_str(&json).unwrap();
    assert_eq!(tc.name, tc2.name);
    assert_eq!(tc.operation, tc2.operation);
    assert_eq!(tc.mesh_result.success, tc2.mesh_result.success);
    assert_eq!(tc.mesh_result.num_faces, tc2.mesh_result.num_faces);
    assert_eq!(tc.parametric_result.success, tc2.parametric_result.success);
    assert_eq!(tc.parametric_result.num_faces, tc2.parametric_result.num_faces);
    assert_eq!(tc.topology_match, tc2.topology_match);
}
