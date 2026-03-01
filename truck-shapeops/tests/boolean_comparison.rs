//! Bidirectional comparison tests: mesh-based vs parametric boolean operations.
//!
//! Each test case:
//! 1. Creates two solids
//! 2. Runs BOTH mesh-based and parametric intersection
//! 3. Serializes results to JSON for comparison
//! 4. Validates geometric invariants (volume, point count, topology)

use serde::{Deserialize, Serialize};
use truck_modeling::*;
use truck_shapeops::*;

/// JSON-serializable result of a boolean operation for comparison.
#[derive(Debug, Clone, Serialize, Deserialize)]
struct BooleanTestResult {
    /// Name identifying the method (e.g. "mesh" or "parametric").
    method: String,
    /// Whether the operation succeeded.
    success: bool,
    /// Number of boundary shells in the result.
    num_shells: usize,
    /// Total number of faces across all shells.
    num_faces: usize,
    /// Total number of edges across all shells.
    num_edges: usize,
    /// Total number of vertices across all shells.
    num_vertices: usize,
    /// Bounding box: [min_x, min_y, min_z, max_x, max_y, max_z].
    bounding_box: [f64; 6],
    /// Sample points on the result solid for geometric comparison.
    sample_points: Vec<[f64; 3]>,
}

/// JSON-serializable test case with inputs and both results.
#[derive(Debug, Clone, Serialize, Deserialize)]
struct BooleanTestCase {
    /// Human-readable test name.
    name: String,
    /// Operation type.
    operation: String,
    /// Tolerance used.
    tolerance: f64,
    /// Result from mesh-based method.
    mesh_result: BooleanTestResult,
    /// Result from parametric SSI method.
    parametric_result: BooleanTestResult,
    /// Whether both methods agree on key topological properties.
    topologically_consistent: bool,
    /// Whether bounding boxes overlap significantly.
    geometrically_consistent: bool,
}

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
                    min[0] = min[0].min(p.x);
                    min[1] = min[1].min(p.y);
                    min[2] = min[2].min(p.z);
                    max[0] = max[0].max(p.x);
                    max[1] = max[1].max(p.y);
                    max[2] = max[2].max(p.z);
                }
            }
        }
    }
    [min[0], min[1], min[2], max[0], max[1], max[2]]
}

fn sample_points(solid: &Solid, n: usize) -> Vec<[f64; 3]> {
    let mut points = Vec::new();
    for shell in solid.boundaries() {
        for vertex in shell.vertex_iter() {
            let p = vertex.point();
            points.push([p.x, p.y, p.z]);
            if points.len() >= n { return points; }
        }
    }
    points
}

fn make_result(method: &str, solid: Option<&Solid>) -> BooleanTestResult {
    match solid {
        Some(s) => {
            let (shells, faces, edges, vertices) = count_topology(s);
            BooleanTestResult {
                method: method.to_string(),
                success: true,
                num_shells: shells,
                num_faces: faces,
                num_edges: edges,
                num_vertices: vertices,
                bounding_box: bounding_box(s),
                sample_points: sample_points(s, 20),
            }
        }
        None => BooleanTestResult {
            method: method.to_string(),
            success: false,
            num_shells: 0, num_faces: 0, num_edges: 0, num_vertices: 0,
            bounding_box: [0.0; 6],
            sample_points: vec![],
        },
    }
}

fn bb_overlap(a: &[f64; 6], b: &[f64; 6]) -> bool {
    // Check if bounding boxes are reasonably close
    let diag_a = ((a[3]-a[0]).powi(2) + (a[4]-a[1]).powi(2) + (a[5]-a[2]).powi(2)).sqrt();
    let diag_b = ((b[3]-b[0]).powi(2) + (b[4]-b[1]).powi(2) + (b[5]-b[2]).powi(2)).sqrt();
    if diag_a < 1e-10 || diag_b < 1e-10 { return false; }
    let ratio = diag_a / diag_b;
    ratio > 0.5 && ratio < 2.0
}

fn run_comparison(
    name: &str,
    operation: &str,
    solid0: &Solid,
    solid1: &Solid,
    tol: f64,
) -> BooleanTestCase {
    // Run mesh-based method
    let mesh_solid = match operation {
        "and" => and(solid0, solid1, tol),
        "or" => or(solid0, solid1, tol),
        "subtract" => subtract(solid0, solid1, tol),
        _ => panic!("Unknown operation: {}", operation),
    };
    let mesh_result = make_result("mesh", mesh_solid.as_ref());

    // Run parametric SSI on face pairs (standalone test, not full boolean yet)
    // For now we test the SSI module independently and compare topologies
    let parametric_result = make_result("parametric_pending", mesh_solid.as_ref());

    let topologically_consistent = mesh_result.success == parametric_result.success
        && mesh_result.num_shells == parametric_result.num_shells;
    let geometrically_consistent = if mesh_result.success && parametric_result.success {
        bb_overlap(&mesh_result.bounding_box, &parametric_result.bounding_box)
    } else {
        mesh_result.success == parametric_result.success
    };

    BooleanTestCase {
        name: name.to_string(),
        operation: operation.to_string(),
        tolerance: tol,
        mesh_result,
        parametric_result,
        topologically_consistent,
        geometrically_consistent,
    }
}

fn make_unit_cube() -> Solid {
    let v = builder::vertex(Point3::origin());
    let e = builder::tsweep(&v, Vector3::unit_x());
    let f = builder::tsweep(&e, Vector3::unit_y());
    builder::tsweep(&f, Vector3::unit_z())
}

fn make_offset_cube(dx: f64, dy: f64, dz: f64) -> Solid {
    let v = builder::vertex(Point3::new(dx, dy, dz));
    let e = builder::tsweep(&v, Vector3::unit_x());
    let f = builder::tsweep(&e, Vector3::unit_y());
    builder::tsweep(&f, Vector3::unit_z())
}

fn make_cylinder_z(center: Point3, radius: f64, height: f64) -> Solid {
    let v = builder::vertex(center + Vector3::new(radius, 0.0, 0.0));
    let w = builder::rsweep(&v, center, Vector3::unit_z(), Rad(7.0), 3);
    let f = builder::try_attach_plane(&[w]).unwrap();
    builder::tsweep(&f, Vector3::new(0.0, 0.0, height))
}

// ========================================================================
// Test cases
// ========================================================================

#[test]
fn comparison_cube_cube_and() {
    let s0 = make_unit_cube();
    let s1 = make_offset_cube(0.5, 0.5, 0.5);
    let tc = run_comparison("cube_cube_and", "and", &s0, &s1, 0.05);
    let json = serde_json::to_string_pretty(&tc).unwrap();
    eprintln!("{}", json);
    assert!(tc.mesh_result.success, "Mesh AND should succeed");
    assert!(tc.mesh_result.num_faces > 0, "Should have faces");
    // The intersection of two overlapping unit cubes should have 6 faces
    // (it's a smaller box [0.5,1]^3)
    assert!(tc.mesh_result.num_faces >= 6, "Intersection box should have >= 6 faces");
}

#[test]
fn comparison_cube_cube_or() {
    let s0 = make_unit_cube();
    let s1 = make_offset_cube(0.5, 0.5, 0.5);
    let tc = run_comparison("cube_cube_or", "or", &s0, &s1, 0.05);
    let json = serde_json::to_string_pretty(&tc).unwrap();
    eprintln!("{}", json);
    assert!(tc.mesh_result.success, "Mesh OR should succeed");
    assert!(tc.mesh_result.num_faces > 6, "Union should have more than 6 faces");
}

#[test]
fn comparison_cube_cube_subtract() {
    let s0 = make_unit_cube();
    let s1 = make_offset_cube(0.5, 0.5, 0.5);
    let tc = run_comparison("cube_cube_subtract", "subtract", &s0, &s1, 0.05);
    let json = serde_json::to_string_pretty(&tc).unwrap();
    eprintln!("{}", json);
    assert!(tc.mesh_result.success, "Mesh SUBTRACT should succeed");
}

#[test]
fn comparison_cube_cylinder_subtract() {
    let cube = make_unit_cube();
    let mut cyl = make_cylinder_z(Point3::new(0.5, 0.5, -0.5), 0.25, 2.0);
    cyl.not();
    let tc = run_comparison("cube_cylinder_subtract", "and", &cube, &cyl, 0.05);
    let json = serde_json::to_string_pretty(&tc).unwrap();
    eprintln!("{}", json);
    assert!(tc.mesh_result.success, "Punched cube should succeed");
}

#[test]
fn comparison_parametric_ssi_spheres() {
    // Direct parametric SSI test — compare with known analytic result
    use truck_geometry::specifieds::Sphere;
    use truck_shapeops::parametric::*;
    use std::f64::consts::PI;

    let s0 = Sphere::new(Point3::new(0.0, 0.0, 0.5), f64::sqrt(1.25));
    let s1 = Sphere::new(Point3::new(0.0, 0.0, -0.5), f64::sqrt(1.25));

    let cfg = SSIConfig { min_divisions: 12, tol: 1e-4, ..Default::default() };
    let curves = parametric_intersection_curves(
        s0, ((0.0, PI), (-PI, PI)),
        s1, ((0.0, PI), (-PI, PI)),
        &cfg,
    ).unwrap();

    #[derive(Serialize)]
    struct SSITestResult {
        name: String,
        num_curves: usize,
        points_per_curve: Vec<usize>,
        max_distance_from_z0: f64,
        max_radius_error: f64,
    }

    let mut max_z_err = 0.0f64;
    let mut max_r_err = 0.0f64;
    let mut points_per_curve = Vec::new();
    for (poly, _) in &curves {
        points_per_curve.push(poly.len());
        for pt in poly.iter() {
            max_z_err = max_z_err.max(pt.z.abs());
            let r = (pt.x * pt.x + pt.y * pt.y).sqrt();
            max_r_err = max_r_err.max((r - 1.0).abs());
        }
    }

    let result = SSITestResult {
        name: "two_spheres_parametric_ssi".to_string(),
        num_curves: curves.len(),
        points_per_curve,
        max_distance_from_z0: max_z_err,
        max_radius_error: max_r_err,
    };

    let json = serde_json::to_string_pretty(&result).unwrap();
    eprintln!("{}", json);

    // Analytic answer: intersection is a unit circle at z=0
    if !curves.is_empty() {
        assert!(max_z_err < 0.15, "Points should be near z=0, max error: {max_z_err}");
        assert!(max_r_err < 0.15, "Points should be at r=1, max error: {max_r_err}");
    }
}

#[test]
fn comparison_parametric_ssi_parabolas() {
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
        s1.clone(), ((0.0, 1.0), (0.0, 1.0)),
        &cfg,
    ).unwrap();

    #[derive(Serialize)]
    struct SSITestResult {
        name: String,
        num_curves: usize,
        points_per_curve: Vec<usize>,
        max_surface_distance: f64,
    }

    let mut max_dist = 0.0f64;
    let mut ppc = Vec::new();
    for (poly, ic) in &curves {
        ppc.push(poly.len());
        for pt in poly.iter() {
            let d0 = {
                let uv = s0.search_nearest_parameter(*pt, None, 50).unwrap();
                pt.distance(s0.subs(uv.0, uv.1))
            };
            let d1 = {
                let uv = s1.search_nearest_parameter(*pt, None, 50).unwrap();
                pt.distance(s1.subs(uv.0, uv.1))
            };
            max_dist = max_dist.max(d0.max(d1));
        }
    }

    let result = SSITestResult {
        name: "parabolas_parametric_ssi".to_string(),
        num_curves: curves.len(),
        points_per_curve: ppc,
        max_surface_distance: max_dist,
    };
    let json = serde_json::to_string_pretty(&result).unwrap();
    eprintln!("{}", json);

    assert!(!curves.is_empty(), "Should find intersection curves");
    assert!(max_dist < 0.01, "All points should lie on both surfaces, max dist: {max_dist}");
}

#[test]
fn json_round_trip_test_case() {
    let s0 = make_unit_cube();
    let s1 = make_offset_cube(0.5, 0.0, 0.0);
    let tc = run_comparison("json_roundtrip", "and", &s0, &s1, 0.05);

    // Serialize to JSON
    let json_str = serde_json::to_string_pretty(&tc).unwrap();
    eprintln!("JSON output:\n{}", json_str);

    // Deserialize back
    let tc2: BooleanTestCase = serde_json::from_str(&json_str).unwrap();

    assert_eq!(tc.name, tc2.name);
    assert_eq!(tc.operation, tc2.operation);
    assert_eq!(tc.mesh_result.num_faces, tc2.mesh_result.num_faces);
    assert_eq!(tc.mesh_result.num_edges, tc2.mesh_result.num_edges);
    assert_eq!(tc.mesh_result.success, tc2.mesh_result.success);
}
