//! Face splitting by intersection curves in 2D parameter space.

use truck_base::cgmath64::*;
use truck_base::tolerance::TOLERANCE;

/// A 2D loop in parameter space.
#[derive(Clone, Debug)]
pub struct ParameterLoop {
    /// Points in (u, v) parameter space.
    pub points: Vec<Point2>,
    /// Signed area: positive = CCW (outline), negative = CW (hole).
    pub signed_area: f64,
}

/// A sub-face created by splitting.
#[derive(Clone, Debug)]
pub struct SplitFaceRegion {
    /// The outer boundary loop.
    pub outline: ParameterLoop,
    /// Holes contained in this region.
    pub holes: Vec<ParameterLoop>,
}

/// Compute signed area of a 2D polygon (positive = CCW).
pub fn signed_area(points: &[Point2]) -> f64 {
    let n = points.len();
    if n < 3 { return 0.0; }
    let mut area = 0.0;
    for i in 0..n {
        let j = (i + 1) % n;
        area += points[i].x * points[j].y - points[j].x * points[i].y;
    }
    area / 2.0
}

/// Check if a point is inside a polygon (winding number algorithm).
pub fn point_in_polygon(point: &Point2, poly: &[Point2]) -> bool {
    let mut winding = 0i32;
    for i in 0..poly.len() {
        let j = (i + 1) % poly.len();
        let (yi, yj) = (poly[i].y, poly[j].y);
        if yi <= point.y {
            if yj > point.y {
                let cross = (poly[j].x - poly[i].x) * (point.y - poly[i].y)
                    - (point.x - poly[i].x) * (poly[j].y - poly[i].y);
                if cross > 0.0 { winding += 1; }
            }
        } else if yj <= point.y {
            let cross = (poly[j].x - poly[i].x) * (point.y - poly[i].y)
                - (point.x - poly[i].x) * (poly[j].y - poly[i].y);
            if cross < 0.0 { winding -= 1; }
        }
    }
    winding != 0
}

/// Split a face into sub-regions using intersection curves in 2D parameter space.
pub fn split_face_by_curves(
    face_boundary: &[Point2],
    face_holes: &[Vec<Point2>],
    intersection_curves_2d: &[Vec<Point2>],
) -> Vec<SplitFaceRegion> {
    if intersection_curves_2d.is_empty() {
        return vec![SplitFaceRegion {
            outline: ParameterLoop { signed_area: signed_area(face_boundary), points: face_boundary.to_vec() },
            holes: face_holes.iter().map(|h| ParameterLoop { signed_area: signed_area(h), points: h.clone() }).collect(),
        }];
    }
    let mut all_loops = Vec::new();
    for curve in intersection_curves_2d {
        if curve.len() < 3 { continue; }
        let a = signed_area(curve);
        if a.abs() < TOLERANCE { continue; }
        all_loops.push(ParameterLoop { points: curve.clone(), signed_area: a });
    }
    all_loops.push(ParameterLoop { points: face_boundary.to_vec(), signed_area: signed_area(face_boundary) });
    for hole in face_holes {
        all_loops.push(ParameterLoop { points: hole.clone(), signed_area: signed_area(hole) });
    }
    let (mut outlines, mut holes): (Vec<_>, Vec<_>) = all_loops.into_iter()
        .partition(|l| l.signed_area > TOLERANCE);
    let holes: Vec<_> = holes.into_iter().filter(|l| l.signed_area < -TOLERANCE).collect();
    outlines.sort_by(|a, b| a.signed_area.partial_cmp(&b.signed_area).unwrap());
    let mut regions: Vec<SplitFaceRegion> = outlines.into_iter()
        .map(|o| SplitFaceRegion { outline: o, holes: Vec::new() }).collect();
    for hole in holes {
        for region in regions.iter_mut() {
            if point_in_polygon(&hole.points[0], &region.outline.points) {
                region.holes.push(hole);
                break;
            }
        }
    }
    regions
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn signed_area_ccw_square() {
        let sq = vec![Point2::new(0.0,0.0), Point2::new(1.0,0.0), Point2::new(1.0,1.0), Point2::new(0.0,1.0)];
        assert!((signed_area(&sq) - 1.0).abs() < 1e-10);
    }

    #[test]
    fn signed_area_cw_square() {
        let sq = vec![Point2::new(0.0,0.0), Point2::new(0.0,1.0), Point2::new(1.0,1.0), Point2::new(1.0,0.0)];
        assert!((signed_area(&sq) + 1.0).abs() < 1e-10);
    }

    #[test]
    fn pip_inside() {
        let sq = vec![Point2::new(0.0,0.0), Point2::new(1.0,0.0), Point2::new(1.0,1.0), Point2::new(0.0,1.0)];
        assert!(point_in_polygon(&Point2::new(0.5, 0.5), &sq));
        assert!(!point_in_polygon(&Point2::new(1.5, 0.5), &sq));
    }

    #[test]
    fn no_intersection_returns_original() {
        let b = vec![Point2::new(0.0,0.0), Point2::new(1.0,0.0), Point2::new(1.0,1.0), Point2::new(0.0,1.0)];
        let r = split_face_by_curves(&b, &[], &[]);
        assert_eq!(r.len(), 1);
        assert_eq!(r[0].outline.points.len(), 4);
    }
}
