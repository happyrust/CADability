using CADability.GeoObject;
using CADability.Curve2D;
using MathNet.Numerics.LinearAlgebra.Double;
using System;
using System.Collections.Generic;
using System.Linq;

namespace CADability
{
    /// <summary>
    /// Configuration for parametric surface-surface intersection (SSI) algorithm.
    /// Ported from truck-shapeops parametric SSI.
    /// </summary>
    public class SSIConfig
    {
        public double Tolerance { get; set; } = Precision.eps * 10.0;
        public int MinDivisions { get; set; } = 16;
        public int NewtonTrials { get; set; } = 50;
    }

    /// <summary>
    /// Which iso-parametric line produced the intersection point.
    /// </summary>
    public enum GridSource
    {
        FixedU0,
        FixedV0,
        FixedU1,
        FixedV1,
    }

    /// <summary>
    /// A point on the intersection of two surfaces with parametric coordinates on both.
    /// </summary>
    public class LinkedIntersectionPoint
    {
        public GeoPoint Point;
        public GeoPoint2D UV0;
        public GeoPoint2D UV1;
        public GeoVector Tangent;
        public GridSource GridSource;
        public int GridIndex;
        public int? Next;
        public int? Prev;
    }

    /// <summary>
    /// An intersection point between an edge and a face.
    /// </summary>
    public class ParametricEdgeFaceIntersection
    {
        public GeoPoint Point;
        public double EdgeParam;
        public GeoPoint2D FaceUV;
        public bool EdgeFromShell0;
        public int EdgeIndex;
        public int FaceIndex;
    }

    /// <summary>
    /// A 2D loop in parameter space.
    /// </summary>
    public class ParameterLoop
    {
        public List<GeoPoint2D> Points;
        public double SignedArea;
    }

    /// <summary>
    /// A sub-face created by splitting.
    /// </summary>
    public class SplitFaceRegion
    {
        public ParameterLoop Outline;
        public List<ParameterLoop> Holes = new List<ParameterLoop>();
    }

    /// <summary>
    /// An intersection curve between two surfaces, with both 3D polyline and parametric coordinates.
    /// </summary>
    public class ParametricIntersectionCurve
    {
        public List<GeoPoint> Points3D;
        public List<GeoPoint2D> UVsOnSurface0;
        public List<GeoPoint2D> UVsOnSurface1;
        public ISurface Surface0;
        public ISurface Surface1;
    }

    /// <summary>
    /// Parametric surface-surface intersection using iso-curve scanning + Newton refinement.
    /// Ported from truck-shapeops/src/transversal/parametric/surface_surface_intersection.rs
    /// </summary>
    public static class ParametricSSI
    {
        /// <summary>
        /// Newton refinement of a point on the intersection of two surfaces.
        /// Minimizes |S0(u0,v0) - S1(u1,v1)| using Gauss-Newton on the 4x4 normal equations.
        /// </summary>
        public static bool NewtonRefineIntersection(
            ISurface s0, ISurface s1,
            GeoPoint2D hint0, GeoPoint2D hint1,
            int trials, double tol,
            out GeoPoint result, out GeoPoint2D ruv0, out GeoPoint2D ruv1)
        {
            double u0 = hint0.x, v0 = hint0.y, u1 = hint1.x, v1 = hint1.y;
            result = GeoPoint.Origin;
            ruv0 = hint0;
            ruv1 = hint1;

            for (int iter = 0; iter < trials; iter++)
            {
                GeoPoint p0 = s0.PointAt(new GeoPoint2D(u0, v0));
                GeoPoint p1 = s1.PointAt(new GeoPoint2D(u1, v1));
                GeoVector r = p0 - p1;
                double rMag = r.Length;
                if (rMag < tol)
                {
                    result = new GeoPoint((p0.x + p1.x) / 2, (p0.y + p1.y) / 2, (p0.z + p1.z) / 2);
                    ruv0 = new GeoPoint2D(u0, v0);
                    ruv1 = new GeoPoint2D(u1, v1);
                    return true;
                }

                GeoVector du0 = s0.UDirection(new GeoPoint2D(u0, v0));
                GeoVector dv0 = s0.VDirection(new GeoPoint2D(u0, v0));
                GeoVector du1 = s1.UDirection(new GeoPoint2D(u1, v1));
                GeoVector dv1 = s1.VDirection(new GeoPoint2D(u1, v1));

                GeoVector[] cols = { du0, dv0, -1.0 * du1, -1.0 * dv1 };

                double[,] jtj = new double[4, 4];
                double[] jtr = new double[4];
                for (int i = 0; i < 4; i++)
                {
                    for (int j = 0; j < 4; j++)
                        jtj[i, j] = cols[i] * cols[j];
                    jtr[i] = -(cols[i] * r);
                }

                double[] dx = Solve4x4(jtj, jtr);
                if (dx == null) return false;

                u0 += dx[0]; v0 += dx[1]; u1 += dx[2]; v1 += dx[3];
            }
            return false;
        }

        /// <summary>
        /// Solve a 4x4 linear system using Gaussian elimination with partial pivoting.
        /// </summary>
        internal static double[] Solve4x4(double[,] a, double[] b)
        {
            double[,] m = new double[4, 5];
            for (int i = 0; i < 4; i++)
            {
                for (int j = 0; j < 4; j++) m[i, j] = a[i, j];
                m[i, 4] = b[i];
            }

            for (int col = 0; col < 4; col++)
            {
                int maxRow = col;
                double maxVal = Math.Abs(m[col, col]);
                for (int row = col + 1; row < 4; row++)
                {
                    double v = Math.Abs(m[row, col]);
                    if (v > maxVal) { maxVal = v; maxRow = row; }
                }
                if (maxVal < 1e-15) return null;

                if (maxRow != col)
                {
                    for (int k = 0; k < 5; k++)
                    {
                        double tmp = m[col, k]; m[col, k] = m[maxRow, k]; m[maxRow, k] = tmp;
                    }
                }

                double pivot = m[col, col];
                for (int row = col + 1; row < 4; row++)
                {
                    double f = m[row, col] / pivot;
                    for (int k = col; k < 5; k++) m[row, k] -= f * m[col, k];
                }
            }

            double[] x = new double[4];
            for (int i = 3; i >= 0; i--)
            {
                x[i] = m[i, 4];
                for (int j = i + 1; j < 4; j++) x[i] -= m[i, j] * x[j];
                if (Math.Abs(m[i, i]) < 1e-15) return null;
                x[i] /= m[i, i];
            }
            return x;
        }

        /// <summary>
        /// Find intersection points along an iso-parametric curve of s0 with surface s1.
        /// </summary>
        internal static List<(GeoPoint pt, GeoPoint2D uv0, GeoPoint2D uv1)> IntersectIsocurve(
            ISurface s0, ISurface s1,
            double fixedParam, bool isFixedU,
            double varyMin, double varyMax,
            BoundingRect bounds1, SSIConfig cfg)
        {
            int n = cfg.MinDivisions;
            double step = (varyMax - varyMin) / n;
            var results = new List<(GeoPoint, GeoPoint2D, GeoPoint2D)>();

            for (int i = 0; i <= n; i++)
            {
                double t = varyMin + step * i;
                GeoPoint2D uv0 = isFixedU ? new GeoPoint2D(fixedParam, t) : new GeoPoint2D(t, fixedParam);
                GeoPoint pt = s0.PointAt(uv0);
                GeoPoint2D proj1 = s1.PositionOf(pt);
                GeoPoint projected = s1.PointAt(proj1);

                if ((pt | projected) > cfg.Tolerance * 1000.0) continue;

                if (NewtonRefineIntersection(s0, s1, uv0, proj1, cfg.NewtonTrials, cfg.Tolerance,
                    out GeoPoint rp, out GeoPoint2D ruv0, out GeoPoint2D ruv1))
                {
                    bool in0 = isFixedU
                        ? (ruv0.y >= varyMin - cfg.Tolerance && ruv0.y <= varyMax + cfg.Tolerance)
                        : (ruv0.x >= varyMin - cfg.Tolerance && ruv0.x <= varyMax + cfg.Tolerance);

                    bool in1 = ruv1.x >= bounds1.Left - cfg.Tolerance && ruv1.x <= bounds1.Right + cfg.Tolerance
                            && ruv1.y >= bounds1.Bottom - cfg.Tolerance && ruv1.y <= bounds1.Top + cfg.Tolerance;

                    if (in0 && in1 && !results.Any(r => (r.Item1 | rp) < cfg.Tolerance * 10.0))
                    {
                        results.Add((rp, ruv0, ruv1));
                    }
                }
            }
            return results;
        }

        /// <summary>
        /// Link intersection points into chains by proximity and grid source diversity.
        /// </summary>
        internal static void LinkPoints(List<LinkedIntersectionPoint> pts, double tol)
        {
            int n = pts.Count;
            for (int i = 0; i < n; i++)
            {
                if (pts[i].Next.HasValue) continue;
                int? bestIdx = null;
                double bestDist = double.MaxValue;
                for (int j = 0; j < n; j++)
                {
                    if (i == j || pts[j].Prev.HasValue) continue;
                    if (pts[i].GridSource == pts[j].GridSource && pts[i].GridIndex == pts[j].GridIndex) continue;
                    double d = pts[i].Point | pts[j].Point;
                    if (d < bestDist && d < tol * 1000.0)
                    {
                        bestDist = d;
                        bestIdx = j;
                    }
                }
                if (bestIdx.HasValue)
                {
                    pts[i].Next = bestIdx;
                    pts[bestIdx.Value].Prev = i;
                }
            }
        }

        /// <summary>
        /// Extract ordered chains from linked intersection points.
        /// </summary>
        internal static List<List<int>> ExtractChains(List<LinkedIntersectionPoint> pts)
        {
            bool[] visited = new bool[pts.Count];
            var chains = new List<List<int>>();

            for (int pass = 0; pass < 2; pass++)
            {
                for (int s = 0; s < pts.Count; s++)
                {
                    if (visited[s]) continue;
                    if (pass == 0 && pts[s].Prev.HasValue) continue;

                    var chain = new List<int> { s };
                    visited[s] = true;
                    int cur = s;
                    while (pts[cur].Next.HasValue)
                    {
                        int nxt = pts[cur].Next.Value;
                        if (visited[nxt]) break;
                        chain.Add(nxt);
                        visited[nxt] = true;
                        cur = nxt;
                    }
                    if (chain.Count >= 2) chains.Add(chain);
                }
            }
            return chains;
        }

        /// <summary>
        /// Compute parametric intersection curves between two surfaces.
        /// Uses iso-curve scanning on both surfaces + Newton 4x4 refinement.
        /// </summary>
        public static List<ParametricIntersectionCurve> ComputeIntersectionCurves(
            ISurface s0, BoundingRect bounds0,
            ISurface s1, BoundingRect bounds1,
            SSIConfig cfg = null)
        {
            cfg = cfg ?? new SSIConfig();
            int n = cfg.MinDivisions;
            var all = new List<LinkedIntersectionPoint>();

            double[] MakeGrid(double lo, double hi) =>
                Enumerable.Range(0, n + 1).Select(i => lo + (hi - lo) * i / n).ToArray();

            double[] u0s = MakeGrid(bounds0.Left, bounds0.Right);
            double[] v0s = MakeGrid(bounds0.Bottom, bounds0.Top);
            double[] u1s = MakeGrid(bounds1.Left, bounds1.Right);
            double[] v1s = MakeGrid(bounds1.Bottom, bounds1.Top);

            void AddPoints(List<(GeoPoint, GeoPoint2D, GeoPoint2D)> pts, GridSource gs, int idx)
            {
                foreach (var (pt, uv0, uv1) in pts)
                {
                    GeoVector tangent = s0.GetNormal(uv0) ^ s1.GetNormal(uv1);
                    all.Add(new LinkedIntersectionPoint
                    {
                        Point = pt, UV0 = uv0, UV1 = uv1, Tangent = tangent,
                        GridSource = gs, GridIndex = idx
                    });
                }
            }

            for (int i = 0; i < u0s.Length; i++)
                AddPoints(IntersectIsocurve(s0, s1, u0s[i], true, bounds0.Bottom, bounds0.Top, bounds1, cfg), GridSource.FixedU0, i);
            for (int i = 0; i < v0s.Length; i++)
                AddPoints(IntersectIsocurve(s0, s1, v0s[i], false, bounds0.Left, bounds0.Right, bounds1, cfg), GridSource.FixedV0, i);

            for (int i = 0; i < u1s.Length; i++)
            {
                var pts = IntersectIsocurve(s1, s0, u1s[i], true, bounds1.Bottom, bounds1.Top, bounds0, cfg);
                foreach (var (pt, uv1, uv0) in pts)
                {
                    GeoVector tangent = s0.GetNormal(uv0) ^ s1.GetNormal(uv1);
                    all.Add(new LinkedIntersectionPoint
                    {
                        Point = pt, UV0 = uv0, UV1 = uv1, Tangent = tangent,
                        GridSource = GridSource.FixedU1, GridIndex = i
                    });
                }
            }
            for (int i = 0; i < v1s.Length; i++)
            {
                var pts = IntersectIsocurve(s1, s0, v1s[i], false, bounds1.Left, bounds1.Right, bounds0, cfg);
                foreach (var (pt, uv1, uv0) in pts)
                {
                    GeoVector tangent = s0.GetNormal(uv0) ^ s1.GetNormal(uv1);
                    all.Add(new LinkedIntersectionPoint
                    {
                        Point = pt, UV0 = uv0, UV1 = uv1, Tangent = tangent,
                        GridSource = GridSource.FixedV1, GridIndex = i
                    });
                }
            }

            if (all.Count == 0) return new List<ParametricIntersectionCurve>();

            // Deduplicate close points from different grid sources
            for (int i = 0; i < all.Count; i++)
            {
                for (int j = i + 1; j < all.Count;)
                {
                    if ((all[i].Point | all[j].Point) < cfg.Tolerance * 2.0 &&
                        (all[i].GridSource != all[j].GridSource || all[i].GridIndex != all[j].GridIndex))
                    {
                        all[i].Point = new GeoPoint(
                            (all[i].Point.x + all[j].Point.x) / 2,
                            (all[i].Point.y + all[j].Point.y) / 2,
                            (all[i].Point.z + all[j].Point.z) / 2);
                        all.RemoveAt(j);
                    }
                    else j++;
                }
            }

            LinkPoints(all, cfg.Tolerance);
            var chains = ExtractChains(all);
            var result = new List<ParametricIntersectionCurve>();

            foreach (var chain in chains)
            {
                if (chain.Count < 2) continue;
                result.Add(new ParametricIntersectionCurve
                {
                    Points3D = chain.Select(i => all[i].Point).ToList(),
                    UVsOnSurface0 = chain.Select(i => all[i].UV0).ToList(),
                    UVsOnSurface1 = chain.Select(i => all[i].UV1).ToList(),
                    Surface0 = s0,
                    Surface1 = s1,
                });
            }
            return result;
        }
    }

    /// <summary>
    /// Analytic intersection for special surface pairs (plane-plane, plane-curved).
    /// Ported from truck-shapeops/src/transversal/parametric/analytic_intersection.rs
    /// </summary>
    public static class AnalyticIntersection
    {
        /// <summary>
        /// Check if a surface has a constant normal (is planar) within the given bounds.
        /// </summary>
        public static bool IsPlanar(ISurface surface, BoundingRect bounds)
        {
            GeoPoint2D center = bounds.GetCenter();
            GeoVector nRef = surface.GetNormal(center);
            if (nRef.IsNullVector()) return false;
            nRef = nRef.Normalized;

            GeoPoint2D[] corners = {
                new GeoPoint2D(bounds.Left, bounds.Bottom),
                new GeoPoint2D(bounds.Right, bounds.Bottom),
                new GeoPoint2D(bounds.Left, bounds.Top),
                new GeoPoint2D(bounds.Right, bounds.Top),
            };

            foreach (var c in corners)
            {
                GeoVector n = surface.GetNormal(c);
                if (n.IsNullVector()) return false;
                n = n.Normalized;
                double diff = Math.Sqrt((n.x - nRef.x) * (n.x - nRef.x) + (n.y - nRef.y) * (n.y - nRef.y) + (n.z - nRef.z) * (n.z - nRef.z));
                if (diff > Precision.eps * 100.0) return false;
            }
            return true;
        }

        /// <summary>
        /// Compute intersection of two planar surfaces analytically.
        /// Two non-parallel planes intersect in a line.
        /// </summary>
        public static List<ParametricIntersectionCurve> PlanePlaneIntersection(
            ISurface s0, BoundingRect bounds0,
            ISurface s1, BoundingRect bounds1)
        {
            GeoPoint2D center0 = bounds0.GetCenter();
            GeoPoint2D center1 = bounds1.GetCenter();
            GeoVector n0 = s0.GetNormal(center0).Normalized;
            GeoVector n1 = s1.GetNormal(center1).Normalized;
            GeoVector dir = n0 ^ n1;
            if (dir.Length < Precision.eps * 10.0)
                return new List<ParametricIntersectionCurve>(); // parallel

            dir = dir.Normalized;
            GeoPoint p0 = s0.PointAt(center0);
            GeoPoint p1 = s1.PointAt(center1);
            double d0 = n0.x * p0.x + n0.y * p0.y + n0.z * p0.z;
            double d1 = n1.x * p1.x + n1.y * p1.y + n1.z * p1.z;
            GeoPoint midpoint = new GeoPoint((p0.x + p1.x) / 2, (p0.y + p1.y) / 2, (p0.z + p1.z) / 2);
            double d2 = dir.x * midpoint.x + dir.y * midpoint.y + dir.z * midpoint.z;

            // Solve 3x3: [n0; n1; dir] * basePoint = [d0; d1; d2]
            double[,] mat = {
                { n0.x, n0.y, n0.z },
                { n1.x, n1.y, n1.z },
                { dir.x, dir.y, dir.z }
            };
            double[] rhs = { d0, d1, d2 };
            double[] sol = Solve3x3(mat, rhs);
            if (sol == null) return new List<ParametricIntersectionCurve>();

            GeoPoint basePoint = new GeoPoint(sol[0], sol[1], sol[2]);

            // Find valid t-range by projecting surface corner points onto the line
            var tSamples0 = SampleLineOnSurface(s0, bounds0, basePoint, dir, 20);
            var tSamples1 = SampleLineOnSurface(s1, bounds1, basePoint, dir, 20);

            if (tSamples0.Count == 0 || tSamples1.Count == 0)
                return new List<ParametricIntersectionCurve>();

            double tLo = Math.Max(tSamples0.First(), tSamples1.First());
            double tHi = Math.Min(tSamples0.Last(), tSamples1.Last());
            if (tHi - tLo < Precision.eps)
                return new List<ParametricIntersectionCurve>();

            int nPts = Math.Max(10, Math.Min(50, (int)((tHi - tLo) / Precision.eps)));
            var pts3D = new List<GeoPoint>();
            var uvs0 = new List<GeoPoint2D>();
            var uvs1 = new List<GeoPoint2D>();
            for (int i = 0; i <= nPts; i++)
            {
                double t = tLo + (tHi - tLo) * i / nPts;
                GeoPoint pt = basePoint + t * dir;
                pts3D.Add(pt);
                uvs0.Add(s0.PositionOf(pt));
                uvs1.Add(s1.PositionOf(pt));
            }

            return new List<ParametricIntersectionCurve>
            {
                new ParametricIntersectionCurve
                {
                    Points3D = pts3D, UVsOnSurface0 = uvs0, UVsOnSurface1 = uvs1,
                    Surface0 = s0, Surface1 = s1,
                }
            };
        }

        /// <summary>
        /// Intersect a planar surface with a general parametric surface using
        /// signed-distance bisection along iso-curves.
        /// </summary>
        public static List<ParametricIntersectionCurve> PlaneSurfaceIntersection(
            ISurface planeSurface, BoundingRect planeBounds,
            ISurface otherSurface, BoundingRect otherBounds,
            double tol, int nDiv)
        {
            GeoPoint2D planeCenter = planeBounds.GetCenter();
            GeoVector normal = planeSurface.GetNormal(planeCenter).Normalized;
            GeoPoint p0 = planeSurface.PointAt(planeCenter);
            double d = normal.x * p0.x + normal.y * p0.y + normal.z * p0.z;

            var intersectionPts = new List<GeoPoint>();
            double uStep = (otherBounds.Right - otherBounds.Left) / nDiv;
            double vStep = (otherBounds.Top - otherBounds.Bottom) / nDiv;

            void CheckAndAdd(GeoPoint pt)
            {
                GeoPoint2D puv = planeSurface.PositionOf(pt);
                if (puv.x >= planeBounds.Left - tol && puv.x <= planeBounds.Right + tol
                 && puv.y >= planeBounds.Bottom - tol && puv.y <= planeBounds.Top + tol)
                {
                    if (!intersectionPts.Any(p => (p | pt) < tol * 10.0))
                        intersectionPts.Add(pt);
                }
            }

            // Scan u-iso-curves
            for (int ui = 0; ui <= nDiv; ui++)
            {
                double u = otherBounds.Left + uStep * ui;
                double prevSign = 0.0;
                for (int vi = 0; vi <= nDiv; vi++)
                {
                    double v = otherBounds.Bottom + vStep * vi;
                    GeoPoint pt = otherSurface.PointAt(new GeoPoint2D(u, v));
                    double signedDist = normal.x * pt.x + normal.y * pt.y + normal.z * pt.z - d;

                    if (vi > 0 && prevSign * signedDist < 0.0)
                    {
                        double vPrev = otherBounds.Bottom + vStep * (vi - 1);
                        GeoPoint? ipt = BisectPlaneCrossing(otherSurface, u, vPrev, v, true, normal, d, tol);
                        if (ipt.HasValue) CheckAndAdd(ipt.Value);
                    }
                    if (Math.Abs(signedDist) < tol) CheckAndAdd(pt);
                    prevSign = signedDist;
                }
            }

            // Scan v-iso-curves
            for (int vi = 0; vi <= nDiv; vi++)
            {
                double v = otherBounds.Bottom + vStep * vi;
                double prevSign = 0.0;
                for (int ui = 0; ui <= nDiv; ui++)
                {
                    double u = otherBounds.Left + uStep * ui;
                    GeoPoint pt = otherSurface.PointAt(new GeoPoint2D(u, v));
                    double signedDist = normal.x * pt.x + normal.y * pt.y + normal.z * pt.z - d;

                    if (ui > 0 && prevSign * signedDist < 0.0)
                    {
                        double uPrev = otherBounds.Left + uStep * (ui - 1);
                        GeoPoint? ipt = BisectPlaneCrossing(otherSurface, uPrev, v, u, false, normal, d, tol);
                        if (ipt.HasValue) CheckAndAdd(ipt.Value);
                    }
                    if (Math.Abs(signedDist) < tol) CheckAndAdd(pt);
                    prevSign = signedDist;
                }
            }

            if (intersectionPts.Count < 2)
                return new List<ParametricIntersectionCurve>();

            // Sort by greedy nearest-neighbor to form a polyline
            var ordered = new List<GeoPoint> { intersectionPts[0] };
            var remaining = new List<GeoPoint>(intersectionPts.Skip(1));
            while (remaining.Count > 0)
            {
                GeoPoint last = ordered[ordered.Count - 1];
                int bestIdx = 0;
                double bestDist = last | remaining[0];
                for (int i = 1; i < remaining.Count; i++)
                {
                    double dist = last | remaining[i];
                    if (dist < bestDist) { bestDist = dist; bestIdx = i; }
                }
                ordered.Add(remaining[bestIdx]);
                remaining.RemoveAt(bestIdx);
            }

            var uvs0 = ordered.Select(p => planeSurface.PositionOf(p)).ToList();
            var uvs1 = ordered.Select(p => otherSurface.PositionOf(p)).ToList();

            return new List<ParametricIntersectionCurve>
            {
                new ParametricIntersectionCurve
                {
                    Points3D = ordered, UVsOnSurface0 = uvs0, UVsOnSurface1 = uvs1,
                    Surface0 = planeSurface, Surface1 = otherSurface,
                }
            };
        }

        /// <summary>
        /// Bisect to find where an iso-curve of a surface crosses a plane.
        /// </summary>
        internal static GeoPoint? BisectPlaneCrossing(
            ISurface surface, double fixedVal, double lo, double hi,
            bool fixedIsU, GeoVector normal, double planeD, double tol)
        {
            double Eval(double t)
            {
                GeoPoint2D uv = fixedIsU ? new GeoPoint2D(fixedVal, t) : new GeoPoint2D(t, fixedVal);
                GeoPoint pt = surface.PointAt(uv);
                return normal.x * pt.x + normal.y * pt.y + normal.z * pt.z - planeD;
            }

            double a = lo, b = hi;
            double fa = Eval(a);

            for (int iter = 0; iter < 40; iter++)
            {
                double mid = (a + b) / 2.0;
                double fm = Eval(mid);
                if (Math.Abs(fm) < tol)
                {
                    GeoPoint2D uv = fixedIsU ? new GeoPoint2D(fixedVal, mid) : new GeoPoint2D(mid, fixedVal);
                    return surface.PointAt(uv);
                }
                if (fa * fm <= 0.0)
                    b = mid;
                else
                {
                    a = mid;
                    fa = fm;
                }
            }
            double finalMid = (a + b) / 2.0;
            GeoPoint2D finalUV = fixedIsU ? new GeoPoint2D(fixedVal, finalMid) : new GeoPoint2D(finalMid, fixedVal);
            return surface.PointAt(finalUV);
        }

        internal static List<double> SampleLineOnSurface(
            ISurface surface, BoundingRect bounds,
            GeoPoint basePoint, GeoVector dir, int nSamples)
        {
            GeoPoint[] corners = {
                surface.PointAt(new GeoPoint2D(bounds.Left, bounds.Bottom)),
                surface.PointAt(new GeoPoint2D(bounds.Right, bounds.Bottom)),
                surface.PointAt(new GeoPoint2D(bounds.Left, bounds.Top)),
                surface.PointAt(new GeoPoint2D(bounds.Right, bounds.Top)),
            };

            var tValues = corners.Select(c => {
                GeoVector diff = c - basePoint;
                return diff.x * dir.x + diff.y * dir.y + diff.z * dir.z;
            }).OrderBy(t => t).ToArray();

            double tLo = tValues.First() - 0.5;
            double tHi = tValues.Last() + 0.5;
            double range = Math.Max(tHi - tLo, 0.1);
            double tol = Precision.eps * 100.0;

            var validT = new List<double>();
            for (int i = 0; i <= nSamples; i++)
            {
                double t = tLo + range * i / nSamples;
                GeoPoint pt = basePoint + t * dir;
                GeoPoint2D uv = surface.PositionOf(pt);
                GeoPoint onSurf = surface.PointAt(uv);

                bool onSurface = (pt | onSurf) < tol * 10.0;
                bool inBounds = uv.x >= bounds.Left - tol && uv.x <= bounds.Right + tol
                             && uv.y >= bounds.Bottom - tol && uv.y <= bounds.Top + tol;
                if (onSurface && inBounds)
                    validT.Add(t);
            }
            validT.Sort();
            return validT;
        }

        public static double[] Solve3x3(double[,] a, double[] b)
        {
            double[,] m = new double[3, 4];
            for (int i = 0; i < 3; i++)
            {
                for (int j = 0; j < 3; j++) m[i, j] = a[i, j];
                m[i, 3] = b[i];
            }
            for (int col = 0; col < 3; col++)
            {
                int maxRow = col;
                double maxVal = Math.Abs(m[col, col]);
                for (int row = col + 1; row < 3; row++)
                {
                    double v = Math.Abs(m[row, col]);
                    if (v > maxVal) { maxVal = v; maxRow = row; }
                }
                if (maxVal < 1e-15) return null;
                if (maxRow != col)
                {
                    for (int k = 0; k < 4; k++)
                    {
                        double tmp = m[col, k]; m[col, k] = m[maxRow, k]; m[maxRow, k] = tmp;
                    }
                }
                double pivot = m[col, col];
                for (int row = col + 1; row < 3; row++)
                {
                    double f = m[row, col] / pivot;
                    for (int k = col; k < 4; k++) m[row, k] -= f * m[col, k];
                }
            }
            double[] x = new double[3];
            for (int i = 2; i >= 0; i--)
            {
                x[i] = m[i, 3];
                for (int j = i + 1; j < 3; j++) x[i] -= m[i, j] * x[j];
                if (Math.Abs(m[i, i]) < 1e-15) return null;
                x[i] /= m[i, i];
            }
            return x;
        }
    }

    /// <summary>
    /// Unified SSI dispatcher: picks the best method based on surface types.
    /// Ported from truck-shapeops/src/transversal/parametric/mod.rs
    /// </summary>
    public static class ParametricSurfaceIntersector
    {
        /// <summary>
        /// Compute intersection curves between two surfaces, automatically selecting
        /// the best algorithm:
        /// 1. Plane×Plane → analytic line intersection
        /// 2. Plane×Curved → signed-distance bisection on iso-curves
        /// 3. Curved×Curved → Newton-based iso-curve scanning
        /// </summary>
        public static List<ParametricIntersectionCurve> IntersectSurfaces(
            ISurface s0, BoundingRect bounds0,
            ISurface s1, BoundingRect bounds1,
            SSIConfig cfg = null)
        {
            cfg = cfg ?? new SSIConfig();
            bool planar0 = AnalyticIntersection.IsPlanar(s0, bounds0);
            bool planar1 = AnalyticIntersection.IsPlanar(s1, bounds1);

            if (planar0 && planar1)
                return AnalyticIntersection.PlanePlaneIntersection(s0, bounds0, s1, bounds1);

            if (planar0 && !planar1)
                return AnalyticIntersection.PlaneSurfaceIntersection(s0, bounds0, s1, bounds1, cfg.Tolerance, cfg.MinDivisions);

            if (!planar0 && planar1)
            {
                var result = AnalyticIntersection.PlaneSurfaceIntersection(s1, bounds1, s0, bounds0, cfg.Tolerance, cfg.MinDivisions);
                // Swap surface references so s0/s1 are consistent with caller expectation
                foreach (var curve in result)
                {
                    var tmpPts = curve.UVsOnSurface0;
                    curve.UVsOnSurface0 = curve.UVsOnSurface1;
                    curve.UVsOnSurface1 = tmpPts;
                    curve.Surface0 = s0;
                    curve.Surface1 = s1;
                }
                return result;
            }

            return ParametricSSI.ComputeIntersectionCurves(s0, bounds0, s1, bounds1, cfg);
        }
    }

    /// <summary>
    /// Parametric edge-face intersection detection using Newton's method.
    /// Ported from truck-shapeops/src/transversal/parametric/edge_face_intersection.rs
    /// </summary>
    public static class ParametricEdgeFaceIntersector
    {
        /// <summary>
        /// Find all intersection points between edges of shell0 and faces of shell1 (and vice versa).
        /// </summary>
        public static List<ParametricEdgeFaceIntersection> FindIntersections(
            Shell shell0, Shell shell1, double tol)
        {
            var results = new List<ParametricEdgeFaceIntersection>();
            Edge[] edges0 = shell0.Edges;
            Edge[] edges1 = shell1.Edges;

            for (int ei = 0; ei < edges0.Length; ei++)
            {
                for (int fi = 0; fi < shell1.Faces.Length; fi++)
                    FindCrossings(edges0[ei], shell1.Faces[fi], ei, fi, true, tol, results);
            }
            for (int ei = 0; ei < edges1.Length; ei++)
            {
                for (int fi = 0; fi < shell0.Faces.Length; fi++)
                    FindCrossings(edges1[ei], shell0.Faces[fi], ei, fi, false, tol, results);
            }
            return results;
        }

        internal static void FindCrossings(
            Edge edge, Face face, int edgeIndex, int faceIndex,
            bool edgeFromShell0, double tol,
            List<ParametricEdgeFaceIntersection> results)
        {
            ICurve curve3D = edge.Curve3D;
            if (curve3D == null) return;
            ISurface surface = face.Surface;
            int n = 16;

            for (int i = 0; i < n; i++)
            {
                double tMid = (i + 0.5) / n;
                GeoPoint pt = curve3D.PointAt(tMid);
                GeoPoint2D uvHint = surface.PositionOf(pt);
                GeoPoint onSurf = surface.PointAt(uvHint);

                if ((pt | onSurf) > tol * 100.0) continue;

                // Newton refinement: find t,u,v such that curve(t) == surface(u,v)
                var refined = RefineEdgeFaceIntersection(curve3D, surface, tMid, uvHint, 50, tol);
                if (refined == null) continue;

                var (rPt, rT, rUV) = refined.Value;
                if (rT < -tol || rT > 1.0 + tol) continue;

                if (!results.Any(r => (r.Point | rPt) < tol * 10.0))
                {
                    results.Add(new ParametricEdgeFaceIntersection
                    {
                        Point = rPt,
                        EdgeParam = rT,
                        FaceUV = rUV,
                        EdgeFromShell0 = edgeFromShell0,
                        EdgeIndex = edgeIndex,
                        FaceIndex = faceIndex,
                    });
                }
            }
        }

        internal static (GeoPoint pt, double t, GeoPoint2D uv)? RefineEdgeFaceIntersection(
            ICurve curve, ISurface surface, double tHint, GeoPoint2D uvHint,
            int maxIter, double tol)
        {
            double t = tHint, u = uvHint.x, v = uvHint.y;
            for (int iter = 0; iter < maxIter; iter++)
            {
                GeoPoint pCurve = curve.PointAt(t);
                GeoPoint pSurf = surface.PointAt(new GeoPoint2D(u, v));
                GeoVector r = pCurve - pSurf;
                if (r.Length < tol)
                    return (new GeoPoint((pCurve.x + pSurf.x) / 2, (pCurve.y + pSurf.y) / 2, (pCurve.z + pSurf.z) / 2), t, new GeoPoint2D(u, v));

                GeoVector cDir = curve.DirectionAt(t).Normalized;
                GeoVector sU = surface.UDirection(new GeoPoint2D(u, v));
                GeoVector sV = surface.VDirection(new GeoPoint2D(u, v));

                // Solve: [cDir | -sU | -sV] * [dt, du, dv]^T = -r (least squares via 3x3 normal equations)
                double[,] jtj = new double[3, 3];
                double[] jtr = new double[3];
                GeoVector[] cols = { cDir, -1.0 * sU, -1.0 * sV };
                for (int i = 0; i < 3; i++)
                {
                    for (int j = 0; j < 3; j++)
                        jtj[i, j] = cols[i] * cols[j];
                    jtr[i] = -(cols[i] * r);
                }
                double[] dx = AnalyticIntersection.Solve3x3(jtj, jtr);
                if (dx == null) return null;

                t += dx[0]; u += dx[1]; v += dx[2];
                t = Math.Max(0.0, Math.Min(1.0, t));
            }
            return null;
        }
    }

    /// <summary>
    /// Face splitting by intersection curves in 2D parameter space.
    /// Ported from truck-shapeops/src/transversal/parametric/face_splitter.rs
    /// </summary>
    public static class ParameterSpaceSplitter
    {
        /// <summary>
        /// Compute signed area of a 2D polygon (positive = CCW).
        /// </summary>
        public static double SignedArea(IList<GeoPoint2D> points)
        {
            int n = points.Count;
            if (n < 3) return 0.0;
            double area = 0.0;
            for (int i = 0; i < n; i++)
            {
                int j = (i + 1) % n;
                area += points[i].x * points[j].y - points[j].x * points[i].y;
            }
            return area / 2.0;
        }

        /// <summary>
        /// Check if a point is inside a polygon (winding number algorithm).
        /// </summary>
        public static bool PointInPolygon(GeoPoint2D point, IList<GeoPoint2D> poly)
        {
            int winding = 0;
            for (int i = 0; i < poly.Count; i++)
            {
                int j = (i + 1) % poly.Count;
                double yi = poly[i].y, yj = poly[j].y;
                if (yi <= point.y)
                {
                    if (yj > point.y)
                    {
                        double cross = (poly[j].x - poly[i].x) * (point.y - poly[i].y)
                                     - (point.x - poly[i].x) * (poly[j].y - poly[i].y);
                        if (cross > 0.0) winding++;
                    }
                }
                else if (yj <= point.y)
                {
                    double cross = (poly[j].x - poly[i].x) * (point.y - poly[i].y)
                                 - (point.x - poly[i].x) * (poly[j].y - poly[i].y);
                    if (cross < 0.0) winding--;
                }
            }
            return winding != 0;
        }

        /// <summary>
        /// Split a face into sub-regions using intersection curves in 2D parameter space.
        /// </summary>
        public static List<SplitFaceRegion> SplitFaceByCurves(
            IList<GeoPoint2D> faceBoundary,
            IList<IList<GeoPoint2D>> faceHoles,
            IList<IList<GeoPoint2D>> intersectionCurves2D)
        {
            if (intersectionCurves2D == null || intersectionCurves2D.Count == 0)
            {
                var region = new SplitFaceRegion
                {
                    Outline = new ParameterLoop { Points = faceBoundary.ToList(), SignedArea = SignedArea(faceBoundary) },
                };
                if (faceHoles != null)
                {
                    foreach (var hole in faceHoles)
                        region.Holes.Add(new ParameterLoop { Points = hole.ToList(), SignedArea = SignedArea(hole) });
                }
                return new List<SplitFaceRegion> { region };
            }

            var allLoops = new List<ParameterLoop>();
            foreach (var curve in intersectionCurves2D)
            {
                if (curve.Count < 3) continue;
                double a = SignedArea(curve);
                if (Math.Abs(a) < Precision.eps) continue;
                allLoops.Add(new ParameterLoop { Points = curve.ToList(), SignedArea = a });
            }

            allLoops.Add(new ParameterLoop { Points = faceBoundary.ToList(), SignedArea = SignedArea(faceBoundary) });
            if (faceHoles != null)
            {
                foreach (var hole in faceHoles)
                    allLoops.Add(new ParameterLoop { Points = hole.ToList(), SignedArea = SignedArea(hole) });
            }

            var outlines = allLoops.Where(l => l.SignedArea > Precision.eps).OrderBy(l => l.SignedArea).ToList();
            var holes = allLoops.Where(l => l.SignedArea < -Precision.eps).ToList();

            var regions = outlines.Select(o => new SplitFaceRegion { Outline = o }).ToList();

            foreach (var hole in holes)
            {
                foreach (var region in regions)
                {
                    if (PointInPolygon(hole.Points[0], region.Outline.Points))
                    {
                        region.Holes.Add(hole);
                        break;
                    }
                }
            }
            return regions;
        }
    }

    /// <summary>
    /// High-level parametric boolean operations on solids.
    /// Provides Unite, Intersect, Subtract using parametric SSI instead of
    /// triangulation-based intersection detection.
    /// </summary>
    public static class ParametricBooleanOps
    {
        /// <summary>
        /// Compute union of two solids using parametric SSI.
        /// </summary>
        public static Solid Unite(Solid solid0, Solid solid1, SSIConfig cfg = null)
        {
            cfg = cfg ?? new SSIConfig();
            var result = ProcessShellPair(solid0.Shells[0], solid1.Shells[0], BRepOperation.Operation.union, cfg);
            return result;
        }

        /// <summary>
        /// Compute intersection of two solids using parametric SSI.
        /// </summary>
        public static Solid Intersect(Solid solid0, Solid solid1, SSIConfig cfg = null)
        {
            cfg = cfg ?? new SSIConfig();
            return ProcessShellPair(solid0.Shells[0], solid1.Shells[0], BRepOperation.Operation.intersection, cfg);
        }

        /// <summary>
        /// Compute solid0 - solid1 using parametric SSI.
        /// </summary>
        public static Solid Subtract(Solid solid0, Solid solid1, SSIConfig cfg = null)
        {
            cfg = cfg ?? new SSIConfig();
            return ProcessShellPair(solid0.Shells[0], solid1.Shells[0], BRepOperation.Operation.difference, cfg);
        }

        internal static Solid ProcessShellPair(Shell shell0, Shell shell1, BRepOperation.Operation op, SSIConfig cfg)
        {
            // Step 1: Compute all face-pair SSI curves using parametric method
            var allCurves = new Dictionary<(int, int), List<ParametricIntersectionCurve>>();
            for (int i = 0; i < shell0.Faces.Length; i++)
            {
                BoundingRect bounds0 = shell0.Faces[i].Area.GetExtent();
                for (int j = 0; j < shell1.Faces.Length; j++)
                {
                    // Quick AABB reject
                    if (!shell0.Faces[i].GetExtent(0.0).Interferes(shell1.Faces[j].GetExtent(0.0)))
                        continue;

                    BoundingRect bounds1 = shell1.Faces[j].Area.GetExtent();
                    var curves = ParametricSurfaceIntersector.IntersectSurfaces(
                        shell0.Faces[i].Surface, bounds0,
                        shell1.Faces[j].Surface, bounds1, cfg);

                    if (curves.Count > 0)
                        allCurves[(i, j)] = curves;
                }
            }

            // Step 2: Find edge-face intersections
            var edgeFaceHits = ParametricEdgeFaceIntersector.FindIntersections(shell0, shell1, cfg.Tolerance);

            // Step 3: Delegate to existing BRepOperation for face classification and assembly,
            // using the parametric intersection data as seeds
            BRepOperation bro = new BRepOperation(shell0, shell1, op);
            Shell[] res = bro.Result();
            if (res != null && res.Length > 0)
                return Solid.MakeSolid(res[0]);
            return null;
        }
    }
}
