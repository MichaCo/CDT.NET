// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
//
// Geometric predicates — exact (arbitrary-precision) paths.

using System.Runtime.CompilerServices;

namespace CDT.Predicates;

/// <summary>
/// Geometric predicates using arbitrary-precision floating-point arithmetic.
/// These produce exact results regardless of degeneracy but are significantly
/// slower than <see cref="PredicatesAdaptive"/>. Provided primarily for
/// reference, verification, and degenerate-case testing.
/// Corresponds to C++ <c>predicates::exact</c>.
/// </summary>
/// <remarks>
/// Reference: https://www.cs.cmu.edu/~quake/robust.html
/// </remarks>
/// <seealso cref="PredicatesAdaptive"/>
public static class PredicatesExact
{
    // =========================================================================
    // Orient2d
    // =========================================================================

    /// <summary>
    /// Exact orient2d predicate for <see cref="double"/> coordinates using
    /// arbitrary-precision arithmetic. Always produces a correct sign even for
    /// nearly-collinear points.
    /// Returns the determinant of {{ax-cx, ay-cy}, {bx-cx, by-cy}}.
    /// Positive = c is left/above the line a→b; zero = collinear; negative = right/below.
    /// </summary>
    /// <param name="ax">X-coordinate of point a.</param>
    /// <param name="ay">Y-coordinate of point a.</param>
    /// <param name="bx">X-coordinate of point b.</param>
    /// <param name="by">Y-coordinate of point b.</param>
    /// <param name="cx">X-coordinate of point c.</param>
    /// <param name="cy">Y-coordinate of point c.</param>
    /// <returns>
    /// A positive value if c is to the left of line a→b,
    /// zero if collinear, or a negative value if to the right.
    /// </returns>
    /// <seealso cref="PredicatesAdaptive.Orient2d(double, double, double, double, double, double)"/>
    [SkipLocalsInit]
    public static double Orient2d(
        double ax, double ay, double bx, double by, double cx, double cy)
    {
        double acx = ax - cx, bcx = bx - cx;
        double acy = ay - cy, bcy = by - cy;

        Span<double> s1 = stackalloc double[4];
        Span<double> s2 = stackalloc double[4];
        Span<double> s3 = stackalloc double[4];
        Span<double> b = stackalloc double[4];

        int bLen = PredicatesAdaptive.TwoTwoDiff(acx, bcy, acy, bcx, b);

        double acxtail = PredicatesAdaptive.MinusTail(ax, cx, acx);
        double bcxtail = PredicatesAdaptive.MinusTail(bx, cx, bcx);
        double acytail = PredicatesAdaptive.MinusTail(ay, cy, acy);
        double bcytail = PredicatesAdaptive.MinusTail(by, cy, bcy);

        int s1Len = PredicatesAdaptive.TwoTwoDiff(acxtail, bcy, acytail, bcx, s1);
        int s2Len = PredicatesAdaptive.TwoTwoDiff(acx, bcytail, acy, bcxtail, s2);
        int s3Len = PredicatesAdaptive.TwoTwoDiff(acxtail, bcytail, acytail, bcxtail, s3);

        Span<double> t1 = stackalloc double[8];
        int t1Len = PredicatesAdaptive.ExpansionSum(b, bLen, s1, s1Len, t1);
        Span<double> t2 = stackalloc double[12];
        int t2Len = PredicatesAdaptive.ExpansionSum(t1, t1Len, s2, s2Len, t2);
        Span<double> d = stackalloc double[16];
        int dLen = PredicatesAdaptive.ExpansionSum(t2, t2Len, s3, s3Len, d);
        return PredicatesAdaptive.MostSignificant(d, dLen);
    }

    /// <summary>
    /// Exact orient2d predicate for <see cref="float"/> coordinates using
    /// arbitrary-precision arithmetic.
    /// Returns the determinant of {{ax-cx, ay-cy}, {bx-cx, by-cy}}.
    /// Positive = c is left/above the line a→b; zero = collinear; negative = right/below.
    /// </summary>
    /// <param name="ax">X-coordinate of point a.</param>
    /// <param name="ay">Y-coordinate of point a.</param>
    /// <param name="bx">X-coordinate of point b.</param>
    /// <param name="by">Y-coordinate of point b.</param>
    /// <param name="cx">X-coordinate of point c.</param>
    /// <param name="cy">Y-coordinate of point c.</param>
    /// <returns>
    /// A positive value if c is to the left of line a→b,
    /// zero if collinear, or a negative value if to the right.
    /// </returns>
    /// <seealso cref="PredicatesAdaptive.Orient2d(float, float, float, float, float, float)"/>
    public static float Orient2d(
        float ax, float ay, float bx, float by, float cx, float cy)
    {
        // Each product of two 24-bit floats is exact in 53-bit double.
        float acx = ax - cx, bcx = bx - cx;
        float acy = ay - cy, bcy = by - cy;
        double exact = (double)acx * bcy - (double)acy * bcx;
        return (float)exact;
    }

    // =========================================================================
    // InCircle
    // =========================================================================

    /// <summary>
    /// Exact incircle predicate for <see cref="double"/> coordinates using
    /// arbitrary-precision arithmetic.
    /// Returns positive if <c>d</c> is inside the circumcircle of (a, b, c),
    /// zero if on, negative if outside.
    /// </summary>
    /// <param name="ax">X-coordinate of triangle vertex a.</param>
    /// <param name="ay">Y-coordinate of triangle vertex a.</param>
    /// <param name="bx">X-coordinate of triangle vertex b.</param>
    /// <param name="by">Y-coordinate of triangle vertex b.</param>
    /// <param name="cx">X-coordinate of triangle vertex c.</param>
    /// <param name="cy">Y-coordinate of triangle vertex c.</param>
    /// <param name="dx">X-coordinate of query point d.</param>
    /// <param name="dy">Y-coordinate of query point d.</param>
    /// <returns>
    /// A positive value if d is inside the circumcircle of (a, b, c),
    /// zero if on, or a negative value if outside.
    /// </returns>
    /// <seealso cref="PredicatesAdaptive.InCircle(double, double, double, double, double, double, double, double)"/>
    [SkipLocalsInit]
    public static double InCircle(
        double ax, double ay, double bx, double by,
        double cx, double cy, double dx, double dy)
    {
        Span<double> abE = stackalloc double[4];
        Span<double> bcE = stackalloc double[4];
        Span<double> cdE = stackalloc double[4];
        Span<double> daE = stackalloc double[4];
        Span<double> acE = stackalloc double[4];
        Span<double> bdE = stackalloc double[4];
        int abLen = PredicatesAdaptive.TwoTwoDiff(ax, by, bx, ay, abE);
        int bcLen = PredicatesAdaptive.TwoTwoDiff(bx, cy, cx, by, bcE);
        int cdLen = PredicatesAdaptive.TwoTwoDiff(cx, dy, dx, cy, cdE);
        int daLen = PredicatesAdaptive.TwoTwoDiff(dx, ay, ax, dy, daE);
        int acLen = PredicatesAdaptive.TwoTwoDiff(ax, cy, cx, ay, acE);
        int bdLen = PredicatesAdaptive.TwoTwoDiff(bx, dy, dx, by, bdE);

        // abc = ab + bc - ac
        Span<double> negAc = stackalloc double[4];
        PredicatesAdaptive.NegateInto(acE, acLen, negAc);
        Span<double> abbc = stackalloc double[8];
        int abbcLen = PredicatesAdaptive.ExpansionSum(abE, abLen, bcE, bcLen, abbc);
        Span<double> abc = stackalloc double[12];
        int abcLen = PredicatesAdaptive.ExpansionSum(abbc, abbcLen, negAc, acLen, abc);

        // bcd = bc + cd - bd
        Span<double> negBd = stackalloc double[4];
        PredicatesAdaptive.NegateInto(bdE, bdLen, negBd);
        Span<double> bccd = stackalloc double[8];
        int bccdLen = PredicatesAdaptive.ExpansionSum(bcE, bcLen, cdE, cdLen, bccd);
        Span<double> bcd = stackalloc double[12];
        int bcdLen = PredicatesAdaptive.ExpansionSum(bccd, bccdLen, negBd, bdLen, bcd);

        // cda = cd + da + ac
        Span<double> cdda = stackalloc double[8];
        int cddaLen = PredicatesAdaptive.ExpansionSum(cdE, cdLen, daE, daLen, cdda);
        Span<double> cda = stackalloc double[12];
        int cdaLen = PredicatesAdaptive.ExpansionSum(cdda, cddaLen, acE, acLen, cda);

        // dab = da + ab + bd
        Span<double> daab = stackalloc double[8];
        int daabLen = PredicatesAdaptive.ExpansionSum(daE, daLen, abE, abLen, daab);
        Span<double> dab = stackalloc double[12];
        int dabLen = PredicatesAdaptive.ExpansionSum(daab, daabLen, bdE, bdLen, dab);

        // adet = bcd*ax*ax + bcd*ay*ay
        Span<double> adet = stackalloc double[96];
        int adetLen = PredicatesAdaptive.ScaleExpansionSum(bcd, bcdLen, ax, ay, adet);

        // bdet = -(cda*bx*bx + cda*by*by)
        Span<double> bdetPos = stackalloc double[96];
        int bdetPosLen = PredicatesAdaptive.ScaleExpansionSum(cda, cdaLen, bx, by, bdetPos);
        Span<double> bdet = stackalloc double[96];
        PredicatesAdaptive.NegateInto(bdetPos, bdetPosLen, bdet);
        int bdetLen = bdetPosLen;

        // cdet = dab*cx*cx + dab*cy*cy
        Span<double> cdet = stackalloc double[96];
        int cdetLen = PredicatesAdaptive.ScaleExpansionSum(dab, dabLen, cx, cy, cdet);

        // ddet = -(abc*dx*dx + abc*dy*dy)
        Span<double> ddetPos = stackalloc double[96];
        int ddetPosLen = PredicatesAdaptive.ScaleExpansionSum(abc, abcLen, dx, dy, ddetPos);
        Span<double> ddet = stackalloc double[96];
        PredicatesAdaptive.NegateInto(ddetPos, ddetPosLen, ddet);
        int ddetLen = ddetPosLen;

        // deter = (adet + bdet) + (cdet + ddet)
        Span<double> ab2 = stackalloc double[192];
        int ab2Len = PredicatesAdaptive.ExpansionSum(adet, adetLen, bdet, bdetLen, ab2);
        Span<double> cd2 = stackalloc double[192];
        int cd2Len = PredicatesAdaptive.ExpansionSum(cdet, cdetLen, ddet, ddetLen, cd2);
        Span<double> deter = stackalloc double[384];
        int deterLen = PredicatesAdaptive.ExpansionSum(ab2, ab2Len, cd2, cd2Len, deter);

        return PredicatesAdaptive.MostSignificant(deter, deterLen);
    }

    /// <summary>
    /// Exact incircle predicate for <see cref="float"/> coordinates using
    /// arbitrary-precision arithmetic.
    /// Returns positive if <c>d</c> is inside the circumcircle of (a, b, c),
    /// zero if on, negative if outside.
    /// </summary>
    /// <param name="ax">X-coordinate of triangle vertex a.</param>
    /// <param name="ay">Y-coordinate of triangle vertex a.</param>
    /// <param name="bx">X-coordinate of triangle vertex b.</param>
    /// <param name="by">Y-coordinate of triangle vertex b.</param>
    /// <param name="cx">X-coordinate of triangle vertex c.</param>
    /// <param name="cy">Y-coordinate of triangle vertex c.</param>
    /// <param name="dx">X-coordinate of query point d.</param>
    /// <param name="dy">Y-coordinate of query point d.</param>
    /// <returns>
    /// A positive value if d is inside the circumcircle of (a, b, c),
    /// zero if on, or a negative value if outside.
    /// </returns>
    /// <seealso cref="PredicatesAdaptive.InCircle(float, float, float, float, float, float, float, float)"/>
    public static float InCircle(
        float ax, float ay, float bx, float by,
        float cx, float cy, float dx, float dy)
    {
        float adx = ax - dx, bdx = bx - dx, cdx = cx - dx;
        float ady = ay - dy, bdy = by - dy, cdy = cy - dy;
        decimal madx = (decimal)adx, mbdx = (decimal)bdx, mcdx = (decimal)cdx;
        decimal mady = (decimal)ady, mbdy = (decimal)bdy, mcdy = (decimal)cdy;
        decimal mdet = (madx * madx + mady * mady) * (mbdx * mcdy - mcdx * mbdy)
                     + (mbdx * mbdx + mbdy * mbdy) * (mcdx * mady - madx * mcdy)
                     + (mcdx * mcdx + mcdy * mcdy) * (madx * mbdy - mbdx * mady);
        return mdet > 0m ? float.Epsilon : mdet < 0m ? -float.Epsilon : 0f;
    }
}
