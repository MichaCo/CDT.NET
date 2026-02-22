// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
//
// Geometric predicates — port of Lenthe/Shewchuk adaptive predicates from
// artem-ogre/CDT (predicates.h, predicates::adaptive namespace).

using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using System.Runtime.Intrinsics;

namespace CDT.Predicates;

/// <summary>
/// Geometric predicates using normal floating-point arithmetic,
/// falling back to arbitrary precision when needed for robustness.
/// These are significantly faster than <see cref="PredicatesExact"/> when
/// the determinant is large (i.e. the non-degenerate case), but produce
/// the same correct result in all cases.
/// Corresponds to C++ <c>predicates::adaptive</c>.
/// </summary>
/// <remarks>
/// Reference: https://www.cs.cmu.edu/~quake/robust.html
/// </remarks>
/// <seealso cref="PredicatesExact"/>
public static class PredicatesAdaptive
{
    // -------------------------------------------------------------------------
    // Error-bound constants (double: eps = 2^-53, float: eps = 2^-24)
    // -------------------------------------------------------------------------
    private const double CcwBoundAD = 3.3306690621773814e-16;
    private const double CcwBoundBD = 2.2204460492503131e-16;
    private const double CcwBoundCD = 1.1093356479670487e-31;
    private const double IccBoundAD = 1.1102230246251565e-15;
    private const double IccBoundBD = 4.440892098500626e-15;
    private const double IccBoundCD = 5.423306525521214e-31;
    private const double ResultErrBound = 3.3306690738754716e-16;
    private const float CcwBoundAF = 1.7881393432617188e-7f;
    private const float IccBoundAF = 5.960464477539063e-7f;
    internal const double SplitterD = 134217729.0; // 2^27 + 1

    // =========================================================================
    // Orient2d
    // =========================================================================

    /// <summary>
    /// Adaptive orient2d predicate for <see cref="double"/> coordinates.
    /// Determines whether point <c>c</c> is to the left of, on, or to the right of
    /// the directed line from <c>a</c> to <c>b</c>.
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
    /// <seealso cref="PredicatesExact.Orient2d(double, double, double, double, double, double)"/>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    [SkipLocalsInit]
    public static double Orient2d(
        double ax, double ay, double bx, double by, double cx, double cy)
    {
        double acx = ax - cx, bcx = bx - cx;
        double acy = ay - cy, bcy = by - cy;
        double detleft = acx * bcy;
        double detright = acy * bcx;
        double det = detleft - detright;

        if ((detleft < 0.0) != (detright < 0.0))
        {
            return det;
        }

        if (detleft == 0.0 || detright == 0.0)
        {
            return det;
        }

        double detsum = Math.Abs(detleft + detright);
        if (Math.Abs(det) >= CcwBoundAD * detsum)
        {
            return det;
        }

        // Stage B
        Span<double> b = stackalloc double[4];
        int bLen = TwoTwoDiff(acx, bcy, acy, bcx, b);
        det = Estimate(b, bLen);
        if (Math.Abs(det) >= CcwBoundBD * detsum)
        {
            return det;
        }

        // Stage C
        double acxtail = MinusTail(ax, cx, acx);
        double bcxtail = MinusTail(bx, cx, bcx);
        double acytail = MinusTail(ay, cy, acy);
        double bcytail = MinusTail(by, cy, bcy);

        if (acxtail == 0.0 && bcxtail == 0.0 && acytail == 0.0 && bcytail == 0.0)
        {
            return det;
        }

        double errC = CcwBoundCD * detsum + ResultErrBound * Math.Abs(det);
        det += (acx * bcytail + bcy * acxtail) - (acy * bcxtail + bcx * acytail);
        if (Math.Abs(det) >= errC)
        {
            return det;
        }

        // Stage D: exact expansion
        return PredicatesExact.Orient2d(ax, ay, bx, by, cx, cy);
    }

    /// <summary>
    /// Adaptive orient2d predicate for <see cref="float"/> coordinates.
    /// Determines whether point <c>c</c> is to the left of, on, or to the right of
    /// the directed line from <c>a</c> to <c>b</c>.
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
    /// <remarks>
    /// All fast-path arithmetic is performed in single precision.
    /// Falls back to exact double computation when needed.
    /// </remarks>
    /// <seealso cref="PredicatesExact.Orient2d(float, float, float, float, float, float)"/>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static float Orient2d(
        float ax, float ay, float bx, float by, float cx, float cy)
    {
        float acx = ax - cx, bcx = bx - cx;
        float acy = ay - cy, bcy = by - cy;
        float detleft = acx * bcy;
        float detright = acy * bcx;
        float det = detleft - detright;

        if ((detleft < 0f) != (detright < 0f))
        {
            return det;
        }

        if (detleft == 0f || detright == 0f)
        {
            return det;
        }

        float detsum = MathF.Abs(detleft + detright);
        if (MathF.Abs(det) >= CcwBoundAF * detsum)
        {
            return det;
        }

        // Exact: each product of two 24-bit floats is exact in 53-bit double.
        double exact = (double)acx * bcy - (double)acy * bcx;
        return (float)exact;
    }

    // =========================================================================
    // InCircle
    // =========================================================================

    /// <summary>
    /// Adaptive incircle predicate for <see cref="double"/> coordinates.
    /// Determines whether point <c>d</c> is inside, on, or outside the circumcircle
    /// of the triangle defined by <c>a</c>, <c>b</c>, <c>c</c> (in CCW order).
    /// Returns positive if <c>d</c> is inside, zero if on, negative if outside.
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
    /// A positive value if d is inside the circumcircle,
    /// zero if on, or a negative value if outside.
    /// </returns>
    /// <seealso cref="PredicatesExact.InCircle(double, double, double, double, double, double, double, double)"/>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    [SkipLocalsInit]
    public static double InCircle(
        double ax, double ay, double bx, double by,
        double cx, double cy, double dx, double dy)
    {
        double adx = ax - dx, bdx = bx - dx, cdx = cx - dx;
        double ady = ay - dy, bdy = by - dy, cdy = cy - dy;

        double bdxcdy = bdx * cdy, cdxbdy = cdx * bdy;
        double cdxady = cdx * ady, adxcdy = adx * cdy;
        double adxbdy = adx * bdy, bdxady = bdx * ady;

        double alift = adx * adx + ady * ady;
        double blift = bdx * bdx + bdy * bdy;
        double clift = cdx * cdx + cdy * cdy;

        double det = alift * (bdxcdy - cdxbdy)
                   + blift * (cdxady - adxcdy)
                   + clift * (adxbdy - bdxady);

        double permanent = (Math.Abs(bdxcdy) + Math.Abs(cdxbdy)) * alift
                         + (Math.Abs(cdxady) + Math.Abs(adxcdy)) * blift
                         + (Math.Abs(adxbdy) + Math.Abs(bdxady)) * clift;

        if (Math.Abs(det) >= IccBoundAD * permanent)
        {
            return det;
        }

        // Stage B
        Span<double> bc = stackalloc double[4];
        Span<double> ca = stackalloc double[4];
        Span<double> ab = stackalloc double[4];
        int bcLen = TwoTwoDiff(bdx, cdy, cdx, bdy, bc);
        int caLen = TwoTwoDiff(cdx, ady, adx, cdy, ca);
        int abLen = TwoTwoDiff(adx, bdy, bdx, ady, ab);

        Span<double> adet = stackalloc double[32];
        int adetLen = ScaleExpansionSum(bc, bcLen, adx, ady, adet);
        Span<double> bdet = stackalloc double[32];
        int bdetLen = ScaleExpansionSum(ca, caLen, bdx, bdy, bdet);
        Span<double> cdet = stackalloc double[32];
        int cdetLen = ScaleExpansionSum(ab, abLen, cdx, cdy, cdet);

        Span<double> abSum = stackalloc double[64];
        int abSumLen = ExpansionSum(adet, adetLen, bdet, bdetLen, abSum);
        Span<double> fin1 = stackalloc double[96];
        int fin1Len = ExpansionSum(abSum, abSumLen, cdet, cdetLen, fin1);

        det = Estimate(fin1, fin1Len);
        if (Math.Abs(det) >= IccBoundBD * permanent)
        {
            return det;
        }

        // Stage C
        double adxtail = MinusTail(ax, dx, adx);
        double adytail = MinusTail(ay, dy, ady);
        double bdxtail = MinusTail(bx, dx, bdx);
        double bdytail = MinusTail(by, dy, bdy);
        double cdxtail = MinusTail(cx, dx, cdx);
        double cdytail = MinusTail(cy, dy, cdy);

        if (adxtail == 0.0 && adytail == 0.0 && bdxtail == 0.0
            && bdytail == 0.0 && cdxtail == 0.0 && cdytail == 0.0)
        {
            return det;
        }

        double errC = IccBoundCD * permanent + ResultErrBound * Math.Abs(det);
        // Stage C correction — direct port of Lenthe predicates.h (Stage C terms).
        // Each group is: lift * (cross-product tails) + cross-product * (lift tails)*2
        det += ((adx * adx + ady * ady) * ((bdx * cdytail + cdy * bdxtail) - (bdy * cdxtail + cdx * bdytail))
              + (bdx * cdy - bdy * cdx) * (adx * adxtail + ady * adytail) * 2.0)
             + ((bdx * bdx + bdy * bdy) * ((cdx * adytail + ady * cdxtail) - (cdy * adxtail + adx * cdytail))
              + (cdx * ady - cdy * adx) * (bdx * bdxtail + bdy * bdytail) * 2.0)
             + ((cdx * cdx + cdy * cdy) * ((adx * bdytail + bdy * adxtail) - (ady * bdxtail + bdx * adytail))
              + (adx * bdy - ady * bdx) * (cdx * cdxtail + cdy * cdytail) * 2.0);
        if (Math.Abs(det) >= errC)
        {
            return det;
        }

        // Stage D: exact
        return PredicatesExact.InCircle(ax, ay, bx, by, cx, cy, dx, dy);
    }

    /// <summary>
    /// Adaptive incircle predicate for <see cref="float"/> coordinates.
    /// Determines whether point <c>d</c> is inside, on, or outside the circumcircle
    /// of the triangle defined by <c>a</c>, <c>b</c>, <c>c</c> (in CCW order).
    /// Returns positive if <c>d</c> is inside, zero if on, negative if outside.
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
    /// A positive value if d is inside the circumcircle,
    /// zero if on, or a negative value if outside.
    /// </returns>
    /// <seealso cref="PredicatesExact.InCircle(float, float, float, float, float, float, float, float)"/>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static float InCircle(
        float ax, float ay, float bx, float by,
        float cx, float cy, float dx, float dy)
    {
        float adx = ax - dx, bdx = bx - dx, cdx = cx - dx;
        float ady = ay - dy, bdy = by - dy, cdy = cy - dy;

        float bdxcdy = bdx * cdy, cdxbdy = cdx * bdy;
        float cdxady = cdx * ady, adxcdy = adx * cdy;
        float adxbdy = adx * bdy, bdxady = bdx * ady;

        float alift = adx * adx + ady * ady;
        float blift = bdx * bdx + bdy * bdy;
        float clift = cdx * cdx + cdy * cdy;

        float det = alift * (bdxcdy - cdxbdy)
                  + blift * (cdxady - adxcdy)
                  + clift * (adxbdy - bdxady);

        float permanent = (MathF.Abs(bdxcdy) + MathF.Abs(cdxbdy)) * alift
                        + (MathF.Abs(cdxady) + MathF.Abs(adxcdy)) * blift
                        + (MathF.Abs(adxbdy) + MathF.Abs(bdxady)) * clift;

        if (MathF.Abs(det) >= IccBoundAF * permanent)
        {
            return det;
        }

        return PredicatesExact.InCircle(ax, ay, bx, by, cx, cy, dx, dy);
    }

    // =========================================================================
    // Shewchuk/Lenthe floating-point expansion primitives (internal)
    // =========================================================================

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static double PlusTail(double a, double b, double x)
    {
        double bv = x - a;
        return (a - (x - bv)) + (b - bv);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static double FastPlusTail(double a, double b, double x)
    {
        return b - (x - a);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static double MinusTail(double a, double b, double x)
    {
        double bv = a - x;
        double av = x + bv;
        return (a - av) + (bv - b);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static (double aHi, double aLo) Split(double a)
    {
        double c = SplitterD * a;
        double aBig = c - a;
        double aHi = c - aBig;
        return (aHi, a - aHi);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static double MultTail(double a, double b, double p)
    {
        double c = SplitterD * a;
        double aBig = c - a;
        double aHi = c - aBig;
        double aLo = a - aHi;
        c = SplitterD * b;
        double bBig = c - b;
        double bHi = c - bBig;
        double bLo = b - bHi;
        double y = p - aHi * bHi;
        y -= aLo * bHi;
        y -= aHi * bLo;
        return aLo * bLo - y;
    }

    /// <summary>
    /// Exact expansion of <c>ax*by - ay*bx</c> (up to 4 non-zero terms).
    /// Matches Lenthe <c>ExpansionBase::TwoTwoDiff</c>.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static int TwoTwoDiff(double ax, double by, double ay, double bx, Span<double> h)
    {
        double axby1 = ax * by;
        double axby0 = MultTail(ax, by, axby1);
        double bxay1 = bx * ay;
        double bxay0 = MultTail(bx, ay, bxay1);

        double i0 = axby0 - bxay0;
        double x0 = MinusTail(axby0, bxay0, i0);
        double j = axby1 + i0;
        double t0 = PlusTail(axby1, i0, j);
        double i1 = t0 - bxay1;
        double x1 = MinusTail(t0, bxay1, i1);
        double x3 = j + i1;
        double x2 = PlusTail(j, i1, x3);

        int n = 0;
        if (x0 != 0.0) { h[n++] = x0; }
        if (x1 != 0.0) { h[n++] = x1; }
        if (x2 != 0.0) { h[n++] = x2; }
        if (x3 != 0.0) { h[n++] = x3; }
        return n;
    }

    /// <summary>
    /// ScaleExpansion: <c>e * b</c> written to <c>h</c>.
    /// Matches Lenthe <c>ExpansionBase::ScaleExpansion</c>.
    /// Output has up to <c>2*elen</c> terms.
    /// </summary>
    internal static int ScaleExpansion(Span<double> e, int elen, double b, Span<double> h)
    {
        if (elen == 0 || b == 0.0)
        {
            return 0;
        }

        var (bHi, bLo) = Split(b);
        double Q = e[0] * b;
        double hh = DekkersPresplit(e[0], bHi, bLo, Q);
        int hIdx = 0;
        if (hh != 0.0) { h[hIdx++] = hh; }

        for (int i = 1; i < elen; i++)
        {
            double Ti = e[i] * b;
            double ti = DekkersPresplit(e[i], bHi, bLo, Ti);
            double Qi = Q + ti;
            hh = PlusTail(Q, ti, Qi);
            if (hh != 0.0) { h[hIdx++] = hh; }
            Q = Ti + Qi;
            hh = FastPlusTail(Ti, Qi, Q);
            if (hh != 0.0) { h[hIdx++] = hh; }
        }

        if (Q != 0.0) { h[hIdx++] = Q; }
        return hIdx;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static double DekkersPresplit(double a, double bHi, double bLo, double p)
    {
        double c = SplitterD * a;
        double aBig = c - a;
        double aHi = c - aBig;
        double aLo = a - aHi;
        double y = p - aHi * bHi;
        y -= aLo * bHi;
        y -= aHi * bLo;
        return aLo * bLo - y;
    }

    /// <summary>
    /// Computes <c>e*s*s + e*t*t</c> as an expansion (two ScaleExpansion calls each, then sum).
    /// Max output: 32 terms for 4-term input (used for InCircle Stage B lift terms).
    /// </summary>
    [SkipLocalsInit]
    internal static int ScaleExpansionSum(Span<double> e, int elen, double s, double t, Span<double> h)
    {
        Span<double> es = stackalloc double[8];
        int esLen = ScaleExpansion(e, elen, s, es);
        Span<double> ess = stackalloc double[16];
        int essLen = ScaleExpansion(es, esLen, s, ess);

        Span<double> et = stackalloc double[8];
        int etLen = ScaleExpansion(e, elen, t, et);
        Span<double> ett = stackalloc double[16];
        int ettLen = ScaleExpansion(et, etLen, t, ett);

        return ExpansionSum(ess, essLen, ett, ettLen, h);
    }

    /// <summary>
    /// Merge-then-accumulate two expansions. Matches Lenthe <c>ExpansionBase::ExpansionSum</c>:
    /// std::merge by |value| (stable), then sequential grow-expansion accumulation.
    /// </summary>
    [SkipLocalsInit]
    internal static int ExpansionSum(Span<double> e, int elen, Span<double> f, int flen, Span<double> h)
    {
        if (elen == 0 && flen == 0) { return 0; }
        if (elen == 0) { f[..flen].CopyTo(h); return flen; }
        if (flen == 0) { e[..elen].CopyTo(h); return elen; }

        int total = elen + flen;

        // Merge sorted by |value| into temporary buffer.
        // Maximum merged size for InCircle Stage D is 192+192=384 ≤ 400, so
        // the stackalloc path is always taken for that call site.
        Span<double> merged = total <= 400 ? stackalloc double[400] : new double[total];
        int ei = 0, fi = 0, mi = 0;
        while (ei < elen && fi < flen)
        {
            if (Math.Abs(f[fi]) < Math.Abs(e[ei]))
            {
                merged[mi++] = f[fi++];
            }
            else
            {
                merged[mi++] = e[ei++];
            }
        }

        while (ei < elen) { merged[mi++] = e[ei++]; }
        while (fi < flen) { merged[mi++] = f[fi++]; }

        // Sequential accumulation
        int hIdx = 0;
        double Q = merged[0];
        double Qnew = merged[1] + Q;
        double hh = FastPlusTail(merged[1], Q, Qnew);
        Q = Qnew;
        if (hh != 0.0) { h[hIdx++] = hh; }

        for (int g = 2; g < mi; g++)
        {
            Qnew = Q + merged[g];
            hh = PlusTail(Q, merged[g], Qnew);
            Q = Qnew;
            if (hh != 0.0) { h[hIdx++] = hh; }
        }

        if (Q != 0.0) { h[hIdx++] = Q; }
        return hIdx;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static double Estimate(Span<double> e, int elen)
    {
        if (Vector256.IsHardwareAccelerated && elen >= 4)
        {
            ref double eRef = ref MemoryMarshal.GetReference(e);
            Vector256<double> acc = Vector256<double>.Zero;
            int i = 0;
            for (; i <= elen - 4; i += 4)
                acc = Vector256.Add(acc, Vector256.LoadUnsafe(ref eRef, (nuint)i));
            double sum = Vector256.Sum(acc);
            for (; i < elen; i++) sum += Unsafe.Add(ref eRef, i);
            return sum;
        }

        double s = 0.0;
        for (int i = 0; i < elen; i++) { s += e[i]; }
        return s;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static double MostSignificant(Span<double> e, int elen)
    {
        for (int i = elen - 1; i >= 0; i--)
        {
            if (e[i] != 0.0) { return e[i]; }
        }
        return 0.0;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static void NegateInto(Span<double> src, int len, Span<double> dst)
    {
        if (Vector256.IsHardwareAccelerated && len >= 4)
        {
            ref double srcRef = ref MemoryMarshal.GetReference(src);
            ref double dstRef = ref MemoryMarshal.GetReference(dst);
            int i = 0;
            for (; i <= len - 4; i += 4)
                Vector256.Negate(Vector256.LoadUnsafe(ref srcRef, (nuint)i))
                         .StoreUnsafe(ref dstRef, (nuint)i);
            for (; i < len; i++) dst[i] = -src[i];
            return;
        }

        for (int i = 0; i < len; i++) { dst[i] = -src[i]; }
    }
}
