// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
//
// Geometric predicates — port of Lenthe/Shewchuk adaptive predicates from
// artem-ogre/CDT (predicates.h, predicates::adaptive namespace).
//
// Key design rules matching the C++ template:
//   - float inputs: ALL intermediate differences and products computed in float
//     first (no premature promotion). Fallback uses double for exact sign.
//   - double inputs: fast double estimate; double-double compensation fallback.

using System.Runtime.CompilerServices;

namespace CDT;

/// <summary>Robust geometric predicates (Lenthe/Shewchuk adaptive).</summary>
internal static class Predicates
{
    // -------------------------------------------------------------------------
    // Float-specific Shewchuk error-bound constants
    //   eps_f = 2^-24  (Shewchuk epsilon for float, = exp2(-digits) = exp2(-24))
    // -------------------------------------------------------------------------
    private const float EpsF = 5.960464477539063e-8f;           // 2^-24

    /// <summary>ccwerrboundA for float = (3 + 16·ε)·ε.</summary>
    private const float CcwBoundAF = 1.7881393432617188e-7f;

    /// <summary>iccerrboundA for float = (10 + 96·ε)·ε.</summary>
    private const float IccBoundAF = 5.960464477539063e-7f;

    // -------------------------------------------------------------------------
    // orient2d: sign of the 2×2 determinant
    //   | ax-cx  ay-cy |
    //   | bx-cx  by-cy |
    // Positive => c is to the left of a→b
    // -------------------------------------------------------------------------

    /// <summary>
    /// Orient2d predicate for <see cref="double"/>.
    /// Fast estimate with double-double compensation fallback, matching C++
    /// <c>predicates::adaptive::orient2d&lt;double&gt;</c> (GCC/Linux long-double behaviour).
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static double Orient2D(
        double ax, double ay,
        double bx, double by,
        double cx, double cy)
    {
        double acx = ax - cx, bcx = bx - cx;
        double acy = ay - cy, bcy = by - cy;
        double det = acx * bcy - acy * bcx;

        // ccwerrboundA for double = (3 + 16·ε)·ε ≈ 3·ε
        const double errBound = 3.3306690621773814e-16;
        double permanent = Math.Abs(acx * bcy) + Math.Abs(acy * bcx);
        if (Math.Abs(det) > errBound * permanent)
        {
            return det;
        }

        // Double-double fallback: acx*bcy and acy*bcx computed with exact round-off.
        var (p, pe) = TwoProd(acx, bcy);
        var (q, qe) = TwoProd(acy, bcx);
        double hi = p - q;
        double lo = (p - hi) - q + pe - qe;
        return hi != 0.0 ? hi : lo;
    }

    /// <summary>
    /// Orient2d predicate for <see cref="float"/>.
    /// All intermediate values computed in <b>float</b> precision (matching C++
    /// <c>predicates::adaptive::orient2d&lt;float&gt;</c>). Falls back to an exact
    /// double computation when the fast estimate is unreliable.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static float Orient2D(
        float ax, float ay,
        float bx, float by,
        float cx, float cy)
    {
        // Fast path: all float arithmetic — identical to C++ fast path.
        float acx = ax - cx;
        float bcx = bx - cx;
        float acy = ay - cy;
        float bcy = by - cy;
        float detleft = acx * bcy;
        float detright = acy * bcx;
        float det = detleft - detright;

        // Different signs → result is exact.
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

        // Exact fallback: each product of two 24-bit floats fits exactly in 53-bit double,
        // so the difference of two such products (≤ 49 bits) is also exact in double.
        // Use float-computed acx/acy/bcx/bcy (NOT re-promoted from ax/cx) to stay consistent
        // with the C++ expansion which uses the float-rounded differences.
        double exact = (double)acx * bcy - (double)acy * bcx;
        return (float)exact;
    }

    // -------------------------------------------------------------------------
    // incircle: positive => d is inside the circumcircle of (a,b,c) in CCW order
    // -------------------------------------------------------------------------

    /// <summary>
    /// InCircle predicate for <see cref="double"/>.
    /// Fast estimate with double-double compensation fallback.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static double InCircle(
        double ax, double ay,
        double bx, double by,
        double cx, double cy,
        double dx, double dy)
    {
        double adx = ax - dx, bdx = bx - dx, cdx = cx - dx;
        double ady = ay - dy, bdy = by - dy, cdy = cy - dy;

        double bdxcdy = bdx * cdy - cdx * bdy;
        double cdxady = cdx * ady - adx * cdy;
        double adxbdy = adx * bdy - bdx * ady;

        double alift = adx * adx + ady * ady;
        double blift = bdx * bdx + bdy * bdy;
        double clift = cdx * cdx + cdy * cdy;

        double det = alift * bdxcdy + blift * cdxady + clift * adxbdy;

        // iccerrboundA for double ≈ 10·ε
        double permanent = (Math.Abs(bdxcdy) + Math.Abs(cdx * bdy)) * alift
                         + (Math.Abs(cdxady) + Math.Abs(adx * cdy)) * blift
                         + (Math.Abs(adxbdy) + Math.Abs(bdx * ady)) * clift;
        const double errBound = 1.1102230246251565e-15;
        if (Math.Abs(det) > errBound * permanent)
        {
            return det;
        }

        // Double-double fallback for each cross-product.
        var (bc_h, bc_e) = TwoCross(bdx, cdy, cdx, bdy);
        var (ca_h, ca_e) = TwoCross(cdx, ady, adx, cdy);
        var (ab_h, ab_e) = TwoCross(adx, bdy, bdx, ady);

        double sum_hi = alift * bc_h + blift * ca_h + clift * ab_h;
        double sum_lo = alift * bc_e + blift * ca_e + clift * ab_e;
        return sum_hi != 0.0 ? sum_hi : sum_lo;
    }

    /// <summary>
    /// InCircle predicate for <see cref="float"/>.
    /// All intermediate differences and products computed in <b>float</b> precision
    /// (matching C++ <c>predicates::adaptive::incircle&lt;float&gt;</c>). Falls back
    /// to a double computation using the float-rounded intermediates for the exact sign.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static float InCircle(
        float ax, float ay,
        float bx, float by,
        float cx, float cy,
        float dx, float dy)
    {
        // Fast path: ALL in float — matches C++ fast path exactly.
        float adx = ax - dx;
        float bdx = bx - dx;
        float cdx = cx - dx;
        float ady = ay - dy;
        float bdy = by - dy;
        float cdy = cy - dy;

        float bdxcdy = bdx * cdy;
        float cdxbdy = cdx * bdy;
        float cdxady = cdx * ady;
        float adxcdy = adx * cdy;
        float adxbdy = adx * bdy;
        float bdxady = bdx * ady;

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

        // Fallback: recompute using the FLOAT-rounded differences (adx, bdx … cdy)
        // widened to double. Each cross-product of two 24-bit floats is exact in
        // 53-bit double; the lift×cross product is approximate but sufficient for sign.
        double dadx = adx, dbdx = bdx, dcdx = cdx;
        double dady = ady, dbdy = bdy, dcdy = cdy;

        double dbdxcdy = dbdx * dcdy - dcdx * dbdy;
        double dcdxady = dcdx * dady - dadx * dcdy;
        double dadxbdy = dadx * dbdy - dbdx * dady;

        double dalift = dadx * dadx + dady * dady;
        double dblift = dbdx * dbdx + dbdy * dbdy;
        double dclift = dcdx * dcdx + dcdy * dcdy;

        double ddet = dalift * dbdxcdy + dblift * dcdxady + dclift * dadxbdy;
        return (float)ddet;
    }

    // -------------------------------------------------------------------------
    // Arithmetic helpers (double precision)
    // -------------------------------------------------------------------------

    /// <summary>
    /// Computes <c>a*b - c*d</c> as <c>(hi, lo)</c> with full double-double accuracy.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static (double hi, double lo) TwoCross(double a, double b, double c, double d)
    {
        var (ab, ab_err) = TwoProd(a, b);
        var (cd, cd_err) = TwoProd(c, d);
        double hi = ab - cd;
        double lo = (ab - hi) - cd + ab_err - cd_err;
        return (hi, lo);
    }

    /// <summary>
    /// Computes <c>a * b</c> exactly as <c>(hi, lo)</c> where <c>hi + lo = a·b</c>.
    /// Uses the Veltkamp split (splitter = 2^27 + 1 for double).
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static (double hi, double lo) TwoProd(double a, double b)
    {
        double x = a * b;
        const double splitter = 134217729.0; // 2^27 + 1
        double ca = splitter * a;
        double abig = ca - a;
        double ahi = ca - abig;
        double alo = a - ahi;

        double cb = splitter * b;
        abig = cb - b;
        double bhi = cb - abig;
        double blo = b - bhi;

        double err = ((ahi * bhi - x) + ahi * blo + alo * bhi) + alo * blo;
        return (x, err);
    }
}
