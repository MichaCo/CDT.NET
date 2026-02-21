// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
//
// Geometric predicates matching the Lenthe formulation used by the C++ CDT library:
//   - float inputs: upcast to double, single-pass computation
//   - double inputs: double-double compensation for near-degenerate cases
//     (emulates the C++ long-double promotion on GCC/Linux targets)

using System.Runtime.CompilerServices;

namespace CDT;

/// <summary>Robust geometric predicates.</summary>
internal static class Predicates
{
    // -------------------------------------------------------------------------
    // orient2d: sign of the 2×2 determinant
    //   | ax-cx  ay-cy |
    //   | bx-cx  by-cy |
    // Positive => c is to the left of a→b
    // -------------------------------------------------------------------------

    /// <summary>
    /// Orient2d predicate for <see cref="double"/>.
    /// Returns positive if <c>(cx,cy)</c> is to the left of <c>(ax,ay)→(bx,by)</c>.
    /// Uses double-double compensation for near-degenerate cases, matching the
    /// behavior of C++ CDT compiled with GCC/Linux (which promotes double to long double).
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

        // Error bound: (3 + 8ε)ε * (|acx*bcy| + |acy*bcx|)
        const double errBound = 3.3306690621773814e-16; // 3ε (ε = machine epsilon)
        double permanent = Math.Abs(acx * bcy) + Math.Abs(acy * bcx);
        if (Math.Abs(det) > errBound * permanent)
        {
            return det;
        }

        // Double-double fallback: compute acx*bcy and acy*bcx with exact error.
        var (p, pe) = TwoProd(acx, bcy);
        var (q, qe) = TwoProd(acy, bcx);
        double hi = p - q;
        double lo = (p - hi) - q + pe - qe; // compensation term
        if (hi != 0.0)
        {
            return hi;
        }

        return lo;
    }

    /// <summary>
    /// Orient2d predicate for <see cref="float"/>.
    /// Upcasts to double and uses the single-pass Lenthe formula.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static float Orient2D(
        float ax, float ay,
        float bx, float by,
        float cx, float cy)
        => (float)((double)(ax - cx) * (by - cy) - (double)(ay - cy) * (bx - cx));

    // -------------------------------------------------------------------------
    // incircle: positive => d is inside the circumcircle of (a,b,c) in CCW order
    // -------------------------------------------------------------------------

    /// <summary>
    /// InCircle predicate for <see cref="double"/>.
    /// Returns positive if <c>d</c> is strictly inside the circumcircle of <c>(a,b,c)</c>.
    /// Uses double-double compensation for near-degenerate cases, matching the
    /// behavior of C++ CDT compiled with GCC/Linux (long double promotion).
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

        // Fast-exit when the estimate is clearly reliable.
        // Permanent = upper bound on rounding error magnitude.
        double permanent = (Math.Abs(bdxcdy) + Math.Abs(cdx * bdy)) * alift
                         + (Math.Abs(cdxady) + Math.Abs(adx * cdy)) * blift
                         + (Math.Abs(adxbdy) + Math.Abs(bdx * ady)) * clift;
        const double errBound = 1.1102230246251565e-15; // 10ε
        if (Math.Abs(det) > errBound * permanent)
        {
            return det;
        }

        // Double-double fallback: compute each cross-product (a*b - c*d)
        // using TwoProd so the error is tracked exactly.
        var (bc_h, bc_e) = TwoCross(bdx, cdy, cdx, bdy);
        var (ca_h, ca_e) = TwoCross(cdx, ady, adx, cdy);
        var (ab_h, ab_e) = TwoCross(adx, bdy, bdx, ady);

        // Scale each cross-product by its lift factor and sum.
        // hi part of result:
        double sum_hi = alift * bc_h + blift * ca_h + clift * ab_h;
        // lo (correction) part:
        double sum_lo = alift * bc_e + blift * ca_e + clift * ab_e;

        if (sum_hi != 0.0)
        {
            return sum_hi;
        }

        return sum_lo;
    }

    /// <summary>
    /// InCircle predicate for <see cref="float"/>.
    /// Upcasts to double and uses the single-pass Lenthe formula, matching C++ CDT float behavior.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static float InCircle(
        float ax, float ay,
        float bx, float by,
        float cx, float cy,
        float dx, float dy)
    {
        // Cast to double first (Lenthe promote<float>=double)
        double adx = (double)ax - dx, bdx = (double)bx - dx, cdx = (double)cx - dx;
        double ady = (double)ay - dy, bdy = (double)by - dy, cdy = (double)cy - dy;
        double alift = adx * adx + ady * ady;
        double blift = bdx * bdx + bdy * bdy;
        double clift = cdx * cdx + cdy * cdy;
        return (float)(alift * (bdx * cdy - cdx * bdy)
                     + blift * (cdx * ady - adx * cdy)
                     + clift * (adx * bdy - bdx * ady));
    }

    // -------------------------------------------------------------------------
    // Arithmetic helpers
    // -------------------------------------------------------------------------

    /// <summary>
    /// Computes <c>a*b - c*d</c> as <c>(hi, lo)</c> where <c>hi + lo = a*b - c*d</c>
    /// with full double-double accuracy.
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
    /// Computes <c>a * b</c> exactly as <c>(hi, lo)</c> where <c>hi + lo = a*b</c>.
    /// Uses the Veltkamp split.
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
