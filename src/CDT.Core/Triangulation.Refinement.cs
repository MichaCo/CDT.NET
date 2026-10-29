// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

using System.Numerics;

namespace CDT;

public sealed partial class Triangulation<T>
    where T : unmanaged, IFloatingPoint<T>, IMinMaxValue<T>, IRootFunctions<T>
{
    /// <summary>
    /// Refines bad-quality triangles by inserting Steiner vertices.
    /// </summary>
    /// <param name="maxVerticesToInsert">Maximum number of vertices to add.</param>
    /// <param name="refinementCriterion">Criterion used to identify bad triangles.</param>
    /// <param name="refinementThreshold">Angle in radians or maximum area, depending on the criterion.</param>
    /// <param name="toEraseOrNull">
    /// Optional mutable set from a <c>Collect*</c> method. Its indices are updated
    /// as refinement creates replacement triangles.
    /// </param>
    /// <param name="minEdgeLength">Do not split edges or triangles at or below this length.</param>
    /// <returns>Counts of refinement attempts that could not be completed.</returns>
    public Unrefined RefineTriangles(
        int maxVerticesToInsert,
        RefinementCriterion refinementCriterion = RefinementCriterion.SmallestAngle,
        T? refinementThreshold = null,
        HashSet<int>? toEraseOrNull = null,
        T? minEdgeLength = null)
    {
        if (IsFinalized) throw new TriangulationFinalizedException();
        ArgumentOutOfRangeException.ThrowIfNegative(maxVerticesToInsert);

        T threshold = refinementThreshold ?? CdtUtils.DegToRad(T.CreateChecked(20));
        T minLength = minEdgeLength ?? T.CreateChecked(1e-6);
        if (_verticesCount > 0 && _kdTree == null) InitKdTree();

        var unrefined = new Unrefined();
        int remainingVertexBudget = maxVerticesToInsert;
        int steinerVerticesOffset = _verticesCount;

        ResolveEncroachedEdges(
            new Queue<Edge>(FindEncroachedFixedEdges()),
            ref remainingVertexBudget,
            steinerVerticesOffset,
            circumcenter: null,
            refinementCriterion,
            threshold,
            toEraseOrNull,
            minLength,
            ref unrefined);

        var badTriangles = new Queue<int>();
        for (int iT = 0; iT < _trianglesCount; iT++)
        {
            var t = _triangles[iT];
            if (!CdtUtils.TouchesSuperTriangle(t) &&
                !(toEraseOrNull?.Contains(iT) ?? false) &&
                IsRefinementNeeded(t, refinementCriterion, threshold))
                badTriangles.Enqueue(iT);
        }

        while (badTriangles.Count > 0 && remainingVertexBudget > 0)
        {
            int iT = badTriangles.Dequeue();
            if (toEraseOrNull?.Contains(iT) ?? false) continue;

            var t = _triangles[iT];
            if (!IsRefinementNeeded(t, refinementCriterion, threshold) ||
                (refinementCriterion == RefinementCriterion.SmallestAngle && IsSmallestAngleFixed(t)))
                continue;

            int v0 = t.V0, v1 = t.V1, v2 = t.V2;
            var p0 = _vertices[v0];
            var p1 = _vertices[v1];
            var p2 = _vertices[v2];
            T shortestEdge = T.Min(
                CdtUtils.Distance(p0, p1),
                T.Min(CdtUtils.Distance(p1, p2), CdtUtils.Distance(p2, p0)));
            if (shortestEdge <= minLength)
            {
                unrefined.ShortEdgeTriangles++;
                continue;
            }

            if (CdtUtils.Orient2D(p0, p1, p2) == T.Zero) continue;
            var circumcenter = CdtUtils.Circumcenter(p0, p1, p2);
            int iCircumcenterTri = FindTriangleAt(circumcenter);
            if (iCircumcenterTri == Indices.NoNeighbor)
            {
                unrefined.CircumcenterOutside++;
                continue;
            }

            var encroachedEdges = EdgesEncroachedBy(circumcenter, iCircumcenterTri);
            int budgetBeforeSplits = remainingVertexBudget;
            var newlyBadTriangles = ResolveEncroachedEdges(
                new Queue<Edge>(encroachedEdges),
                ref remainingVertexBudget,
                steinerVerticesOffset,
                circumcenter,
                refinementCriterion,
                threshold,
                toEraseOrNull,
                minLength,
                ref unrefined);
            if (remainingVertexBudget == 0) break;

            if (remainingVertexBudget != budgetBeforeSplits || newlyBadTriangles.Count > 0)
            {
                foreach (int badTriangle in newlyBadTriangles)
                    badTriangles.Enqueue(badTriangle);
                badTriangles.Enqueue(iT);
                continue;
            }
            if (encroachedEdges.Count > 0) continue;

            var centerTriangle = _triangles[iCircumcenterTri];
            var location = CdtUtils.LocatePointTriangle(
                circumcenter,
                _vertices[centerTriangle.V0],
                _vertices[centerTriangle.V1],
                _vertices[centerTriangle.V2]);
            if (location == PtTriLocation.OnVertex)
            {
                unrefined.CircumcenterOnVertex++;
                continue;
            }

            int iTopo = CdtUtils.IsOnEdge(location)
                ? centerTriangle.GetNeighbor(CdtUtils.EdgeNeighborFromLocation(location))
                : Indices.NoNeighbor;
            bool outsideInputHull =
                CdtUtils.TouchesSuperTriangle(centerTriangle) &&
                (iTopo == Indices.NoNeighbor ||
                 CdtUtils.TouchesSuperTriangle(_triangles[iTopo]));
            if ((toEraseOrNull?.Contains(iCircumcenterTri) ?? false) ||
                (iTopo != Indices.NoNeighbor && (toEraseOrNull?.Contains(iTopo) ?? false)) ||
                outsideInputHull)
            {
                unrefined.CircumcenterOutside++;
                continue;
            }

            int iVert = _verticesCount;
            AddNewVertex(circumcenter, Indices.NoNeighbor);
            var stack = new Stack<int>(4);
            if (iTopo == Indices.NoNeighbor)
                InsertVertexInsideTriangle(iVert, iCircumcenterTri, stack);
            else
                InsertVertexOnEdge(iVert, iCircumcenterTri, iTopo, handleFixedSplitEdge: true, stack);
            EnsureDelaunayByEdgeFlips(iVert, stack);
            TryAddVertexToLocator(iVert);

            int start = _vertTris[iVert];
            int curr = start;
            do
            {
                var adjacent = _triangles[curr];
                if (!(toEraseOrNull?.Contains(curr) ?? false) &&
                    !CdtUtils.TouchesSuperTriangle(adjacent) &&
                    IsRefinementNeeded(adjacent, refinementCriterion, threshold))
                    badTriangles.Enqueue(curr);
                curr = adjacent.Next(iVert).triIndex;
            } while (curr != start && curr != Indices.NoNeighbor);
            remainingVertexBudget--;
        }

        if (refinementCriterion == RefinementCriterion.SmallestAngle)
        {
            for (int iT = 0; iT < _trianglesCount; iT++)
            {
                var t = _triangles[iT];
                if (CdtUtils.TouchesSuperTriangle(t) ||
                    (toEraseOrNull?.Contains(iT) ?? false))
                    continue;
                if (IsRefinementNeeded(t, refinementCriterion, threshold) && IsSmallestAngleFixed(t))
                    unrefined.SharpFixedCorner++;
            }
        }
        return unrefined;
    }

    /// <summary>Finds fixed edges encroached by one of their opposed vertices.</summary>
    public List<Edge> FindEncroachedFixedEdges()
    {
        if (IsFinalized) throw new TriangulationFinalizedException();
        var encroached = new List<Edge>();
        foreach (var edge in _fixedEdges)
        {
            if (IsEdgeEncroached(edge)) encroached.Add(edge);
        }
        encroached.Sort(static (a, b) =>
        {
            int first = a.V1.CompareTo(b.V1);
            return first != 0 ? first : a.V2.CompareTo(b.V2);
        });
        return encroached;
    }

    /// <summary>Finds triangles that do not meet the requested refinement criterion.</summary>
    public List<int> FindUnrefinedTriangles(
        RefinementCriterion refinementCriterion = RefinementCriterion.SmallestAngle,
        T? refinementThreshold = null)
    {
        T threshold = refinementThreshold ?? CdtUtils.DegToRad(T.CreateChecked(20));
        bool hasSuperTriangle = !IsFinalized;
        var unrefined = new List<int>();
        for (int iT = 0; iT < _trianglesCount; iT++)
        {
            var t = _triangles[iT];
            if (hasSuperTriangle && CdtUtils.TouchesSuperTriangle(t)) continue;
            if (IsRefinementNeeded(t, refinementCriterion, threshold))
                unrefined.Add(iT);
        }
        return unrefined;
    }

    private bool IsRefinementNeeded(Triangle t, RefinementCriterion criterion, T threshold)
    {
        var a = _vertices[t.V0];
        var b = _vertices[t.V1];
        var c = _vertices[t.V2];
        return criterion switch
        {
            RefinementCriterion.SmallestAngle => CdtUtils.SmallestAngle(a, b, c) < threshold,
            RefinementCriterion.LargestArea => CdtUtils.Area(a, b, c) > threshold,
            _ => throw new ArgumentOutOfRangeException(nameof(criterion)),
        };
    }

    private bool IsSmallestAngleFixed(Triangle t)
    {
        int apexIndex = 0;
        T shortestSide = CdtUtils.DistanceSquared(_vertices[t.V1], _vertices[t.V2]);
        for (int i = 1; i < 3; i++)
        {
            T side = CdtUtils.DistanceSquared(
                _vertices[t.GetVertex(CdtUtils.Ccw(i))],
                _vertices[t.GetVertex(CdtUtils.Cw(i))]);
            if (side < shortestSide)
            {
                shortestSide = side;
                apexIndex = i;
            }
        }
        int apex = t.GetVertex(apexIndex);
        return _fixedEdges.Contains(new Edge(apex, t.GetVertex(CdtUtils.Ccw(apexIndex)))) &&
               _fixedEdges.Contains(new Edge(apex, t.GetVertex(CdtUtils.Cw(apexIndex))));
    }

    private bool IsEdgeEncroached(Edge edge)
    {
        var (iT, iTopo) = EdgeTriangles(edge.V1, edge.V2);
        if (iT == Indices.NoNeighbor) return false;
        var t = _triangles[iT];
        int edgeIndex = CdtUtils.EdgeNeighborIndex(t, edge.V1, edge.V2);
        int opposed = t.GetVertex(CdtUtils.OpposedVertexIndex(edgeIndex));
        if (CdtUtils.IsEncroachingOnEdge(_vertices[opposed], _vertices[edge.V1], _vertices[edge.V2]))
            return true;
        return iTopo != Indices.NoNeighbor &&
               CdtUtils.IsEncroachingOnEdge(
                   _vertices[CdtUtils.OpposedVertex(_triangles[iTopo], iT)],
                   _vertices[edge.V1],
                   _vertices[edge.V2]);
    }

    private List<Edge> EdgesEncroachedBy(V2d<T> vertex, int startTriangle)
    {
        var encroached = new HashSet<Edge>();
        var traversed = new HashSet<int> { startTriangle };
        var toTraverse = new Stack<int>();
        toTraverse.Push(startTriangle);
        while (toTraverse.Count > 0)
        {
            var t = _triangles[toTraverse.Pop()];
            for (int i = 0; i < 3; i++)
            {
                var edge = new Edge(t.GetVertex(CdtUtils.Ccw(i)), t.GetVertex(CdtUtils.Cw(i)));
                if (_fixedEdges.Contains(edge))
                {
                    if (CdtUtils.IsEncroachingOnEdge(vertex, _vertices[edge.V1], _vertices[edge.V2]))
                        encroached.Add(edge);
                    continue;
                }

                int iN = t.GetNeighbor(CdtUtils.OpposedNeighborIndex(i));
                if (iN == Indices.NoNeighbor || !traversed.Add(iN)) continue;
                var neighbor = _triangles[iN];
                if (CdtUtils.IsInCircumcircle(
                    vertex,
                    _vertices[neighbor.V0],
                    _vertices[neighbor.V1],
                    _vertices[neighbor.V2]))
                    toTraverse.Push(iN);
            }
        }
        var result = encroached.ToList();
        result.Sort(static (a, b) =>
        {
            int first = a.V1.CompareTo(b.V1);
            return first != 0 ? first : a.V2.CompareTo(b.V2);
        });
        return result;
    }

    private List<int> ResolveEncroachedEdges(
        Queue<Edge> encroachedEdges,
        ref int remainingVertexBudget,
        int steinerVerticesOffset,
        V2d<T>? circumcenter,
        RefinementCriterion refinementCriterion,
        T badTriangleThreshold,
        HashSet<int>? toEraseOrNull,
        T minEdgeLength,
        ref Unrefined unrefined)
    {
        var badTriangles = new List<int>();
        while (encroachedEdges.Count > 0 && remainingVertexBudget > 0)
        {
            var edge = encroachedEdges.Dequeue();
            if (!_fixedEdges.Contains(edge)) continue;
            if (CdtUtils.Distance(_vertices[edge.V1], _vertices[edge.V2]) <= minEdgeLength)
            {
                unrefined.ShortEdges++;
                continue;
            }

            int? splitVertex = SplitEncroachedEdge(
                edge, steinerVerticesOffset, toEraseOrNull, ref unrefined);
            if (splitVertex is not int iSplitVert) continue;
            remainingVertexBudget--;

            int start = _vertTris[iSplitVert];
            int iT = start;
            do
            {
                var t = _triangles[iT];
                if (circumcenter.HasValue &&
                    !(toEraseOrNull?.Contains(iT) ?? false) &&
                    !CdtUtils.TouchesSuperTriangle(t) &&
                    IsRefinementNeeded(t, refinementCriterion, badTriangleThreshold))
                    badTriangles.Add(iT);

                for (int i = 0; i < 3; i++)
                {
                    var triEdge = new Edge(t.GetVertex(i), t.GetVertex(CdtUtils.Cw(i)));
                    if (!_fixedEdges.Contains(triEdge)) continue;
                    if (IsEdgeEncroached(triEdge) ||
                        (circumcenter.HasValue &&
                         CdtUtils.IsEncroachingOnEdge(
                             circumcenter.Value,
                             _vertices[triEdge.V1],
                             _vertices[triEdge.V2])))
                        encroachedEdges.Enqueue(triEdge);
                }
                iT = t.Next(iSplitVert).triIndex;
            } while (iT != start && iT != Indices.NoNeighbor);
        }
        return badTriangles;
    }

    private int? SplitEncroachedEdge(
        Edge edge,
        int steinerVerticesOffset,
        HashSet<int>? toEraseOrNull,
        ref Unrefined unrefined)
    {
        var start = _vertices[edge.V1];
        var end = _vertices[edge.V2];
        var (iT, iTopo) = EdgeTriangles(edge.V1, edge.V2);
        if (iT == Indices.NoNeighbor || iTopo == Indices.NoNeighbor)
        {
            unrefined.SplitVertexInvalid++;
            return null;
        }

        T split = T.One / (T.One + T.One);
        if (edge.V1 < steinerVerticesOffset &&
            edge.V2 >= steinerVerticesOffset &&
            HasAnotherFixedEdgeAtSmallAngle(edge.V1, edge))
        {
            T length = CdtUtils.Distance(start, end);
            T halfLength = length / (T.One + T.One);
            T nearestPowerOfTwo = T.One;
            while (halfLength > nearestPowerOfTwo)
                nearestPowerOfTwo += nearestPowerOfTwo;
            while (halfLength < T.CreateChecked(0.75) * nearestPowerOfTwo)
                nearestPowerOfTwo /= T.One + T.One;
            split = nearestPowerOfTwo / length;
        }

        var splitVertex = new V2d<T>(
            CdtUtils.Lerp(start.X, end.X, split),
            CdtUtils.Lerp(start.Y, end.Y, split));
        if (!IsEdgeSplitVertexValid(splitVertex, iT, iTopo))
        {
            unrefined.SplitVertexInvalid++;
            return null;
        }

        int iSplit = SplitFixedEdgeAt(edge, splitVertex, iT, iTopo);
        if (toEraseOrNull != null)
        {
            if (toEraseOrNull.Contains(iT)) toEraseOrNull.Add(_trianglesCount - 2);
            if (toEraseOrNull.Contains(iTopo)) toEraseOrNull.Add(_trianglesCount - 1);
        }
        return iSplit;
    }

    private bool HasAnotherFixedEdgeAtSmallAngle(int vertex, Edge excludeEdge)
    {
        int other = excludeEdge.V1 == vertex ? excludeEdge.V2 : excludeEdge.V1;
        int start = _vertTris[vertex];
        int iT = start;
        do
        {
            var t = _triangles[iT];
            var (iNext, otherVertex) = t.Next(vertex);
            var candidate = new Edge(vertex, otherVertex);
            if (candidate != excludeEdge &&
                _fixedEdges.Contains(candidate) &&
                CdtUtils.IsAngleAtApexSmall(
                    _vertices[vertex],
                    _vertices[other],
                    _vertices[otherVertex]))
                return true;
            iT = iNext;
        } while (iT != start && iT != Indices.NoNeighbor);
        return false;
    }

    private int FindTriangleAt(V2d<T> position)
    {
        if (_trianglesCount == 0 || _verticesCount == 0) return Indices.NoNeighbor;
        int startVertex = _kdTree?.Nearest(position.X, position.Y, _vertices) ?? 0;
        int iT = WalkTriangles(startVertex, position);
        if ((uint)iT < (uint)_trianglesCount)
        {
            var t = _triangles[iT];
            if (CdtUtils.LocatePointTriangle(
                    position, _vertices[t.V0], _vertices[t.V1], _vertices[t.V2]) != PtTriLocation.Outside)
                return iT;
        }
        for (int i = 0; i < _trianglesCount; i++)
        {
            var t = _triangles[i];
            if (CdtUtils.LocatePointTriangle(
                    position, _vertices[t.V0], _vertices[t.V1], _vertices[t.V2]) != PtTriLocation.Outside)
                return i;
        }
        return Indices.NoNeighbor;
    }
}
