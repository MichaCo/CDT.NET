// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

using System.Numerics;
using System.Runtime.CompilerServices;

namespace CDT;

/// <summary>
/// Errors thrown by the triangulation.
/// </summary>
public class TriangulationException : Exception
{
    /// <inheritdoc/>
    public TriangulationException(string message) : base(message) { }
}

/// <summary>Thrown when a duplicate vertex is detected during insertion.</summary>
public sealed class DuplicateVertexException : TriangulationException
{
    /// <summary>First duplicate vertex index.</summary>
    public int V1 { get; }

    /// <summary>Second duplicate vertex index (same position as V1).</summary>
    public int V2 { get; }

    /// <inheritdoc/>
    public DuplicateVertexException(int v1, int v2)
        : base($"Duplicate vertex detected: #{v1} is a duplicate of #{v2}")
    {
        V1 = v1;
        V2 = v2;
    }
}

/// <summary>Thrown when intersecting constraint edges are detected and <see cref="IntersectingConstraintEdges.NotAllowed"/> is set.</summary>
public sealed class IntersectingConstraintsException : TriangulationException
{
    /// <summary>First constraint edge.</summary>
    public Edge E1 { get; }

    /// <summary>Second constraint edge.</summary>
    public Edge E2 { get; }

    /// <inheritdoc/>
    public IntersectingConstraintsException(Edge e1, Edge e2)
        : base($"Intersecting constraint edges: ({e1.V1},{e1.V2}) ∩ ({e2.V1},{e2.V2})")
    {
        E1 = e1;
        E2 = e2;
    }
}

/// <summary>
/// 2D constrained Delaunay triangulation.
/// Supports both constrained and conforming modes.
/// </summary>
/// <typeparam name="T">Floating-point coordinate type (float or double).</typeparam>
public sealed class Triangulation<T>
    where T : unmanaged, IFloatingPoint<T>, IMinMaxValue<T>, IRootFunctions<T>
{
    // -------------------------------------------------------------------------
    // Public state (read-only views)
    // -------------------------------------------------------------------------

    /// <summary>All vertices in the triangulation (including super-triangle vertices while not finalized).</summary>
    public IReadOnlyList<V2d<T>> Vertices => _vertices;

    /// <summary>All triangles in the triangulation.</summary>
    public IReadOnlyList<Triangle> Triangles => _triangles;

    /// <summary>Set of constraint (fixed) edges.</summary>
    public IReadOnlySet<Edge> FixedEdges => _fixedEdges;

    /// <summary>
    /// Stores count of overlapping boundaries for a fixed edge.
    /// Only has entries for edges that represent overlapping boundaries.
    /// </summary>
    public IReadOnlyDictionary<Edge, ushort> OverlapCount => _overlapCount;

    /// <summary>
    /// Stores the list of original edges represented by a given fixed edge.
    /// Only populated when edges were split or overlap.
    /// </summary>
    public IReadOnlyDictionary<Edge, IReadOnlyList<Edge>> PieceToOriginals => _pieceToOriginalsView;

    // -------------------------------------------------------------------------
    // Private fields
    // -------------------------------------------------------------------------

    private readonly List<V2d<T>> _vertices = new();
    private readonly List<Triangle> _triangles = new();
    private readonly HashSet<Edge> _fixedEdges = new();
    private readonly Dictionary<Edge, ushort> _overlapCount = new();
    private readonly Dictionary<Edge, List<Edge>> _pieceToOriginals = new();
    private readonly IReadOnlyDictionary<Edge, IReadOnlyList<Edge>> _pieceToOriginalsView;

    private readonly VertexInsertionOrder _insertionOrder;
    private readonly IntersectingConstraintEdges _intersectingEdgesStrategy;
    private readonly T _minDistToConstraintEdge;
    private SuperGeometryType _superGeomType;
    private int _nTargetVerts;

    // For each vertex: one adjacent triangle index
    private readonly List<int> _vertTris = new();

    // KD-tree for nearest-point location
    private KdTree<T>? _kdTree;

    // -------------------------------------------------------------------------
    // Construction
    // -------------------------------------------------------------------------

    /// <summary>
    /// Creates a triangulation with default settings:
    /// <see cref="VertexInsertionOrder.Auto"/>,
    /// <see cref="IntersectingConstraintEdges.NotAllowed"/>,
    /// zero minimum distance.
    /// </summary>
    public Triangulation()
        : this(VertexInsertionOrder.Auto, IntersectingConstraintEdges.NotAllowed, T.Zero)
    { }

    /// <summary>Creates a triangulation with the specified insertion order.</summary>
    public Triangulation(VertexInsertionOrder insertionOrder)
        : this(insertionOrder, IntersectingConstraintEdges.NotAllowed, T.Zero)
    { }

    /// <summary>Creates a triangulation with explicit settings.</summary>
    public Triangulation(
        VertexInsertionOrder insertionOrder,
        IntersectingConstraintEdges intersectingEdgesStrategy,
        T minDistToConstraintEdge)
    {
        _insertionOrder = insertionOrder;
        _intersectingEdgesStrategy = intersectingEdgesStrategy;
        _minDistToConstraintEdge = minDistToConstraintEdge;
        _superGeomType = SuperGeometryType.SuperTriangle;
        _nTargetVerts = 0;
        _pieceToOriginalsView = new CovariantReadOnlyDictionary<Edge, List<Edge>, IReadOnlyList<Edge>>(_pieceToOriginals);
    }

    // -------------------------------------------------------------------------
    // Public API – vertex insertion
    // -------------------------------------------------------------------------

    /// <summary>Inserts a list of vertices into the triangulation.</summary>
    public void InsertVertices(IReadOnlyList<V2d<T>> newVertices)
    {
        if (newVertices.Count == 0) return;

        bool isFirstInsertion = _kdTree == null && _vertices.Count == 0;

        // Build bounding box of new vertices
        var box = new Box2d<T>();
        box.Envelop(newVertices);

        if (isFirstInsertion)
        {
            AddSuperTriangle(box);
        }
        else if (_kdTree == null)
        {
            // Subsequent calls: initialize the KD tree with all existing vertices
            InitKdTree();
        }

        int insertStart = _vertices.Count;
        foreach (var v in newVertices)
        {
            AddNewVertex(v, Indices.NoNeighbor);
        }

        if (_insertionOrder == VertexInsertionOrder.Auto && isFirstInsertion)
        {
            // Use BFS KD-tree ordering for the first bulk insertion
            // Walk-start comes from the BFS parent, not the KD tree
            InsertVertices_KDTreeBFS(insertStart, box);
        }
        else if (_insertionOrder == VertexInsertionOrder.Auto)
        {
            // Subsequent calls: randomized with KD-tree walk-start
            InsertVertices_Randomized(insertStart);
        }
        else
        {
            // AsProvided: sequential order, KD-tree walk-start
            for (int iV = insertStart; iV < _vertices.Count; iV++)
            {
                InsertVertex(iV);
            }
        }
    }

    // -------------------------------------------------------------------------
    // Public API – edge insertion (constrained DT)
    // -------------------------------------------------------------------------

    /// <summary>Inserts constraint edges (constrained Delaunay triangulation).</summary>
    public void InsertEdges(IReadOnlyList<Edge> edges)
    {
        var remaining = new List<Edge>(4);
        var tppTasks = new List<TriangulatePseudoPolygonTask>(8);
        foreach (var e in edges)
        {
            remaining.Clear();
            remaining.Add(new Edge(e.V1 + _nTargetVerts, e.V2 + _nTargetVerts));
            while (remaining.Count > 0)
            {
                Edge edge = remaining[^1];
                remaining.RemoveAt(remaining.Count - 1);
                InsertEdgeIteration(edge, new Edge(e.V1 + _nTargetVerts, e.V2 + _nTargetVerts), remaining, tppTasks);
            }
        }
    }

    // -------------------------------------------------------------------------
    // Public API – conforming DT
    // -------------------------------------------------------------------------

    /// <summary>
    /// Inserts constraint edges for a conforming Delaunay triangulation.
    /// May add new vertices (midpoints) until edges are represented.
    /// </summary>
    public void ConformToEdges(IReadOnlyList<Edge> edges)
    {
        var remaining = new List<ConformToEdgeTask>(8);
        foreach (var e in edges)
        {
            var shifted = new Edge(e.V1 + _nTargetVerts, e.V2 + _nTargetVerts);
            remaining.Clear();
            remaining.Add(new ConformToEdgeTask(shifted, new List<Edge> { shifted }, 0));
            while (remaining.Count > 0)
            {
                var task = remaining[^1];
                remaining.RemoveAt(remaining.Count - 1);
                ConformToEdgeIteration(task.Edge, task.Originals, task.Overlaps, remaining);
            }
        }
    }

    // -------------------------------------------------------------------------
    // Public API – finalization
    // -------------------------------------------------------------------------

    /// <summary>Removes the super-triangle to produce a convex hull triangulation.</summary>
    public void EraseSuperTriangle()
    {
        if (_superGeomType != SuperGeometryType.SuperTriangle) return;
        var toErase = new HashSet<int>();
        for (int i = 0; i < _triangles.Count; i++)
        {
            if (TriangleUtils.TouchesSuperTriangle(_triangles[i]))
                toErase.Add(i);
        }
        FinalizeTriangulation(toErase);
    }

    /// <summary>Removes all outer triangles (flood-fill from super-triangle vertex until a constraint edge).</summary>
    public void EraseOuterTriangles()
    {
        if (_vertTris[0] == Indices.NoNeighbor)
            throw new TriangulationException("No vertex triangle data – already finalized?");
        var seeds = new Stack<int>();
        seeds.Push(_vertTris[0]);
        var toErase = GrowToBoundary(seeds);
        FinalizeTriangulation(toErase);
    }

    /// <summary>
    /// Removes outer triangles and automatically detects and removes holes
    /// using even-odd depth rule.
    /// </summary>
    public void EraseOuterTrianglesAndHoles()
    {
        var depths = CalculateTriangleDepths();
        var toErase = new HashSet<int>();
        for (int i = 0; i < _triangles.Count; i++)
        {
            if (depths[i] % 2 == 0) toErase.Add(i);
        }
        FinalizeTriangulation(toErase);
    }

    /// <summary>
    /// Indicates whether the triangulation has been finalized (i.e., one of the
    /// Erase methods was called). Further modification is not possible.
    /// </summary>
    public bool IsFinalized => _vertTris.Count == 0 && _vertices.Count > 0;

    // -------------------------------------------------------------------------
    // Internal helpers – super-triangle setup
    // -------------------------------------------------------------------------

    private void AddSuperTriangle(Box2d<T> box)
    {
        _nTargetVerts = Indices.SuperTriangleVertexCount;
        _superGeomType = SuperGeometryType.SuperTriangle;

        T two = T.One + T.One;
        T cx = (box.Min.X + box.Max.X) / two;
        T cy = (box.Min.Y + box.Max.Y) / two;
        T w = box.Max.X - box.Min.X;
        T h = box.Max.Y - box.Min.Y;
        T r = T.Max(w, h);
        r = T.Max(two * r, T.One);

        // Guard against very large numbers
        while (cy <= cy - r) r = two * r;

        T R = two * r;
        T cos30 = ParseT("0.8660254037844386");
        T shiftX = R * cos30;

        var v1 = new V2d<T>(cx - shiftX, cy - r);
        var v2 = new V2d<T>(cx + shiftX, cy - r);
        var v3 = new V2d<T>(cx, cy + R);

        AddNewVertex(v1, 0);
        AddNewVertex(v2, 0);
        AddNewVertex(v3, 0);

        AddTriangle(new Triangle(0, 1, 2, Indices.NoNeighbor, Indices.NoNeighbor, Indices.NoNeighbor));

        if (_insertionOrder != VertexInsertionOrder.Auto)
        {
            InitKdTree();
        }
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private void AddNewVertex(V2d<T> pos, int iTri)
    {
        _vertices.Add(pos);
        _vertTris.Add(iTri);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private int AddTriangle() => AddTriangle(new Triangle());

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private int AddTriangle(Triangle t)
    {
        int idx = _triangles.Count;
        _triangles.Add(t);
        return idx;
    }

    // -------------------------------------------------------------------------
    // Internal helpers – vertex insertion
    // -------------------------------------------------------------------------

    private void InsertVertex(int iVert, int walkStart)
    {
        var (iT, iTopo) = WalkingSearchTrianglesAt(iVert, walkStart);
        var stack = iTopo == Indices.NoNeighbor
            ? InsertVertexInsideTriangle(iVert, iT)
            : InsertVertexOnEdge(iVert, iT, iTopo, handleFixedSplitEdge: true);
        EnsureDelaunayByEdgeFlips(iVert, stack);
        TryAddVertexToLocator(iVert);
    }

    private void InsertVertex(int iVert)
    {
        // Walk-start from KD-tree nearest point, or vertex 0 as fallback
        int near = _kdTree != null
            ? _kdTree.Nearest(_vertices[iVert].X, _vertices[iVert].Y, _vertices)
            : 0;
        InsertVertex(iVert, near);
    }

    private void InsertVertices_Randomized(int superGeomVertCount)
    {
        int count = _vertices.Count - superGeomVertCount;
        var indices = new int[count];
        for (int i = 0; i < count; i++) { indices[i] = superGeomVertCount + i; }
        // Matches C++ detail::random_shuffle which uses SplitMix64(state=0) fresh per call.
        ulong state = 0UL;
        for (int i = count - 1; i > 0; i--)
        {
            int j = (int)(SplitMix64(ref state) % (ulong)(i + 1));
            (indices[i], indices[j]) = (indices[j], indices[i]);
        }
        foreach (int iV in indices) { InsertVertex(iV); }
    }

    private void InsertVertices_KDTreeBFS(int superGeomVertCount, Box2d<T> box)
    {
        int vertexCount = _vertices.Count - superGeomVertCount;
        if (vertexCount <= 0) { return; }

        var indices = new int[vertexCount];
        for (int i = 0; i < vertexCount; i++) { indices[i] = superGeomVertCount + i; }

        // Matches C++ insertVertices_KDTreeBFS: BFS with portable_nth_element and
        // box updated from the actual split-vertex coordinate at each level.
        var queue = new Queue<(int lo, int hi, T boxMinX, T boxMinY, T boxMaxX, T boxMaxY, int parent)>();
        queue.Enqueue((0, vertexCount, box.Min.X, box.Min.Y, box.Max.X, box.Max.Y, 0));

        while (queue.Count > 0)
        {
            var (lo, hi, boxMinX, boxMinY, boxMaxX, boxMaxY, parent) = queue.Dequeue();
            int len = hi - lo;
            if (len == 0) { continue; }
            if (len == 1) { InsertVertex(indices[lo], parent); continue; }

            int midPos = lo + len / 2;

            if (T.CreateChecked(boxMaxX - boxMinX) >= T.CreateChecked(boxMaxY - boxMinY))
            {
                NthElement(indices, lo, midPos, hi, (a, b) => _vertices[a].X.CompareTo(_vertices[b].X));
                T split = _vertices[indices[midPos]].X;
                InsertVertex(indices[midPos], parent);
                if (lo < midPos) { queue.Enqueue((lo, midPos, boxMinX, boxMinY, split, boxMaxY, indices[midPos])); }
                if (midPos + 1 < hi) { queue.Enqueue((midPos + 1, hi, split, boxMinY, boxMaxX, boxMaxY, indices[midPos])); }
            }
            else
            {
                NthElement(indices, lo, midPos, hi, (a, b) => _vertices[a].Y.CompareTo(_vertices[b].Y));
                T split = _vertices[indices[midPos]].Y;
                InsertVertex(indices[midPos], parent);
                if (lo < midPos) { queue.Enqueue((lo, midPos, boxMinX, boxMinY, boxMaxX, split, indices[midPos])); }
                if (midPos + 1 < hi) { queue.Enqueue((midPos + 1, hi, boxMinX, split, boxMaxX, boxMaxY, indices[midPos])); }
            }
        }
    }

    // -------------------------------------------------------------------------
    // nth_element — port of LLVM's portable_nth_element (matches C++ CDT exactly)
    // -------------------------------------------------------------------------

    /// <summary>
    /// Port of LLVM libcxx <c>nth_element</c> used by the C++ CDT for
    /// <c>insertVertices_KDTreeBFS</c>. Rearranges <paramref name="arr"/>
    /// in [<paramref name="lo"/>, <paramref name="hi"/>) so the element at
    /// position <paramref name="nth"/> is the one that would be there after a
    /// full sort; elements before it are ≤ it and elements after are ≥ it.
    /// Tie-breaking matches C++ because the algorithm is identical.
    /// </summary>
    private static void NthElement(int[] arr, int lo, int nth, int hi, Comparison<int> cmp)
    {
        int first = lo, last = hi;
        while (true)
        {
            if (nth == last) { return; }
            int len = last - first;
            switch (len)
            {
                case 0: case 1: return;
                case 2:
                    if (cmp(arr[last - 1], arr[first]) < 0) { (arr[first], arr[last - 1]) = (arr[last - 1], arr[first]); }
                    return;
                case 3:
                    NthSort3(arr, first, first + 1, last - 1, cmp);
                    return;
            }
            if (len <= 7) { NthSelectionSort(arr, first, last, cmp); return; }

            int m = first + len / 2;
            int lm1 = last - 1;
            uint nSwaps = NthSort3(arr, first, m, lm1, cmp);

            int i = first, j = lm1;

            if (cmp(arr[i], arr[m]) >= 0)   // !comp(*i, *m)
            {
                while (true)
                {
                    --j;
                    if (i == j)
                    {
                        ++i;
                        j = last;
                        if (cmp(arr[first], arr[--j]) >= 0)   // !comp(*first, *(last-1))
                        {
                            while (true)
                            {
                                if (i == j) { return; }
                                if (cmp(arr[first], arr[i]) < 0)
                                {
                                    (arr[i], arr[j]) = (arr[j], arr[i]);
                                    ++nSwaps;
                                    ++i;
                                    break;
                                }
                                ++i;
                            }
                        }
                        if (i == j) { return; }
                        while (true)
                        {
                            while (cmp(arr[first], arr[i]) >= 0) { ++i; }
                            do { --j; } while (cmp(arr[first], arr[j]) < 0);
                            if (i >= j) { break; }
                            (arr[i], arr[j]) = (arr[j], arr[i]);
                            ++nSwaps;
                            ++i;
                        }
                        if (nth < i) { return; }
                        first = i;
                        goto ContinueOuter;
                    }
                    if (cmp(arr[j], arr[m]) < 0)
                    {
                        (arr[i], arr[j]) = (arr[j], arr[i]);
                        ++nSwaps;
                        break;
                    }
                }
            }

            ++i;
            if (i < j)
            {
                while (true)
                {
                    while (cmp(arr[i], arr[m]) < 0) { ++i; }
                    do { --j; } while (cmp(arr[j], arr[m]) >= 0);
                    if (i >= j) { break; }
                    (arr[i], arr[j]) = (arr[j], arr[i]);
                    ++nSwaps;
                    if (m == i) { m = j; }
                    ++i;
                }
            }

            if (i != m && cmp(arr[m], arr[i]) < 0)
            {
                (arr[i], arr[m]) = (arr[m], arr[i]);
                ++nSwaps;
            }

            if (nth == i) { return; }

            if (nSwaps == 0)
            {
                if (nth < i)
                {
                    int ck = m = first;
                    while (++ck != i)
                    {
                        if (cmp(arr[ck], arr[m]) < 0) { goto NotSorted; }
                        m = ck;
                    }
                    return;
                }
                else
                {
                    int ck = m = i;
                    while (++ck != last)
                    {
                        if (cmp(arr[ck], arr[m]) < 0) { goto NotSorted; }
                        m = ck;
                    }
                    return;
                }
            }

        NotSorted:
            if (nth < i) { last = i; }
            else { first = i + 1; }
        ContinueOuter:;
        }
    }

    /// <summary>Sorts three elements at positions x, y, z — port of LLVM sort3.</summary>
    private static uint NthSort3(int[] arr, int x, int y, int z, Comparison<int> cmp)
    {
        uint r = 0;
        if (cmp(arr[y], arr[x]) >= 0)    // !c(*y, *x): x <= y
        {
            if (cmp(arr[z], arr[y]) >= 0) { return r; }   // y <= z: done
            (arr[y], arr[z]) = (arr[z], arr[y]); r = 1;
            if (cmp(arr[y], arr[x]) < 0) { (arr[x], arr[y]) = (arr[y], arr[x]); r = 2; }
            return r;
        }
        if (cmp(arr[z], arr[y]) < 0)    // x > y, y > z
        {
            (arr[x], arr[z]) = (arr[z], arr[x]); r = 1; return r;
        }
        // x > y, y <= z
        (arr[x], arr[y]) = (arr[y], arr[x]); r = 1;
        if (cmp(arr[z], arr[y]) < 0) { (arr[y], arr[z]) = (arr[z], arr[y]); r = 2; }
        return r;
    }

    /// <summary>Selection sort for small ranges — port of LLVM selection_sort.</summary>
    private static void NthSelectionSort(int[] arr, int first, int last, Comparison<int> cmp)
    {
        for (int i = first; i < last - 1; i++)
        {
            int minIdx = i;
            for (int k = i + 1; k < last; k++)
            {
                if (cmp(arr[k], arr[minIdx]) < 0) { minIdx = k; }
            }
            if (minIdx != i) { (arr[i], arr[minIdx]) = (arr[minIdx], arr[i]); }
        }
    }

    private List<Edge> InsertVertex_FlipFixedEdges(int iV)
    {
        var flipped = new List<Edge>();
        // Use KD-tree if available, otherwise fall back to vertex 0 (first super-triangle vertex)
        int near = _kdTree != null
            ? _kdTree.Nearest(_vertices[iV].X, _vertices[iV].Y, _vertices)
            : 0;
        var (iT, iTopo) = WalkingSearchTrianglesAt(iV, near);
        var stack = iTopo == Indices.NoNeighbor
            ? InsertVertexInsideTriangle(iV, iT)
            : InsertVertexOnEdge(iV, iT, iTopo, handleFixedSplitEdge: false);

        int _dbgFlipIter2 = 0;
        while (stack.Count > 0)
        {
            if (++_dbgFlipIter2 > 1_000_000) throw new InvalidOperationException($"InsertVertex_FlipFixed infinite loop, iV={iV}");
            int tri = stack.Pop();
            EdgeFlipInfo(tri, iV,
                out int itopo, out int iv2, out int iv3, out int iv4,
                out int n1, out int n2, out int n3, out int n4);

            if (itopo != Indices.NoNeighbor && IsFlipNeeded(iV, iv2, iv3, iv4))
            {
                var flippedEdge = new Edge(iv2, iv4);
                if (_fixedEdges.Contains(flippedEdge))
                    flipped.Add(flippedEdge);
                FlipEdge(tri, itopo, iV, iv2, iv3, iv4, n1, n2, n3, n4);
                stack.Push(tri);
                stack.Push(itopo);
            }
        }
        TryAddVertexToLocator(iV);
        return flipped;
    }

    private void EnsureDelaunayByEdgeFlips(int iV1, Stack<int> triStack)
    {
        int _dbgFlipIter = 0;
        while (triStack.Count > 0)
        {
            if (++_dbgFlipIter > 1_000_000) throw new InvalidOperationException($"EnsureDelaunayByEdgeFlips infinite loop, iV1={iV1}, stack size={triStack.Count}");

            int iT = triStack.Pop();
            EdgeFlipInfo(iT, iV1,
                out int iTopo, out int iV2, out int iV3, out int iV4,
                out int n1, out int n2, out int n3, out int n4);
            if (iTopo != Indices.NoNeighbor && IsFlipNeeded(iV1, iV2, iV3, iV4))
            {
                FlipEdge(iT, iTopo, iV1, iV2, iV3, iV4, n1, n2, n3, n4);
                triStack.Push(iT);
                triStack.Push(iTopo);
            }
        }
    }

    // -------------------------------------------------------------------------
    // Locate triangle containing a vertex
    // -------------------------------------------------------------------------

    private (int iT, int iTopo) WalkingSearchTrianglesAt(int iVert, int startVertex)
    {
        var v = _vertices[iVert];
        int iT = WalkTriangles(startVertex, v);
        var t = _triangles[iT];

        var loc = LocatePointTriangle(v, _vertices[t.V0], _vertices[t.V1], _vertices[t.V2]);

        if (loc == PtTriLocation.Outside)
        {
            // Walk hit a degenerate cycle; fall back to brute-force linear scan
            iT = FindTriangleLinear(v, out loc);
            t = _triangles[iT];
        }

        if (loc == PtTriLocation.OnVertex)
        {
            int iDupe = _vertices[t.V0] == v ? t.V0 : _vertices[t.V1] == v ? t.V1 : t.V2;
            throw new DuplicateVertexException(iVert - _nTargetVerts, iDupe - _nTargetVerts);
        }

        int iNeigh = TriangleUtils.IsOnEdge(loc)
            ? t.GetNeighbor(TriangleUtils.EdgeNeighborFromLocation(loc))
            : Indices.NoNeighbor;
        return (iT, iNeigh);
    }

    /// <summary>Brute-force O(n) fallback: scan all triangles to find the one containing <paramref name="pos"/>.</summary>
    private int FindTriangleLinear(V2d<T> pos, out PtTriLocation loc)
    {
        for (int i = 0; i < _triangles.Count; i++)
        {
            var t = _triangles[i];
            loc = LocatePointTriangle(pos, _vertices[t.V0], _vertices[t.V1], _vertices[t.V2]);
            if (loc != PtTriLocation.Outside)
            {
                return i;
            }
        }
        throw new TriangulationException($"No triangle found for point ({pos.X}, {pos.Y}).");
    }

    private int WalkTriangles(int startVertex, V2d<T> pos)
    {
        int currTri = _vertTris[startVertex];
        // SplitMix64 with state=0, fresh per call — matches C++ detail::SplitMix64RandGen.
        ulong prngState = 0UL;
        for (int guard = 0; guard < 1_000_000; guard++)
        {
            var t = _triangles[currTri];
            bool found = true;
            int offset = (int)(SplitMix64(ref prngState) % 3UL);
            for (int i = 0; i < 3; i++)
            {
                int idx = (i + offset) % 3;
                int vStart = t.GetVertex(idx);
                int vEnd = t.GetVertex(TriangleUtils.Ccw(idx));
                var loc = LocatePointLine(pos, _vertices[vStart], _vertices[vEnd]);
                int iN = t.GetNeighbor(idx);
                if (loc == PtLineLocation.Right && iN != Indices.NoNeighbor)
                {
                    currTri = iN;
                    found = false;
                    break;
                }
            }
            if (found) { return currTri; }
        }
        // Walk did not converge (very degenerate triangulation) — let caller fall back.
        return currTri;
    }

    /// <summary>
    /// SplitMix64 PRNG — direct port of C++ <c>detail::SplitMix64RandGen::operator()</c>.
    /// Initial state 0 and a fresh instance per <see cref="WalkTriangles"/> call
    /// matches C++ behavior exactly.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static ulong SplitMix64(ref ulong state)
    {
        ulong z = (state += 0x9e3779b97f4a7c15UL);
        z = (z ^ (z >> 30)) * 0xbf58476d1ce4e5b9UL;
        z = (z ^ (z >> 27)) * 0x94d049bb133111ebUL;
        return z ^ (z >> 31);
    }

    // -------------------------------------------------------------------------
    // Insert vertex inside triangle or on edge
    // -------------------------------------------------------------------------

    private Stack<int> InsertVertexInsideTriangle(int v, int iT)
    {
        int iNewT1 = AddTriangle();
        int iNewT2 = AddTriangle();

        var t = _triangles[iT];
        int v1 = t.V0, v2 = t.V1, v3 = t.V2;
        int n1 = t.N0, n2 = t.N1, n3 = t.N2;

        _triangles[iNewT1] = new Triangle(v2, v3, v, n2, iNewT2, iT);
        _triangles[iNewT2] = new Triangle(v3, v1, v, n3, iT, iNewT1);
        _triangles[iT] = new Triangle(v1, v2, v, n1, iNewT1, iNewT2);

        SetAdjacentTriangle(v, iT);
        SetAdjacentTriangle(v3, iNewT1);
        ChangeNeighbor(n2, iT, iNewT1);
        ChangeNeighbor(n3, iT, iNewT2);

        var stack = new Stack<int>(3);
        stack.Push(iT);
        stack.Push(iNewT1);
        stack.Push(iNewT2);
        return stack;
    }

    private Stack<int> InsertVertexOnEdge(int v, int iT1, int iT2, bool handleFixedSplitEdge)
    {
        int iTnew1 = AddTriangle();
        int iTnew2 = AddTriangle();

        var t1 = _triangles[iT1];
        int i1 = TriangleUtils.NeighborIndex(t1, iT2);
        int v1 = t1.GetVertex(TriangleUtils.OpposedVertexIndex(i1));
        int v2 = t1.GetVertex(TriangleUtils.Ccw(TriangleUtils.OpposedVertexIndex(i1)));
        int n1 = t1.GetNeighbor(TriangleUtils.OpposedVertexIndex(i1));
        int n4 = t1.GetNeighbor(TriangleUtils.Cw(TriangleUtils.OpposedVertexIndex(i1)));

        var t2 = _triangles[iT2];
        int i2 = TriangleUtils.NeighborIndex(t2, iT1);
        int v3 = t2.GetVertex(TriangleUtils.OpposedVertexIndex(i2));
        int v4 = t2.GetVertex(TriangleUtils.Ccw(TriangleUtils.OpposedVertexIndex(i2)));
        int n3 = t2.GetNeighbor(TriangleUtils.OpposedVertexIndex(i2));
        int n2 = t2.GetNeighbor(TriangleUtils.Cw(TriangleUtils.OpposedVertexIndex(i2)));

        _triangles[iT1] = new Triangle(v, v1, v2, iTnew1, n1, iT2);
        _triangles[iT2] = new Triangle(v, v2, v3, iT1, n2, iTnew2);
        _triangles[iTnew1] = new Triangle(v, v4, v1, iTnew2, n4, iT1);
        _triangles[iTnew2] = new Triangle(v, v3, v4, iT2, n3, iTnew1);

        SetAdjacentTriangle(v, iT1);
        SetAdjacentTriangle(v4, iTnew1);
        ChangeNeighbor(n4, iT1, iTnew1);
        ChangeNeighbor(n3, iT2, iTnew2);

        if (handleFixedSplitEdge)
        {
            var sharedEdge = new Edge(v2, v4);
            if (_fixedEdges.Contains(sharedEdge))
                SplitFixedEdge(sharedEdge, v);
        }

        var stack = new Stack<int>(4);
        stack.Push(iT1);
        stack.Push(iTnew2);
        stack.Push(iT2);
        stack.Push(iTnew1);
        return stack;
    }

    // -------------------------------------------------------------------------
    // Edge flip
    // -------------------------------------------------------------------------

    private void EdgeFlipInfo(
        int iT, int iV1,
        out int iTopo, out int iV2, out int iV3, out int iV4,
        out int n1, out int n2, out int n3, out int n4)
    {
        var t = _triangles[iT];
        if (t.V0 == iV1)
        {
            iV2 = t.V1; iV4 = t.V2;
            n1 = t.N0; n3 = t.N2;
            iTopo = t.N1;
        }
        else if (t.V1 == iV1)
        {
            iV2 = t.V2; iV4 = t.V0;
            n1 = t.N1; n3 = t.N0;
            iTopo = t.N2;
        }
        else
        {
            iV2 = t.V0; iV4 = t.V1;
            n1 = t.N2; n3 = t.N1;
            iTopo = t.N0;
        }

        n2 = Indices.NoNeighbor;
        n4 = Indices.NoNeighbor;
        iV3 = Indices.NoVertex;

        if (iTopo == Indices.NoNeighbor) return;

        var tOpo = _triangles[iTopo];
        int oi = TriangleUtils.NeighborIndex(tOpo, iT);
        int ov = TriangleUtils.OpposedVertexIndex(oi);
        iV3 = tOpo.GetVertex(ov);
        // n2 and n4 use the NEIGHBOR position (oi), not the vertex position (ov)
        n2 = tOpo.GetNeighbor(TriangleUtils.Ccw(oi));
        n4 = tOpo.GetNeighbor(TriangleUtils.Cw(oi));
    }

    private bool IsFlipNeeded(int iV1, int iV2, int iV3, int iV4)
    {
        if (_fixedEdges.Contains(new Edge(iV2, iV4))) return false;

        var v1 = _vertices[iV1];
        var v2 = _vertices[iV2];
        var v3 = _vertices[iV3];
        var v4 = _vertices[iV4];

        if (_superGeomType == SuperGeometryType.SuperTriangle)
        {
            int st = Indices.SuperTriangleVertexCount;
            if (iV1 < st)
            {
                if (iV2 < st)
                    return LocatePointLine(v2, v3, v4) == LocatePointLine(v1, v3, v4);
                if (iV4 < st)
                    return LocatePointLine(v4, v2, v3) == LocatePointLine(v1, v2, v3);
                return false;
            }
            if (iV3 < st)
            {
                if (iV2 < st)
                    return LocatePointLine(v2, v1, v4) == LocatePointLine(v3, v1, v4);
                if (iV4 < st)
                    return LocatePointLine(v4, v2, v1) == LocatePointLine(v3, v2, v1);
                return false;
            }
            if (iV2 < st)
                return LocatePointLine(v2, v3, v4) == LocatePointLine(v1, v3, v4);
            if (iV4 < st)
                return LocatePointLine(v4, v2, v3) == LocatePointLine(v1, v2, v3);
        }
        return IsInCircumcircle(v1, v2, v3, v4);
    }

    private void FlipEdge(
        int iT, int iTopo,
        int v1, int v2, int v3, int v4,
        int n1, int n2, int n3, int n4)
    {
        _triangles[iT] = new Triangle(v4, v1, v3, n3, iTopo, n4);
        _triangles[iTopo] = new Triangle(v2, v3, v1, n2, iT, n1);
        ChangeNeighbor(n1, iT, iTopo);
        ChangeNeighbor(n4, iTopo, iT);
        if (!IsFinalized)
        {
            SetAdjacentTriangle(v4, iT);
            SetAdjacentTriangle(v2, iTopo);
        }
    }

    // -------------------------------------------------------------------------
    // Constraint edge insertion
    // -------------------------------------------------------------------------

    private void InsertEdgeIteration(
        Edge edge, Edge originalEdge,
        List<Edge> remaining,
        List<TriangulatePseudoPolygonTask> tppIterations)
    {
        int iA = edge.V1, iB = edge.V2;
        if (iA == iB) return;

        if (HasEdge(iA, iB))
        {
            FixEdge(edge, originalEdge);
            return;
        }

        var a = _vertices[iA];
        var b = _vertices[iB];
        T distTol = _minDistToConstraintEdge == T.Zero
            ? T.Zero
            : _minDistToConstraintEdge * TriangleUtils.Distance(a, b);

        var (iT, iVL, iVR) = IntersectedTriangle(iA, a, b, distTol);
        if (iT == Indices.NoNeighbor)
        {
            var edgePart = new Edge(iA, iVL);
            FixEdge(edgePart, originalEdge);
            remaining.Add(new Edge(iVL, iB));
            return;
        }

        var polyL = new List<int>(8) { iA, iVL };
        var polyR = new List<int>(8) { iA, iVR };
        var outerTris = new Dictionary<Edge, int>
        {
            [new Edge(iA, iVL)] = TriangleUtils.EdgeNeighbor(_triangles[iT], iA, iVL),
            [new Edge(iA, iVR)] = TriangleUtils.EdgeNeighbor(_triangles[iT], iA, iVR),
        };
        var intersected = new List<int>(8) { iT };

        int iV = iA;
        var t = _triangles[iT];

        while (!t.ContainsVertex(iB))
        {
            int iTopo = TriangleUtils.OpposedTriangle(t, iV);
            var tOpo = _triangles[iTopo];
            int iVopo = TriangleUtils.OpposedVertex(tOpo, iT);

            HandleIntersectingEdgeStrategy(iVL, iVR, iA, iB, iT, iTopo, originalEdge, a, b, distTol, remaining, tppIterations, out bool @return);
            if (@return) return;

            var loc = LocatePointLine(_vertices[iVopo], a, b, distTol);
            if (loc == PtLineLocation.Left)
            {
                var e = new Edge(polyL[^1], iVopo);
                int outer = TriangleUtils.EdgeNeighbor(tOpo, e.V1, e.V2);
                if (!outerTris.TryAdd(e, outer)) outerTris[e] = Indices.NoNeighbor;
                polyL.Add(iVopo);
                iV = iVL;
                iVL = iVopo;
            }
            else if (loc == PtLineLocation.Right)
            {
                var e = new Edge(polyR[^1], iVopo);
                int outer = TriangleUtils.EdgeNeighbor(tOpo, e.V1, e.V2);
                if (!outerTris.TryAdd(e, outer)) outerTris[e] = Indices.NoNeighbor;
                polyR.Add(iVopo);
                iV = iVR;
                iVR = iVopo;
            }
            else // on line
            {
                iB = iVopo;
            }

            intersected.Add(iTopo);
            iT = iTopo;
            t = _triangles[iT];
        }

        outerTris[new Edge(polyL[^1], iB)] = TriangleUtils.EdgeNeighbor(t, polyL[^1], iB);
        outerTris[new Edge(polyR[^1], iB)] = TriangleUtils.EdgeNeighbor(t, polyR[^1], iB);
        polyL.Add(iB);
        polyR.Add(iB);

        // Ensure start/end vertices have valid non-intersected triangle
        if (_vertTris[iA] == intersected[0]) PivotVertexTriangleCW(iA);
        if (_vertTris[iB] == intersected[^1]) PivotVertexTriangleCW(iB);

        polyR.Reverse();

        // Re-use intersected triangles
        int iTL = intersected[^1]; intersected.RemoveAt(intersected.Count - 1);
        int iTR = intersected[^1]; intersected.RemoveAt(intersected.Count - 1);

        TriangulatePseudoPolygon(polyL, outerTris, iTL, iTR, intersected, tppIterations);
        TriangulatePseudoPolygon(polyR, outerTris, iTR, iTL, intersected, tppIterations);

        if (iB != edge.V2)
        {
            FixEdge(new Edge(iA, iB), originalEdge);
            remaining.Add(new Edge(iB, edge.V2));
        }
        else
        {
            FixEdge(edge, originalEdge);
        }
    }

    private void HandleIntersectingEdgeStrategy(
        int iVL, int iVR, int iA, int iB, int iT, int iTopo,
        Edge originalEdge, V2d<T> a, V2d<T> b, T distTol,
        List<Edge> remaining, List<TriangulatePseudoPolygonTask> tppIterations,
        out bool @return)
    {
        @return = false;
        var edgeLR = new Edge(iVL, iVR);
        switch (_intersectingEdgesStrategy)
        {
            case IntersectingConstraintEdges.NotAllowed:
                if (_fixedEdges.Contains(edgeLR))
                {
                    var e1 = originalEdge;
                    var e2 = edgeLR;
                    if (_pieceToOriginals.TryGetValue(e2, out var origE2) && origE2.Count > 0) e2 = origE2[0];
                    e1 = new Edge(e1.V1 - _nTargetVerts, e1.V2 - _nTargetVerts);
                    e2 = new Edge(e2.V1 - _nTargetVerts, e2.V2 - _nTargetVerts);
                    throw new IntersectingConstraintsException(e1, e2);
                }
                break;

            case IntersectingConstraintEdges.TryResolve:
                if (_fixedEdges.Contains(edgeLR))
                {
                    var newV = IntersectionPosition(_vertices[iA], _vertices[iB], _vertices[iVL], _vertices[iVR]);
                    int iNewVert = SplitFixedEdgeAt(edgeLR, newV, iT, iTopo);
                    remaining.Add(new Edge(iA, iNewVert));
                    remaining.Add(new Edge(iNewVert, iB));
                    @return = true;
                }
                break;
        }
    }

    // -------------------------------------------------------------------------
    // Conforming edge insertion
    // -------------------------------------------------------------------------

    private void ConformToEdgeIteration(
        Edge edge, List<Edge> originals, ushort overlaps,
        List<ConformToEdgeTask> remaining)
    {
        int iA = edge.V1, iB = edge.V2;
        if (iA == iB) return;

        if (HasEdge(iA, iB))
        {
            FixEdge(edge);
            if (overlaps > 0) _overlapCount[edge] = overlaps;
            if (originals.Count > 0 && edge != originals[0])
                InsertUnique(_pieceToOriginals.GetOrAdd(edge), originals);
            return;
        }

        var a = _vertices[iA];
        var b = _vertices[iB];
        T distTol = _minDistToConstraintEdge == T.Zero
            ? T.Zero
            : _minDistToConstraintEdge * TriangleUtils.Distance(a, b);

        var (iT, iVleft, iVright) = IntersectedTriangle(iA, a, b, distTol);
        if (iT == Indices.NoNeighbor)
        {
            var part = new Edge(iA, iVleft);
            FixEdge(part);
            if (overlaps > 0) _overlapCount[part] = overlaps;
            InsertUnique(_pieceToOriginals.GetOrAdd(part), originals);
            remaining.Add(new ConformToEdgeTask(new Edge(iVleft, iB), originals, overlaps));
            return;
        }

        int iV = iA;
        var t = _triangles[iT];
        while (!t.ContainsVertex(iB))
        {
            int iTopo = TriangleUtils.OpposedTriangle(t, iV);
            var tOpo = _triangles[iTopo];
            int iVopo = TriangleUtils.OpposedVertex(tOpo, iT);
            var vOpo = _vertices[iVopo];

            HandleConformIntersecting(iVleft, iVright, iA, iB, iT, iTopo,
                originals, overlaps, remaining, out bool @return);
            if (@return) return;

            iT = iTopo;
            t = _triangles[iT];
            var loc = LocatePointLine(vOpo, a, b, distTol);
            if (loc == PtLineLocation.Left) { iV = iVleft; iVleft = iVopo; }
            else if (loc == PtLineLocation.Right) { iV = iVright; iVright = iVopo; }
            else iB = iVopo; // on line
        }

        if (iB != edge.V2)
            remaining.Add(new ConformToEdgeTask(new Edge(iB, edge.V2), originals, overlaps));

        // Insert midpoint and recurse
        int iMid = _vertices.Count;
        var start = _vertices[iA];
        var end = _vertices[iB];
        T two = T.One + T.One;
        AddNewVertex(new V2d<T>((start.X + end.X) / two, (start.Y + end.Y) / two), Indices.NoNeighbor);

        var flippedFixed = InsertVertex_FlipFixedEdges(iMid);

        remaining.Add(new ConformToEdgeTask(new Edge(iMid, iB), originals, overlaps));
        remaining.Add(new ConformToEdgeTask(new Edge(iA, iMid), originals, overlaps));

        // Re-insert flipped fixed edges
        foreach (var fe in flippedFixed)
        {
            _fixedEdges.Remove(fe);
            ushort prevOv = _overlapCount.TryGetValue(fe, out var ov) ? ov : (ushort)0;
            _overlapCount.Remove(fe);
            var prevOrig = _pieceToOriginals.TryGetValue(fe, out var po) ? po : new List<Edge> { fe };
            remaining.Add(new ConformToEdgeTask(fe, prevOrig, prevOv));
        }
    }

    private void HandleConformIntersecting(
        int iVleft, int iVright, int iA, int iB, int iT, int iTopo,
        List<Edge> originals, ushort overlaps,
        List<ConformToEdgeTask> remaining,
        out bool @return)
    {
        @return = false;
        var edgeLR = new Edge(iVleft, iVright);
        switch (_intersectingEdgesStrategy)
        {
            case IntersectingConstraintEdges.NotAllowed:
                if (_fixedEdges.Contains(edgeLR))
                {
                    var e1 = _pieceToOriginals.TryGetValue(edgeLR, out var po1) && po1.Count > 0 ? po1[0] : edgeLR;
                    throw new IntersectingConstraintsException(
                        new Edge(e1.V1 - _nTargetVerts, e1.V2 - _nTargetVerts),
                        edgeLR);
                }
                break;
            case IntersectingConstraintEdges.TryResolve:
                if (_fixedEdges.Contains(edgeLR))
                {
                    var newV = IntersectionPosition(_vertices[iA], _vertices[iB], _vertices[iVleft], _vertices[iVright]);
                    int iNewVert = SplitFixedEdgeAt(edgeLR, newV, iT, iTopo);
                    remaining.Add(new ConformToEdgeTask(new Edge(iNewVert, iB), originals, overlaps));
                    remaining.Add(new ConformToEdgeTask(new Edge(iA, iNewVert), originals, overlaps));
                    @return = true;
                }
                break;
        }
    }

    // -------------------------------------------------------------------------
    // Pseudo-polygon triangulation (for constrained edges)
    // -------------------------------------------------------------------------

    // Task: (iA, iB, iT, iParent, iInParent)
    private readonly record struct TriangulatePseudoPolygonTask(int IA, int IB, int IT, int IParent, int IInParent);

    private void TriangulatePseudoPolygon(
        List<int> poly,
        Dictionary<Edge, int> outerTris,
        int iT, int iN,
        List<int> trianglesToReuse,
        List<TriangulatePseudoPolygonTask> iterations)
    {
        iterations.Clear();
        iterations.Add(new TriangulatePseudoPolygonTask(0, poly.Count - 1, iT, iN, 0));
        while (iterations.Count > 0)
        {
            TriangulatePseudoPolygonIteration(poly, outerTris, trianglesToReuse, iterations);
        }
    }

    private void TriangulatePseudoPolygonIteration(
        List<int> poly,
        Dictionary<Edge, int> outerTris,
        List<int> trianglesToReuse,
        List<TriangulatePseudoPolygonTask> iterations)
    {
        var (iA, iB, iT, iParent, iInParent) = iterations[^1];
        iterations.RemoveAt(iterations.Count - 1);

        int iC = FindDelaunayPoint(poly, iA, iB);
        int a = poly[iA], b = poly[iB], c = poly[iC];

        // Second part (after c)
        if (iB - iC > 1)
        {
            int iNext = trianglesToReuse[^1]; trianglesToReuse.RemoveAt(trianglesToReuse.Count - 1);
            iterations.Add(new TriangulatePseudoPolygonTask(iC, iB, iNext, iT, 1));
        }
        else
        {
            var outerEdge = new Edge(b, c);
            int outerTri = outerTris[outerEdge];
            var tri = _triangles[iT]; tri.N1 = Indices.NoNeighbor; _triangles[iT] = tri;
            if (outerTri != Indices.NoNeighbor)
            {
                tri = _triangles[iT]; tri.N1 = outerTri; _triangles[iT] = tri;
                ChangeNeighbor(outerTri, c, b, iT);
            }
            else outerTris[outerEdge] = iT;
        }

        // First part (before c)
        if (iC - iA > 1)
        {
            int iNext = trianglesToReuse[^1]; trianglesToReuse.RemoveAt(trianglesToReuse.Count - 1);
            iterations.Add(new TriangulatePseudoPolygonTask(iA, iC, iNext, iT, 2));
        }
        else
        {
            var outerEdge = new Edge(c, a);
            int outerTri = outerTris[outerEdge];
            var tri = _triangles[iT]; tri.N2 = Indices.NoNeighbor; _triangles[iT] = tri;
            if (outerTri != Indices.NoNeighbor)
            {
                tri = _triangles[iT]; tri.N2 = outerTri; _triangles[iT] = tri;
                ChangeNeighbor(outerTri, c, a, iT);
            }
            else outerTris[outerEdge] = iT;
        }

        // Finalize triangle
        var parentTri = _triangles[iParent]; parentTri.SetNeighbor(iInParent, iT); _triangles[iParent] = parentTri;
        var tFinal = _triangles[iT]; tFinal.N0 = iParent; tFinal.V0 = a; tFinal.V1 = b; tFinal.V2 = c; _triangles[iT] = tFinal;
        SetAdjacentTriangle(c, iT);
    }

    private int FindDelaunayPoint(List<int> poly, int iA, int iB)
    {
        var a = _vertices[poly[iA]];
        var b = _vertices[poly[iB]];
        int best = iA + 1;
        var bestV = _vertices[poly[best]];
        for (int i = iA + 1; i < iB; i++)
        {
            var v = _vertices[poly[i]];
            if (IsInCircumcircle(v, a, b, bestV))
            {
                best = i;
                bestV = v;
            }
        }
        return best;
    }

    // -------------------------------------------------------------------------
    // Triangle search helpers
    // -------------------------------------------------------------------------

    private (int iT, int iVleft, int iVright) IntersectedTriangle(
        int iA, V2d<T> a, V2d<T> b, T tolerance)
    {
        int startTri = _vertTris[iA];
        int iT = startTri;
        do
        {
            var t = _triangles[iT];
            int i = TriangleUtils.VertexIndex(t, iA);
            int iP2 = t.GetVertex(TriangleUtils.Ccw(i));
            var p2 = _vertices[iP2];
            T orientP2 = Orient2D(p2, a, b);
            var locP2 = TriangleUtils.ClassifyOrientation(orientP2, tolerance);
            if (locP2 == PtLineLocation.Right)
            {
                int iP1 = t.GetVertex(TriangleUtils.Cw(i));
                var p1 = _vertices[iP1];
                T orientP1 = Orient2D(p1, a, b);
                var locP1 = TriangleUtils.ClassifyOrientation(orientP1, T.Zero);
                if (locP1 == PtLineLocation.OnLine)
                    return (Indices.NoNeighbor, iP1, iP1);
                if (locP1 == PtLineLocation.Left)
                {
                    if (tolerance != T.Zero)
                    {
                        T absp1 = T.Abs(orientP1), absp2 = T.Abs(orientP2);
                        T closestOrient; int iClosest;
                        if (absp1 <= absp2) { closestOrient = orientP1; iClosest = iP1; }
                        else { closestOrient = orientP2; iClosest = iP2; }
                        if (TriangleUtils.ClassifyOrientation(closestOrient, tolerance) == PtLineLocation.OnLine)
                            return (Indices.NoNeighbor, iClosest, iClosest);
                    }
                    return (iT, iP1, iP2);
                }
            }
            (iT, _) = t.Next(iA);
        } while (iT != startTri);

        throw new TriangulationException("Could not find vertex triangle intersected by an edge.");
    }

    // -------------------------------------------------------------------------
    // Topology changes
    // -------------------------------------------------------------------------

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private void SetAdjacentTriangle(int v, int iT)
    {
        _vertTris[v] = iT;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private void ChangeNeighbor(int iT, int oldN, int newN)
    {
        if (iT == Indices.NoNeighbor) return;
        var t = _triangles[iT];
        if (t.N0 == oldN) t.N0 = newN;
        else if (t.N1 == oldN) t.N1 = newN;
        else t.N2 = newN;
        _triangles[iT] = t;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private void ChangeNeighbor(int iT, int va, int vb, int newN)
    {
        if (iT == Indices.NoNeighbor) return;
        var t = _triangles[iT];
        t.SetNeighbor(TriangleUtils.EdgeNeighborIndex(t, va, vb), newN);
        _triangles[iT] = t;
    }

    private void PivotVertexTriangleCW(int v)
    {
        int iT = _vertTris[v];
        var (iNext, _) = _triangles[iT].Next(v);
        _vertTris[v] = iNext;
    }

    // -------------------------------------------------------------------------
    // Fixed edge management
    // -------------------------------------------------------------------------

    private void FixEdge(Edge edge)
    {
        if (!_fixedEdges.Add(edge))
        {
            _overlapCount.TryGetValue(edge, out ushort cur);
            _overlapCount[edge] = (ushort)(cur + 1);
        }
    }

    private void FixEdge(Edge edge, Edge originalEdge)
    {
        FixEdge(edge);
        if (edge != originalEdge)
            InsertUnique(_pieceToOriginals.GetOrAdd(edge), originalEdge);
    }

    private void SplitFixedEdge(Edge edge, int iSplitVert)
    {
        var half1 = new Edge(edge.V1, iSplitVert);
        var half2 = new Edge(iSplitVert, edge.V2);
        _fixedEdges.Remove(edge);
        FixEdge(half1);
        FixEdge(half2);
        if (_overlapCount.TryGetValue(edge, out ushort ov))
        {
            _overlapCount.TryGetValue(half1, out ushort h1); _overlapCount[half1] = (ushort)(h1 + ov);
            _overlapCount.TryGetValue(half2, out ushort h2); _overlapCount[half2] = (ushort)(h2 + ov);
            _overlapCount.Remove(edge);
        }
        var newOrig = new List<Edge> { edge };
        if (_pieceToOriginals.TryGetValue(edge, out var originals))
        {
            newOrig = originals;
            _pieceToOriginals.Remove(edge);
        }
        InsertUnique(_pieceToOriginals.GetOrAdd(half1), newOrig);
        InsertUnique(_pieceToOriginals.GetOrAdd(half2), newOrig);
    }

    private int SplitFixedEdgeAt(Edge edge, V2d<T> splitVert, int iT, int iTopo)
    {
        int iSplit = _vertices.Count;
        AddNewVertex(splitVert, Indices.NoNeighbor);
        var stack = InsertVertexOnEdge(iSplit, iT, iTopo, handleFixedSplitEdge: false);
        TryAddVertexToLocator(iSplit);
        EnsureDelaunayByEdgeFlips(iSplit, stack);
        SplitFixedEdge(edge, iSplit);
        return iSplit;
    }

    // -------------------------------------------------------------------------
    // Edge query
    // -------------------------------------------------------------------------

    private bool HasEdge(int va, int vb)
    {
        int startTri = _vertTris[va];
        int iT = startTri;
        do
        {
            var t = _triangles[iT];
            if (t.ContainsVertex(vb)) return true;
            (iT, _) = t.Next(va);
        } while (iT != startTri && iT != Indices.NoNeighbor);
        return false;
    }

    // -------------------------------------------------------------------------
    // Finalization
    // -------------------------------------------------------------------------

    private HashSet<int> GrowToBoundary(Stack<int> seeds)
    {
        var traversed = new HashSet<int>();
        while (seeds.Count > 0)
        {
            int iT = seeds.Pop();
            traversed.Add(iT);
            var t = _triangles[iT];
            for (int i = 0; i < 3; i++)
            {
                int va = t.GetVertex(TriangleUtils.Ccw(i));
                int vb = t.GetVertex(TriangleUtils.Cw(i));
                var opEdge = new Edge(va, vb);
                if (_fixedEdges.Contains(opEdge)) continue;
                int iN = t.GetNeighbor(TriangleUtils.OpposedNeighborIndex(i));
                if (iN != Indices.NoNeighbor && !traversed.Contains(iN))
                    seeds.Push(iN);
            }
        }
        return traversed;
    }

    private void FinalizeTriangulation(HashSet<int> removedTriangles)
    {
        _vertTris.Clear();

        if (_superGeomType == SuperGeometryType.SuperTriangle)
        {
            _vertices.RemoveRange(0, Indices.SuperTriangleVertexCount);
            RemapEdgesNoSuperTriangle(_fixedEdges);
            RemapEdgesNoSuperTriangle(_overlapCount);
            RemapEdgesNoSuperTriangle(_pieceToOriginals);
        }

        RemoveTriangles(removedTriangles);

        if (_superGeomType == SuperGeometryType.SuperTriangle)
        {
            int offset = Indices.SuperTriangleVertexCount;
            for (int i = 0; i < _triangles.Count; i++)
            {
                var t = _triangles[i];
                t.V0 -= offset; t.V1 -= offset; t.V2 -= offset;
                _triangles[i] = t;
            }
        }
    }

    private static Edge ShiftEdge(Edge e, int offset) => new(e.V1 - offset, e.V2 - offset);

    private void RemapEdgesNoSuperTriangle(HashSet<Edge> edges)
    {
        var updated = new HashSet<Edge>(edges.Count);
        foreach (var e in edges) updated.Add(ShiftEdge(e, Indices.SuperTriangleVertexCount));
        edges.Clear();
        foreach (var e in updated) edges.Add(e);
    }

    private void RemapEdgesNoSuperTriangle<TVal>(Dictionary<Edge, TVal> dict)
    {
        var updated = new Dictionary<Edge, TVal>(dict.Count);
        foreach (var kv in dict) updated[ShiftEdge(kv.Key, Indices.SuperTriangleVertexCount)] = kv.Value;
        dict.Clear();
        foreach (var kv in updated) dict[kv.Key] = kv.Value;
    }

    private void RemapEdgesNoSuperTriangle(Dictionary<Edge, List<Edge>> dict)
    {
        var updated = new Dictionary<Edge, List<Edge>>(dict.Count);
        foreach (var kv in dict)
        {
            var newKey = ShiftEdge(kv.Key, Indices.SuperTriangleVertexCount);
            var newList = new List<Edge>(kv.Value.Count);
            foreach (var e in kv.Value) newList.Add(ShiftEdge(e, Indices.SuperTriangleVertexCount));
            updated[newKey] = newList;
        }
        dict.Clear();
        foreach (var kv in updated) dict[kv.Key] = kv.Value;
    }

    private void RemoveTriangles(HashSet<int> removed)
    {
        if (removed.Count == 0) return;
        // Build compact mapping: old index → new index
        var mapping = new int[_triangles.Count];
        int newIdx = 0;
        for (int i = 0; i < _triangles.Count; i++)
        {
            if (removed.Contains(i)) { mapping[i] = Indices.NoNeighbor; continue; }
            mapping[i] = newIdx++;
        }
        // Compact triangle list
        int write = 0;
        for (int i = 0; i < _triangles.Count; i++)
        {
            if (removed.Contains(i)) continue;
            _triangles[write++] = _triangles[i];
        }
        _triangles.RemoveRange(write, _triangles.Count - write);
        // Re-map neighbor indices
        for (int i = 0; i < _triangles.Count; i++)
        {
            var t = _triangles[i];
            t.N0 = t.N0 == Indices.NoNeighbor ? Indices.NoNeighbor : (removed.Contains(t.N0) ? Indices.NoNeighbor : mapping[t.N0]);
            t.N1 = t.N1 == Indices.NoNeighbor ? Indices.NoNeighbor : (removed.Contains(t.N1) ? Indices.NoNeighbor : mapping[t.N1]);
            t.N2 = t.N2 == Indices.NoNeighbor ? Indices.NoNeighbor : (removed.Contains(t.N2) ? Indices.NoNeighbor : mapping[t.N2]);
            _triangles[i] = t;
        }
    }

    // -------------------------------------------------------------------------
    // Layer depth / hole detection
    // -------------------------------------------------------------------------

    private ushort[] CalculateTriangleDepths()
    {
        var depths = new ushort[_triangles.Count];
        for (int i = 0; i < depths.Length; i++) depths[i] = ushort.MaxValue;

        // Find a triangle touching the super-triangle vertex 0
        int seedTri = _vertTris.Count > 0 ? _vertTris[0] : Indices.NoNeighbor;
        if (seedTri == Indices.NoNeighbor && _triangles.Count > 0) seedTri = 0;

        var layerSeeds = new Stack<int>();
        layerSeeds.Push(seedTri);
        ushort depth = 0;

        while (layerSeeds.Count > 0)
        {
            var nextLayer = PeelLayer(layerSeeds, depth, depths);
            layerSeeds = new Stack<int>(nextLayer.Keys);
            depth++;
        }
        return depths;
    }

    private Dictionary<int, ushort> PeelLayer(
        Stack<int> seeds, ushort layerDepth, ushort[] triDepths)
    {
        var behindBoundary = new Dictionary<int, ushort>();
        while (seeds.Count > 0)
        {
            int iT = seeds.Pop();
            triDepths[iT] = Math.Min(triDepths[iT], layerDepth);
            behindBoundary.Remove(iT);

            var t = _triangles[iT];
            for (int i = 0; i < 3; i++)
            {
                int va = t.GetVertex(TriangleUtils.Ccw(i));
                int vb = t.GetVertex(TriangleUtils.Cw(i));
                var opEdge = new Edge(va, vb);
                int iN = t.GetNeighbor(TriangleUtils.OpposedNeighborIndex(i));
                if (iN == Indices.NoNeighbor || triDepths[iN] <= layerDepth) continue;

                if (_fixedEdges.Contains(opEdge))
                {
                    ushort nextDepth = layerDepth;
                    if (_overlapCount.TryGetValue(opEdge, out ushort ov))
                        nextDepth += ov;
                    nextDepth++;
                    if (!behindBoundary.ContainsKey(iN) || behindBoundary[iN] > nextDepth)
                        behindBoundary[iN] = nextDepth;
                }
                else
                {
                    seeds.Push(iN);
                }
            }
        }
        return behindBoundary;
    }

    // -------------------------------------------------------------------------
    // KD-tree helpers
    // -------------------------------------------------------------------------

    private void InitKdTree()
    {
        var box = new Box2d<T>();
        box.Envelop(_vertices);
        _kdTree = new KdTree<T>(box.Min.X, box.Min.Y, box.Max.X, box.Max.Y);
        for (int i = 0; i < _vertices.Count; i++)
            _kdTree.Insert(i, _vertices);
    }

    private void TryAddVertexToLocator(int iV)
    {
        // Only add to the locator if it's already initialized (matches C++ behavior)
        _kdTree?.Insert(iV, _vertices);
    }

    // -------------------------------------------------------------------------
    // Type-specific geometric operations (dispatched via T)
    // -------------------------------------------------------------------------

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static PtTriLocation LocatePointTriangle(V2d<T> p, V2d<T> v1, V2d<T> v2, V2d<T> v3)
    {
        // We cast to the correct type dispatch at compile time
        if (typeof(T) == typeof(double))
        {
            return TriangleUtils.LocatePointTriangle(
                Unsafe.As<V2d<T>, V2d<double>>(ref Unsafe.AsRef(in p)),
                Unsafe.As<V2d<T>, V2d<double>>(ref Unsafe.AsRef(in v1)),
                Unsafe.As<V2d<T>, V2d<double>>(ref Unsafe.AsRef(in v2)),
                Unsafe.As<V2d<T>, V2d<double>>(ref Unsafe.AsRef(in v3)));
        }
        else
        {
            return TriangleUtils.LocatePointTriangle(
                Unsafe.As<V2d<T>, V2d<float>>(ref Unsafe.AsRef(in p)),
                Unsafe.As<V2d<T>, V2d<float>>(ref Unsafe.AsRef(in v1)),
                Unsafe.As<V2d<T>, V2d<float>>(ref Unsafe.AsRef(in v2)),
                Unsafe.As<V2d<T>, V2d<float>>(ref Unsafe.AsRef(in v3)));
        }
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static PtLineLocation LocatePointLine(V2d<T> p, V2d<T> v1, V2d<T> v2, T tolerance = default)
    {
        if (typeof(T) == typeof(double))
        {
            return TriangleUtils.LocatePointLine(
                Unsafe.As<V2d<T>, V2d<double>>(ref Unsafe.AsRef(in p)),
                Unsafe.As<V2d<T>, V2d<double>>(ref Unsafe.AsRef(in v1)),
                Unsafe.As<V2d<T>, V2d<double>>(ref Unsafe.AsRef(in v2)),
                Unsafe.As<T, double>(ref Unsafe.AsRef(in tolerance)));
        }
        else
        {
            return TriangleUtils.LocatePointLine(
                Unsafe.As<V2d<T>, V2d<float>>(ref Unsafe.AsRef(in p)),
                Unsafe.As<V2d<T>, V2d<float>>(ref Unsafe.AsRef(in v1)),
                Unsafe.As<V2d<T>, V2d<float>>(ref Unsafe.AsRef(in v2)),
                Unsafe.As<T, float>(ref Unsafe.AsRef(in tolerance)));
        }
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static T Orient2D(V2d<T> p, V2d<T> v1, V2d<T> v2)
    {
        if (typeof(T) == typeof(double))
        {
            double r = Predicates.Orient2D(
                Unsafe.As<T, double>(ref Unsafe.AsRef(in v1.X)),
                Unsafe.As<T, double>(ref Unsafe.AsRef(in v1.Y)),
                Unsafe.As<T, double>(ref Unsafe.AsRef(in v2.X)),
                Unsafe.As<T, double>(ref Unsafe.AsRef(in v2.Y)),
                Unsafe.As<T, double>(ref Unsafe.AsRef(in p.X)),
                Unsafe.As<T, double>(ref Unsafe.AsRef(in p.Y)));
            return Unsafe.As<double, T>(ref r);
        }
        else
        {
            float r = Predicates.Orient2D(
                Unsafe.As<T, float>(ref Unsafe.AsRef(in v1.X)),
                Unsafe.As<T, float>(ref Unsafe.AsRef(in v1.Y)),
                Unsafe.As<T, float>(ref Unsafe.AsRef(in v2.X)),
                Unsafe.As<T, float>(ref Unsafe.AsRef(in v2.Y)),
                Unsafe.As<T, float>(ref Unsafe.AsRef(in p.X)),
                Unsafe.As<T, float>(ref Unsafe.AsRef(in p.Y)));
            return Unsafe.As<float, T>(ref r);
        }
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool IsInCircumcircle(V2d<T> p, V2d<T> v1, V2d<T> v2, V2d<T> v3)
    {
        if (typeof(T) == typeof(double))
        {
            return TriangleUtils.IsInCircumcircle(
                Unsafe.As<V2d<T>, V2d<double>>(ref Unsafe.AsRef(in p)),
                Unsafe.As<V2d<T>, V2d<double>>(ref Unsafe.AsRef(in v1)),
                Unsafe.As<V2d<T>, V2d<double>>(ref Unsafe.AsRef(in v2)),
                Unsafe.As<V2d<T>, V2d<double>>(ref Unsafe.AsRef(in v3)));
        }
        else
        {
            return TriangleUtils.IsInCircumcircle(
                Unsafe.As<V2d<T>, V2d<float>>(ref Unsafe.AsRef(in p)),
                Unsafe.As<V2d<T>, V2d<float>>(ref Unsafe.AsRef(in v1)),
                Unsafe.As<V2d<T>, V2d<float>>(ref Unsafe.AsRef(in v2)),
                Unsafe.As<V2d<T>, V2d<float>>(ref Unsafe.AsRef(in v3)));
        }
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private V2d<T> IntersectionPosition(V2d<T> a, V2d<T> b, V2d<T> c, V2d<T> d)
    {
        if (typeof(T) == typeof(double))
        {
            var r = TriangleUtils.IntersectionPosition(
                Unsafe.As<V2d<T>, V2d<double>>(ref Unsafe.AsRef(in a)),
                Unsafe.As<V2d<T>, V2d<double>>(ref Unsafe.AsRef(in b)),
                Unsafe.As<V2d<T>, V2d<double>>(ref Unsafe.AsRef(in c)),
                Unsafe.As<V2d<T>, V2d<double>>(ref Unsafe.AsRef(in d)));
            return Unsafe.As<V2d<double>, V2d<T>>(ref r);
        }
        else
        {
            var r = TriangleUtils.IntersectionPosition(
                Unsafe.As<V2d<T>, V2d<float>>(ref Unsafe.AsRef(in a)),
                Unsafe.As<V2d<T>, V2d<float>>(ref Unsafe.AsRef(in b)),
                Unsafe.As<V2d<T>, V2d<float>>(ref Unsafe.AsRef(in c)),
                Unsafe.As<V2d<T>, V2d<float>>(ref Unsafe.AsRef(in d)));
            return Unsafe.As<V2d<float>, V2d<T>>(ref r);
        }
    }

    // -------------------------------------------------------------------------
    // Misc helpers
    // -------------------------------------------------------------------------

    private readonly record struct ConformToEdgeTask(Edge Edge, List<Edge> Originals, ushort Overlaps);

    private static void InsertUnique(List<Edge> to, Edge elem)
    {
        if (!to.Contains(elem)) to.Add(elem);
    }

    private static void InsertUnique(List<Edge> to, IEnumerable<Edge> from)
    {
        foreach (var e in from) InsertUnique(to, e);
    }

    private static T ParseT(string s)
        => T.Parse(s, System.Globalization.CultureInfo.InvariantCulture);
}

internal static class DictionaryExtensions
{
    public static TVal GetOrAdd<TKey, TVal>(this Dictionary<TKey, TVal> dict, TKey key)
        where TKey : notnull
        where TVal : new()
    {
        if (!dict.TryGetValue(key, out var val))
        {
            val = new TVal();
            dict[key] = val;
        }
        return val;
    }
}

/// <summary>
/// Provides a covariant read-only view over a <see cref="Dictionary{TKey,TInner}"/>
/// where <typeparamref name="TInner"/> is assignable to <typeparamref name="TOuter"/>.
/// </summary>
internal sealed class CovariantReadOnlyDictionary<TKey, TInner, TOuter>(Dictionary<TKey, TInner> inner)
    : IReadOnlyDictionary<TKey, TOuter>
    where TKey : notnull
    where TInner : TOuter
{
    public TOuter this[TKey key] => inner[key];
    public IEnumerable<TKey> Keys => inner.Keys;
    public IEnumerable<TOuter> Values => inner.Values.Cast<TOuter>();
    public int Count => inner.Count;
    public bool ContainsKey(TKey key) => inner.ContainsKey(key);
    public bool TryGetValue(TKey key, [System.Diagnostics.CodeAnalysis.MaybeNullWhen(false)] out TOuter value)
    {
        if (inner.TryGetValue(key, out var v)) { value = v!; return true; }
        value = default!;
        return false;
    }
    public IEnumerator<KeyValuePair<TKey, TOuter>> GetEnumerator() =>
        inner.Select(kv => new KeyValuePair<TKey, TOuter>(kv.Key, kv.Value)).GetEnumerator();
    System.Collections.IEnumerator System.Collections.IEnumerable.GetEnumerator() => GetEnumerator();
}