// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
// Contribution of original implementation:
// Andre Fecteau <andre.fecteau1@gmail.com>

using System.Numerics;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;

namespace CDT;

/// <summary>
/// Incrementally-built 2D KD-tree for nearest-point queries.
/// An external point buffer is referenced by index to keep the tree compact.
/// </summary>
/// <typeparam name="T">Coordinate type.</typeparam>
internal sealed class KdTree<T>
    where T : IFloatingPoint<T>, IMinMaxValue<T>, IRootFunctions<T>
{
    private const int NumVerticesInLeaf = 32;
    private const int InitialStackDepth = 64;

    private enum SplitDir { X, Y }

    private struct Node
    {
        // children[0] == children[1] => leaf
        public int Child0, Child1;
        public List<int>? Data; // non-null only for leaf nodes

        public bool IsLeaf => Child0 == Child1;

        public static Node NewLeaf() => new() { Child0 = 0, Child1 = 0, Data = new List<int>(NumVerticesInLeaf) };
    }

    private readonly struct NearestTask
    {
        public readonly int NodeIndex;
        public readonly T MinX, MinY, MaxX, MaxY;
        public readonly SplitDir Dir;
        public readonly T DistSq;

        public NearestTask(int node, T minX, T minY, T maxX, T maxY, SplitDir dir, T distSq)
        {
            NodeIndex = node; MinX = minX; MinY = minY; MaxX = maxX; MaxY = maxY;
            Dir = dir; DistSq = distSq;
        }
    }

    private readonly List<Node> _nodes = new(64);
    private int _root;
    private SplitDir _rootDir = SplitDir.X;

    private T _minX, _minY, _maxX, _maxY;
    private bool _boxInitialized;
    private int _size;

    private NearestTask[] _stack = new NearestTask[InitialStackDepth];

    /// <summary>Initializes an empty KD-tree with no bounding box pre-set.</summary>
    public KdTree()
    {
        _minX = T.MinValue; _minY = T.MinValue;
        _maxX = T.MaxValue; _maxY = T.MaxValue;
        _root = AddNewNode();
        _boxInitialized = false;
    }

    /// <summary>Initializes an empty KD-tree with a known bounding box.</summary>
    public KdTree(T minX, T minY, T maxX, T maxY)
    {
        _minX = minX; _minY = minY; _maxX = maxX; _maxY = maxY;
        _root = AddNewNode();
        _boxInitialized = true;
    }

    /// <summary>Number of points stored.</summary>
    public int Size => _size;

    /// <summary>Inserts point at index <paramref name="iPoint"/> from the external buffer.</summary>
    public void Insert(int iPoint, IReadOnlyList<V2d<T>> points)
    {
        _size++;
        T px = points[iPoint].X, py = points[iPoint].Y;

        // Extend tree if the point falls outside the current box
        while (!IsInsideBox(px, py, _minX, _minY, _maxX, _maxY))
        {
            ExtendTree(px, py);
        }

        int node = _root;
        T minX = _minX, minY = _minY, maxX = _maxX, maxY = _maxY;
        SplitDir dir = _rootDir;

        while (true)
        {
            ref Node n = ref CollectionsMarshal.AsSpan(_nodes)[node];
            if (n.IsLeaf)
            {
                // Add point if capacity not reached
                if (n.Data!.Count < NumVerticesInLeaf)
                {
                    n.Data.Add(iPoint);
                    return;
                }

                // Lazy box initialization
                if (!_boxInitialized)
                {
                    InitializeRootBox(points);
                    minX = _minX; minY = _minY; maxX = _maxX; maxY = _maxY;
                }

                // Split the full leaf
                T mid = GetMid(minX, minY, maxX, maxY, dir);
                int c1 = AddNewNode(), c2 = AddNewNode();
                n = ref CollectionsMarshal.AsSpan(_nodes)[node]; // re-acquire after possible resize
                n.Child0 = c1; n.Child1 = c2;

                // Move existing points to children
                foreach (int ip in n.Data!)
                {
                    T cx = points[ip].X, cy = points[ip].Y;
                    int target = WhichChild(cx, cy, mid, dir);
                    _nodes[target == 0 ? c1 : c2].Data!.Add(ip);
                }
                n.Data = null; // inner node – no data list needed
            }

            T midVal = GetMid(minX, minY, maxX, maxY, dir);
            int childIdx = WhichChild(px, py, midVal, dir);
            if (dir == SplitDir.X)
            {
                if (childIdx == 0) maxX = midVal; else minX = midVal;
            }
            else
            {
                if (childIdx == 0) maxY = midVal; else minY = midVal;
            }

            node = childIdx == 0 ? _nodes[node].Child0 : _nodes[node].Child1;
            dir = dir == SplitDir.X ? SplitDir.Y : SplitDir.X;
        }
    }

    /// <summary>Finds the nearest point to <paramref name="query"/> in the external buffer.</summary>
    public int Nearest(T qx, T qy, IReadOnlyList<V2d<T>> points)
    {
        int resultIdx = 0;
        T minDistSq = T.MaxValue;
        int stackTop = -1;

        T rootDistSq = DistSqToBox(qx, qy, _minX, _minY, _maxX, _maxY);
        PushStack(ref stackTop, new NearestTask(_root, _minX, _minY, _maxX, _maxY, _rootDir, rootDistSq));

        while (stackTop >= 0)
        {
            NearestTask task = _stack[stackTop--];
            if (task.DistSq > minDistSq) continue;

            ref Node n = ref CollectionsMarshal.AsSpan(_nodes)[task.NodeIndex];
            if (n.IsLeaf)
            {
                foreach (int ip in n.Data!)
                {
                    T dx = points[ip].X - qx;
                    T dy = points[ip].Y - qy;
                    T d2 = dx * dx + dy * dy;
                    if (d2 < minDistSq) { minDistSq = d2; resultIdx = ip; }
                }
            }
            else
            {
                T mid = GetMid(task.MinX, task.MinY, task.MaxX, task.MaxY, task.Dir);
                SplitDir childDir = task.Dir == SplitDir.X ? SplitDir.Y : SplitDir.X;
                bool afterSplit = IsAfterSplit(qx, qy, mid, task.Dir);

                T dSqFarther = FartherBoxDistSq(qx, qy, task.MinX, task.MinY, task.MaxX, task.MaxY, mid, task.Dir);

                if (stackTop + 2 >= _stack.Length)
                    Array.Resize(ref _stack, _stack.Length + InitialStackDepth);

                if (afterSplit)
                {
                    // Closer = child1, Farther = child0
                    T fMinX = task.MinX, fMinY = task.MinY, fMaxX = task.MaxX, fMaxY = task.MaxY;
                    T cMinX = task.MinX, cMinY = task.MinY, cMaxX = task.MaxX, cMaxY = task.MaxY;
                    if (task.Dir == SplitDir.X) { fMaxX = mid; cMinX = mid; }
                    else { fMaxY = mid; cMinY = mid; }

                    if (dSqFarther <= minDistSq)
                        PushStack(ref stackTop, new NearestTask(n.Child0, fMinX, fMinY, fMaxX, fMaxY, childDir, dSqFarther));
                    PushStack(ref stackTop, new NearestTask(n.Child1, cMinX, cMinY, cMaxX, cMaxY, childDir, task.DistSq));
                }
                else
                {
                    // Closer = child0, Farther = child1
                    T cMinX = task.MinX, cMinY = task.MinY, cMaxX = task.MaxX, cMaxY = task.MaxY;
                    T fMinX = task.MinX, fMinY = task.MinY, fMaxX = task.MaxX, fMaxY = task.MaxY;
                    if (task.Dir == SplitDir.X) { cMaxX = mid; fMinX = mid; }
                    else { cMaxY = mid; fMinY = mid; }

                    if (dSqFarther <= minDistSq)
                        PushStack(ref stackTop, new NearestTask(n.Child1, fMinX, fMinY, fMaxX, fMaxY, childDir, dSqFarther));
                    PushStack(ref stackTop, new NearestTask(n.Child0, cMinX, cMinY, cMaxX, cMaxY, childDir, task.DistSq));
                }
            }
        }
        return resultIdx;
    }

    // -------------------------------------------------------------------------
    // Helpers
    // -------------------------------------------------------------------------

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private void PushStack(ref int top, NearestTask task)
    {
        _stack[++top] = task;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool IsInsideBox(T px, T py, T minX, T minY, T maxX, T maxY)
        => px >= minX && px <= maxX && py >= minY && py <= maxY;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static T GetMid(T minX, T minY, T maxX, T maxY, SplitDir dir)
    {
        T two = T.One + T.One;
        return dir == SplitDir.X ? (minX + maxX) / two : (minY + maxY) / two;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool IsAfterSplit(T px, T py, T split, SplitDir dir)
        => dir == SplitDir.X ? px > split : py > split;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static int WhichChild(T px, T py, T split, SplitDir dir)
        => IsAfterSplit(px, py, split, dir) ? 1 : 0;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static T DistSqToBox(T px, T py, T minX, T minY, T maxX, T maxY)
    {
        T dx = T.Max(T.Max(minX - px, T.Zero), px - maxX);
        T dy = T.Max(T.Max(minY - py, T.Zero), py - maxY);
        return dx * dx + dy * dy;
    }

    private static T FartherBoxDistSq(T px, T py, T minX, T minY, T maxX, T maxY, T mid, SplitDir dir)
    {
        if (dir == SplitDir.X)
        {
            T dx = px - mid;
            T dy = T.Max(T.Max(minY - py, T.Zero), py - maxY);
            return dx * dx + dy * dy;
        }
        else
        {
            T dx = T.Max(T.Max(minX - px, T.Zero), px - maxX);
            T dy = py - mid;
            return dx * dx + dy * dy;
        }
    }

    private int AddNewNode()
    {
        int idx = _nodes.Count;
        _nodes.Add(Node.NewLeaf());
        return idx;
    }

    private void ExtendTree(T px, T py)
    {
        int newRoot = AddNewNode();
        int newLeaf = AddNewNode();
        ref Node nr = ref CollectionsMarshal.AsSpan(_nodes)[newRoot];

        switch (_rootDir)
        {
            case SplitDir.X:
                _rootDir = SplitDir.Y;
                if (py < _minY)
                {
                    _minY -= _maxY - _minY;
                    nr.Child0 = newLeaf; nr.Child1 = _root;
                }
                else
                {
                    _maxY += _maxY - _minY;
                    nr.Child0 = _root; nr.Child1 = newLeaf;
                }
                break;
            case SplitDir.Y:
                _rootDir = SplitDir.X;
                if (px < _minX)
                {
                    _minX -= _maxX - _minX;
                    nr.Child0 = newLeaf; nr.Child1 = _root;
                }
                else
                {
                    _maxX += _maxX - _minX;
                    nr.Child0 = _root; nr.Child1 = newLeaf;
                }
                break;
        }
        _root = newRoot;
    }

    private void InitializeRootBox(IReadOnlyList<V2d<T>> points)
    {
        Node rootNode = _nodes[_root];
        T mxn = points[rootNode.Data![0]].X, myn = points[rootNode.Data[0]].Y;
        T mxx = mxn, mxy = myn;
        foreach (int ip in rootNode.Data)
        {
            T cx = points[ip].X, cy = points[ip].Y;
            if (cx < mxn) mxn = cx;
            if (cx > mxx) mxx = cx;
            if (cy < myn) myn = cy;
            if (cy > mxy) mxy = cy;
        }

        // Ensure non-zero size
        T padding = T.One;
        if (mxn == mxx) { mxn -= padding; mxx += padding; }
        if (myn == mxy) { myn -= padding; mxy += padding; }

        _minX = mxn; _minY = myn; _maxX = mxx; _maxY = mxy;
        _boxInitialized = true;
    }
}
