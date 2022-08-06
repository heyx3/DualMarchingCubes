using System;
using System.Linq;
using System.Collections.Generic;
using UnityEngine;

public class Controller : MonoBehaviour
{
    public float Dropoff = 100.0f;
    public float HorizontalScale = 0.01f,
                 VerticalScale = 0.05f;

    public float Threshold = 0.5f,
                 NoiseCurve = 0.3f,
                 DropoffCurve = 3.0f;
    public float SamplingSize = 1.0f;


    [Serializable]
    public class OctreeNode
    {
        public Bounds Bounds;

        [NonSerialized]
        public int Depth;

        /// <summary>
        /// Set to null if this node is a leaf.
        /// Children are ordered in this fashion:
        /// <para></para>
        ///    0)   -X   -Y   -Z
        /// <para></para>
        ///    1)   +X   -Y   -Z
        /// <para></para>
        ///    2)   +X   -Y   +Z
        /// <para></para>
        ///    3)   -X   -Y   +Z
        /// <para></para>
        ///    4)   -X   +Y   -Z
        /// <para></para>
        ///    5)   +X   +Y   -Z
        /// <para></para>
        ///    6)   +X   +Y   +Z
        /// <para></para>
        ///    7)   -X   +Y   +Z
        /// </summary>
        [NonSerialized]
        public OctreeNode[] Children;

        public OctreeNode(Bounds bounds, int depth, OctreeNode[] children = null)
        {
            Bounds = bounds;
            Depth = depth;
            Children = children;
        }

        /// <summary>
        /// Calculates the number of samples that can be taken within this node,
        ///     given the physical spacing between each sample.
        /// Should always return at least two samples along each axis
        ///     (one for each corner of the region).
        /// </summary>
        public Vector3Int CountSamples(float sampleInterval)
        {
            return new Vector3Int(
                1 + Mathf.CeilToInt(Bounds.size.x / sampleInterval),
                1 + Mathf.CeilToInt(Bounds.size.y / sampleInterval),
                1 + Mathf.CeilToInt(Bounds.size.z / sampleInterval)
            );
        }


        public IEnumerable<OctreeNode> Leaves
        {
            get
            {
                if (Children == null)
                    yield return (this);
                else foreach (var leaf in Children.SelectMany(c => c.Leaves))
                    yield return leaf;
            }
        }
    }

    /// <summary>
    /// A vertex straddling two-to-eight connected octree nodes.
    /// </summary>
    public struct DualCell
    {
        /// <summary>
        /// Ordered the same way as the children of an OctreeNode.
        /// Some of the values are likely to be duplicates
        ///     (wherever different-sized nodes meet, the larger one will appear more than once).
        /// </summary>
        public OctreeNode n0, n1, n2, n3, n4, n5, n6, n7;

        public Vector3 Vertex { get { return n0.Bounds.max; } }
    }


    public OctreeNode TreeRoot = new OctreeNode(
        new Bounds(new Vector3(0, 5, 0),
                   new Vector3(25, 10, 25)),
        0
    );
    public List<DualCell> DualGrid = new List<DualCell>();

    [SerializeField]
    private Vector3 samplePoint = Vector3.zero;
    [SerializeField]
    private float sampleRadiusScale = 1.5f;

    [SerializeField]
    private bool resetOctree = false;
    [SerializeField]
    private int octreeItersPerFrame = 1,
                dualCellItersPerFrame = 1;
    [SerializeField]
    private int octreeDepthDisplay = -1;

    [SerializeField]
    private string INFO_rootSamples = "";


    private System.Collections.IEnumerator Start()
    {
        while (true)
        {
            resetOctree = false;
            TreeRoot.Children = null;
            DualGrid.Clear();

            //Generate the octree.
            bool needsMoreExpansion = true;
            int iterCount = 1;
            while (needsMoreExpansion && !resetOctree)
            {
                needsMoreExpansion = false;
                foreach (var leaf in GetLeaves().ToList())
                {
                    if (iterCount % octreeItersPerFrame == 0)
                        yield return new WaitForSeconds(0.4f / Mathf.Pow((leaf.Depth + 1), 2.0f));
                    iterCount += 1;

                    if (TrySplitOctree(leaf))
                        needsMoreExpansion = true;
                    if (resetOctree)
                        break;
                }
            }

            //Generate the dual grid.
            iterCount = 1;
            foreach (var cell in TraverseNode(TreeRoot))
            {
                DualGrid.Add(cell);

                if (iterCount % dualCellItersPerFrame == 0)
                    yield return new WaitForSeconds(0.05f);
                iterCount += 1;
            }

            //Wait for a reset flag.
            while (!resetOctree)
                yield return null;
        }
    }

    private void OnDrawGizmos()
    {
        INFO_rootSamples = TreeRoot.CountSamples(SamplingSize).ToString();

        Gizmos.color = Color.black;
        Gizmos.DrawSphere(samplePoint,
                          Mathf.Min(0.1f, sampleRadiusScale * SampleField(samplePoint)));

        Gizmos.color = Color.white;
        Gizmos.DrawWireCube(TreeRoot.Bounds.center, TreeRoot.Bounds.size);

        foreach (var node in GetLeaves())
            if (octreeDepthDisplay > 0 && octreeDepthDisplay == node.Depth)
                GizmoOctree(node);
        if (octreeDepthDisplay < 0)
            foreach (var cell in DualGrid)
                GizmoDualCells(cell);
    }
    private void Update()
    {
        
    }

    private IEnumerable<OctreeNode> GetLeaves() { return TreeRoot.Leaves; }

    private bool TrySplitOctree(OctreeNode node)
    {
        //Sample inside the node to see if there are any surfaces.
        bool isBelow = false,
             isAbove = false;
        float samplingSize = SamplingSize * Mathf.Log(node.Depth + 2);
        Vector3Int nSamples = node.CountSamples(samplingSize);

        //If the node is too small, don't try splitting.
        if (nSamples.x < 3 || nSamples.y < 3 || nSamples.z < 3)
            return false;

        //Ignore the samples on the boundaries.
        for (int z = 1; z < nSamples.z - 1; ++z)
        {
            float zF = Mathf.LerpUnclamped(node.Bounds.min.z,
                                           node.Bounds.max.z,
                                           z / (float)nSamples.z);

            for (int y = 1; y < nSamples.y - 1; ++y)
            {
                float yF = Mathf.LerpUnclamped(node.Bounds.min.y, node.Bounds.max.y,
                                               y / (float)nSamples.y);

                for (int x = 1; x < nSamples.x - 1; ++x)
                {
                    float xF = Mathf.LerpUnclamped(node.Bounds.min.x, node.Bounds.max.x,
                                                   x / (float)nSamples.x);

                    bool aboveThreshold = SampleField(new Vector3(xF, yF, zF)) > Threshold;
                    isBelow |= !aboveThreshold;
                    isAbove |= aboveThreshold;

                    if (isBelow & isAbove)
                        break;
                }

                if (isBelow & isAbove)
                    break;
            }
        }

        //If there's something interesting going on in the node, break it up.
        if (isBelow & isAbove)
        {
            Vector3 newSize = node.Bounds.size * 0.5f,
                    halfNewSize = newSize * 0.5f,
                    min = node.Bounds.min + halfNewSize,
                    max = node.Bounds.max - halfNewSize;
            node.Children = new[]
            {
                new OctreeNode(new Bounds(new Vector3(min.x, min.y, min.z), newSize),
                               node.Depth + 1),
                new OctreeNode(new Bounds(new Vector3(max.x, min.y, min.z), newSize),
                               node.Depth + 1),
                new OctreeNode(new Bounds(new Vector3(min.x, max.y, min.z), newSize),
                               node.Depth + 1),
                new OctreeNode(new Bounds(new Vector3(max.x, max.y, min.z), newSize),
                               node.Depth + 1),

                new OctreeNode(new Bounds(new Vector3(min.x, min.y, max.z), newSize),
                               node.Depth + 1),
                new OctreeNode(new Bounds(new Vector3(max.x, min.y, max.z), newSize),
                               node.Depth + 1),
                new OctreeNode(new Bounds(new Vector3(min.x, max.y, max.z), newSize),
                               node.Depth + 1),
                new OctreeNode(new Bounds(new Vector3(max.x, max.y, max.z), newSize),
                               node.Depth + 1)
            };

            return true;
        }
        else
        {
            return false;
        }
    }

    private void GizmoOctree(OctreeNode node)
    {
        UnityEngine.Random.InitState(node.Depth);
        Gizmos.color = Color.HSVToRGB(
            Mathf.Repeat(node.Depth / 1.6180339887f, 1.0f),
            Mathf.LerpUnclamped(0.5f, 1.0f,
                                Mathf.Repeat(node.Depth / 0.3078954f, 1.0f)),
            Mathf.LerpUnclamped(0.75f, 1.0f,
                                Mathf.Repeat(node.Depth / 0.132078954f, 1.0f))
        );
        Gizmos.DrawWireCube(node.Bounds.center, node.Bounds.size);
    }
    private void GizmoDualCells(DualCell cell)
    {
        GizmoOctree(cell.n0);
        GizmoOctree(cell.n1);
        GizmoOctree(cell.n2);
        GizmoOctree(cell.n3);
        GizmoOctree(cell.n4);
        GizmoOctree(cell.n5);
        GizmoOctree(cell.n5);
        GizmoOctree(cell.n7);

        Gizmos.color = new Color(0.4f, 0.4f, 0.4f, 0.3f);
        Gizmos.DrawSphere(cell.Vertex, 0.2f);
    }

    private float SampleField(Vector3 pos)
    {
        pos = new Vector3(pos.x * HorizontalScale, pos.y * VerticalScale, pos.z * VerticalScale);

        float heightmapNoise = Mathf.Pow(Mathf.PerlinNoise(pos.x, pos.z),
                                         NoiseCurve);
        float verticalNoise = Mathf.Pow(Mathf.PerlinNoise(pos.y, 123.456f),
                                        NoiseCurve);

        float strength = Mathf.Pow(1 - Mathf.Clamp01(pos.y / Dropoff),
                                   DropoffCurve);

        return heightmapNoise * verticalNoise * strength;
    }



    /*
        Recursive algorithm that traverses the dual grid.
        Each vertex in the octree corresponds to a cell in the dual grid,
            so this algorithm ultimately iterates over every point in the octree,
            representing it as a set of 8 surrounding nodes.
        Note that most groups of 8 nodes will contain duplicates,
            due to large nodes sitting next to smaller ones.

        In this coordinate system, Y is up, Z is right, X is forward.
     */

    /// <summary>
    /// Traverses through every vertex connecting 2-8 octrees, within the given tree node.
    /// </summary>
    private IEnumerable<DualCell> TraverseNode(OctreeNode node)
    {
        //If this node is a leaf, there is nothing left to traverse.
        if (node.Children == null)
            yield break;

        //Traverse the child nodes.
        var iterations = node.Children.SelectMany(TraverseNode);
        //Traverse the faces in-between each pair of children.
        //   * Shared X faces:
        iterations = iterations.Concat(TraverseFace(node.Children[0], node.Children[1], 0))
                               .Concat(TraverseFace(node.Children[3], node.Children[2], 0))
                               .Concat(TraverseFace(node.Children[4], node.Children[5], 0))
                               .Concat(TraverseFace(node.Children[7], node.Children[6], 0));
        //   * Shared Y faces:
        iterations = iterations.Concat(TraverseFace(node.Children[0], node.Children[4], 1))
                               .Concat(TraverseFace(node.Children[1], node.Children[5], 1))
                               .Concat(TraverseFace(node.Children[3], node.Children[7], 1))
                               .Concat(TraverseFace(node.Children[2], node.Children[6], 1));
        //   * Shared Z faces:
        iterations = iterations.Concat(TraverseFace(node.Children[0], node.Children[3], 2))
                               .Concat(TraverseFace(node.Children[1], node.Children[2], 2))
                               .Concat(TraverseFace(node.Children[4], node.Children[7], 2))
                               .Concat(TraverseFace(node.Children[5], node.Children[6], 2));
        //Traverse the edges between each quartet of children.
        //    * Shared X edge:
        iterations = iterations.Concat(TraverseEdge(node.Children[0], node.Children[3],
                                                    node.Children[7], node.Children[4],
                                                    0))
                               .Concat(TraverseEdge(node.Children[1], node.Children[2],
                                                    node.Children[6], node.Children[5],
                                                    0));
        //    * Shared Y edge:
        iterations = iterations.Concat(TraverseEdge(node.Children[0], node.Children[1],
                                                    node.Children[2], node.Children[3],
                                                    1))
                               .Concat(TraverseEdge(node.Children[4], node.Children[5],
                                                    node.Children[6], node.Children[7],
                                                    1));
        //    * Shared Z edge:
        //      NOTE: I flipped the Y order from the reference article because it seems incorrect.
        iterations = iterations.Concat(TraverseEdge(node.Children[3], node.Children[2],
                                                    node.Children[6], node.Children[7],
                                                    2))
                               .Concat(TraverseEdge(node.Children[0], node.Children[1],
                                                    node.Children[5], node.Children[4],
                                                    2));
        //    * The vertex at the center of the child nodes:
        iterations = iterations.Concat(TraverseVertex(node.Children[0], node.Children[1],
                                                      node.Children[2], node.Children[3],
                                                      node.Children[4], node.Children[5],
                                                      node.Children[6], node.Children[7]));

        foreach (var result in iterations)
            yield return result;
    }
    /// <summary>
    /// Traverses through every cell of the dual grid which sits on the face between the two nodes.
    /// </summary>
    /// <param name="n0">The node on the "min" side of the face.</param>
    /// <param name="n1">The node on the "max" side of the face.</param>
    /// <param name="axis">The perpendicular axis (0=X, 1=Y, 2=Z) of the face.</param>
    private IEnumerable<DualCell> TraverseFace(OctreeNode n0, OctreeNode n1, int axis)
    {
        //If both nodes are leaves of the octree, then there's no vertices on this face.
        if (n0.Children == null && n1.Children == null)
            yield break;

        //Get the 4 child nodes touching each side of the face.
        //The children should be sorted so they line up across the face
        //    (e.x. cA0 touches cB0, cA3 touches cB3).
        OctreeNode cA0, cA1, cA2, cA3,
                   cB0, cB1, cB2, cB3;
        //Parent node 0:
        if (n0.Children == null)
            cA0 = cA1 = cA2 = cA3 = n0;
        else switch (axis)
        {
            case 0: //X
                cA0 = n0.Children[1];
                cA1 = n0.Children[2];
                cA2 = n0.Children[6];
                cA3 = n0.Children[5];
            break;
            case 1: //Y
                cA0 = n0.Children[4];
                cA1 = n0.Children[5];
                cA2 = n0.Children[6];
                cA3 = n0.Children[7];
            break;
            case 2: //Z
                cA0 = n0.Children[3];
                cA1 = n0.Children[2];
                cA2 = n0.Children[6];
                cA3 = n0.Children[7];
            break;
            default: throw new Exception($"Invalid axis: {axis}");
        }
        //Parent node 1:
        if (n1.Children == null)
            cB0 = cB1 = cB2 = cB3 = n1;
        else switch (axis)
        {
            case 0: //X
                cB0 = n1.Children[0];
                cB1 = n1.Children[4];
                cB2 = n1.Children[7];
                cB3 = n1.Children[3];
            break;
            case 1: //Y
                cB0 = n1.Children[0];
                cB1 = n1.Children[1];
                cB2 = n1.Children[2];
                cB3 = n1.Children[3];
            break;
            case 2: //Z
                cB0 = n1.Children[0];
                cB1 = n1.Children[1];
                cB2 = n1.Children[5];
                cB3 = n1.Children[4];
            break;
            default: throw new Exception($"Invalid axis: {axis}");
        }

        //Iterate over the vertices (dual cells) within these 8 children.

        //The children are directly touching each other, through smaller versions of this face:
        var iterations = TraverseFace(cA0, cB0, axis)
                 .Concat(TraverseFace(cA1, cB1, axis))
                 .Concat(TraverseFace(cA2, cB2, axis))
                 .Concat(TraverseFace(cA3, cB3, axis));

        //Within this face's child nodes there are also 4 edges and 1 vertex.
        switch (axis)
        {
            case 0: //X
                iterations = iterations
                      .Concat(TraverseEdge(cA0, cB0, cB3, cA3, 1))
                      .Concat(TraverseEdge(cA1, cB1, cB2, cA2, 1))
                      .Concat(TraverseEdge(cA0, cB0, cB1, cA1, 2))
                      .Concat(TraverseEdge(cA3, cB3, cB2, cA2, 2))
                      .Concat(TraverseVertex(cA0, cB0, cB3, cA3,
                                             cA1, cB1, cB2, cA2));
            break;
            case 1: //Y
                iterations = iterations
                      .Concat(TraverseEdge(cA0, cA3, cB3, cB0, 0))
                      .Concat(TraverseEdge(cA1, cA2, cB2, cB1, 0))
                      .Concat(TraverseEdge(cA0, cA1, cB1, cB0, 2))
                      .Concat(TraverseEdge(cA3, cA2, cB2, cB3, 2))
                      .Concat(TraverseVertex(cA0, cA1, cA2, cA3,
                                             cB0, cB1, cB2, cB3));
            break;
            case 2: //Z
                iterations = iterations
                      .Concat(TraverseEdge(cA0, cB0, cB3, cA3, 0))
                      .Concat(TraverseEdge(cA1, cB1, cB2, cA2, 0))
                      .Concat(TraverseEdge(cA0, cA1, cB1, cB0, 1))
                      .Concat(TraverseEdge(cA3, cA2, cB2, cB3, 1))
                      .Concat(TraverseVertex(cA0, cA1, cB1, cB0,
                                             cA3, cA2, cB2, cB3));
            break;
            default: throw new Exception($"Invalid axis: {axis}");
        }

        foreach (var result in iterations)
            yield return result;
    }
    /// <summary>
    /// Traverses through every cell of the dual grid which sits on
    ///     the edge shared by four nodes.
    /// The parameters are named based on their relative position.
    /// For example, n00 touches faces with n01 and n10 at the min corner,
    ///     while n11 is at the max corner also touching faces with n01 and n10.
    /// <para></para>
    /// The orientation of n01 vs n10 depends on which axis we're looking at.
    /// <para></para>
    /// For X edges, the order is (MinYZ, MinYMaxZ, MaxYZ, MaxYMinZ).
    /// <para></para>
    /// For Y edges, the order is (MinXZ, MaxXMinZ, MaxXZ, MinXMaxZ).
    /// <para></para>
    /// For Z edges, the order is (MinXY, MaxXMinY, MaxXY, MinXMaxY).
    /// </summary>
    /// <param name="axis">The axis (0=X, 1=Y, 2=Z) that the edge points along.</param>
    /// <returns></returns>
    private IEnumerable<DualCell> TraverseEdge(OctreeNode n00, OctreeNode n10,
                                               OctreeNode n11, OctreeNode n01,
                                               int axis)
    {
        //If all children are leaves, then there's no vertices on this edge.
        if (n00.Children == null && n01.Children == null &&
            n10.Children == null && n11.Children == null)
        {
            yield break;
        }

        //Each node has 2 children touching this edge.
        //Gather these 8 smaller nodes,
        //    using the parents as a stand-in for any child that doesn't exist.
        //Here the parents are referred to as A, B, C, and D (for n00, n10, n11, and n01, respectively).
        OctreeNode cA0, cA1,   cB0, cB1,   cC0, cC1,   cD0, cD1;
        //Their iteration order to match child-node ordering is different per-axis,
        //    and explicitly listed in the next section.

        //Parent node 00 (A):
        if (n00.Children == null)
            cA0 = cA1 = n00;
        else switch (axis)
        {
            case 0: //X
                cA0 = n00.Children[7];
                cA1 = n00.Children[6];
            break;
            case 1: //Y
                cA0 = n00.Children[2];
                cA1 = n00.Children[6];
            break;
            case 2: //Z
                cA0 = n00.Children[5];
                cA1 = n00.Children[6];
            break;
            default: throw new Exception($"Invalid axis: {axis}");
        }
        //Parent node 10 (B):
        if (n10.Children == null)
            cB0 = cB1 = n10;
        else switch (axis)
        {
            case 0: //X
                cB0 = n10.Children[4];
                cB1 = n10.Children[5];
            break;
            case 1: //Y
                cB0 = n10.Children[3];
                cB1 = n10.Children[7];
            break;
            case 2: //Z
                cB0 = n10.Children[4];
                cB1 = n10.Children[7];
            break;
            default: throw new Exception($"Invalid axis: {axis}");
        }
        //Parent node 11 (C):
        if (n11.Children == null)
            cC0 = cC1 = n11;
        else switch (axis)
        {
            case 0: //X
                cC0 = n11.Children[0];
                cC1 = n11.Children[1];
            break;
            case 1: //Y
                cC0 = n11.Children[0];
                cC1 = n11.Children[4];
            break;
            case 2: //Z
                cC0 = n11.Children[0];
                cC1 = n11.Children[3];
            break;
            default: throw new Exception($"Invalid axis: {axis}");
        }
        //Parent node 01 (D):
        if (n01.Children == null)
            cD0 = cD1 = n01;
        else switch (axis)
        {
            case 0: //X
                cD0 = n01.Children[3];
                cD1 = n01.Children[2];
            break;
            case 1: //Y
                cD0 = n01.Children[1];
                cD1 = n01.Children[5];
            break;
            case 2: //Z
                cD0 = n01.Children[1];
                cD1 = n01.Children[2];
            break;
            default: throw new Exception($"Invalid axis: {axis}");
        }

        //Child-like ordering of these inner nodes:
        //    (For X edges) [ cA0, cA1, cB1, cB0, cD0, cD1, cC1, cC0 ]
        //    (For Y edges) [ cA0, cB0, cC0, cD0, cA1, cB1, cC1, cD1 ]
        //    (For Z edges) [ cA0, cB0, cB1, cA1, cD0, cC0, cC1, cC1 ]

        //Traverse the two "child edges" making up this larger edge.
        //Fun fact: for all 3 caases, the child-edge traversals
        //    work out to the exact same ordering!
        var iterator = TraverseEdge(cA0, cB0, cC0, cD0, axis)
               .Concat(TraverseEdge(cA1, cB1, cC1, cD1, axis));

        //Traverse the vertex at the center of this edge.
        switch (axis)
        {
            case 0: iterator = iterator.Concat(
                TraverseVertex(cA0, cA1, cB1, cB0,
                               cD0, cD1, cC1, cC0)
            ); break;
            case 1: iterator = iterator.Concat(
                TraverseVertex(cA0, cB0, cC0, cD0,
                               cA1, cB1, cC1, cD1)
            ); break;
            case 2: iterator = iterator.Concat(
                TraverseVertex(cA0, cB0, cB1, cA1,
                               cD0, cC0, cC1, cD1)
            ); break;
            default: throw new NotImplementedException(axis.ToString());
        }

        foreach (var result in iterator)
            yield return result;
    }
    /// <summary>
    /// Traverses through every cell off the dual grid which touches
    ///     the vertex shared by eight nodes.
    /// The nodes must be ordered in the same way as a group of 8 children.
    /// </summary>
    private IEnumerable<DualCell> TraverseVertex(OctreeNode n0, OctreeNode n1, OctreeNode n2, OctreeNode n3,
                                                 OctreeNode n4, OctreeNode n5, OctreeNode n6, OctreeNode n7)
    {
        if (n0.Children != null || n1.Children != null ||
            n2.Children != null || n3.Children != null ||
            n4.Children != null || n5.Children != null ||
            n6.Children != null || n7.Children != null)
        {
            n0 = (n0.Children == null) ? n0 : n0.Children[6];
            n1 = (n1.Children == null) ? n1 : n1.Children[7];
            n2 = (n2.Children == null) ? n2 : n2.Children[4];
            n3 = (n3.Children == null) ? n3 : n3.Children[5];
            n4 = (n4.Children == null) ? n4 : n4.Children[2];
            n5 = (n5.Children == null) ? n5 : n5.Children[3];
            n6 = (n6.Children == null) ? n6 : n6.Children[0];
            n7 = (n7.Children == null) ? n7 : n7.Children[1];
            foreach (var result in TraverseVertex(n0, n1, n2, n3, n4, n5, n6, n7))
                yield return result;
        }
        else
        {
            var dc = new DualCell();
            dc.n0 = n0;
            dc.n1 = n1;
            dc.n2 = n2;
            dc.n3 = n3;
            dc.n4 = n4;
            dc.n5 = n5;
            dc.n6 = n6;
            dc.n7 = n7;
            yield return dc;
        }
    }
}
