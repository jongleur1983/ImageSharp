// Copyright (c) Six Labors and contributors.
// Licensed under the Apache License, Version 2.0.

using System;
using System.Collections.Generic;
using System.Numerics;

using SixLabors.ImageSharp.PixelFormats;
using SixLabors.Primitives;
using SixLabors.Shapes;

namespace SixLabors.ImageSharp.Processing
{
    /// <summary>
    /// Provides an implementation of a brush for painting gradients between multiple color positions in 2D coordinates.
    /// It works similarly with the class in System.Drawing.Drawing2D of the same name.
    /// </summary>
    public sealed class PathGradientBrush : IBrush
    {
        private readonly Edge[] edges;

        private readonly Color centerColor;

        /// <summary>
        /// Initializes a new instance of the <see cref="PathGradientBrush"/> class.
        /// </summary>
        /// <param name="points">Points that constitute a polygon that represents the gradient area.</param>
        /// <param name="colors">Array of colors that correspond to each point in the polygon.</param>
        /// <param name="centerColor">Color at the center of the gradient area to which the other colors converge.</param>
        public PathGradientBrush(PointF[] points, Color[] colors, Color centerColor)
        {
            if (points == null)
            {
                throw new ArgumentNullException(nameof(points));
            }

            if (points.Length < 3)
            {
                throw new ArgumentOutOfRangeException(
                    nameof(points),
                    "There must be at least 3 lines to construct a path gradient brush.");
            }

            if (colors == null)
            {
                throw new ArgumentNullException(nameof(colors));
            }

            if (colors.Length == 0)
            {
                throw new ArgumentOutOfRangeException(
                    nameof(colors),
                    "One or more color is needed to construct a path gradient brush.");
            }

            int size = points.Length;

            var lines = new ILineSegment[size];

            for (int i = 0; i < size; i++)
            {
                lines[i] = new LinearLineSegment(points[i % size], points[(i + 1) % size]);
            }

            this.centerColor = centerColor;

            Color ColorAt(int index) => colors[index % colors.Length];

            var theEdges = new Edge[lines.Length];
            for (int i = 0; i < lines.Length; i++)
            {
                theEdges[i] = new Edge(new Path(lines[i]), ColorAt(i), ColorAt(i + 1));
            }

            this.edges = theEdges;
        }

        /// <summary>
        /// Initializes a new instance of the <see cref="PathGradientBrush"/> class.
        /// </summary>
        /// <param name="points">Points that constitute a polygon that represents the gradient area.</param>
        /// <param name="colors">Array of colors that correspond to each point in the polygon.</param>
        public PathGradientBrush(PointF[] points, Color[] colors)
            : this(points, colors, CalculateCenterColor(colors))
        {
        }

        /// <inheritdoc />
        public BrushApplicator<TPixel> CreateApplicator<TPixel>(
            ImageFrame<TPixel> source,
            RectangleF region,
            GraphicsOptions options)
            where TPixel : struct, IPixel<TPixel>
        {
            return new PathGradientBrushApplicator<TPixel>(source, this.edges, this.centerColor, options);
        }

        private static Color CalculateCenterColor(Color[] colors)
        {
            if (colors == null)
            {
                throw new ArgumentNullException(nameof(colors));
            }

            if (colors.Length == 0)
            {
                throw new ArgumentOutOfRangeException(
                    nameof(colors),
                    "One or more color is needed to construct a path gradient brush.");
            }

            Vector4 sum = Vector4.Zero;
            for (int i = 0; i < colors.Length; i++)
            {
                sum += colors[i].ToVector4();
            }

            return new Color(sum / colors.Length);
        }

        private static float DistanceBetween(PointF p1, PointF p2) => ((Vector2)(p2 - p1)).Length();

        private struct Intersection
        {
            public Intersection(PointF point, float distance)
            {
                this.Point = point;
                this.Distance = distance;
            }

            public PointF Point { get; }

            public float Distance { get; }
        }

        /// <summary>
        /// An edge of the polygon that represents the gradient area.
        /// </summary>
        private class Edge
        {
            private readonly Path path;

            private readonly float length;

            private readonly PointF[] buffer;

            public Edge(Path path, Color startColor, Color endColor)
            {
                this.path = path;

                IReadOnlyList<ILineSegment> segments = path.LineSegments;
                int segmentCount = segments.Count;

                Vector2? mayFirstPoint = null;
                Vector2 lastPoint = default;

                for (int i = 0; i < segmentCount; i++)
                {
                    foreach (PointF p in segments[i].Flatten())
                    {
                        lastPoint = p;
                        if (mayFirstPoint is null)
                        {
                            mayFirstPoint = p;
                        }
                    }
                }

                if (mayFirstPoint is Vector2 firstPoint)
                {
                    this.Start = firstPoint;
                    this.StartColor = startColor.ToVector4();

                    this.End = lastPoint;
                    this.EndColor = endColor.ToVector4();

                    this.length = DistanceBetween(lastPoint, firstPoint);
                    this.buffer = new PointF[this.path.MaxIntersections];
                }

                ThrowNoPointsFound();

                void ThrowNoPointsFound() => throw new ArgumentException("no points found in path", nameof(path));
            }

            public PointF Start { get; }

            public Vector4 StartColor { get; }

            public PointF End { get; }

            public Vector4 EndColor { get; }

            public Intersection? FindIntersection(PointF start, PointF end)
            {
                int intersections = this.path.FindIntersections(start, end, this.buffer);

                if (intersections == 0)
                {
                    return null;
                }

                float minDistanceToPSquared = float.MaxValue;
                Intersection? minDistanceIntersection = null;

                for (int i = 0; i < intersections; i++)
                {
                    PointF p = this.buffer[i];
                    float distanceToPSquared = ((Vector2)(p - start)).LengthSquared();
                    if (distanceToPSquared < minDistanceToPSquared)
                    {
                        minDistanceIntersection = new Intersection(p, distanceToPSquared);
                        minDistanceToPSquared = distanceToPSquared;
                    }
                }

                return minDistanceIntersection;
            }

            public Vector4 ColorAt(float distance)
            {
                float ratio = this.length > 0 ? distance / this.length : 0;

                return Vector4.Lerp(this.StartColor, this.EndColor, ratio);
            }

            public Vector4 ColorAt(PointF point) => this.ColorAt(DistanceBetween(point, this.Start));
        }

        /// <summary>
        /// The path gradient brush applicator.
        /// </summary>
        private class PathGradientBrushApplicator<TPixel> : BrushApplicator<TPixel>
            where TPixel : struct, IPixel<TPixel>
        {
            private readonly PointF center;

            private readonly Vector4 centerColor;

            private readonly float maxDistance;

            private readonly Edge[] edges;

            /// <summary>
            /// Initializes a new instance of the <see cref="PathGradientBrushApplicator{TPixel}"/> class.
            /// </summary>
            /// <param name="source">The source image.</param>
            /// <param name="edges">Edges of the polygon.</param>
            /// <param name="centerColor">Color at the center of the gradient area to which the other colors converge.</param>
            /// <param name="options">The options.</param>
            public PathGradientBrushApplicator(
                ImageFrame<TPixel> source,
                Edge[] edges,
                Color centerColor,
                GraphicsOptions options)
                : base(source, options)
            {
                this.edges = edges;

                var points = new PointF[edges.Length];

                PointF sum = PointF.Empty;

                // we use the SQUARED distance to determine the maximum as it's faster to calculate.
                // Distance is Sqrt(a^2 + b^2), DistanceSquared is just a^2+b^2.
                // by using Squared values first, we don't need the SquareRoot each time, but only once at the end.
                float maxDistanceSquared = -1;

                for (int i = 0; i < edges.Length; i++)
                {
                    PointF p = edges[i].Start;
                    points[i] = p;
                    sum += p;

                    float distanceSquared = ((Vector2)(p - this.center)).LengthSquared();
                    maxDistanceSquared = MathF.Max(distanceSquared, maxDistanceSquared);
                }

                this.center = sum / edges.Length;
                this.centerColor = centerColor.ToVector4();

                this.maxDistance = MathF.Sqrt(maxDistanceSquared);
            }

            /// <inheritdoc />
            internal override TPixel this[int x, int y]
            {
                get
                {
                    var point = new PointF(x, y);

                    if (point == this.center)
                    {
                        return new Color(this.centerColor).ToPixel<TPixel>();
                    }

                    Vector2 direction = Vector2.Normalize(point - this.center);

                    PointF end = point + (PointF)(direction * this.maxDistance);

                    (Edge edge, Intersection? info) = this.FindIntersection(point, end);

                    if (!info.HasValue)
                    {
                        return Color.Transparent.ToPixel<TPixel>();
                    }

                    PointF intersection = info.Value.Point;

                    Vector4 edgeColor = edge.ColorAt(intersection);

                    float length = DistanceBetween(intersection, this.center);
                    float ratio = length > 0 ? DistanceBetween(intersection, point) / length : 0;

                    Vector4 color = Vector4.Lerp(edgeColor, this.centerColor, ratio);

                    return new Color(color).ToPixel<TPixel>();
                }
            }

            private (Edge edge, Intersection? info) FindIntersection(PointF start, PointF end)
            {
                (Edge edge, Intersection? info) closest = default;

                foreach (Edge edge in this.edges)
                {
                    Intersection? intersection = edge.FindIntersection(start, end);

                    if (!intersection.HasValue)
                    {
                        continue;
                    }

                    if (closest.info == null || closest.info.Value.Distance > intersection.Value.Distance)
                    {
                        closest = (edge, intersection);
                    }
                }

                return closest;
            }

            /// <inheritdoc />
            public override void Dispose()
            {
            }
        }
    }
}
