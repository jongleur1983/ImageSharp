﻿using System;
using System.Numerics;

using SixLabors.ImageSharp.Advanced;
using SixLabors.ImageSharp.Memory;
using SixLabors.ImageSharp.PixelFormats;
using SixLabors.ImageSharp.PixelFormats.PixelBlenders;
using SixLabors.Primitives;

namespace SixLabors.ImageSharp.Processing.Drawing.Brushes
{
    /// <summary>
    /// Provides an implementation of a brush for painting gradients within areas.
    /// Supported right now:
    /// - a set of colors in relative distances to each other.
    /// - two points to gradient along.
    /// </summary>
    /// <typeparam name="TPixel">The pixel format</typeparam>
    public class LinearGradientBrush<TPixel> : IBrush<TPixel>
        where TPixel : struct, IPixel<TPixel>
    {
        private readonly Point p1;

        private readonly Point p2;

        private readonly Tuple<float, TPixel>[] keyColors;

        /// <summary>
        /// Initializes a new instance of the <see cref="LinearGradientBrush{TPixel}"/> class.
        /// </summary>
        /// <param name="p1">Start point</param>
        /// <param name="p2">End point</param>
        /// <param name="keyColors">a set of color keys and where they are. The double must be in range [0..1] and is relative between p1 and p2.</param>
        public LinearGradientBrush(Point p1, Point p2, params Tuple<float, TPixel>[] keyColors)
        {
            this.p1 = p1;
            this.p2 = p2;
            this.keyColors = keyColors;
        }

        /// <inheritdoc />
        public BrushApplicator<TPixel> CreateApplicator(ImageFrame<TPixel> source, RectangleF region, GraphicsOptions options)
            => new LinearGradientBrushApplicator(source, this.p1, this.p2, this.keyColors, region, options);

        /// <summary>
        /// The linear gradient brush applicator.
        /// </summary>
        private class LinearGradientBrushApplicator : BrushApplicator<TPixel>
        {
            private readonly Point start;

            private readonly Point end;

            private readonly Tuple<float, TPixel>[] colorStops;

            /// <summary>
            /// the vector along the gradient, x component
            /// </summary>
            private readonly float alongX;

            /// <summary>
            /// the vector along the gradient, y component
            /// </summary>
            private readonly float alongY;

            /// <summary>
            /// the vector perpendicular to the gradient, y component
            /// </summary>
            private readonly float acrossY;

            /// <summary>
            /// the vector perpendicular to the gradient, x component
            /// </summary>
            private readonly float acrossX;

            /// <summary>
            /// helper to speed up calculation as these dont't change
            /// </summary>
            private readonly float aYcX;

            /// <summary>
            /// helper to speed up calculation as these dont't change
            /// </summary>
            private readonly float aXcY;

            /// <summary>
            /// helper to speed up calculation as these dont't change
            /// </summary>
            private readonly float aXcX;

            /// <summary>
            /// Initializes a new instance of the <see cref="LinearGradientBrushApplicator" /> class.
            /// </summary>
            /// <param name="source">The source</param>
            /// <param name="start">start point of the gradient</param>
            /// <param name="end">end point of the gradient</param>
            /// <param name="colorStops">tuple list of colors and their respective position between 0 and 1 on the line</param>
            /// <param name="region">the region, copied from SolidColorBrush, not sure if necessary! TODO</param>
            /// <param name="options">the graphics options</param>
            public LinearGradientBrushApplicator(
                ImageFrame<TPixel> source,
                Point start,
                Point end,
                Tuple<float, TPixel>[] colorStops,
                RectangleF region, // TODO: use region, compare with other Brushes for reference.
                GraphicsOptions options)
                : base(source, options)
            {
                this.start = start;
                this.end = end;
                this.colorStops = colorStops; // TODO: requires colorStops to be sorted by Item1!

                // the along vector:
                this.alongX = this.start.X - this.end.X;
                this.alongY = this.start.Y - this.end.Y;

                // the cross vector:
                this.acrossX = this.alongY;
                this.acrossY = -this.alongX;

                // some helpers:
                this.aYcX = this.alongY * this.acrossX;
                this.aXcY = this.alongX * this.acrossY;
                this.aXcX = this.alongX * this.acrossX;
            }

            /// <summary>
            /// Gets the color for a single pixel
            /// </summary>
            /// <param name="x">The x.</param>
            /// <param name="y">The y.</param>
            internal override TPixel this[int x, int y]
            {
                get
                {
                    // the following formula is the result of the linear equation system that forms the vector.
                    // TODO: this formula should be abstracted as it's the only difference between linear and radial gradient!
                    float onCompleteGradient = this.RatioOnGradient(x, y);

                    var localGradientFrom = this.colorStops[0];
                    Tuple<float, TPixel> localGradientTo = null;

                    // TODO: ensure colorStops has at least 2 items (technically 1 would be okay, but that's no gradient)
                    foreach (var colorStop in this.colorStops)
                    {
                        localGradientTo = colorStop;
                        if (colorStop.Item1 >= onCompleteGradient)
                        {
                            // we're done here, so break it!
                            break;
                        }

                        localGradientFrom = localGradientTo;
                    }

                    TPixel resultColor = default;
                    if (localGradientFrom.Item2.Equals(localGradientTo.Item2))
                    {
                        resultColor = localGradientFrom.Item2;
                    }
                    else
                    {
                        var fromAsVector = localGradientFrom.Item2.ToVector4();
                        var toAsVector = localGradientTo.Item2.ToVector4();
                        float onLocalGradient = (onCompleteGradient - localGradientFrom.Item1) / localGradientTo.Item1; // TODO:

                        Vector4 result = PorterDuffFunctions.Normal(
                            fromAsVector,
                            toAsVector,
                            onLocalGradient);

                        // TODO: when resultColor is a struct, what does PackFromVector4 do here?
                        resultColor.PackFromVector4(result);
                    }

                    return resultColor;
                }
            }

            private float RatioOnGradient(int x, int y)
            {
                return ((x / this.acrossX) - (this.alongX * y / this.aYcX))
                       / (1 - (this.aXcY / this.aXcX));
            }

            internal override void Apply(Span<float> scanline, int x, int y)
            {
                base.Apply(scanline, x, y);

                // Span<TPixel> destinationRow = this.Target.GetPixelRowSpan(y).Slice(x, scanline.Length);
                // MemoryManager memoryManager = this.Target.MemoryManager;
                // using (IBuffer<float> amountBuffer = memoryManager.Allocate<float>(scanline.Length))
                // {
                //     Span<float> amountSpan = amountBuffer.Span;
                //
                //     for (int i = 0; i < scanline.Length; i++)
                //     {
                //         amountSpan[i] = scanline[i] * this.Options.BlendPercentage;
                //     }
                //
                //     this.Blender.Blend(memoryManager, destinationRow, destinationRow, this.Colors.Span, amountSpan);
                // }
            }

            /// <inheritdoc />
            public override void Dispose()
            {
            }
        }
    }
}