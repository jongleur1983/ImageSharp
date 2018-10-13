﻿// Copyright (c) Six Labors and contributors.
// Licensed under the Apache License, Version 2.0.

using System.Numerics;
using System.Runtime.CompilerServices;

namespace SixLabors.ImageSharp.PixelFormats
{
    /// <summary>
    /// Packed pixel type containing a single 8 bit normalized gray values.
    /// <para>
    /// Ranges from [0, 0, 0, 1] to [1, 1, 1, 1] in vector form.
    /// </para>
    /// </summary>
    public struct Gray8 : IPixel<Gray8>, IPackedVector<byte>
    {
        /// <summary>
        /// The maximum byte value.
        /// </summary>
        private static readonly Vector4 MaxBytes = new Vector4(255F);

        /// <summary>
        /// The half vector value.
        /// </summary>
        private static readonly Vector4 Half = new Vector4(0.5F);

        /// <summary>
        /// Initializes a new instance of the <see cref="Gray8"/> struct.
        /// </summary>
        /// <param name="luminance">The luminance component.</param>
        public Gray8(byte luminance) => this.PackedValue = luminance;

        /// <inheritdoc />
        public byte PackedValue { get; set; }

        /// <summary>
        /// Compares two <see cref="Gray8"/> objects for equality.
        /// </summary>
        /// <param name="left">The <see cref="Gray8"/> on the left side of the operand.</param>
        /// <param name="right">The <see cref="Gray8"/> on the right side of the operand.</param>
        /// <returns>
        /// True if the <paramref name="left"/> parameter is equal to the <paramref name="right"/> parameter; otherwise, false.
        /// </returns>
        [MethodImpl(InliningOptions.ShortMethod)]
        public static bool operator ==(Gray8 left, Gray8 right) => left.Equals(right);

        /// <summary>
        /// Compares two <see cref="Gray8"/> objects for equality.
        /// </summary>
        /// <param name="left">The <see cref="Gray8"/> on the left side of the operand.</param>
        /// <param name="right">The <see cref="Gray8"/> on the right side of the operand.</param>
        /// <returns>
        /// True if the <paramref name="left"/> parameter is not equal to the <paramref name="right"/> parameter; otherwise, false.
        /// </returns>
        [MethodImpl(InliningOptions.ShortMethod)]
        public static bool operator !=(Gray8 left, Gray8 right) => !left.Equals(right);

        /// <inheritdoc />
        public PixelOperations<Gray8> CreatePixelOperations() => new PixelOperations<Gray8>();

        /// <inheritdoc/>
        [MethodImpl(InliningOptions.ShortMethod)]
        public void PackFromScaledVector4(Vector4 vector) => this.PackFromVector4(vector);

        /// <inheritdoc/>
        [MethodImpl(InliningOptions.ShortMethod)]
        public Vector4 ToScaledVector4() => this.ToVector4();

        /// <inheritdoc />
        [MethodImpl(InliningOptions.ShortMethod)]
        public void PackFromVector4(Vector4 vector)
        {
            vector *= MaxBytes;
            vector += Half;
            vector = Vector4.Clamp(vector, Vector4.Zero, MaxBytes);

            this.PackedValue = ImageMaths.Get8BitBT709Luminance((byte)vector.X, (byte)vector.Y, (byte)vector.Z);
        }

        /// <inheritdoc />
        [MethodImpl(InliningOptions.ShortMethod)]
        public Vector4 ToVector4()
        {
            float rgb = this.PackedValue / 255F;
            return new Vector4(rgb, rgb, rgb, 1F);
        }

        /// <inheritdoc/>
        [MethodImpl(InliningOptions.ShortMethod)]
        public void PackFromArgb32(Argb32 source) => this.PackedValue = ImageMaths.Get8BitBT709Luminance(source.R, source.G, source.B);

        /// <inheritdoc/>
        [MethodImpl(InliningOptions.ShortMethod)]
        public void PackFromBgra32(Bgra32 source) => this.PackedValue = ImageMaths.Get8BitBT709Luminance(source.R, source.G, source.B);

        /// <inheritdoc/>
        [MethodImpl(InliningOptions.ShortMethod)]
        public void PackFromGray8(Gray8 source) => this.PackedValue = source.PackedValue;

        /// <inheritdoc/>
        [MethodImpl(InliningOptions.ShortMethod)]
        public void PackFromGray16(Gray16 source) => this.PackedValue = ImageMaths.DownScaleFrom16BitTo8Bit(source.PackedValue);

        /// <inheritdoc />
        [MethodImpl(InliningOptions.ShortMethod)]
        public void PackFromRgba32(Rgba32 source) => this.PackedValue = ImageMaths.Get8BitBT709Luminance(source.R, source.G, source.B);

        /// <inheritdoc />
        [MethodImpl(InliningOptions.ShortMethod)]
        public Rgba32 ToRgba32() => new Rgba32(this.PackedValue, this.PackedValue, this.PackedValue, byte.MaxValue);

        /// <inheritdoc/>
        [MethodImpl(InliningOptions.ShortMethod)]
        public void PackFromRgb48(Rgb48 source)
            => this.PackedValue = ImageMaths.Get8BitBT709Luminance(
                ImageMaths.DownScaleFrom16BitTo8Bit(source.R),
                ImageMaths.DownScaleFrom16BitTo8Bit(source.G),
                ImageMaths.DownScaleFrom16BitTo8Bit(source.B));

        /// <inheritdoc/>
        [MethodImpl(InliningOptions.ShortMethod)]
        public void PackFromRgba64(Rgba64 source)
            => this.PackedValue = ImageMaths.Get8BitBT709Luminance(
                ImageMaths.DownScaleFrom16BitTo8Bit(source.R),
                ImageMaths.DownScaleFrom16BitTo8Bit(source.G),
                ImageMaths.DownScaleFrom16BitTo8Bit(source.B));

        /// <inheritdoc />
        public override bool Equals(object obj) => obj is Gray8 other && this.Equals(other);

        /// <inheritdoc />
        [MethodImpl(InliningOptions.ShortMethod)]
        public bool Equals(Gray8 other) => this.PackedValue.Equals(other.PackedValue);

        /// <inheritdoc />
        public override string ToString() => $"Gray8({this.PackedValue}";

        /// <inheritdoc />
        [MethodImpl(InliningOptions.ShortMethod)]
        public override int GetHashCode() => this.PackedValue.GetHashCode();
    }
}