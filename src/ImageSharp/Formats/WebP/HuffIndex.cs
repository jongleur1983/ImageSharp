// Copyright (c) Six Labors and contributors.
// Licensed under the Apache License, Version 2.0.

namespace SixLabors.ImageSharp.Formats.WebP
{
    /// <summary>
    /// Five Huffman codes are used at each meta code.
    /// </summary>
    public static class HuffIndex
    {
        /// <summary>
        /// Green + length prefix codes + color cache codes.
        /// </summary>
        public const int Green = 0;

        /// <summary>
        /// Red.
        /// </summary>
        public const int Red = 1;

        /// <summary>
        /// Blue.
        /// </summary>
        public const int Blue = 2;

        /// <summary>
        /// Alpha.
        /// </summary>
        public const int Alpha = 3;

        /// <summary>
        /// Distance prefix codes.
        /// </summary>
        public const int Dist = 4;
    }
}
