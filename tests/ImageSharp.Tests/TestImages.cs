// Copyright (c) Six Labors and contributors.
// Licensed under the Apache License, Version 2.0.

using System.Linq;

// ReSharper disable InconsistentNaming
// ReSharper disable MemberHidesStaticFromOuterClass
namespace SixLabors.ImageSharp.Tests
{
    /// <summary>
    /// Class that contains all the relative test image paths in the TestImages/Formats directory.
    /// Use with <see cref="WithFileAttribute"/>, <see cref="WithFileCollectionAttribute"/> or <see cref="FileTestBase"/>.
    /// </summary>
    public static class TestImages
    {
        public static class Png
        {
            public const string P1 = "Png/pl.png";
            public const string Pd = "Png/pd.png";
            public const string Blur = "Png/blur.png";
            public const string Indexed = "Png/indexed.png";
            public const string Splash = "Png/splash.png";
            public const string Cross = "Png/cross.png";
            public const string Powerpoint = "Png/pp.png";
            public const string SplashInterlaced = "Png/splash-interlaced.png";
            public const string Interlaced = "Png/interlaced.png";
            public const string Palette8Bpp = "Png/palette-8bpp.png";
            public const string Bpp1 = "Png/bpp1.png";
            public const string Gray4Bpp = "Png/gray_4bpp.png";
            public const string L16Bit = "Png/gray-16.png";
            public const string GrayA8Bit = "Png/gray-alpha-8.png";
            public const string GrayA8BitInterlaced = "Png/rollsroyce.png";
            public const string GrayAlpha1BitInterlaced = "Png/iftbbn0g01.png";
            public const string GrayAlpha2BitInterlaced = "Png/iftbbn0g02.png";
            public const string Gray4BitInterlaced = "Png/iftbbn0g04.png";
            public const string GrayAlpha16Bit = "Png/gray-alpha-16.png";
            public const string GrayTrns16BitInterlaced = "Png/gray-16-tRNS-interlaced.png";
            public const string Rgb24BppTrans = "Png/rgb-8-tRNS.png";
            public const string Rgb48Bpp = "Png/rgb-48bpp.png";
            public const string Rgb48BppInterlaced = "Png/rgb-48bpp-interlaced.png";
            public const string Rgb48BppTrans = "Png/rgb-16-tRNS.png";
            public const string Rgba64Bpp = "Png/rgb-16-alpha.png";
            public const string CalliphoraPartial = "Png/CalliphoraPartial.png";
            public const string CalliphoraPartialGrayscale = "Png/CalliphoraPartialGrayscale.png";
            public const string Bike = "Png/Bike.png";
            public const string BikeGrayscale = "Png/BikeGrayscale.png";
            public const string SnakeGame = "Png/SnakeGame.png";
            public const string Icon = "Png/icon.png";
            public const string Kaboom = "Png/kaboom.png";
            public const string PDSrc = "Png/pd-source.png";
            public const string PDDest = "Png/pd-dest.png";
            public const string Gray1BitTrans = "Png/gray-1-trns.png";
            public const string Gray2BitTrans = "Png/gray-2-tRNS.png";
            public const string Gray4BitTrans = "Png/gray-4-tRNS.png";
            public const string L8BitTrans = "Png/gray-8-tRNS.png";
            public const string LowColorVariance = "Png/low-variance.png";
            public const string PngWithMetadata = "Png/PngWithMetaData.png";
            public const string InvalidTextData = "Png/InvalidTextData.png";
            public const string David = "Png/david.png";

            // Filtered test images from http://www.schaik.com/pngsuite/pngsuite_fil_png.html
            public const string Filter0 = "Png/filter0.png";
            public const string Filter1 = "Png/filter1.png";
            public const string Filter2 = "Png/filter2.png";
            public const string Filter3 = "Png/filter3.png";
            public const string Filter4 = "Png/filter4.png";

            // Filter changing per scanline
            public const string FilterVar = "Png/filterVar.png";

            public const string VimImage1 = "Png/vim16x16_1.png";
            public const string VimImage2 = "Png/vim16x16_2.png";

            public const string VersioningImage1 = "Png/versioning-1_1.png";
            public const string VersioningImage2 = "Png/versioning-1_2.png";

            public const string Banner7Adam7InterlaceMode = "Png/banner7-adam.png";
            public const string Banner8Index = "Png/banner8-index.png";

            public const string Ratio1x4 = "Png/ratio-1x4.png";
            public const string Ratio4x1 = "Png/ratio-4x1.png";

            public const string Ducky = "Png/ducky.png";
            public const string Rainbow = "Png/rainbow.png";

            // Issue 1014: https://github.com/SixLabors/ImageSharp/issues/1014
            public const string Issue1014_1 = "Png/issues/Issue_1014_1.png";
            public const string Issue1014_2 = "Png/issues/Issue_1014_2.png";
            public const string Issue1014_3 = "Png/issues/Issue_1014_3.png";
            public const string Issue1014_4 = "Png/issues/Issue_1014_4.png";
            public const string Issue1014_5 = "Png/issues/Issue_1014_5.png";
            public const string Issue1014_6 = "Png/issues/Issue_1014_6.png";
            public const string Issue1127 = "Png/issues/Issue_1127.png";

            public static class Bad
            {
                // Odd chunk lengths
                public const string ChunkLength1 = "Png/chunklength1.png";
                public const string ChunkLength2 = "Png/chunklength2.png";
                public const string CorruptedChunk = "Png/big-corrupted-chunk.png";
                public const string ZlibOverflow = "Png/zlib-overflow.png";
                public const string ZlibOverflow2 = "Png/zlib-overflow2.png";
                public const string ZlibZtxtBadHeader = "Png/zlib-ztxt-bad-header.png";
                public const string Issue1047_BadEndChunk = "Png/issues/Issue_1047.png";
            }

            public static readonly string[] All =
            {
                P1, Pd, Blur, Splash, Cross,
                Powerpoint, SplashInterlaced, Interlaced,
                Filter0, Filter1, Filter2, Filter3, Filter4,
                FilterVar, VimImage1, VimImage2, VersioningImage1,
                VersioningImage2, Ratio4x1, Ratio1x4
            };
        }

        public static class Jpeg
        {
            public static class Progressive
            {
                public const string Fb = "Jpg/progressive/fb.jpg";
                public const string Progress = "Jpg/progressive/progress.jpg";
                public const string Festzug = "Jpg/progressive/Festzug.jpg";

                public static class Bad
                {
                    public const string BadEOF = "Jpg/progressive/BadEofProgressive.jpg";
                    public const string ExifUndefType = "Jpg/progressive/ExifUndefType.jpg";
                }

                public static readonly string[] All = { Fb, Progress, Festzug };
            }

            public static class Baseline
            {
                public static class Bad
                {
                    public const string BadEOF = "Jpg/baseline/badeof.jpg";
                    public const string BadRST = "Jpg/baseline/badrst.jpg";
                }

                public const string Cmyk = "Jpg/baseline/cmyk.jpg";
                public const string Exif = "Jpg/baseline/exif.jpg";
                public const string Floorplan = "Jpg/baseline/Floorplan.jpg";
                public const string Calliphora = "Jpg/baseline/Calliphora.jpg";
                public const string Ycck = "Jpg/baseline/ycck.jpg";
                public const string Turtle420 = "Jpg/baseline/turtle.jpg";
                public const string GammaDalaiLamaGray = "Jpg/baseline/gamma_dalai_lama_gray.jpg";
                public const string Hiyamugi = "Jpg/baseline/Hiyamugi.jpg";
                public const string Snake = "Jpg/baseline/Snake.jpg";
                public const string Lake = "Jpg/baseline/Lake.jpg";
                public const string Jpeg400 = "Jpg/baseline/jpeg400jfif.jpg";
                public const string Jpeg420Exif = "Jpg/baseline/jpeg420exif.jpg";
                public const string Jpeg444 = "Jpg/baseline/jpeg444.jpg";
                public const string Jpeg420Small = "Jpg/baseline/jpeg420small.jpg";
                public const string Testorig420 = "Jpg/baseline/testorig.jpg";
                public const string MultiScanBaselineCMYK = "Jpg/baseline/MultiScanBaselineCMYK.jpg";
                public const string Ratio1x1 = "Jpg/baseline/ratio-1x1.jpg";
                public const string LowContrast = "Jpg/baseline/AsianCarvingLowContrast.jpg";
                public const string Testorig12bit = "Jpg/baseline/testorig12.jpg";
                public const string YcckSubsample1222 = "Jpg/baseline/ycck-subsample-1222.jpg";

                public static readonly string[] All =
                {
                    Cmyk, Ycck, Exif, Floorplan,
                    Calliphora, Turtle420, GammaDalaiLamaGray,
                    Hiyamugi, Jpeg400, Jpeg420Exif, Jpeg444,
                    Ratio1x1, Testorig12bit, YcckSubsample1222
                };
            }

            public static class Issues
            {
                public const string CriticalEOF214 = "Jpg/issues/Issue214-CriticalEOF.jpg";
                public const string MissingFF00ProgressiveGirl159 = "Jpg/issues/Issue159-MissingFF00-Progressive-Girl.jpg";
                public const string MissingFF00ProgressiveBedroom159 = "Jpg/issues/Issue159-MissingFF00-Progressive-Bedroom.jpg";
                public const string BadCoeffsProgressive178 = "Jpg/issues/Issue178-BadCoeffsProgressive-Lemon.jpg";
                public const string BadZigZagProgressive385 = "Jpg/issues/Issue385-BadZigZag-Progressive.jpg";
                public const string MultiHuffmanBaseline394 = "Jpg/issues/Issue394-MultiHuffmanBaseline-Speakers.jpg";
                public const string NoEoiProgressive517 = "Jpg/issues/Issue517-No-EOI-Progressive.jpg";
                public const string BadRstProgressive518 = "Jpg/issues/Issue518-Bad-RST-Progressive.jpg";
                public const string InvalidCast520 = "Jpg/issues/Issue520-InvalidCast.jpg";
                public const string DhtHasWrongLength624 = "Jpg/issues/Issue624-DhtHasWrongLength-Progressive-N.jpg";
                public const string ExifDecodeOutOfRange694 = "Jpg/issues/Issue694-Decode-Exif-OutOfRange.jpg";
                public const string InvalidEOI695 = "Jpg/issues/Issue695-Invalid-EOI.jpg";
                public const string ExifResizeOutOfRange696 = "Jpg/issues/Issue696-Resize-Exif-OutOfRange.jpg";
                public const string InvalidAPP0721 = "Jpg/issues/Issue721-InvalidAPP0.jpg";
                public const string OrderedInterleavedProgressive723A = "Jpg/issues/Issue723-Ordered-Interleaved-Progressive-A.jpg";
                public const string OrderedInterleavedProgressive723B = "Jpg/issues/Issue723-Ordered-Interleaved-Progressive-B.jpg";
                public const string OrderedInterleavedProgressive723C = "Jpg/issues/Issue723-Ordered-Interleaved-Progressive-C.jpg";
                public const string ExifGetString750Transform = "Jpg/issues/issue750-exif-tranform.jpg";
                public const string ExifGetString750Load = "Jpg/issues/issue750-exif-load.jpg";
                public const string IncorrectQuality845 = "Jpg/issues/Issue845-Incorrect-Quality99.jpg";
                public const string IncorrectColorspace855 = "Jpg/issues/issue855-incorrect-colorspace.jpg";
                public const string IncorrectResize1006 = "Jpg/issues/issue1006-incorrect-resize.jpg";
                public const string ExifResize1049 = "Jpg/issues/issue1049-exif-resize.jpg";
                public const string BadSubSampling1076 = "Jpg/issues/issue-1076-invalid-subsampling.jpg";

                public static class Fuzz
                {
                    public const string NullReferenceException797 = "Jpg/issues/fuzz/Issue797-NullReferenceException.jpg";
                    public const string AccessViolationException798 = "Jpg/issues/fuzz/Issue798-AccessViolationException.jpg";
                    public const string DivideByZeroException821 = "Jpg/issues/fuzz/Issue821-DivideByZeroException.jpg";
                    public const string DivideByZeroException822 = "Jpg/issues/fuzz/Issue822-DivideByZeroException.jpg";
                    public const string NullReferenceException823 = "Jpg/issues/fuzz/Issue823-NullReferenceException.jpg";
                    public const string IndexOutOfRangeException824A = "Jpg/issues/fuzz/Issue824-IndexOutOfRangeException-A.jpg";
                    public const string IndexOutOfRangeException824B = "Jpg/issues/fuzz/Issue824-IndexOutOfRangeException-B.jpg";
                    public const string IndexOutOfRangeException824C = "Jpg/issues/fuzz/Issue824-IndexOutOfRangeException-C.jpg";
                    public const string IndexOutOfRangeException824D = "Jpg/issues/fuzz/Issue824-IndexOutOfRangeException-D.jpg";
                    public const string IndexOutOfRangeException824E = "Jpg/issues/fuzz/Issue824-IndexOutOfRangeException-E.jpg";
                    public const string IndexOutOfRangeException824F = "Jpg/issues/fuzz/Issue824-IndexOutOfRangeException-F.jpg";
                    public const string IndexOutOfRangeException824G = "Jpg/issues/fuzz/Issue824-IndexOutOfRangeException-G.jpg";
                    public const string IndexOutOfRangeException824H = "Jpg/issues/fuzz/Issue824-IndexOutOfRangeException-H.jpg";
                    public const string ArgumentOutOfRangeException825A = "Jpg/issues/fuzz/Issue825-ArgumentOutOfRangeException-A.jpg";
                    public const string ArgumentOutOfRangeException825B = "Jpg/issues/fuzz/Issue825-ArgumentOutOfRangeException-B.jpg";
                    public const string ArgumentOutOfRangeException825C = "Jpg/issues/fuzz/Issue825-ArgumentOutOfRangeException-C.jpg";
                    public const string ArgumentOutOfRangeException825D = "Jpg/issues/fuzz/Issue825-ArgumentOutOfRangeException-D.jpg";
                    public const string ArgumentException826A = "Jpg/issues/fuzz/Issue826-ArgumentException-A.jpg";
                    public const string ArgumentException826B = "Jpg/issues/fuzz/Issue826-ArgumentException-B.jpg";
                    public const string ArgumentException826C = "Jpg/issues/fuzz/Issue826-ArgumentException-C.jpg";
                    public const string AccessViolationException827 = "Jpg/issues/fuzz/Issue827-AccessViolationException.jpg";
                    public const string ExecutionEngineException839 = "Jpg/issues/fuzz/Issue839-ExecutionEngineException.jpg";
                    public const string AccessViolationException922 = "Jpg/issues/fuzz/Issue922-AccessViolationException.jpg";
                }
            }

            public static readonly string[] All = Baseline.All.Concat(Progressive.All).ToArray();

            public static class BenchmarkSuite
            {
                public const string Jpeg400_SmallMonochrome = Baseline.Jpeg400;
                public const string Jpeg420Exif_MidSizeYCbCr = Baseline.Jpeg420Exif;
                public const string Lake_Small444YCbCr = Baseline.Lake;

                // A few large images from the "issues" set are actually very useful for benchmarking:
                public const string MissingFF00ProgressiveBedroom159_MidSize420YCbCr = Issues.MissingFF00ProgressiveBedroom159;
                public const string BadRstProgressive518_Large444YCbCr = Issues.BadRstProgressive518;
                public const string ExifGetString750Transform_Huge420YCbCr = Issues.ExifGetString750Transform;
            }
        }

        public static class Bmp
        {
            // Note: The inverted images have been generated by altering the BitmapInfoHeader using a hex editor.
            // As such, the expected pixel output will be the reverse of the unaltered equivalent images.
            public const string Car = "Bmp/Car.bmp";
            public const string F = "Bmp/F.bmp";
            public const string NegHeight = "Bmp/neg_height.bmp";
            public const string CoreHeader = "Bmp/BitmapCoreHeaderQR.bmp";
            public const string V5Header = "Bmp/BITMAPV5HEADER.bmp";
            public const string RLE24 = "Bmp/rgb24rle24.bmp";
            public const string RLE24Cut = "Bmp/rle24rlecut.bmp";
            public const string RLE24Delta = "Bmp/rle24rlecut.bmp";
            public const string RLE8 = "Bmp/RunLengthEncoded.bmp";
            public const string RLE8Cut = "Bmp/pal8rlecut.bmp";
            public const string RLE8Delta = "Bmp/pal8rletrns.bmp";
            public const string Rle8Delta320240 = "Bmp/rle8-delta-320x240.bmp";
            public const string Rle8Blank160120 = "Bmp/rle8-blank-160x120.bmp";
            public const string RLE8Inverted = "Bmp/RunLengthEncoded-inverted.bmp";
            public const string RLE4 = "Bmp/pal4rle.bmp";
            public const string RLE4Cut = "Bmp/pal4rlecut.bmp";
            public const string RLE4Delta = "Bmp/pal4rletrns.bmp";
            public const string Rle4Delta320240 = "Bmp/rle4-delta-320x240.bmp";
            public const string Bit1 = "Bmp/pal1.bmp";
            public const string Bit1Pal1 = "Bmp/pal1p1.bmp";
            public const string Bit4 = "Bmp/pal4.bmp";
            public const string Bit8 = "Bmp/test8.bmp";
            public const string Bit8Gs = "Bmp/pal8gs.bmp";
            public const string Bit8Inverted = "Bmp/test8-inverted.bmp";
            public const string Bit16 = "Bmp/test16.bmp";
            public const string Bit16Inverted = "Bmp/test16-inverted.bmp";
            public const string Bit32Rgb = "Bmp/rgb32.bmp";
            public const string Bit32Rgba = "Bmp/rgba32.bmp";
            public const string Rgb16 = "Bmp/rgb16.bmp";

            // Note: This format can be called OS/2 BMPv1, or Windows BMPv2
            public const string WinBmpv2 = "Bmp/pal8os2v1_winv2.bmp";

            public const string WinBmpv3 = "Bmp/rgb24.bmp";
            public const string WinBmpv4 = "Bmp/pal8v4.bmp";
            public const string WinBmpv5 = "Bmp/pal8v5.bmp";
            public const string Bit8Palette4 = "Bmp/pal8-0.bmp";
            public const string Os2v2Short = "Bmp/pal8os2v2-16.bmp";
            public const string Os2v2 = "Bmp/pal8os2v2.bmp";
            public const string Os2BitmapArray = "Bmp/ba-bm.bmp";
            public const string Os2BitmapArray9s = "Bmp/9S.BMP";
            public const string Os2BitmapArrayDiamond = "Bmp/DIAMOND.BMP";
            public const string Os2BitmapArrayMarble = "Bmp/GMARBLE.BMP";
            public const string Os2BitmapArraySkater = "Bmp/SKATER.BMP";
            public const string Os2BitmapArraySpade = "Bmp/SPADE.BMP";
            public const string Os2BitmapArraySunflower = "Bmp/SUNFLOW.BMP";
            public const string Os2BitmapArrayWarpd = "Bmp/WARPD.BMP";
            public const string Os2BitmapArrayPines = "Bmp/PINES.BMP";
            public const string LessThanFullSizedPalette = "Bmp/pal8os2sp.bmp";
            public const string Pal8Offset = "Bmp/pal8offs.bmp";
            public const string OversizedPalette = "Bmp/pal8oversizepal.bmp";
            public const string Rgb24LargePalette = "Bmp/rgb24largepal.bmp";
            public const string InvalidPaletteSize = "Bmp/invalidPaletteSize.bmp";
            public const string Rgb24jpeg = "Bmp/rgb24jpeg.bmp";
            public const string Rgb24png = "Bmp/rgb24png.bmp";
            public const string Rgba32v4 = "Bmp/rgba32v4.bmp";

            // Bitmap images with compression type BITFIELDS.
            public const string Rgb32bfdef = "Bmp/rgb32bfdef.bmp";
            public const string Rgb32bf = "Bmp/rgb32bf.bmp";
            public const string Rgb16bfdef = "Bmp/rgb16bfdef.bmp";
            public const string Rgb16565 = "Bmp/rgb16-565.bmp";
            public const string Rgb16565pal = "Bmp/rgb16-565pal.bmp";
            public const string Issue735 = "Bmp/issue735.bmp";
            public const string Rgba32bf56AdobeV3 = "Bmp/rgba32h56.bmp";
            public const string Rgb32h52AdobeV3 = "Bmp/rgb32h52.bmp";
            public const string Rgba321010102 = "Bmp/rgba32-1010102.bmp";
            public const string RgbaAlphaBitfields = "Bmp/rgba32abf.bmp";

            public static readonly string[] BitFields =
            {
                  Rgb32bfdef,
                  Rgb32bf,
                  Rgb16565,
                  Rgb16bfdef,
                  Rgb16565pal,
                  Issue735,
            };

            public static readonly string[] Miscellaneous =
            {
                Car,
                F,
                NegHeight
            };

            public static readonly string[] Benchmark =
            {
                Car,
                F,
                NegHeight,
                CoreHeader,
                V5Header,
                RLE4,
                RLE8,
                RLE8Inverted,
                Bit1,
                Bit1Pal1,
                Bit4,
                Bit8,
                Bit8Inverted,
                Bit16,
                Bit16Inverted,
                Bit32Rgb
            };
        }

        public static class Gif
        {
            public const string Rings = "Gif/rings.gif";
            public const string Giphy = "Gif/giphy.gif";
            public const string Cheers = "Gif/cheers.gif";
            public const string Receipt = "Gif/receipt.gif";
            public const string Trans = "Gif/trans.gif";
            public const string Kumin = "Gif/kumin.gif";
            public const string Leo = "Gif/leo.gif";
            public const string Ratio4x1 = "Gif/base_4x1.gif";
            public const string Ratio1x4 = "Gif/base_1x4.gif";
            public const string LargeComment = "Gif/large_comment.gif";

            public static class Issues
            {
                public const string BadAppExtLength = "Gif/issues/issue405_badappextlength252.gif";
                public const string BadAppExtLength_2 = "Gif/issues/issue405_badappextlength252-2.gif";
                public const string BadDescriptorWidth = "Gif/issues/issue403_baddescriptorwidth.gif";
            }

            public static readonly string[] All = { Rings, Giphy, Cheers, Trans, Kumin, Leo, Ratio4x1, Ratio1x4 };
        }

        public static class Tga
        {
            public const string Bit15 = "Tga/rgb15.tga";
            public const string Bit15Rle = "Tga/rgb15rle.tga";
            public const string Bit16 = "Tga/targa_16bit.tga";
            public const string Bit16PalRle = "Tga/ccm8.tga";
            public const string Bit24 = "Tga/targa_24bit.tga";
            public const string Bit24TopLeft = "Tga/targa_24bit_pal_origin_topleft.tga";
            public const string Bit24RleTopLeft = "Tga/targa_24bit_rle_origin_topleft.tga";
            public const string Bit32 = "Tga/targa_32bit.tga";
            public const string Grey = "Tga/targa_8bit.tga";
            public const string GreyRle = "Tga/targa_8bit_rle.tga";
            public const string Bit16Rle = "Tga/targa_16bit_rle.tga";
            public const string Bit24Rle = "Tga/targa_24bit_rle.tga";
            public const string Bit32Rle = "Tga/targa_32bit_rle.tga";
            public const string Bit16Pal = "Tga/targa_16bit_pal.tga";
            public const string Bit24Pal = "Tga/targa_24bit_pal.tga";
        }

        public static class WebP
        {
            public static class Animated
            {
                public const string Animated1 = "WebP/animated-webp.webp";
                public const string Animated2 = "WebP/animated2.webp";
                public const string Animated3 = "WebP/animated3.webp";
                public const string Animated4 = "WebP/animated_lossy.webp";
            }

            public static class Lossless
            {
                public const string WithExif = "WebP/exif_lossless.webp";
                public const string WithIccp = "WebP/iccp_lossless.webp";
                public const string NoTransform1 = "WebP/lossless_vec_1_0.webp";
                public const string NoTransform2 = "WebP/lossless_vec_2_0.webp";
                public const string GreenTransform1 = "WebP/lossless1.webp";
                public const string GreenTransform2 = "WebP/lossless2.webp";
                public const string GreenTransform3 = "WebP/lossless3.webp";
                public const string GreenTransform4 = "WebP/lossless_vec_1_4.webp";
                public const string GreenTransform5 = "WebP/lossless_vec_2_4.webp";
                public const string CrossColorTransform1 = "WebP/lossless_vec_1_8.webp";
                public const string CrossColorTransform2 = "WebP/lossless_vec_2_8.webp";
                public const string PredictorTransform1 = "WebP/lossless_vec_1_2.webp";
                public const string PredictorTransform2 = "WebP/lossless_vec_2_2.webp";
                public const string ColorIndexTransform1 = "WebP/lossless4.webp";
                public const string ColorIndexTransform2 = "WebP/lossless_vec_1_1.webp";
                public const string ColorIndexTransform3 = "WebP/lossless_vec_1_5.webp";
                public const string ColorIndexTransform4 = "WebP/lossless_vec_2_1.webp";
                public const string ColorIndexTransform5 = "WebP/lossless_vec_2_5.webp";
                public const string TwoTransforms1 = "Webp/lossless_vec_1_10.webp"; // cross_color, predictor
                public const string TwoTransforms2 = "Webp/lossless_vec_1_12.webp"; // cross_color, substract_green
                public const string TwoTransforms3 = "Webp/lossless_vec_1_13.webp"; // color_indexing, cross_color
                public const string TwoTransforms4 = "Webp/lossless_vec_1_3.webp"; // color_indexing, predictor
                public const string TwoTransforms5 = "Webp/lossless_vec_1_6.webp"; // substract_green, predictor
                public const string TwoTransforms6 = "Webp/lossless_vec_1_7.webp"; // color_indexing, predictor
                public const string TwoTransforms7 = "Webp/lossless_vec_1_9.webp"; // color_indexing, cross_color
                public const string TwoTransforms8 = "Webp/lossless_vec_2_10.webp"; // predictor, cross_color
                public const string TwoTransforms9 = "Webp/lossless_vec_2_12.webp"; // substract_green, cross_color
                public const string TwoTransforms10 = "Webp/lossless_vec_2_13.webp"; // color_indexing, cross_color
                public const string TwoTransforms11 = "Webp/lossless_vec_2_3.webp"; // color_indexing, predictor
                public const string TwoTransforms12 = "Webp/lossless_vec_2_6.webp"; // substract_green, predictor
                public const string TwoTransforms13 = "Webp/lossless_vec_2_9.webp"; // color_indexing, predictor
                public const string ThreeTransforms1 = "Webp/color_cache_bits_11.webp"; // substract_green, predictor, cross_color
                public const string ThreeTransforms2 = "Webp/lossless_vec_1_11.webp"; // color_indexing, predictor, cross_color
                public const string ThreeTransforms3 = "Webp/lossless_vec_1_14.webp"; // substract_green, predictor, cross_color
                public const string ThreeTransforms4 = "Webp/lossless_vec_1_15.webp"; // color_indexing, predictor, cross_color
                public const string ThreeTransforms5 = "Webp/lossless_vec_2_11.webp"; // color_indexing, predictor, cross_color
                public const string ThreeTransforms6 = "Webp/lossless_vec_2_14.webp"; // substract_green, predictor, cross_color
                public const string ThreeTransforms7 = "Webp/lossless_vec_2_15.webp"; // color_indexing, predictor, cross_color
                public const string BikeThreeTransforms = "Webp/bike_lossless.webp"; // substract_green, predictor, cross_color

                // Invalid / corrupted images
                // Below images have errors according to webpinfo. The error message webpinfo gives is "Truncated data detected when parsing RIFF payload."
                public const string LossLessCorruptImage1 = "Webp/lossless_big_random_alpha.webp"; // substract_green, predictor, cross_color.
                public const string LossLessCorruptImage2 = "Webp/lossless_vec_2_7.webp"; // color_indexing, predictor.
                public const string LossLessCorruptImage3 = "Webp/lossless_color_transform.webp"; // cross_color, predictor
                public const string LossLessCorruptImage4 = "Webp/near_lossless_75.webp"; // predictor, cross_color.
            }

            public static class Lossy
            {
                public const string WithExif = "WebP/exif_lossy.webp";
                public const string WithIccp = "WebP/iccp_lossy.webp";

                // Lossy images without macroblock filtering.
                public const string Bike = "WebP/bike_lossy.webp";
                public const string NoFilter01 = "WebP/vp80-01-intra-1400.webp";
                public const string NoFilter02 = "WebP/vp80-00-comprehensive-010.webp";
                public const string NoFilter03 = "WebP/vp80-00-comprehensive-005.webp";
                public const string NoFilter04 = "WebP/vp80-01-intra-1417.webp";
                public const string NoFilter05 = "WebP/vp80-02-inter-1402.webp";
                public const string NoFilter06 = "WebP/test.webp";

                // Lossy images with a simple filter.
                public const string SimpleFilter01 = "WebP/segment01.webp";
                public const string SimpleFilter02 = "WebP/segment02.webp";
                public const string SimpleFilter03 = "WebP/vp80-00-comprehensive-003.webp";
                public const string SimpleFilter04 = "WebP/vp80-00-comprehensive-007.webp";
                public const string SimpleFilter05 = "WebP/test-nostrong.webp";

                // Lossy images with a complex filter.
                public const string IccpComplexFilter = "WebP/iccp_lossy.webp";
                public const string VeryShort = "WebP/very_short.webp";
                public const string BikeComplexFilter = "WebP/bike_lossy_complex_filter.webp";
                public const string ComplexFilter01 = "WebP/vp80-02-inter-1418.webp";
                public const string ComplexFilter02 = "WebP/vp80-02-inter-1418.webp";
                public const string ComplexFilter03 = "WebP/vp80-00-comprehensive-002.webp";
                public const string ComplexFilter04 = "WebP/vp80-00-comprehensive-006.webp";
                public const string ComplexFilter05 = "WebP/vp80-00-comprehensive-009.webp";
                public const string ComplexFilter06 = "WebP/vp80-00-comprehensive-012.webp";
                public const string ComplexFilter07 = "WebP/vp80-00-comprehensive-015.webp";
                public const string ComplexFilter08 = "WebP/vp80-00-comprehensive-016.webp";
                public const string ComplexFilter09 = "WebP/vp80-00-comprehensive-017.webp";

                // Lossy with partitions.
                public const string Partitions01 = "WebP/vp80-04-partitions-1404.webp";
                public const string Partitions02 = "WebP/vp80-04-partitions-1405.webp";
                public const string Partitions03 = "WebP/vp80-04-partitions-1406.webp";

                // Lossy with segmentation.
                public const string SegmentationNoFilter01 = "WebP/vp80-03-segmentation-1401.webp";
                public const string SegmentationNoFilter02 = "WebP/vp80-03-segmentation-1403.webp";
                public const string SegmentationNoFilter03 = "WebP/vp80-03-segmentation-1407.webp";
                public const string SegmentationNoFilter04 = "WebP/vp80-03-segmentation-1408.webp";
                public const string SegmentationNoFilter05 = "WebP/vp80-03-segmentation-1409.webp";
                public const string SegmentationNoFilter06 = "WebP/vp80-03-segmentation-1410.webp";
                public const string SegmentationComplexFilter01 = "WebP/vp80-03-segmentation-1413.webp";
                public const string SegmentationComplexFilter02 = "WebP/vp80-03-segmentation-1425.webp";
                public const string SegmentationComplexFilter03 = "WebP/vp80-03-segmentation-1426.webp";
                public const string SegmentationComplexFilter04 = "WebP/vp80-03-segmentation-1427.webp";
                public const string SegmentationComplexFilter05 = "WebP/vp80-03-segmentation-1432.webp";

                // Lossy with sharpness level.
                public const string Sharpness01 = "WebP/vp80-05-sharpness-1428.webp";
                public const string Sharpness02 = "WebP/vp80-05-sharpness-1429.webp";
                public const string Sharpness03 = "WebP/vp80-05-sharpness-1430.webp";
                public const string Sharpness04 = "WebP/vp80-05-sharpness-1431.webp";
                public const string Sharpness05 = "WebP/vp80-05-sharpness-1433.webp";
                public const string Sharpness06 = "WebP/vp80-05-sharpness-1434.webp";

                // Very small images (all with complex filter).
                public const string Small01 = "WebP/small_13x1.webp";
                public const string Small02 = "WebP/small_1x1.webp";
                public const string Small03 = "WebP/small_1x13.webp";
                public const string Small04 = "WebP/small_31x13.webp";

                // Lossy images with an alpha channel.
                public const string Alpha1 = "WebP/lossy_alpha1.webp";
                public const string Alpha2 = "WebP/lossy_alpha2.webp";
                public const string AlphaNoCompression = "WebP/alpha_no_compression.webp";
                public const string AlphaNoCompressionNoFilter = "WebP/alpha_filter_0_method_0.webp";
                public const string AlphaCompressedNoFilter = "WebP/alpha_filter_0_method_1.webp";
                public const string AlphaNoCompressionHorizontalFilter = "WebP/alpha_filter_1_method_0.webp";
                public const string AlphaCompressedHorizontalFilter = "WebP/alpha_filter_1_method_1.webp";
                public const string AlphaNoCompressionVerticalFilter = "WebP/alpha_filter_2_method_0.webp";
                public const string AlphaCompressedVerticalFilter = "WebP/alpha_filter_2_method_1.webp";
                public const string AlphaNoCompressionGradientFilter = "WebP/alpha_filter_3_method_0.webp";
                public const string AlphaCompressedGradientFilter = "WebP/alpha_filter_3_method_1.webp";
            }
        }
    }
}
