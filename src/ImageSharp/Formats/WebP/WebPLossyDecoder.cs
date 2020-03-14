// Copyright (c) Six Labors and contributors.
// Licensed under the Apache License, Version 2.0.

using System;
using System.Buffers;
using System.Linq;
using System.Runtime.InteropServices;

using SixLabors.ImageSharp.Memory;
using SixLabors.ImageSharp.PixelFormats;

namespace SixLabors.ImageSharp.Formats.WebP
{
    internal sealed class WebPLossyDecoder
    {
        /// <summary>
        /// A bit reader for reading lossy webp streams.
        /// </summary>
        private readonly Vp8BitReader bitReader;

        /// <summary>
        /// Used for allocating memory during processing operations.
        /// </summary>
        private readonly MemoryAllocator memoryAllocator;

        /// <summary>
        /// The global configuration.
        /// </summary>
        private readonly Configuration configuration;

        /// <summary>
        /// Initializes a new instance of the <see cref="WebPLossyDecoder"/> class.
        /// </summary>
        /// <param name="bitReader">Bitreader to read from the stream.</param>
        /// <param name="memoryAllocator">Used for allocating memory during processing operations.</param>
        /// <param name="configuration">The configuration.</param>
        public WebPLossyDecoder(Vp8BitReader bitReader, MemoryAllocator memoryAllocator, Configuration configuration)
        {
            this.bitReader = bitReader;
            this.memoryAllocator = memoryAllocator;
            this.configuration = configuration;
        }

        public void Decode<TPixel>(Buffer2D<TPixel> pixels, int width, int height, WebPImageInfo info)
            where TPixel : unmanaged, IPixel<TPixel>
        {
            // Paragraph 9.2: color space and clamp type follow.
            sbyte colorSpace = (sbyte)this.bitReader.ReadValue(1);
            sbyte clampType = (sbyte)this.bitReader.ReadValue(1);
            var pictureHeader = new Vp8PictureHeader()
                                   {
                                       Width = (uint)width,
                                       Height = (uint)height,
                                       XScale = info.XScale,
                                       YScale = info.YScale,
                                       ColorSpace = colorSpace,
                                       ClampType = clampType
                                   };

            // Paragraph 9.3: Parse the segment header.
            var proba = new Vp8Proba();
            Vp8SegmentHeader vp8SegmentHeader = this.ParseSegmentHeader(proba);

            using (var decoder = new Vp8Decoder(info.Vp8FrameHeader, pictureHeader, vp8SegmentHeader, proba, this.memoryAllocator))
            {
                Vp8Io io = InitializeVp8Io(decoder, pictureHeader);

                // Paragraph 9.4: Parse the filter specs.
                this.ParseFilterHeader(decoder);
                decoder.PrecomputeFilterStrengths();

                // Paragraph 9.5: Parse partitions.
                this.ParsePartitions(decoder);

                // Paragraph 9.6: Dequantization Indices.
                this.ParseDequantizationIndices(decoder);

                // Ignore the value of update probabilities.
                this.bitReader.ReadBool();

                // Paragraph 13.4: Parse probabilities.
                this.ParseProbabilities(decoder);

                // Decode image data.
                this.ParseFrame(decoder, io);

                if (info.Features?.Alpha is true)
                {
                    using (var alphaDecoder = new AlphaDecoder(
                        width,
                        height,
                        info.Features.AlphaData,
                        info.Features.AlphaChunkHeader,
                        this.memoryAllocator,
                        this.configuration))
                    {
                        alphaDecoder.Decode();
                        this.DecodePixelValues(width, height, decoder.Pixels.Memory.Span, pixels, alphaDecoder.Alpha);
                    }
                }
                else
                {
                    this.DecodePixelValues(width, height, decoder.Pixels.Memory.Span, pixels);
                }
            }
        }

        private void DecodePixelValues<TPixel>(int width, int height, Span<byte> pixelData, Buffer2D<TPixel> pixels, IMemoryOwner<byte> alpha = null)
            where TPixel : unmanaged, IPixel<TPixel>
        {
            if (alpha != null)
            {
                TPixel color = default;
                Span<byte> alphaSpan = alpha.Memory.Span;
                for (int y = 0; y < height; y++)
                {
                    Span<TPixel> pixelRow = pixels.GetRowSpan(y);
                    for (int x = 0; x < width; x++)
                    {
                        int offset = (y * width) + x;
                        int idxBgr = offset * 3;
                        byte b = pixelData[idxBgr];
                        byte g = pixelData[idxBgr + 1];
                        byte r = pixelData[idxBgr + 2];
                        byte a = alphaSpan[offset];
                        color.FromBgra32(new Bgra32(r, g, b, a));
                        pixelRow[x] = color;
                    }
                }

                return;
            }

            for (int y = 0; y < height; y++)
            {
                Span<byte> row = pixelData.Slice(y * width * 3, width * 3);
                Span<TPixel> pixelSpan = pixels.GetRowSpan(y);
                PixelOperations<TPixel>.Instance.FromBgr24Bytes(
                    this.configuration,
                    row,
                    pixelSpan,
                    width);
            }
        }

        private void ParseFrame(Vp8Decoder dec, Vp8Io io)
        {
            for (dec.MbY = 0; dec.MbY < dec.BottomRightMbY; ++dec.MbY)
            {
                // Parse bitstream for this row.
                long bitreaderIdx = dec.MbY & dec.NumPartsMinusOne;
                Vp8BitReader bitreader = dec.Vp8BitReaders[bitreaderIdx];

                // Parse intra mode mode row.
                for (int mbX = 0; mbX < dec.MbWidth; ++mbX)
                {
                    this.ParseIntraMode(dec, mbX);
                }

                for (; dec.MbX < dec.MbWidth; ++dec.MbX)
                {
                    this.DecodeMacroBlock(dec, bitreader);
                }

                // Prepare for next scanline.
                this.InitScanline(dec);

                // Reconstruct, filter and emit the row.
                this.ProcessRow(dec, io);
            }
        }

        private void ParseIntraMode(Vp8Decoder dec, int mbX)
        {
            Vp8MacroBlockData block = dec.MacroBlockData[mbX];
            Span<byte> top = dec.IntraT.AsSpan(4 * mbX, 4);
            byte[] left = dec.IntraL;

            if (dec.SegmentHeader.UpdateMap)
            {
                // Hardcoded tree parsing.
                block.Segment = this.bitReader.GetBit((int)dec.Probabilities.Segments[0]) is 0
                                    ? (byte)this.bitReader.GetBit((int)dec.Probabilities.Segments[1])
                                    : (byte)(this.bitReader.GetBit((int)dec.Probabilities.Segments[2]) + 2);
            }
            else
            {
                // default for intra
                block.Segment = 0;
            }

            if (dec.UseSkipProbability)
            {
                block.Skip = (byte)this.bitReader.GetBit(dec.SkipProbability);
            }

            block.IsI4x4 = this.bitReader.GetBit(145) is 0;
            if (!block.IsI4x4)
            {
                // Hardcoded 16x16 intra-mode decision tree.
                int yMode = this.bitReader.GetBit(156) != 0 ?
                                this.bitReader.GetBit(128) != 0 ? WebPConstants.TmPred : WebPConstants.HPred :
                                this.bitReader.GetBit(163) != 0 ? WebPConstants.VPred : WebPConstants.DcPred;
                block.Modes[0] = (byte)yMode;
                for (int i = 0; i < left.Length; i++)
                {
                    left[i] = (byte)yMode;
                    top[i] = (byte)yMode;
                }
            }
            else
            {
                Span<byte> modes = block.Modes.AsSpan();
                for (int y = 0; y < 4; ++y)
                {
                    int yMode = left[y];
                    for (int x = 0; x < 4; ++x)
                    {
                        byte[] prob = WebPLookupTables.ModesProba[top[x], yMode];
                        int i = WebPConstants.YModesIntra4[this.bitReader.GetBit(prob[0])];
                        while (i > 0)
                        {
                            i = WebPConstants.YModesIntra4[(2 * i) + this.bitReader.GetBit(prob[i])];
                        }

                        yMode = -i;
                        top[x] = (byte)yMode;
                    }

                    top.CopyTo(modes);
                    modes = modes.Slice(4);
                    left[y] = (byte)yMode;
                }
            }

            // Hardcoded UVMode decision tree.
            block.UvMode = (byte)(this.bitReader.GetBit(142) is 0 ? 0 :
                           this.bitReader.GetBit(114) is 0 ? 2 :
                           this.bitReader.GetBit(183) != 0 ? 1 : 3);
        }

        private void InitScanline(Vp8Decoder dec)
        {
            Vp8MacroBlock left = dec.LeftMacroBlock;
            left.NoneZeroAcDcCoeffs = 0;
            left.NoneZeroDcCoeffs = 0;
            for (int i = 0; i < dec.IntraL.Length; i++)
            {
                dec.IntraL[i] = 0;
            }

            dec.MbX = 0;
        }

        private void ProcessRow(Vp8Decoder dec, Vp8Io io)
        {
            this.ReconstructRow(dec);
            this.FinishRow(dec, io);
        }

        private void ReconstructRow(Vp8Decoder dec)
        {
            int mby = dec.MbY;
            int yOff = (WebPConstants.Bps * 1) + 8;
            int uOff = yOff + (WebPConstants.Bps * 16) + WebPConstants.Bps;
            int vOff = uOff + 16;

            Span<byte> yuv = dec.YuvBuffer.Memory.Span;
            Span<byte> yDst = yuv.Slice(yOff);
            Span<byte> uDst = yuv.Slice(uOff);
            Span<byte> vDst = yuv.Slice(vOff);

            // Initialize left-most block.
            for (int i = 0; i < 16; ++i)
            {
                yuv[(i * WebPConstants.Bps) - 1 + yOff] = 129;
            }

            for (int i = 0; i < 8; ++i)
            {
                yuv[(i * WebPConstants.Bps) - 1 + uOff] = 129;
                yuv[(i * WebPConstants.Bps) - 1 + vOff] = 129;
            }

            // Init top-left sample on left column too.
            if (mby > 0)
            {
                yuv[yOff - 1 - WebPConstants.Bps] = yuv[uOff - 1 - WebPConstants.Bps] = yuv[vOff - 1 - WebPConstants.Bps] = 129;
            }
            else
            {
                // We only need to do this init once at block (0,0).
                // Afterward, it remains valid for the whole topmost row.
                Span<byte> tmp = yuv.Slice(yOff - WebPConstants.Bps - 1, 16 + 4 + 1);
                for (int i = 0; i < tmp.Length; ++i)
                {
                    tmp[i] = 127;
                }

                tmp = yuv.Slice(uOff - WebPConstants.Bps - 1, 8 + 1);
                for (int i = 0; i < tmp.Length; ++i)
                {
                    tmp[i] = 127;
                }

                tmp = yuv.Slice(vOff - WebPConstants.Bps - 1, 8 + 1);
                for (int i = 0; i < tmp.Length; ++i)
                {
                    tmp[i] = 127;
                }
            }

            // Reconstruct one row.
            for (int mbx = 0; mbx < dec.MbWidth; ++mbx)
            {
                Vp8MacroBlockData block = dec.MacroBlockData[mbx];

                // Rotate in the left samples from previously decoded block. We move four
                // pixels at a time for alignment reason, and because of in-loop filter.
                if (mbx > 0)
                {
                    for (int i = -1; i < 16; ++i)
                    {
                        int srcIdx = (i * WebPConstants.Bps) + 12 + yOff;
                        int dstIdx = (i * WebPConstants.Bps) - 4 + yOff;
                        yuv.Slice(srcIdx, 4).CopyTo(yuv.Slice(dstIdx));
                    }

                    for (int i = -1; i < 8; ++i)
                    {
                        int srcIdx = (i * WebPConstants.Bps) + 4 + uOff;
                        int dstIdx = (i * WebPConstants.Bps) - 4 + uOff;
                        yuv.Slice(srcIdx, 4).CopyTo(yuv.Slice(dstIdx));
                        srcIdx = (i * WebPConstants.Bps) + 4 + vOff;
                        dstIdx = (i * WebPConstants.Bps) - 4 + vOff;
                        yuv.Slice(srcIdx, 4).CopyTo(yuv.Slice(dstIdx));
                    }
                }

                // Bring top samples into the cache.
                Vp8TopSamples topYuv = dec.YuvTopSamples[mbx];
                short[] coeffs = block.Coeffs;
                uint bits = block.NonZeroY;
                if (mby > 0)
                {
                    topYuv.Y.CopyTo(yuv.Slice(yOff - WebPConstants.Bps));
                    topYuv.U.CopyTo(yuv.Slice(uOff - WebPConstants.Bps));
                    topYuv.V.CopyTo(yuv.Slice(vOff - WebPConstants.Bps));
                }

                // Predict and add residuals.
                if (block.IsI4x4)
                {
                    Span<byte> topRight = yuv.Slice(yOff - WebPConstants.Bps + 16);
                    if (mby > 0)
                    {
                        if (mbx >= dec.MbWidth - 1)
                        {
                            // On rightmost border.
                            topRight[0] = topYuv.Y[15];
                            topRight[1] = topYuv.Y[15];
                            topRight[2] = topYuv.Y[15];
                            topRight[3] = topYuv.Y[15];
                        }
                        else
                        {
                            dec.YuvTopSamples[mbx + 1].Y.AsSpan(0, 4).CopyTo(topRight);
                        }
                    }

                    // Replicate the top-right pixels below.
                    Span<uint> topRightUint = MemoryMarshal.Cast<byte, uint>(yuv.Slice(yOff - WebPConstants.Bps + 16));
                    topRightUint[WebPConstants.Bps] = topRightUint[2 * WebPConstants.Bps] = topRightUint[3 * WebPConstants.Bps] = topRightUint[0];

                    // Predict and add residuals for all 4x4 blocks in turn.
                    for (int n = 0; n < 16; ++n, bits <<= 2)
                    {
                        int offset = yOff + WebPConstants.Scan[n];
                        Span<byte> dst = yuv.Slice(offset);
                        byte lumaMode = block.Modes[n];
                        switch (lumaMode)
                        {
                            case 0:
                                LossyUtils.DC4(dst, yuv, offset);
                                break;
                            case 1:
                                LossyUtils.TM4(dst, yuv, offset);
                                break;
                            case 2:
                                LossyUtils.VE4(dst, yuv, offset);
                                break;
                            case 3:
                                LossyUtils.HE4(dst, yuv, offset);
                                break;
                            case 4:
                                LossyUtils.RD4(dst, yuv, offset);
                                break;
                            case 5:
                                LossyUtils.VR4(dst, yuv, offset);
                                break;
                            case 6:
                                LossyUtils.LD4(dst, yuv, offset);
                                break;
                            case 7:
                                LossyUtils.VL4(dst, yuv, offset);
                                break;
                            case 8:
                                LossyUtils.HD4(dst, yuv, offset);
                                break;
                            case 9:
                                LossyUtils.HU4(dst, yuv, offset);
                                break;
                        }

                        this.DoTransform(bits,  coeffs.AsSpan(n * 16), dst);
                    }
                }
                else
                {
                    // 16x16
                    int mode = CheckMode(mbx, mby, block.Modes[0]);
                    switch (mode)
                    {
                        case 0:
                            LossyUtils.DC16(yDst, yuv, yOff);
                            break;
                        case 1:
                            LossyUtils.TM16(yDst, yuv, yOff);
                            break;
                        case 2:
                            LossyUtils.VE16(yDst, yuv, yOff);
                            break;
                        case 3:
                            LossyUtils.HE16(yDst, yuv, yOff);
                            break;
                        case 4:
                            LossyUtils.DC16NoTop(yDst, yuv, yOff);
                            break;
                        case 5:
                            LossyUtils.DC16NoLeft(yDst, yuv, yOff);
                            break;
                        case 6:
                            LossyUtils.DC16NoTopLeft(yDst);
                            break;
                    }

                    if (bits != 0)
                    {
                        for (int n = 0; n < 16; ++n, bits <<= 2)
                        {
                            this.DoTransform(bits, coeffs.AsSpan(n * 16), yDst.Slice(WebPConstants.Scan[n]));
                        }
                    }
                }

                // Chroma
                uint bitsUv = block.NonZeroUv;
                int chromaMode = CheckMode(mbx, mby, block.UvMode);
                switch (chromaMode)
                {
                    case 0:
                        LossyUtils.DC8uv(uDst, yuv, uOff);
                        LossyUtils.DC8uv(vDst, yuv, vOff);
                        break;
                    case 1:
                        LossyUtils.TM8uv(uDst, yuv, uOff);
                        LossyUtils.TM8uv(vDst, yuv, vOff);
                        break;
                    case 2:
                        LossyUtils.VE8uv(uDst, yuv, uOff);
                        LossyUtils.VE8uv(vDst, yuv, vOff);
                        break;
                    case 3:
                        LossyUtils.HE8uv(uDst, yuv, uOff);
                        LossyUtils.HE8uv(vDst, yuv, vOff);
                        break;
                    case 4:
                        LossyUtils.DC8uvNoTop(uDst, yuv, uOff);
                        LossyUtils.DC8uvNoTop(vDst, yuv, vOff);
                        break;
                    case 5:
                        LossyUtils.DC8uvNoLeft(uDst, yuv, uOff);
                        LossyUtils.DC8uvNoLeft(vDst, yuv, vOff);
                        break;
                    case 6:
                        LossyUtils.DC8uvNoTopLeft(uDst);
                        LossyUtils.DC8uvNoTopLeft(vDst);
                        break;
                }

                this.DoUVTransform(bitsUv >> 0, coeffs.AsSpan(16 * 16), uDst);
                this.DoUVTransform(bitsUv >> 8, coeffs.AsSpan(20 * 16), vDst);

                // Stash away top samples for next block.
                if (mby < dec.MbHeight - 1)
                {
                    yDst.Slice(15 * WebPConstants.Bps, 16).CopyTo(topYuv.Y);
                    uDst.Slice(7 * WebPConstants.Bps, 8).CopyTo(topYuv.U);
                    vDst.Slice(7 * WebPConstants.Bps, 8).CopyTo(topYuv.V);
                }

                // Transfer reconstructed samples from yuv_buffer cache to final destination.
                Span<byte> yOut = dec.CacheY.Memory.Span.Slice(dec.CacheYOffset + (mbx * 16));
                Span<byte> uOut = dec.CacheU.Memory.Span.Slice(dec.CacheUvOffset + (mbx * 8));
                Span<byte> vOut = dec.CacheV.Memory.Span.Slice(dec.CacheUvOffset + (mbx * 8));
                for (int j = 0; j < 16; ++j)
                {
                    yDst.Slice(j * WebPConstants.Bps, Math.Min(16, yOut.Length)).CopyTo(yOut.Slice(j * dec.CacheYStride));
                }

                for (int j = 0; j < 8; ++j)
                {
                    uDst.Slice(j * WebPConstants.Bps, Math.Min(8, uOut.Length)).CopyTo(uOut.Slice(j * dec.CacheUvStride));
                    vDst.Slice(j * WebPConstants.Bps, Math.Min(8, vOut.Length)).CopyTo(vOut.Slice(j * dec.CacheUvStride));
                }
            }
        }

        private void FilterRow(Vp8Decoder dec)
        {
            int mby = dec.MbY;
            for (int mbx = dec.TopLeftMbX; mbx < dec.BottomRightMbX; ++mbx)
            {
                this.DoFilter(dec, mbx, mby);
            }
        }

        private void DoFilter(Vp8Decoder dec, int mbx, int mby)
        {
            int yBps = dec.CacheYStride;
            Vp8FilterInfo filterInfo = dec.FilterInfo[mbx];
            int iLevel = filterInfo.InnerLevel;
            int limit = filterInfo.Limit;

            if (limit is 0)
            {
                return;
            }

            if (dec.Filter is LoopFilter.Simple)
            {
                int offset = dec.CacheYOffset + (mbx * 16);
                if (mbx > 0)
                {
                    LossyUtils.SimpleHFilter16(dec.CacheY.Memory.Span, offset, yBps, limit + 4);
                }

                if (filterInfo.UseInnerFiltering > 0)
                {
                    LossyUtils.SimpleHFilter16i(dec.CacheY.Memory.Span, offset, yBps, limit);
                }

                if (mby > 0)
                {
                    LossyUtils.SimpleVFilter16(dec.CacheY.Memory.Span, offset, yBps, limit + 4);
                }

                if (filterInfo.UseInnerFiltering > 0)
                {
                    LossyUtils.SimpleVFilter16i(dec.CacheY.Memory.Span, offset, yBps, limit);
                }
            }
            else if (dec.Filter is LoopFilter.Complex)
            {
                int uvBps = dec.CacheUvStride;
                int yOffset = dec.CacheYOffset + (mbx * 16);
                int uvOffset = dec.CacheUvOffset + (mbx * 8);
                int hevThresh = filterInfo.HighEdgeVarianceThreshold;
                if (mbx > 0)
                {
                    LossyUtils.HFilter16(dec.CacheY.Memory.Span, yOffset, yBps, limit + 4, iLevel, hevThresh);
                    LossyUtils.HFilter8(dec.CacheU.Memory.Span, dec.CacheV.Memory.Span, uvOffset, uvBps, limit + 4, iLevel, hevThresh);
                }

                if (filterInfo.UseInnerFiltering > 0)
                {
                    LossyUtils.HFilter16i(dec.CacheY.Memory.Span, yOffset, yBps, limit, iLevel, hevThresh);
                    LossyUtils.HFilter8i(dec.CacheU.Memory.Span, dec.CacheV.Memory.Span, uvOffset, uvBps, limit, iLevel, hevThresh);
                }

                if (mby > 0)
                {
                    LossyUtils.VFilter16(dec.CacheY.Memory.Span, yOffset, yBps, limit + 4, iLevel, hevThresh);
                    LossyUtils.VFilter8(dec.CacheU.Memory.Span, dec.CacheV.Memory.Span, uvOffset, uvBps, limit + 4, iLevel, hevThresh);
                }

                if (filterInfo.UseInnerFiltering > 0)
                {
                    LossyUtils.VFilter16i(dec.CacheY.Memory.Span, yOffset, yBps, limit, iLevel, hevThresh);
                    LossyUtils.VFilter8i(dec.CacheU.Memory.Span, dec.CacheV.Memory.Span, uvOffset, uvBps, limit, iLevel, hevThresh);
                }
            }
        }

        private void FinishRow(Vp8Decoder dec, Vp8Io io)
        {
            int extraYRows = WebPConstants.FilterExtraRows[(int)dec.Filter];
            int ySize = extraYRows * dec.CacheYStride;
            int uvSize = (extraYRows / 2) * dec.CacheUvStride;
            Span<byte> yDst = dec.CacheY.Memory.Span;
            Span<byte> uDst = dec.CacheU.Memory.Span;
            Span<byte> vDst = dec.CacheV.Memory.Span;
            int mby = dec.MbY;
            bool isFirstRow = mby is 0;
            bool isLastRow = mby >= dec.BottomRightMbY - 1;
            bool filterRow = (dec.Filter != LoopFilter.None) &&
                             (dec.MbY >= dec.TopLeftMbY) && (dec.MbY <= dec.BottomRightMbY);

            if (filterRow)
            {
                this.FilterRow(dec);
            }

            int yStart = mby * 16;
            int yEnd = (mby + 1) * 16;
            if (!isFirstRow)
            {
                yStart -= extraYRows;
                io.Y = yDst;
                io.U = uDst;
                io.V = vDst;
            }
            else
            {
                io.Y = dec.CacheY.Memory.Span.Slice(dec.CacheYOffset);
                io.U = dec.CacheU.Memory.Span.Slice(dec.CacheUvOffset);
                io.V = dec.CacheV.Memory.Span.Slice(dec.CacheUvOffset);
            }

            if (!isLastRow)
            {
                yEnd -= extraYRows;
            }

            if (yEnd > io.CropBottom)
            {
                yEnd = io.CropBottom; // make sure we don't overflow on last row.
            }

            if (yStart < yEnd)
            {
                io.Y = io.Y.Slice(io.CropLeft);
                io.U = io.U.Slice(io.CropLeft);
                io.V = io.V.Slice(io.CropLeft);

                io.MbY = yStart - io.CropTop;
                io.MbW = io.CropRight - io.CropLeft;
                io.MbH = yEnd - yStart;
                this.EmitRgb(dec, io);
            }

            // Rotate top samples if needed.
            if (!isLastRow)
            {
                yDst.Slice(16 * dec.CacheYStride, ySize).CopyTo(dec.CacheY.Memory.Span);
                uDst.Slice(8 * dec.CacheUvStride, uvSize).CopyTo(dec.CacheU.Memory.Span);
                vDst.Slice(8 * dec.CacheUvStride, uvSize).CopyTo(dec.CacheV.Memory.Span);
            }
        }

        private int EmitRgb(Vp8Decoder dec, Vp8Io io)
        {
            Span<byte> buf = dec.Pixels.Memory.Span;
            int numLinesOut = io.MbH; // a priori guess.
            Span<byte> curY = io.Y;
            Span<byte> curU = io.U;
            Span<byte> curV = io.V;
            Span<byte> tmpYBuffer = dec.TmpYBuffer.Memory.Span;
            Span<byte> tmpUBuffer = dec.TmpUBuffer.Memory.Span;
            Span<byte> tmpVBuffer = dec.TmpVBuffer.Memory.Span;
            Span<byte> topU = tmpUBuffer;
            Span<byte> topV = tmpVBuffer;
            int bpp = 3;
            int bufferStride = bpp * io.Width;
            int dstStartIdx = io.MbY * bufferStride;
            Span<byte> dst = buf.Slice(dstStartIdx);
            int yEnd = io.MbY + io.MbH;
            int mbw = io.MbW;
            int uvw = (mbw + 1) / 2;
            int y = io.MbY;

            if (y is 0)
            {
                // First line is special cased. We mirror the u/v samples at boundary.
                this.UpSample(curY, null, curU, curV, curU, curV, dst, null, mbw);
            }
            else
            {
                // We can finish the left-over line from previous call.
                this.UpSample(tmpYBuffer, curY, topU, topV, curU, curV, buf.Slice(dstStartIdx - bufferStride), dst, mbw);
                numLinesOut++;
            }

            // Loop over each output pairs of row.
            for (; y + 2 < yEnd; y += 2)
            {
                topU = curU;
                topV = curV;
                curU = curU.Slice(io.UvStride);
                curV = curV.Slice(io.UvStride);
                this.UpSample(curY.Slice(io.YStride), curY.Slice(2 * io.YStride), topU, topV, curU, curV, dst.Slice(bufferStride), dst.Slice(2 * bufferStride), mbw);
                curY = curY.Slice(2 * io.YStride);
                dst = dst.Slice(2 * bufferStride);
            }

            // Move to last row.
            curY = curY.Slice(io.YStride);
            if (io.CropTop + yEnd < io.CropBottom)
            {
                // Save the unfinished samples for next call (as we're not done yet).
                curY.Slice(0, mbw).CopyTo(tmpYBuffer);
                curU.Slice(0, uvw).CopyTo(tmpUBuffer);
                curV.Slice(0, uvw).CopyTo(tmpVBuffer);

                // The upsampler leaves a row unfinished behind (except for the very last row).
                numLinesOut--;
            }
            else
            {
                // Process the very last row of even-sized picture.
                if ((yEnd & 1) is 0)
                {
                    this.UpSample(curY, null, curU, curV, curU, curV, dst.Slice(bufferStride), null, mbw);
                }
            }

            return numLinesOut;
        }

        private void UpSample(Span<byte> topY, Span<byte> bottomY, Span<byte> topU, Span<byte> topV, Span<byte> curU, Span<byte> curV, Span<byte> topDst, Span<byte> bottomDst, int len)
        {
            int xStep = 3;
            int lastPixelPair = (len - 1) >> 1;
            uint tluv = LossyUtils.LoadUv(topU[0], topV[0]); // top-left sample
            uint luv = LossyUtils.LoadUv(curU[0], curV[0]); // left-sample
            uint uv0 = ((3 * tluv) + luv + 0x00020002u) >> 2;
            LossyUtils.YuvToBgr(topY[0], (int)(uv0 & 0xff), (int)(uv0 >> 16), topDst);

            if (bottomY != null)
            {
                uv0 = ((3 * luv) + tluv + 0x00020002u) >> 2;
                LossyUtils.YuvToBgr(bottomY[0], (int)uv0 & 0xff, (int)(uv0 >> 16), bottomDst);
            }

            for (int x = 1; x <= lastPixelPair; ++x)
            {
                uint tuv = LossyUtils.LoadUv(topU[x], topV[x]); // top sample
                uint uv = LossyUtils.LoadUv(curU[x], curV[x]); // sample

                // Precompute invariant values associated with first and second diagonals.
                uint avg = tluv + tuv + luv + uv + 0x00080008u;
                uint diag12 = (avg + (2 * (tuv + luv))) >> 3;
                uint diag03 = (avg + (2 * (tluv + uv))) >> 3;
                uv0 = (diag12 + tluv) >> 1;
                uint uv1 = (diag03 + tuv) >> 1;
                LossyUtils.YuvToBgr(topY[(2 * x) - 1], (int)(uv0 & 0xff), (int)(uv0 >> 16), topDst.Slice(((2 * x) - 1) * xStep));
                LossyUtils.YuvToBgr(topY[(2 * x) - 0], (int)(uv1 & 0xff), (int)(uv1 >> 16), topDst.Slice(((2 * x) - 0) * xStep));

                if (bottomY != null)
                {
                    uv0 = (diag03 + luv) >> 1;
                    uv1 = (diag12 + uv) >> 1;
                    LossyUtils.YuvToBgr(bottomY[(2 * x) - 1], (int)(uv0 & 0xff), (int)(uv0 >> 16), bottomDst.Slice(((2 * x) - 1) * xStep));
                    LossyUtils.YuvToBgr(bottomY[(2 * x) + 0], (int)(uv1 & 0xff), (int)(uv1 >> 16), bottomDst.Slice(((2 * x) + 0) * xStep));
                }

                tluv = tuv;
                luv = uv;
            }

            if ((len & 1) is 0)
            {
                uv0 = ((3 * tluv) + luv + 0x00020002u) >> 2;
                LossyUtils.YuvToBgr(topY[len - 1], (int)(uv0 & 0xff), (int)(uv0 >> 16), topDst.Slice((len - 1) * xStep));
                if (bottomY != null)
                {
                    uv0 = ((3 * luv) + tluv + 0x00020002u) >> 2;
                    LossyUtils.YuvToBgr(bottomY[len - 1], (int)(uv0 & 0xff), (int)(uv0 >> 16), bottomDst.Slice((len - 1) * xStep));
                }
            }
        }

        private void DoTransform(uint bits, Span<short> src, Span<byte> dst)
        {
            switch (bits >> 30)
            {
                case 3:
                    LossyUtils.Transform(src, dst, false);
                    break;
                case 2:
                    LossyUtils.TransformAc3(src, dst);
                    break;
                case 1:
                    LossyUtils.TransformDc(src, dst);
                    break;
                default:
                    break;
            }
        }

        private void DoUVTransform(uint bits, Span<short> src, Span<byte> dst)
        {
            // any non-zero coeff at all?
            if ((bits & 0xff) > 0)
            {
                // any non-zero AC coefficient?
                if ((bits & 0xaa) > 0)
                {
                    LossyUtils.TransformUv(src, dst); // note we don't use the AC3 variant for U/V.
                }
                else
                {
                    LossyUtils.TransformDcuv(src, dst);
                }
            }
        }

        private void DecodeMacroBlock(Vp8Decoder dec, Vp8BitReader bitreader)
        {
            Vp8MacroBlock left = dec.LeftMacroBlock;
            Vp8MacroBlock macroBlock = dec.CurrentMacroBlock;
            Vp8MacroBlockData blockData = dec.CurrentBlockData;
            int skip = dec.UseSkipProbability ? blockData.Skip : 0;

            if (skip is 0)
            {
                this.ParseResiduals(dec, bitreader, macroBlock);
            }
            else
            {
                left.NoneZeroAcDcCoeffs = macroBlock.NoneZeroAcDcCoeffs = 0;
                if (!blockData.IsI4x4)
                {
                    left.NoneZeroDcCoeffs = macroBlock.NoneZeroDcCoeffs = 0;
                }

                blockData.NonZeroY = 0;
                blockData.NonZeroUv = 0;
            }

            // Store filter info.
            if (dec.Filter != LoopFilter.None)
            {
                Vp8FilterInfo precomputedFilterInfo = dec.FilterStrength[blockData.Segment, blockData.IsI4x4 ? 1 : 0];
                dec.FilterInfo[dec.MbX] = (Vp8FilterInfo)precomputedFilterInfo.DeepClone();
                dec.FilterInfo[dec.MbX].UseInnerFiltering |= (byte)(skip is 0 ? 1 : 0);
            }
        }

        private bool ParseResiduals(Vp8Decoder dec, Vp8BitReader br, Vp8MacroBlock mb)
        {
            uint nonZeroY = 0;
            uint nonZeroUv = 0;
            int first;
            int dstOffset = 0;
            Vp8MacroBlockData block = dec.CurrentBlockData;
            Vp8QuantMatrix q = dec.DeQuantMatrices[block.Segment];
            Vp8BandProbas[,] bands = dec.Probabilities.BandsPtr;
            Vp8BandProbas[] acProba;
            Vp8MacroBlock leftMb = dec.LeftMacroBlock;
            short[] dst = block.Coeffs;
            for (int i = 0; i < dst.Length; i++)
            {
                dst[i] = 0;
            }

            if (!block.IsI4x4)
            {
                // Parse DC
                var dc = new short[16];
                int ctx = (int)(mb.NoneZeroDcCoeffs + leftMb.NoneZeroDcCoeffs);
                int nz = this.GetCoeffs(br, GetBandsRow(bands, 1), ctx, q.Y2Mat, 0, dc);
                mb.NoneZeroDcCoeffs = leftMb.NoneZeroDcCoeffs = (uint)(nz > 0 ? 1 : 0);
                if (nz > 1)
                {
                    // More than just the DC -> perform the full transform.
                    this.TransformWht(dc, dst);
                }
                else
                {
                    // Only DC is non-zero -> inlined simplified transform.
                    int dc0 = (dc[0] + 3) >> 3;
                    for (int i = 0; i < 16 * 16; i += 16)
                    {
                        dst[i] = (short)dc0;
                    }
                }

                first = 1;
                acProba = GetBandsRow(bands, 0);
            }
            else
            {
                first = 0;
                acProba = GetBandsRow(bands, 3);
            }

            byte tnz = (byte)(mb.NoneZeroAcDcCoeffs & 0x0f);
            byte lnz = (byte)(leftMb.NoneZeroAcDcCoeffs & 0x0f);

            for (int y = 0; y < 4; ++y)
            {
                int l = lnz & 1;
                uint nzCoeffs = 0;
                for (int x = 0; x < 4; ++x)
                {
                    int ctx = l + (tnz & 1);
                    int nz = this.GetCoeffs(br, acProba, ctx, q.Y1Mat, first, dst.AsSpan(dstOffset));
                    l = (nz > first) ? 1 : 0;
                    tnz = (byte)((tnz >> 1) | (l << 7));
                    nzCoeffs = NzCodeBits(nzCoeffs, nz, dst[dstOffset] != 0 ? 1 : 0);
                    dstOffset += 16;
                }

                tnz >>= 4;
                lnz = (byte)((lnz >> 1) | (l << 7));
                nonZeroY = (nonZeroY << 8) | nzCoeffs;
            }

            uint outTnz = tnz;
            uint outLnz = (uint)(lnz >> 4);

            for (int ch = 0; ch < 4; ch += 2)
            {
                uint nzCoeffs = 0;
                tnz = (byte)(mb.NoneZeroAcDcCoeffs >> (4 + ch));
                lnz = (byte)(leftMb.NoneZeroAcDcCoeffs >> (4 + ch));
                for (int y = 0; y < 2; ++y)
                {
                    int l = lnz & 1;
                    for (int x = 0; x < 2; ++x)
                    {
                        int ctx = l + (tnz & 1);
                        int nz = this.GetCoeffs(br, GetBandsRow(bands, 2), ctx, q.UvMat, 0, dst.AsSpan(dstOffset));
                        l = (nz > 0) ? 1 : 0;
                        tnz = (byte)((tnz >> 1) | (l << 3));
                        nzCoeffs = NzCodeBits(nzCoeffs, nz, dst[dstOffset] != 0 ? 1 : 0);
                        dstOffset += 16;
                    }

                    tnz >>= 2;
                    lnz = (byte)((lnz >> 1) | (l << 5));
                }

                // Note: we don't really need the per-4x4 details for U/V blocks.
                nonZeroUv |= nzCoeffs << (4 * ch);
                outTnz |= (uint)((tnz << 4) << ch);
                outLnz |= (uint)((lnz & 0xf0) << ch);
            }

            mb.NoneZeroAcDcCoeffs = outTnz;
            leftMb.NoneZeroAcDcCoeffs = outLnz;

            block.NonZeroY = nonZeroY;
            block.NonZeroUv = nonZeroUv;

            return (nonZeroY | nonZeroUv) is 0;
        }

        private int GetCoeffs(Vp8BitReader br, Vp8BandProbas[] prob, int ctx, int[] dq, int n, Span<short> coeffs)
        {
            // Returns the position of the last non - zero coeff plus one.
            Vp8ProbaArray p = prob[n].Probabilities[ctx];
            for (; n < 16; ++n)
            {
                if (br.GetBit(p.Probabilities[0]) is 0)
                {
                    // Previous coeff was last non - zero coeff.
                    return n;
                }

                // Sequence of zero coeffs.
                while (br.GetBit(p.Probabilities[1]) is 0)
                {
                    p = prob[++n].Probabilities[0];
                    if (n is 16)
                    {
                        return 16;
                    }
                }

                // Non zero coeffs.
                int v;
                if (br.GetBit(p.Probabilities[2]) is 0)
                {
                    v = 1;
                    p = prob[n + 1].Probabilities[1];
                }
                else
                {
                    v = this.GetLargeValue(br, p.Probabilities);
                    p = prob[n + 1].Probabilities[2];
                }

                int idx = n > 0 ? 1 : 0;
                coeffs[WebPConstants.Zigzag[n]] = (short)(br.GetSigned(v) * dq[idx]);
            }

            return 16;
        }

        private int GetLargeValue(Vp8BitReader br, byte[] p)
        {
            // See section 13 - 2: http://tools.ietf.org/html/rfc6386#section-13.2
            int v;
            if (br.GetBit(p[3]) is 0)
            {
                if (br.GetBit(p[4]) is 0)
                {
                    v = 2;
                }
                else
                {
                    v = 3 + br.GetBit(p[5]);
                }
            }
            else
            {
                if (br.GetBit(p[6]) is 0)
                {
                    if (br.GetBit(p[7]) is 0)
                    {
                        v = 5 + br.GetBit(159);
                    }
                    else
                    {
                        v = 7 + (2 * br.GetBit(165));
                        v += br.GetBit(145);
                    }
                }
                else
                {
                    int bit1 = br.GetBit(p[8]);
                    int bit0 = br.GetBit(p[9 + bit1]);
                    int cat = (2 * bit1) + bit0;
                    v = 0;
                    byte[] tab = null;
                    switch (cat)
                    {
                        case 0:
                            tab = WebPConstants.Cat3;
                            break;
                        case 1:
                            tab = WebPConstants.Cat4;
                            break;
                        case 2:
                            tab = WebPConstants.Cat5;
                            break;
                        case 3:
                            tab = WebPConstants.Cat6;
                            break;
                        default:
                            WebPThrowHelper.ThrowImageFormatException("VP8 parsing error");
                            break;
                    }

                    for (int i = 0; i < tab.Length; i++)
                    {
                        v += v + br.GetBit(tab[i]);
                    }

                    v += 3 + (8 << cat);
                }
            }

            return v;
        }

        /// <summary>
        /// Paragraph 14.3: Implementation of the Walsh-Hadamard transform inversion.
        /// </summary>
        private void TransformWht(short[] input, short[] output)
        {
            var tmp = new int[16];
            for (int i = 0; i < 4; ++i)
            {
                int a0 = input[0 + i] + input[12 + i];
                int a1 = input[4 + i] + input[8 + i];
                int a2 = input[4 + i] - input[8 + i];
                int a3 = input[0 + i] - input[12 + i];
                tmp[0 + i] = a0 + a1;
                tmp[8 + i] = a0 - a1;
                tmp[4 + i] = a3 + a2;
                tmp[12 + i] = a3 - a2;
            }

            int outputOffset = 0;
            for (int i = 0; i < 4; ++i)
            {
                int dc = tmp[0 + (i * 4)] + 3;
                int a0 = dc + tmp[3 + (i * 4)];
                int a1 = tmp[1 + (i * 4)] + tmp[2 + (i * 4)];
                int a2 = tmp[1 + (i * 4)] - tmp[2 + (i * 4)];
                int a3 = dc - tmp[3 + (i * 4)];
                output[outputOffset + 0] = (short)((a0 + a1) >> 3);
                output[outputOffset + 16] = (short)((a3 + a2) >> 3);
                output[outputOffset + 32] = (short)((a0 - a1) >> 3);
                output[outputOffset + 48] = (short)((a3 - a2) >> 3);
                outputOffset += 64;
            }
        }

        private Vp8SegmentHeader ParseSegmentHeader(Vp8Proba proba)
        {
            var vp8SegmentHeader = new Vp8SegmentHeader
            {
                UseSegment = this.bitReader.ReadBool()
            };
            if (vp8SegmentHeader.UseSegment)
            {
                vp8SegmentHeader.UpdateMap = this.bitReader.ReadBool();
                bool updateData = this.bitReader.ReadBool();
                if (updateData)
                {
                    vp8SegmentHeader.Delta = this.bitReader.ReadBool();
                    bool hasValue;
                    for (int i = 0; i < vp8SegmentHeader.Quantizer.Length; i++)
                    {
                        hasValue = this.bitReader.ReadBool();
                        int quantizeValue = hasValue ? this.bitReader.ReadSignedValue(7) : 0;
                        vp8SegmentHeader.Quantizer[i] = (byte)quantizeValue;
                    }

                    for (int i = 0; i < vp8SegmentHeader.FilterStrength.Length; i++)
                    {
                        hasValue = this.bitReader.ReadBool();
                        int filterStrengthValue = hasValue ? this.bitReader.ReadSignedValue(6) : 0;
                        vp8SegmentHeader.FilterStrength[i] = (byte)filterStrengthValue;
                    }

                    if (vp8SegmentHeader.UpdateMap)
                    {
                        for (int s = 0; s < proba.Segments.Length; ++s)
                        {
                            hasValue = this.bitReader.ReadBool();
                            proba.Segments[s] = hasValue ? this.bitReader.ReadValue(8) : 255;
                        }
                    }
                }
            }
            else
            {
                vp8SegmentHeader.UpdateMap = false;
            }

            return vp8SegmentHeader;
        }

        private void ParseFilterHeader(Vp8Decoder dec)
        {
            Vp8FilterHeader vp8FilterHeader = dec.FilterHeader;
            vp8FilterHeader.LoopFilter = this.bitReader.ReadBool() ? LoopFilter.Simple : LoopFilter.Complex;
            vp8FilterHeader.Level = (int)this.bitReader.ReadValue(6);
            vp8FilterHeader.Sharpness = (int)this.bitReader.ReadValue(3);
            vp8FilterHeader.UseLfDelta = this.bitReader.ReadBool();

            dec.Filter = (vp8FilterHeader.Level is 0) ? LoopFilter.None : vp8FilterHeader.LoopFilter;
            if (vp8FilterHeader.UseLfDelta)
            {
                // Update lf-delta?
                if (this.bitReader.ReadBool())
                {
                    bool hasValue;
                    for (int i = 0; i < vp8FilterHeader.RefLfDelta.Length; i++)
                    {
                        hasValue = this.bitReader.ReadBool();
                        if (hasValue)
                        {
                            vp8FilterHeader.RefLfDelta[i] = this.bitReader.ReadSignedValue(6);
                        }
                    }

                    for (int i = 0; i < vp8FilterHeader.ModeLfDelta.Length; i++)
                    {
                        hasValue = this.bitReader.ReadBool();
                        if (hasValue)
                        {
                            vp8FilterHeader.ModeLfDelta[i] = this.bitReader.ReadSignedValue(6);
                        }
                    }
                }
            }

            int extraRows = WebPConstants.FilterExtraRows[(int)dec.Filter];
            int extraY = extraRows * dec.CacheYStride;
            int extraUv = (extraRows / 2) * dec.CacheUvStride;
            dec.CacheYOffset = extraY;
            dec.CacheUvOffset = extraUv;
        }

        private void ParsePartitions(Vp8Decoder dec)
        {
            uint size = this.bitReader.Remaining - this.bitReader.PartitionLength;
            int startIdx = (int)this.bitReader.PartitionLength;
            Span<byte> sz = this.bitReader.Data.AsSpan(startIdx);
            int sizeLeft = (int)size;
            dec.NumPartsMinusOne = (1 << (int)this.bitReader.ReadValue(2)) - 1;
            int lastPart = dec.NumPartsMinusOne;

            int partStart = startIdx + (lastPart * 3);
            sizeLeft -= lastPart * 3;
            for (int p = 0; p < lastPart; ++p)
            {
                int pSize = sz[0] | (sz[1] << 8) | (sz[2] << 16);
                if (pSize > sizeLeft)
                {
                    pSize = sizeLeft;
                }

                dec.Vp8BitReaders[p] = new Vp8BitReader(this.bitReader.Data, (uint)pSize, partStart);
                partStart += pSize;
                sizeLeft -= pSize;
                sz = sz.Slice(3);
            }

            dec.Vp8BitReaders[lastPart] = new Vp8BitReader(this.bitReader.Data, (uint)sizeLeft, partStart);
        }

        private void ParseDequantizationIndices(Vp8Decoder decoder)
        {
            Vp8SegmentHeader vp8SegmentHeader = decoder.SegmentHeader;

            int baseQ0 = (int)this.bitReader.ReadValue(7);
            bool hasValue = this.bitReader.ReadBool();
            int dqy1Dc = hasValue ? this.bitReader.ReadSignedValue(4) : 0;
            hasValue = this.bitReader.ReadBool();
            int dqy2Dc = hasValue ? this.bitReader.ReadSignedValue(4) : 0;
            hasValue = this.bitReader.ReadBool();
            int dqy2Ac = hasValue ? this.bitReader.ReadSignedValue(4) : 0;
            hasValue = this.bitReader.ReadBool();
            int dquvDc = hasValue ? this.bitReader.ReadSignedValue(4) : 0;
            hasValue = this.bitReader.ReadBool();
            int dquvAc = hasValue ? this.bitReader.ReadSignedValue(4) : 0;
            for (int i = 0; i < WebPConstants.NumMbSegments; ++i)
            {
                int q;
                if (vp8SegmentHeader.UseSegment)
                {
                    q = vp8SegmentHeader.Quantizer[i];
                    if (!vp8SegmentHeader.Delta)
                    {
                        q += baseQ0;
                    }
                }
                else
                {
                    if (i > 0)
                    {
                        decoder.DeQuantMatrices[i] = decoder.DeQuantMatrices[0];
                        continue;
                    }
                    else
                    {
                        q = baseQ0;
                    }
                }

                Vp8QuantMatrix m = decoder.DeQuantMatrices[i];
                m.Y1Mat[0] = WebPLookupTables.DcTable[Clip(q + dqy1Dc, 127)];
                m.Y1Mat[1] = WebPLookupTables.AcTable[Clip(q + 0, 127)];
                m.Y2Mat[0] = WebPLookupTables.DcTable[Clip(q + dqy2Dc, 127)] * 2;

                // For all x in [0..284], x*155/100 is bitwise equal to (x*101581) >> 16.
                // The smallest precision for that is '(x*6349) >> 12' but 16 is a good word size.
                m.Y2Mat[1] = (WebPLookupTables.AcTable[Clip(q + dqy2Ac, 127)] * 101581) >> 16;
                if (m.Y2Mat[1] < 8)
                {
                    m.Y2Mat[1] = 8;
                }

                m.UvMat[0] = WebPLookupTables.DcTable[Clip(q + dquvDc, 117)];
                m.UvMat[1] = WebPLookupTables.AcTable[Clip(q + dquvAc, 127)];

                // For dithering strength evaluation.
                m.UvQuant = q + dquvAc;
            }
        }

        private void ParseProbabilities(Vp8Decoder dec)
        {
            Vp8Proba proba = dec.Probabilities;

            for (int t = 0; t < WebPConstants.NumTypes; ++t)
            {
                for (int b = 0; b < WebPConstants.NumBands; ++b)
                {
                    for (int c = 0; c < WebPConstants.NumCtx; ++c)
                    {
                        for (int p = 0; p < WebPConstants.NumProbas; ++p)
                        {
                            byte prob = WebPLookupTables.CoeffsUpdateProba[t, b, c, p];
                            int v = this.bitReader.GetBit(prob) != 0
                                        ? (int)this.bitReader.ReadValue(8)
                                        : WebPLookupTables.DefaultCoeffsProba[t, b, c, p];
                            proba.Bands[t, b].Probabilities[c].Probabilities[p] = (byte)v;
                        }
                    }
                }

                for (int b = 0; b < 16 + 1; ++b)
                {
                    proba.BandsPtr[t, b] = proba.Bands[t, WebPConstants.Bands[b]];
                }
            }

            dec.UseSkipProbability = this.bitReader.ReadBool();
            if (dec.UseSkipProbability)
            {
                dec.SkipProbability = (byte)this.bitReader.ReadValue(8);
            }
        }

        private static Vp8Io InitializeVp8Io(Vp8Decoder dec, Vp8PictureHeader pictureHeader)
        {
            var io = default(Vp8Io);
            io.Width = (int)pictureHeader.Width;
            io.Height = (int)pictureHeader.Height;
            io.UseCropping = false;
            io.CropTop = 0;
            io.CropLeft = 0;
            io.CropRight = io.Width;
            io.CropBottom = io.Height;
            io.UseScaling = false;
            io.ScaledWidth = io.Width;
            io.ScaledHeight = io.ScaledHeight;
            io.MbW = io.Width;
            io.MbH = io.Height;
            io.YStride = (int)(16 * ((pictureHeader.Width + 15) >> 4));
            io.UvStride = (int)(8 * ((pictureHeader.Width + 15) >> 4));

            int intraPredModeSize = 4 * dec.MbWidth;
            dec.IntraT = new byte[intraPredModeSize];

            int extraPixels = WebPConstants.FilterExtraRows[(int)dec.Filter];
            if (dec.Filter is LoopFilter.Complex)
            {
                // For complex filter, we need to preserve the dependency chain.
                dec.TopLeftMbX = 0;
                dec.TopLeftMbY = 0;
            }
            else
            {
                // For simple filter, we can filter only the cropped region. We include 'extraPixels' on
                // the other side of the boundary, since vertical or horizontal filtering of the previous
                // macroblock can modify some abutting pixels.
                dec.TopLeftMbX = (io.CropLeft - extraPixels) >> 4;
                dec.TopLeftMbY = (io.CropTop - extraPixels) >> 4;
                if (dec.TopLeftMbX < 0)
                {
                    dec.TopLeftMbX = 0;
                }

                if (dec.TopLeftMbY < 0)
                {
                    dec.TopLeftMbY = 0;
                }
            }

            // We need some 'extra' pixels on the right/bottom.
            dec.BottomRightMbY = (io.CropBottom + 15 + extraPixels) >> 4;
            dec.BottomRightMbX = (io.CropRight + 15 + extraPixels) >> 4;
            if (dec.BottomRightMbX > dec.MbWidth)
            {
                dec.BottomRightMbX = dec.MbWidth;
            }

            if (dec.BottomRightMbY > dec.MbHeight)
            {
                dec.BottomRightMbY = dec.MbHeight;
            }

            return io;
        }

        private static uint NzCodeBits(uint nzCoeffs, int nz, int dcNz)
        {
            nzCoeffs <<= 2;
            nzCoeffs |= (uint)((nz > 3) ? 3 : (nz > 1) ? 2 : dcNz);
            return nzCoeffs;
        }

        private static Vp8BandProbas[] GetBandsRow(Vp8BandProbas[,] bands, int rowIdx)
        {
            Vp8BandProbas[] bandsRow = Enumerable.Range(0, bands.GetLength(1)).Select(x => bands[rowIdx, x]).ToArray();
            return bandsRow;
        }

        private static int CheckMode(int mbx, int mby, int mode)
        {
            // B_DC_PRED
            if (mode is 0)
            {
                if (mbx is 0)
                {
                    return (mby is 0)
                               ? 6 // B_DC_PRED_NOTOPLEFT
                               : 5; // B_DC_PRED_NOLEFT
                }

                return (mby is 0)
                           ? 4 // B_DC_PRED_NOTOP
                           : 0; // B_DC_PRED
            }

            return mode;
        }

        private static int Clip(int value, int max)
        {
            return value < 0 ? 0 : value > max ? max : value;
        }
    }
}
