// ReSharper disable InconsistentNaming
namespace ImageSharp.Tests.Common
{
    using System;
    using System.Runtime.CompilerServices;

    using Xunit;

    using static TestStructs;

    public unsafe class Buffer2DTests
    {
        // ReSharper disable once ClassNeverInstantiated.Local
        private class Assert : Xunit.Assert
        {
            public static void SpanPointsTo<T>(BufferSpan<T> span, Buffer<T> buffer, int bufferOffset = 0)
                where T : struct
            {
                ref T actual = ref span.DangerousGetPinnableReference();
                ref T expected = ref Unsafe.Add(ref buffer[0], bufferOffset);

                Assert.True(Unsafe.AreSame(ref expected, ref actual), "span does not point to the expected position");
            }
        }

        [Theory]
        [InlineData(7, 42)]
        [InlineData(1025, 17)]
        public void Construct(int width, int height)
        {
            using (Buffer2D<Foo> buffer = new Buffer2D<Foo>(width, height))
            {
                Assert.Equal(width, buffer.Width);
                Assert.Equal(height, buffer.Height);
                Assert.Equal(width * height, buffer.Length);
            }
        }

        [Theory]
        [InlineData(7, 42)]
        [InlineData(1025, 17)]
        public void Construct_FromExternalArray(int width, int height)
        {
            Foo[] array = new Foo[width * height + 10];
            using (Buffer2D<Foo> buffer = new Buffer2D<Foo>(array, width, height))
            {
                Assert.Equal(width, buffer.Width);
                Assert.Equal(height, buffer.Height);
                Assert.Equal(width * height, buffer.Length);
            }
        }


        [Fact]
        public void CreateClean()
        {
            for (int i = 0; i < 100; i++)
            {
                using (Buffer2D<int> buffer = Buffer2D<int>.CreateClean(42, 42))
                {
                    for (int j = 0; j < buffer.Length; j++)
                    {
                        Assert.Equal(0, buffer.Array[j]);
                        buffer.Array[j] = 666;
                    }
                }
            }
        }

        [Theory]
        [InlineData(7, 42, 0)]
        [InlineData(7, 42, 10)]
        [InlineData(17, 42, 41)]
        public void GetRowSpanY(int width, int height, int y)
        {
            using (Buffer2D<Foo> buffer = new Buffer2D<Foo>(width, height))
            {
                BufferSpan<Foo> span = buffer.GetRowSpan(y);

                Assert.Equal(width * y, span.Start);
                Assert.Equal(width, span.Length);
                Assert.SpanPointsTo(span, buffer, width * y);
            }
        }

        [Theory]
        [InlineData(7, 42, 0, 0)]
        [InlineData(7, 42, 3, 10)]
        [InlineData(17, 42, 0, 41)]
        public void GetRowSpanXY(int width, int height, int x, int y)
        {
            using (Buffer2D<Foo> buffer = new Buffer2D<Foo>(width, height))
            {
                BufferSpan<Foo> span = buffer.GetRowSpan(x, y);

                Assert.Equal(width * y + x, span.Start);
                Assert.Equal(width - x, span.Length);
                Assert.SpanPointsTo(span, buffer, width * y + x);
            }
        }

        [Theory]
        [InlineData(42, 8, 0, 0)]
        [InlineData(400, 1000, 20, 10)]
        [InlineData(99, 88, 98, 87)]
        public void Indexer(int width, int height, int x, int y)
        {
            using (Buffer2D<Foo> buffer = new Buffer2D<Foo>(width, height))
            {
                Foo[] array = buffer.Array;

                ref Foo actual = ref buffer[x, y];

                ref Foo expected = ref array[y * width + x];

                Assert.True(Unsafe.AreSame(ref expected, ref actual));
            }
        }
    }
}