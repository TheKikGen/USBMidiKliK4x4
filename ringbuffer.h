/*
__ __| |           |  /_) |     ___|             |           |
  |   __ \   _ \  ' /  | |  / |      _ \ __ \   |      _` | __ \   __|
  |   | | |  __/  . \  |   <  |   |  __/ |   |  |     (   | |   |\__ \
 _|  _| |_|\___| _|\_\_|_|\_\\____|\___|_|  _| _____|\__,_|_.__/ ____/
  -----------------------------------------------------------------------------
 *  RING BUFFER TEMPLATE CLASS.
 *
 * Original work from Francois Best for the Arduino MIDI Library
 * license    MIT - Copyright (c) 2016 Francois Best
 *
 * Modified by TheKikGen Labs for the UsbMidiKliK4x4 project.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 *
 */

#pragma once


template<typename DataType, int Size>
class RingBuffer
{
public:
     RingBuffer();
    ~RingBuffer();

public:
    int available() const;

public:
    void write(DataType inData) volatile ;
    void write(const DataType* inData, int inSize) volatile;
    void flush() volatile;

public:
    DataType read() volatile;
    void readBytes(DataType* outData, int inSize) volatile;

private:
    volatile DataType mData[Size];
    volatile DataType* mWriteHead;
    volatile DataType* mReadHead;
};

template<typename DataType, int Size>
RingBuffer<DataType, Size>::RingBuffer()
    : mWriteHead(mData)
    , mReadHead(mData)
{
    memset((void*)mData, DataType(0), Size * sizeof(DataType)) ;
}

template<typename DataType, int Size>
RingBuffer<DataType, Size>::~RingBuffer()
{
}

// -----------------------------------------------------------------------------

template<typename DataType, int Size>
int RingBuffer<DataType, Size>::available() const
{
    if (mReadHead == mWriteHead)
    {
        return 0;
    }
    else if (mWriteHead > mReadHead)
    {
        return int(mWriteHead - mReadHead);
    }
    else
    {
        return int(mWriteHead - mData) + Size - int(mReadHead - mData);
    }
}

// -----------------------------------------------------------------------------

template<typename DataType, int Size>
void RingBuffer<DataType, Size>::write(DataType inData) volatile
{
    *mWriteHead++ = inData;
    if (mWriteHead >= mData + Size)
    {
        mWriteHead = mData;
    }
}

template<typename DataType, int Size>
void RingBuffer<DataType, Size>::write(const DataType* inData, int inSize) volatile
{
    for (int i = 0; i < inSize; ++i)
    {
        write(inData[i]);
    }
}

template<typename DataType, int Size>
void RingBuffer<DataType, Size>::flush() volatile
{
    memset(mData, DataType(0), Size * sizeof(DataType));
    mReadHead  = mData;
    mWriteHead = mData;
}

// -----------------------------------------------------------------------------

template<typename DataType, int Size>
DataType RingBuffer<DataType, Size>::read() volatile
{
    const DataType data = *mReadHead++;
    if (mReadHead >= mData + Size)
    {
        mReadHead = mData;
    }
    return data;
}

template<typename DataType, int Size>
void RingBuffer<DataType, Size>::readBytes(DataType* outData, int inSize) volatile
{
    for (int i = 0; i < inSize; ++i)
    {
        outData[i] = read();
    }
}
