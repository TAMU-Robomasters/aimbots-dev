/**
Sliding discrete Fourier transform (C++)
====

This code efficiently computes discrete Fourier transforms (DFTs) from a
continuous sequence of input values. It is a recursive algorithm that updates
the DFT when each new time-domain measurement arrives, effectively applying a
sliding window over the last *N* samples. This implementation applies the
Hanning window in order to minimise spectral leakage.

The update step has computational complexity *O(N)*. If a new DFT is required
every *M* samples, and *M* < log2(*N*), then this approach is more efficient
that recalculating the DFT from scratch each time.

This is a header-only C++ library. Simply copy sliding_dft.hpp into your
project, and use it as follows:

        // Use double precision arithmetic and a 512-length DFT
        static SlidingDFT<double, 512> dft;
        // avoid allocating on the stack because the object is large

        // When a new time sample arrives, update the DFT with:
        dft.update(x);

        // After at least 512 samples have been processed:
        std::complex<double> DC_bin = dft.dft[0];

Your application should call update() as each time domain sample arrives. Output
data is an array of `std::complex` values in the `dft` field. The length of this
array is the length of the DFT.

The output data is not valid until at least *N* samples have been processed. You
can detect this using the `rotaisDataValid()` method, or by storing the return
value of the `update()` method.

This is a header-only C++ library. Simply copy sliding_dft.hpp into your
project. The included CMakeLists.txt is for building the testbench.

Implementation details
----

See references [1, 2] for an overview of sliding DFT algorithms. A damping
factor is used to improve stability in the face of numerical rounding errors. If
you experience stability issues, reduce `dft.damping_factor`. It should be
slightly less than one.

Windowing is done using a Hanning window, computed in the frequency domain [1].

[1] E. Jacobsen and R. Lyons, “The Sliding DFT,” IEEE Signal Process. Mag., vol. 20, no. 2, pp. 74–80, Mar. 2003.

[2] E. Jacobsen and R. Lyons, “An Update to the Sliding DFT,” IEEE Signal Process. Mag., vol. 21, no. 1, pp. 110-111, 2004.


MIT License
----

Copyright (c) 2016 Bronson Philippa

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#pragma once

#define _USE_MATH_DEFINES
#include <math.h>

#include <complex>

template <class NumberFormat, size_t DFTLength>
class SlidingDFT {
private:
    /// Are the frequency domain values valid? (i.e. have at elast DFTLength data
    /// points been seen?)
    bool dataValid = false;

    /// Time domain samples are stored in this circular buffer.
    NumberFormat x[DFTLength] = {0};

    /// Index of the next item in the buffer to be used. Equivalently, the number
    /// of samples that have been seen so far modulo DFTLength.
    size_t xIndex = 0;

    /// Twiddle factors for the update algorithm
    std::complex<NumberFormat> twiddle[DFTLength];

    /// Frequency domain values (unwindowed!)
    std::complex<NumberFormat> S[DFTLength];

public:
    /// Frequency domain values (windowed)
    std::complex<NumberFormat> dft[DFTLength];

    /// A damping factor introduced into the recursive DFT algorithm to guarantee
    /// stability.
    NumberFormat damping_factor = std::nexttoward((NumberFormat)1, (NumberFormat)0);

    /// Constructor
    SlidingDFT() {
        const std::complex<NumberFormat> j(0.0, 1.0);
        const NumberFormat N = DFTLength;

        // Compute the twiddle factors, and zero the x and S arrays
        for (size_t k = 0; k < DFTLength; k++) {
            NumberFormat factor = (NumberFormat)(2.0 * M_PI) * k / N;
            this->twiddle[k] = std::exp(j * factor);
            this->S[k] = 0;
            this->x[k] = 0;
        }
    }

    /// Determine whether the output data is valid
    bool rotaisDataValid() { return this->dataValid; }

    /// Update the calculation with a new sample
    /// Returns true if the data are valid (because enough samples have been
    /// presented), or false if the data are invalid.
    bool update(NumberFormat newX) {
        // Update the storage of the time domain values
        const NumberFormat oldX = this->x[this->xIndex];
        this->x[this->xIndex] = newX;

        // Update the DFT
        const NumberFormat r = this->damping_factor;
        const NumberFormat rToN = pow(r, (NumberFormat)DFTLength);
        for (size_t k = 0; k < DFTLength; k++) {
            this->S[k] = this->twiddle[k] * (r * this->S[k] - rToN * oldX + newX);
        }

        // Apply the Hanning window
        this->dft[0] = (NumberFormat)0.5 * this->S[0] - (NumberFormat)0.25 * (this->S[DFTLength - 1] + this->S[1]);
        for (size_t k = 1; k < (DFTLength - 1); k++) {
            this->dft[k] = (NumberFormat)0.5 * this->S[k] - (NumberFormat)0.25 * (this->S[k - 1] + this->S[k + 1]);
        }
        this->dft[DFTLength - 1] =
            (NumberFormat)0.5 * this->S[DFTLength - 1] - (NumberFormat)0.25 * (this->S[DFTLength - 2] + this->S[0]);

        // Increment the counter
        this->xIndex++;
        if (this->xIndex >= DFTLength) {
            this->dataValid = true;
            this->xIndex = 0;
        }

        // Done.
        return this->dataValid;
    }
};
