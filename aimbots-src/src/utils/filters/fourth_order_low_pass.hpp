#pragma once

#include <array>
#include <vector>

namespace src::Utils::Filters {


//TODO: make acceleration reset when kalman is reset
//From matlab

struct SOSSection {
    std::array<double,3> b{}, a{};
    std::array<double,2> xh{}, yh{}; // history for this section
};

struct YawVelocityFourthOrderLPF {
    YawVelocityFourthOrderLPF()
    {

        for (size_t i = 0; i < sos.size(); ++i) {
            SOSSection s;
            // unpack
            s.b = { sos[i][0], sos[i][1], sos[i][2]};
            s.a = { sos[i][3], sos[i][4], sos[i][5] }; // a0 should be 1
            sections.push_back(s);
        }
    }

    double processSample(double x) {
        double acc = x * gain;
        for (auto &sec : sections) {
            // Direct‑form II transposed for a single second‑order:
            double yn = sec.b[0]*acc 
                      + sec.xh[0];
            sec.xh[0] = sec.b[1]*acc - sec.a[1]*yn + sec.xh[1];
            sec.xh[1] = sec.b[2]*acc - sec.a[2]*yn;
            acc = yn;
        };
        return acc;
    }

private:
    double gain = 8.9849e-07;
    static constexpr auto sos = std::array<std::array<double,6>,2>{{
        // section1: {b0, b1, b2, a0, a1, a2}
        { 1.0000,    2.0000,    1.0000,    1.0000,   -1.8866,    0.8903 },
        // section2:
        { 1.0000,    2.0000,    1.0000,    1.0000,  -1.9492,    0.9531 }
    }};
    std::vector<SOSSection> sections;
};


struct YawAccelerationFourthOrderLPF {
    YawAccelerationFourthOrderLPF()
    {

        for (size_t i = 0; i < sos.size(); ++i) {
            SOSSection s;
            // unpack
            s.b = { sos[i][0], sos[i][1], sos[i][2]};
            s.a = { sos[i][3], sos[i][4], sos[i][5] }; // a0 should be 1
            sections.push_back(s);
        }
    }

    double processSample(double x) {
        double acc = x * gain;
        for (auto &sec : sections) {
            // Direct‑form II transposed for a single second‑order:
            double yn = sec.b[0]*acc 
                      + sec.xh[0];
            sec.xh[0] = sec.b[1]*acc - sec.a[1]*yn + sec.xh[1];
            sec.xh[1] = sec.b[2]*acc - sec.a[2]*yn;
            acc = yn;
        };
        return acc;
    }

private:
    double gain = 1.5106e-05;
    static constexpr auto sos = std::array<std::array<double,6>,2>{{
        // section1: {b0, b1, b2, a0, a1, a2}
        { 1.0000, 1.0000, 0, 1.0000, -0.9510, 0 }
        // section2:
        { 1.0000, 2.0000, 1.0000, 1.0000, -1.9485, 0.9510 }
    }};
    std::vector<SOSSection> sections;
};

struct targetDistanceFourthOrderLPF {
    targetDistanceFourthOrderLPF()
    {

        for (size_t i = 0; i < sos.size(); ++i) {
            SOSSection s;
            // unpack
            s.b = { sos[i][0], sos[i][1], sos[i][2]};
            s.a = { sos[i][3], sos[i][4], sos[i][5] }; // a0 should be 1
            sections.push_back(s);
        }
    }

    double processSample(double x) {
        double acc = x * gain;
        for (auto &sec : sections) {
            // Direct‑form II transposed for a single second‑order:
            double yn = sec.b[0]*acc 
                      + sec.xh[0];
            sec.xh[0] = sec.b[1]*acc - sec.a[1]*yn + sec.xh[1];
            sec.xh[1] = sec.b[2]*acc - sec.a[2]*yn;
            acc = yn;
        };
        return acc;
    }

private:
    double gain = 1.5106e-05;
    static constexpr auto sos = std::array<std::array<double,6>,2>{{
        // section1: {b0, b1, b2, a0, a1, a2}
        { 1.0000, 1.0000, 0, 1.0000, -0.9510, 0 },
        // section2:
        { 1.0000, 2.0000, 1.0000, 1.0000, -1.9485, 0.9510 }
    }};
    std::vector<SOSSection> sections;
};
}  // namespace src::Utils::Filters
