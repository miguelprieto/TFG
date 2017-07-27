/*
 * low_pass.h
 */

#ifndef __LOW_PASS_H
#define __LOW_PASS_H

#include "filter.h"

class LowPassFilter : public Filter {
 public:
 LowPassFilter() : Filter() { };
 LowPassFilter(float dt) : Filter(dt) { };
    void set_cutoff_frequecy(float hz);
    float apply(float v);
 protected:
    float m_alpha, m_alpha_1;
};

class LowPassFilter2P : public LowPassFilter {
 public:
    void set_cutoff_frequecy(float hz);
    float apply(float v);
 protected:
    float m_a1, m_a2;
    float m_b0, m_b1, m_b2;
    float m_delay_element_1, m_delay_element_2;
};

#endif
