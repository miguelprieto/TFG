/*
 * low_pass.cpp
 */

#include <math.h>
#include "low_pass.h"

#define TWO_PI   6.28318530717959
#define M_PI     (TWO_PI / 2)

void LowPassFilter::set_cutoff_frequecy(float hz)
{
    float rc = 1.0 / (TWO_PI * hz);
    m_alpha = m_dt / (rc + m_dt);
    m_alpha_1 = 1 - m_alpha;
}


float LowPassFilter::apply(float val)
{
    float v = val * m_alpha + m_prev_value * m_alpha_1;
    m_prev_value = v;
    return v;
}

// ------------------------------------------------------------------

void LowPassFilter2P::set_cutoff_frequecy(float hz)
{
    float fr = m_dt * hz; // sample_freq/_cutoff_freq;
    float omega = tan(M_PI/fr);
    float c = 1.0f + 2.0f * cosf(M_PI / 4.0f) * omega + omega * omega;
    m_b0 = omega * omega / c;
    m_b1 = 2.0f * m_b0;
    m_b2 = m_b0;
    m_a1 = 2.0f * (omega * omega - 1.0f) / c;
    m_a2 = (1.0f - 2.0f * cosf(M_PI / 4.0f) * omega + omega * omega) / c;

    m_delay_element_1 = m_delay_element_2 = 0;
}


float LowPassFilter2P::apply(float v)
{
    float delay_element_0 = v - m_delay_element_1 * m_a1 - m_delay_element_2 * m_a2;
    float output = delay_element_0 * m_b0 + m_delay_element_1 * m_b1 + m_delay_element_2 * m_b2;

    m_delay_element_2 = m_delay_element_1;
    m_delay_element_1 = delay_element_0;

    return output;
}
