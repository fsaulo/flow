#include "LPFilter.h"

#include <iostream>

double LPFilter::update(double input) {
    m_output = (1.0 - m_alpha) * input + m_alpha * m_output;
    return m_output;
}

void LPFilter::initialize(const double& value) {
    m_output = value;
}

void LPFilter::initialize(const double& value, double alpha)
{
    if (alpha < 0.0)
    m_alpha = 0.0;

    if (alpha >= 1.0)
        m_alpha = 0.99;

    m_output = value;
}

void LPFilter2d::initialize(const std::vector<double>& xy)
{
    m_filter_x.initialize(xy[0]);
    m_filter_y.initialize(xy[1]);
}

std::vector<double> LPFilter2d::update(const std::vector<double>& xy)
{
    std::vector<double> result_vect = {
        m_filter_x.update(xy[0]),
        m_filter_y.update(xy[1]),
    };

    return result_vect;
}
