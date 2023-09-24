#include "DCOffsetFilter.h"

void DCOffsetFilter::initialize(const double value)
{
    m_window_buffer.clear();
    m_sum = value;
}

double DCOffsetFilter::update(const double sample) {
    if (m_window_size == 0)
        return sample;

    // Add the new sample to the sum
    m_sum += sample;

    // Add the sample to the window buffer
    m_window_buffer.push_back(sample);

    // If the window buffer is larger than the specified window size,
    // remove the oldest sample from the sum and the buffer
    if (m_window_buffer.size() > m_window_size) {
      m_sum -= m_window_buffer.front();
      m_window_buffer.pop_front();
    }

    // Calculate the average and subtract it from the sample
    double average = m_sum / m_window_buffer.size();
    return sample - average;
}

void DCOffsetFilter2d::initialize(const std::vector<double>& xy)
{
    m_dc_removal_x.initialize(xy[0]);
    m_dc_removal_y.initialize(xy[1]);
}

std::vector<double> DCOffsetFilter2d::update(const std::vector<double>& xy)
{
    std::vector<double> result = {
        m_dc_removal_x.update(xy[0]),
        m_dc_removal_y.update(xy[1])
    };

    return result;
}
