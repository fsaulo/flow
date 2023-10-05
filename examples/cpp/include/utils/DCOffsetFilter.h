#ifndef DC_OFFSET_FILTER_H
#define DC_OFFSET_FILTER_H

#include <deque>

class DCOffsetFilter {
public:
  DCOffsetFilter(int windowSize) : m_window_size(windowSize), m_sum(0), m_moving_sum(0) {
    m_alpha = 0.9;
    m_result = 0;
    m_sample_1 = 0;
  }

  double removeOffset(double sample) {
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

  double update(double sample) {
      // double value = removeOffset(sample);
      // m_moving_sum += value;
      // return m_moving_sum / m_window_buffer.size();
    m_result = sample - m_sample_1 + m_alpha * m_result;
    m_sample_1 = sample;
    return m_result;
  }

private:
  int m_window_size;
  double m_sum;
  double m_moving_sum;
  double m_sample_1;
  double m_result;
  double m_alpha;
  std::deque<double> m_window_buffer;
};  

#endif // DC_OFFSET_FILTER_H
