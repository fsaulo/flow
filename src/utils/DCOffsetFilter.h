#ifndef DC_OFFSET_FILTER_H
#define DC_OFFSET_FILTER_H

#include <deque>
#include <vector>

class DCOffsetFilter
{
private:
  int m_window_size;
  double m_sum;
  double m_moving_sum;
  std::deque<double> m_window_buffer;

public:
  DCOffsetFilter(int window_size) : m_window_size(window_size), m_sum(0), m_moving_sum(0) {
  }

  void initialize(const double value);
  double update(const double sample);
};  

class DCOffsetFilter2d : public DCOffsetFilter
{
private:
  DCOffsetFilter m_dc_removal_x;
  DCOffsetFilter m_dc_removal_y;

public:
  DCOffsetFilter2d(int window_size) : DCOffsetFilter(window_size), m_dc_removal_x(window_size), m_dc_removal_y(window_size) {
  }

  void initialize(const std::vector<double>& xy);
  std::vector<double> update(const std::vector<double>& xy);

};
#endif // DC_OFFSET_FILTER_H
