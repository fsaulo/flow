#ifndef DC_OFFSET_FILTER_H
#define DC_OFFSET_FILTER_H

#include <deque>

class DCOffsetFilter {
public:
  DCOffsetFilter(int windowSize) : windowSize_(windowSize), sum_(0), movingSum_(0) {}

  double removeOffset(double sample) {
    if (windowSize_ == 0) 
        return sample;

    // Add the new sample to the sum
    sum_ += sample;

    // Add the sample to the window buffer
    windowBuffer_.push_back(sample);

    // If the window buffer is larger than the specified window size,
    // remove the oldest sample from the sum and the buffer
    if (windowBuffer_.size() > windowSize_) {
      sum_ -= windowBuffer_.front();
      windowBuffer_.pop_front();
    }

    // Calculate the average and subtract it from the sample
    double average = sum_ / windowBuffer_.size();
    return sample - average;
  }

  double update(double sample) {
      double value = removeOffset(sample);
      movingSum_ += value;
      return movingSum_ / windowBuffer_.size();
  }

private:
  int windowSize_;
  double sum_;
  double movingSum_;
  std::deque<double> windowBuffer_;
};  

#endif // DC_OFFSET_FILTER_H
