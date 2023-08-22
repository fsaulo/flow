#include <iostream>
#include <deque>

class DCOffsetRemover {
public:
  DCOffsetRemover(int windowSize) : windowSize_(windowSize), sum_(0) {}

  double RemoveOffset(double sample) {
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

private:
  int windowSize_;
  double sum_;
  std::deque<double> windowBuffer_;
};  
