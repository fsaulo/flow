#ifndef LP_FILTER_H
#define LP_FILTER_H

#include <vector>

class LPFilter
{
private:
    double m_alpha;
    double m_output;

public:
    LPFilter(double alpha = 0.1) : m_alpha(alpha), m_output(0.0) {
    }

    void initialize(const double& initial_value);
    void initialize(const double& initial_value, double alpha);
    double update(double input);
};

class LPFilter2d : public LPFilter
{
private:
    LPFilter m_filter_x;
    LPFilter m_filter_y;

public:
    LPFilter2d(double alpha = 0.1) : m_filter_x(alpha), m_filter_y(alpha) {
    }

    void initialize(const std::vector<double>& xy);
    std::vector<double> update(const std::vector<double>& xy);
};

#endif // LP_FILTER_H
