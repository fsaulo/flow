#ifndef LK_ESTIMATOR_H
#define LK_ESTIMATOR_H

#include <string>

#include "params.h"

void ClearFile(std::string& filename);
void EstimatorCallback(const std::string& pipeline);
void MavlinkMessageCallback(void);

#endif // LK_ESTIMATOR_H
