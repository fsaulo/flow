#ifndef LK_ESTIMATOR_H
#define LK_ESTIMATOR_H

#include <iostream>

#include "params.h"

void ClearFile(std::string& filename);
void EstimatorCallback(const std::string& pipeline);
void MavMessageCallback(void);

#endif // LK_ESTIMATOR_H
