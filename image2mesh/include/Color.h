#pragma once

#include <map>
#include <vtkColor.h>

namespace std {
	bool operator <(const vtkColor3ub &c1, const vtkColor3ub &c2);
}

// color -> class number
typedef std::map<vtkColor3ub, int> ColorMap;
const int UNKNOWN_CLASS = -1;