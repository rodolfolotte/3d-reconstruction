#include "Color.h"

namespace std {
	bool operator <(const vtkColor3ub &c1, const vtkColor3ub &c2)
	{
		if (c1[0] < c2[0])
			return true;
		else if (c1[0] == c2[0]) {
			if (c1[1] < c2[1])
				return true;
			else if (c1[1] == c2[1]) {
				if (c1[2] < c2[2])
					return true;
			}
		}

		return false;
	}
}