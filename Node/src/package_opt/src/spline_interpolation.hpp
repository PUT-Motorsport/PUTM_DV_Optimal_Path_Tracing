#pragma once

#include <vector>
#include <utility>

namespace opt {

std::pair<std::vector<double>, std::vector<double>> track_spline_interpolation(std::vector<double> const &x, std::vector<double> const &y);

}   //namespace opt
