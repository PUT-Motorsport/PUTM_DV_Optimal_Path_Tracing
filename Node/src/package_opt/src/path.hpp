#pragma once

#include <vector>
#include <utility>  //pair
#include <type_traits>

namespace opt::path {
    
    template<typename T>
    [[nodiscard]] std::pair<std::vector<T>, std::vector<T>> 
    rrt_path_smoothing(std::vector<T> const &rrt_path_x, std::vector<T> const &rrt_path_y) noexcept {
        static_assert(std::is_floating_point_v<T>);
        
    };

}   //namespace opt::path