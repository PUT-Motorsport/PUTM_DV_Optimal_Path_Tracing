#pragma once

#include <vector>
#include <utility>  //pair
#include <span>
#include <type_traits>

namespace opt::path {
    
    template<typename T>
    [[nodiscard]] std::pair<std::vector<T>, std::vector<T>> 
    rrt_path_smoothing(std::span<T> rrt_path_x, std::span<T> rrt_path_y) noexcept {
        static_assert(std::is_floating_point_v<T>);
        
    };

}   //namespace opt::path