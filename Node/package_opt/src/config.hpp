#pragma once

//All configurations and numerical constants can be found in this file

namespace opt::config{

    constexpr auto track_discretization_interval{15};   //todo: find optimal value

    constexpr auto track_bounds_search_narrowing_heuristic{40};   //tbd
    //this is used to narrow the search for the closest point on the other side; it assumes that the closest point
    // must be somewhere in <value - heuristic, value + heuristic>

    constexpr auto track_bounds_safety_region{0.5};   //tbd
    //minimal distance to the track bounds during optimization
}   //namespace opt::config
