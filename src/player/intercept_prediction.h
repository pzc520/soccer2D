#ifndef SOCCER2D_INTERCEPT_PREDICTION_H
#define SOCCER2D_INTERCEPT_PREDICTION_H

#include <algorithm>
#include <cmath>
#include <limits>

// Geometry-only reach prediction. The caller supplies ball/player inertia
// positions and the player type's dash-distance model, so this stays usable
// by both defensive behaviors without duplicating server physics.
namespace InterceptPrediction {

template < typename Point, typename ReachCycles >
int dashCyclesToControl( const Point & player_pos,
                         const Point & ball_pos,
                         const double kickable_area,
                         ReachCycles cycles_to_reach )
{
    const double distance = std::hypot( player_pos.x - ball_pos.x,
                                        player_pos.y - ball_pos.y );
    if ( ! std::isfinite( distance ) ) return std::numeric_limits< int >::max();

    const double dash_distance = std::max( 0.0, distance - kickable_area );
    return cycles_to_reach( dash_distance );
}

template < typename Point, typename ReachCycles >
bool canReachAtCycle( const Point & player_pos,
                      const Point & ball_pos,
                      const int cycle,
                      const double kickable_area,
                      ReachCycles cycles_to_reach )
{
    return cycle >= 0
        && dashCyclesToControl( player_pos, ball_pos,
                                kickable_area, cycles_to_reach ) <= cycle;
}

template < typename BallPosition, typename PlayerPosition, typename ReachCycles >
int earliestReachCycle( const int max_cycle,
                        const double kickable_area,
                        BallPosition ball_position,
                        PlayerPosition player_position,
                        ReachCycles cycles_to_reach )
{
    for ( int cycle = 0; cycle <= max_cycle; ++cycle )
    {
        if ( canReachAtCycle( player_position( cycle ),
                             ball_position( cycle ),
                             cycle,
                             kickable_area,
                             cycles_to_reach ) )
        {
            return cycle;
        }
    }
    return max_cycle + 1;
}

} // namespace InterceptPrediction

#endif
