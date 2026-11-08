#include "../src/player/intercept_prediction.h"

#include <cassert>
#include <cmath>
#include <limits>

struct Point {
    double x;
    double y;
};

int main()
{
    const auto stationary = []( int ) { return Point{ 0.0, 0.0 }; };
    const auto one_meter_per_cycle = []( double distance ) {
        return static_cast< int >( std::ceil( distance ) );
    };

    // A moving ball is met earlier than its starting position would suggest.
    const auto approaching_ball = []( int cycle ) {
        return Point{ 10.0 - cycle, 0.0 };
    };
    assert( InterceptPrediction::earliestReachCycle( 10, 0.0,
                                                     approaching_ball,
                                                     stationary,
                                                     one_meter_per_cycle ) == 5 );

    // A ball moving away requires more time; the horizon is respected.
    const auto departing_ball = []( int cycle ) {
        return Point{ 5.0 + 0.5 * cycle, 0.0 };
    };
    assert( InterceptPrediction::earliestReachCycle( 10, 0.0,
                                                     departing_ball,
                                                     stationary,
                                                     one_meter_per_cycle ) == 10 );
    assert( InterceptPrediction::earliestReachCycle( 5, 0.0,
                                                     departing_ball,
                                                     stationary,
                                                     one_meter_per_cycle ) == 6 );

    // A faster player can reach the ball before a nearer, slower player.
    const auto fixed_ball = []( int ) { return Point{ 10.0, 0.0 }; };
    const auto near_player = []( int ) { return Point{ 8.0, 0.0 }; };
    const auto far_player = []( int ) { return Point{ 0.0, 0.0 }; };
    const auto slow_reach = []( double distance ) {
        return static_cast< int >( std::ceil( distance / 0.1 ) );
    };
    const auto fast_reach = []( double distance ) {
        return static_cast< int >( std::ceil( distance / 2.0 ) );
    };
    assert( InterceptPrediction::earliestReachCycle( 20, 0.0,
                                                     fixed_ball, near_player,
                                                     slow_reach ) == 20 );
    assert( InterceptPrediction::earliestReachCycle( 20, 0.0,
                                                     fixed_ball, far_player,
                                                     fast_reach ) == 5 );

    // Existing player momentum changes the reachable cycle as well.
    const auto coasting_player = []( int cycle ) {
        return Point{ 0.5 * cycle, 0.0 };
    };
    assert( InterceptPrediction::earliestReachCycle( 10, 0.0,
                                                     fixed_ball, coasting_player,
                                                     one_meter_per_cycle ) == 7 );

    // The control radius allows an earlier interception.
    assert( InterceptPrediction::earliestReachCycle( 10, 1.0,
                                                     fixed_ball, far_player,
                                                     one_meter_per_cycle ) == 9 );

    // Invalid observations must not be treated as an immediate interception.
    const Point unknown_ball{ std::numeric_limits< double >::quiet_NaN(), 0.0 };
    assert( ! InterceptPrediction::canReachAtCycle( far_player( 0 ),
                                                    unknown_ball, 10, 1.0,
                                                    one_meter_per_cycle ) );
}
