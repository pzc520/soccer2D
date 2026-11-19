// -*-c++-*-

/*!
  \file bhv_strict_check_shoot.cpp
  \brief strict checked shoot behavior using ShootGenerator
*/

/*
 *Copyright:

 Copyright (C) Hidehisa AKIYAMA

 This code is free software; you can redistribute it and/or
 modify it under the terms of the GNU Lesser General Public
 License as published by the Free Software Foundation; either
 version 3 of the License, or (at your option) any later version.

 This library is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 Lesser General Public License for more details.

 You should have received a copy of the GNU Lesser General Public
 License along with this library; if not, write to the Free Software
 Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

 *EndCopyright:
 */

/////////////////////////////////////////////////////////////////////

#ifdef HAVE_CONFIG_H
#include <config.h>
#endif

#include "bhv_strict_check_shoot.h"

#include "shoot_generator.h"

#include "basic_actions/neck_turn_to_goalie_or_scan.h"
#include "basic_actions/neck_turn_to_point.h"
#include "basic_actions/body_smart_kick.h"

#include <rcsc/player/player_agent.h>
#include <rcsc/player/debug_client.h>
#include <rcsc/common/logger.h>

#include <algorithm>
#include <array>
#include <vector>

using namespace rcsc;

/*-------------------------------------------------------------------*/
/*!

*/
bool
Bhv_StrictCheckShoot::execute( PlayerAgent * agent )
{
    const WorldModel & wm = agent->world();

    if ( ! wm.self().isKickable() )
    {
        std::cerr << __FILE__ << ": " << __LINE__
                  << " not ball kickable!"
                  << std::endl;
        dlog.addText( Logger::ACTION,
                      __FILE__":  not kickable" );
        return false;
    }

    const ShootGenerator::Container & cont = ShootGenerator::instance().courses( wm );

    // update
    if ( cont.empty() )
    {
        dlog.addText( Logger::SHOOT,
                      __FILE__": no shoot course" );
        return false;
    }

    const ShootGenerator::ScoreCmp score_cmp;
    const auto best_shoot = std::min_element( cont.begin(), cont.end(), score_cmp );

    if ( ! wm.ball().velValid() )
    {
        // Keep the existing ball-control fallback when a kick cannot be simulated.
        if ( Body_SmartKick( best_shoot->target_point_,
                             best_shoot->first_ball_speed_,
                             best_shoot->first_ball_speed_ * 0.99,
                             3 ).execute( agent ) )
        {
            agent->setNeckAction( new Neck_TurnToGoalieOrScan( -1 ) );
            return true;
        }
        return false;
    }

    const auto try_shot = [&]( const ShootGenerator::Course & course ) {
        const double one_step_speed
            = KickTable::calc_max_velocity( ( course.target_point_ - wm.ball().pos() ).th(),
                                            wm.self().kickRate(),
                                            wm.ball().vel() ).r();

        dlog.addText( Logger::SHOOT,
                      __FILE__": shoot[%d] target=(%.2f, %.2f) first_speed=%f one_kick_max_speed=%f",
                      course.index_,
                      course.target_point_.x,
                      course.target_point_.y,
                      course.first_ball_speed_,
                      one_step_speed );

        const bool direct = one_step_speed > course.first_ball_speed_ * 0.99
                      && Body_SmartKick( course.target_point_,
                                         one_step_speed,
                                         one_step_speed * 0.99 - 0.0001,
                                         1 ).tryKick( agent );
        if ( ! direct
             && ! Body_SmartKick( course.target_point_,
                                  course.first_ball_speed_,
                                  course.first_ball_speed_ * 0.99,
                                  3 ).tryKick( agent ) )
        {
            return false;
        }

        agent->debugClient().addMessage( "Shoot" );
        agent->debugClient().setTarget( course.target_point_ );
        if ( direct )
        {
            agent->setNeckAction( new Neck_TurnToGoalieOrScan( -1 ) );
            agent->debugClient().addMessage( "Force1Step" );
        }
        else if ( ! doTurnNeckToShootPoint( agent, course.target_point_ ) )
        {
            agent->setNeckAction( new Neck_TurnToGoalieOrScan( -1 ) );
        }
        return true;
    };

    if ( try_shot( *best_shoot ) )
    {
        return true;
    }

    // Try at most four other directions in score order.
    constexpr std::size_t max_alternatives = 4;
    std::vector<const ShootGenerator::Course *> alternatives;
    alternatives.reserve( cont.size() - 1 );
    for ( const auto & course : cont )
    {
        if ( ! course.target_point_.equals( best_shoot->target_point_ ) )
        {
            alternatives.push_back( &course );
        }
    }
    std::stable_sort( alternatives.begin(), alternatives.end(),
                      [&]( const auto *lhs, const auto *rhs ) {
                          return score_cmp( *lhs, *rhs );
                      } );

    std::array<Vector2D, max_alternatives> attempted_targets;
    auto attempted_end = attempted_targets.begin();
    for ( const auto *course : alternatives )
    {
        if ( attempted_end == attempted_targets.end() )
        {
            break;
        }
        if ( std::any_of( attempted_targets.begin(), attempted_end,
                         [&]( const Vector2D & target ) {
                             return target.equals( course->target_point_ );
                         } ) )
        {
            continue;
        }
        *attempted_end++ = course->target_point_;
        if ( try_shot( *course ) )
        {
            dlog.addText( Logger::SHOOT,
                          __FILE__": alternate shot selected target=(%.2f, %.2f)",
                          course->target_point_.x, course->target_point_.y );
            return true;
        }
    }

    dlog.addText( Logger::SHOOT,
                  __FILE__": failed" );
    return false;
}


/*-------------------------------------------------------------------*/
/*!

 */
bool
Bhv_StrictCheckShoot::doTurnNeckToShootPoint( PlayerAgent * agent,
                                              const Vector2D & shoot_point )
{
    const double angle_buf = 10.0; // Magic Number

    if ( ! agent->effector().queuedNextCanSeeWithTurnNeck( shoot_point, angle_buf ) )
    {
        dlog.addText( Logger::TEAM,
                      __FILE__": cannot look the shoot point(%.2f %.2f)",
                      shoot_point.x,
                      shoot_point.y );
        return false;
    }

#if 0
    const WorldModel & wm = agent->world();
    if ( wm.seeTime() == wm.time() )
    {
        double current_width = wm.self().viewWidth().width();
        AngleDeg target_angle = ( shoot_point - wm.self().pos() ).th();
        double angle_diff = ( target_angle - wm.self().face() ).abs();

        if ( angle_diff < current_width*0.5 - angle_buf )
        {
            dlog.addText( Logger::TEAM,
                          __FILE__": already seen. width=%.1f, diff=%.1f. shoot point(%.2f %.2f)",
                          current_width,
                          angle_diff,
                          shoot_point.x,
                          shoot_point.y );
            return false;
        }
    }
#endif

    dlog.addText( Logger::TEAM,
                  __FILE__": turn_neck to the shoot point(%.2f %.2f)",
                  shoot_point.x,
                  shoot_point.y );
    agent->debugClient().addMessage( "Shoot:NeckToTarget" );

    agent->setNeckAction( new Neck_TurnToPoint( shoot_point ) );

    return true;
}
