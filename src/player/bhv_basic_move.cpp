// -*-c++-*-

/*
 *Copyright:

 Cyrus2D
 Modified by Omid Amini, Nader Zare
 
 Gliders2d
 Modified by Mikhail Prokopenko, Peter Wang

 Copyright (C) Hidehisa AKIYAMA

 This code is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 3, or (at your option)
 any later version.

 This code is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this code; see the file COPYING.  If not, write to
 the Free Software Foundation, 675 Mass Ave, Cambridge, MA 02139, USA.

 *EndCopyright:
 */

/////////////////////////////////////////////////////////////////////

#ifdef HAVE_CONFIG_H
#include <config.h>
#endif

#include "bhv_basic_move.h"
#include "strategy.h"
#include "bhv_basic_tackle.h"
#include "neck_offensive_intercept_neck.h"
#include "bhv_basic_block.h"
#include "intercept_prediction.h"

#include "basic_actions/basic_actions.h"
#include "basic_actions/body_go_to_point.h"
#include "basic_actions/body_intercept.h"
#include "basic_actions/neck_turn_to_ball_or_scan.h"
#include "basic_actions/neck_scan_field.h"
#include "basic_actions/neck_turn_to_low_conf_teammate.h"

#include <rcsc/player/player_agent.h>
#include <rcsc/player/debug_client.h>
#include <rcsc/player/intercept_table.h>

#include <rcsc/common/logger.h>
#include <rcsc/common/server_param.h>

#include "neck_offensive_intercept_neck.h"
#include <rcsc/player/soccer_intention.h>
#include "bhv_unmark.h"

using namespace rcsc;

/*-------------------------------------------------------------------*/
/*!

 */
bool
Bhv_BasicMove::execute( PlayerAgent * agent )
{
    dlog.addText( Logger::TEAM,
                  __FILE__": Bhv_BasicMove" );

    const WorldModel & wm = agent->world();

    //-----------------------------------------------
    // tackle
    // G2d: tackle probability
    double doTackleProb = 0.8;
    if (wm.ball().pos().x < 0.0)
    {
      doTackleProb = 0.55;
    }

    if ( Bhv_BasicTackle( doTackleProb, 80.0 ).execute( agent ) )
    {
        return true;
    }

    /*--------------------------------------------------------*/
    // chase ball
    const int self_min = wm.interceptTable().selfStep();
    const int mate_min = wm.interceptTable().teammateStep();
    const int opp_min = wm.interceptTable().opponentStep();

    const Vector2D target_point = Strategy::i().getPosition( wm.self().unum() );

    // G2d: to retrieve opp team name
    // C2D: Helios 18 Tune removed -> replace with BNN
    // bool helios2018 = false;
    // if (wm.opponentTeamName().find("HELIOS2018") != std::string::npos)
	// helios2018 = true;
//    if (std::min(self_min, mate_min) < opp_min){
//
//    }else{
//        if (Bhv_BasicBlock().execute(agent)){
//            return true;
//        }
//    }
    // G2d: role
    int role = Strategy::i().roleNumber( wm.self().unum() );

    // G2D: blocking

    Vector2D ball = wm.ball().pos();

    double block_d = -10.0;

    Vector2D me = wm.self().pos();
    Vector2D homePos = target_point;
    int num = role;

    if (ball.x < block_d)
    {
        double block_line = -38.0;

    //  if (helios2018)
    //      block_line = -48.0;

    // acknowledgement: fragments of Marlik-2012 code
        if( (num == 2 || num == 3) && homePos.x < block_line &&
            !( num == 2 && ball.x < -46.0 && ball.y > -18.0 && ball.y < -6.0 &&
               opp_min <= 3 && opp_min <= mate_min && ball.dist(me) < 9.5 ) &&
            !( num == 3 && ball.x < -46.0 && ball.y <  18.0 && ball.y >  6.0  &&
               opp_min <= 3 && opp_min <= mate_min && ball.dist(me) < 9.5 ) ) // do not block in this situation
        {
            // do nothing
        }
        else if( (num == 2 || num == 3) && fabs(wm.ball().pos().y) > 22.0 )
        {
            // do nothing
        }
        else if (Bhv_BasicBlock().execute(agent)){
            return true;
        }

    } // end of block


    // The intercept table already compares reach cycles in normal play.
    // Near our goal, estimate the other players' reach cycles to decide whether
    // this defender should be the second player covering the ball.
    const bool danger = ball.x < -36.0;
    const bool second_defender = danger && role >= 2 && role <= 5;
    bool may_press = true;
    if ( second_defender )
    {
        int faster_mates = 0;
        const int search_limit = std::min( self_min, 30 );
        const auto & player_steps = wm.interceptTable().playerMap();
        for ( const PlayerObject * tm : wm.teammatesFromSelf() )
        {
            if ( faster_mates >= 2 ) break;
            if ( tm->isGhost() || tm->isTackling()
                 || tm->posCount() > 5 )
            {
                continue;
            }

            // Prefer librcsc's full interception simulation. Estimate locally
            // only when the table has no entry for this visible teammate.
            const auto found = player_steps.find( tm );
            int reach_cycle = search_limit + 1;
            if ( found != player_steps.end() )
            {
                reach_cycle = found->second;
            }
            else if ( tm->playerTypePtr() )
            {
                reach_cycle = InterceptPrediction::earliestReachCycle(
                    search_limit,
                    tm->playerTypePtr()->kickableArea(),
                    [&]( int cycle ) { return wm.ball().inertiaPoint( cycle ); },
                    [&]( int cycle ) { return tm->inertiaPoint( cycle ); },
                    [&]( double distance ) {
                        return tm->playerTypePtr()->cyclesToReachDistance( distance );
                    } );
            }

            // Ties are broken by uniform number.
            if ( reach_cycle <= search_limit
                 && ( reach_cycle < self_min
                      || ( reach_cycle == self_min
                           && tm->unum() < wm.self().unum() ) ) )
            {
                ++faster_mates;
            }
        }
        may_press = faster_mates < 2;
    }

    int pressing;
    if ( role >= 2 && role <= 5 ) pressing = danger ? 12 : 5;
    else if ( role >= 6 && role <= 8 ) pressing = danger ? 10 : 8;
    else pressing = danger ? 8 : 4;

    if ( ! wm.kickableTeammate()
         && may_press
         && self_min <= mate_min + ( second_defender ? 1 : 0 )
         && ( self_min <= 2 || self_min < opp_min + pressing ) )
    {
        dlog.addText( Logger::TEAM,
                      __FILE__": intercept" );
        Body_Intercept().execute( agent );
        agent->setNeckAction( new Neck_OffensiveInterceptNeck() );

        return true;
    }



// G2D : offside trap
    double first = 0.0, second = 0.0;
    const auto t3_end = wm.teammatesFromSelf().end();
        for ( auto it = wm.teammatesFromSelf().begin();
              it != t3_end;
              ++it )
        {
            double x = (*it)->pos().x;
            if ( x < second )
            {
                second = x;
                if ( second < first )
                {
                    std::swap( first, second );
                }
            }
        }

   double buf1 = 3.5;
   double buf2 = 4.5;

   if( me.x < -37.0 && opp_min < mate_min &&
       (homePos.x > -37.5 || wm.ball().inertiaPoint(opp_min).x > -36.0 ) &&
         second + buf1 > me.x && wm.ball().pos().x > me.x + buf2)
   {
        Body_GoToPoint( rcsc::Vector2D( me.x + 15.0, me.y ),
                        0.5, ServerParam::i().maxDashPower(), // maximum dash power
                        ServerParam::i().maxDashPower(),     // preferred dash speed
                        2,                                  // preferred reach cycle
                        true,                              // save recovery
                        5.0 ).execute( agent );

        if (wm.kickableOpponent() && wm.ball().distFromSelf() < 12.0) // C2D
            agent->setNeckAction(new Neck_TurnToBall());
        else
            agent->setNeckAction(new Neck_TurnToBallOrScan(4)); // C2D
        return true;
   }

    if (std::min(self_min, mate_min) < opp_min){
        if (Bhv_Unmark().execute(agent))
            return true;
    }
    const double dash_power = Strategy::get_normal_dash_power( wm );

    double dist_thr = wm.ball().distFromSelf() * 0.1;
    if ( dist_thr < 1.0 ) dist_thr = 1.0;

    dlog.addText( Logger::TEAM,
                  __FILE__": Bhv_BasicMove target=(%.1f %.1f) dist_thr=%.2f",
                  target_point.x, target_point.y,
                  dist_thr );

    agent->debugClient().addMessage( "BasicMove%.0f", dash_power );
    agent->debugClient().setTarget( target_point );
    agent->debugClient().addCircle( target_point, dist_thr );

    if ( ! Body_GoToPoint( target_point, dist_thr, dash_power
                           ).execute( agent ) )
    {
        Body_TurnToBall().execute( agent );
    }

    if ( wm.kickableOpponent()
         && wm.ball().distFromSelf() < 18.0 )
    {
        agent->setNeckAction( new Neck_TurnToBall() );
    }
    else
    {
        agent->setNeckAction( new Neck_TurnToBallOrScan( 0 ) );
    }

    return true;
}
