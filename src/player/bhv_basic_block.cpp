// -*-c++-*-

/*
    Cyrus2D
    Modified by Nader Zare, Omid Amini.
*/

#ifdef HAVE_CONFIG_H
#include <config.h>
#endif
#include "bhv_basic_block.h"
#include "strategy.h"
#include "bhv_basic_tackle.h"
#include "neck_offensive_intercept_neck.h"
#include "intercept_prediction.h"

#include "basic_actions/body_turn_to_point.h"
#include "basic_actions/neck_turn_to_ball_or_scan.h"
#include <rcsc/player/player_agent.h>
#include <rcsc/player/intercept_table.h>
#include <rcsc/common/logger.h>
#include <rcsc/common/server_param.h>

#include "basic_actions/basic_actions.h"
#include "basic_actions/body_go_to_point.h"
#include "basic_actions/body_intercept.h"

#include <algorithm>
#include <array>

#define DEBUG_BLOCK
using namespace rcsc;

int Bhv_BasicBlock::last_block_cycle = -1;
Vector2D Bhv_BasicBlock::last_block_pos = Vector2D::INVALIDATED;

bool Bhv_BasicBlock::execute(PlayerAgent *agent)
{
    const WorldModel &wm = agent->world();
    const int self_min = wm.interceptTable().selfStep();
    const int mate_min = wm.interceptTable().teammateStep();
    const int opp_min = wm.interceptTable().opponentStep();
    if ( ! wm.kickableTeammate()
         && self_min < InterceptPrediction::unreachable_cycle
         && self_min <= mate_min
         && ( self_min <= 3 || self_min < opp_min + 3 ) )
    {
        dlog.addText( Logger::TEAM,
                      __FILE__": intercept" );
        Body_Intercept().execute( agent );
        agent->setNeckAction( new Neck_OffensiveInterceptNeck() );

        return true;
    }

    if (opp_min >= InterceptPrediction::unreachable_cycle)
    {
        last_block_cycle = -1;
        return false;
    }

    const Vector2D ball_inertia = wm.ball().inertiaPoint(opp_min);
    const ServerParam &server = ServerParam::i();
    if (!InterceptPrediction::isBallWithinPitch(ball_inertia, server.pitchHalfLength(),
                                                server.pitchHalfWidth(), server.ballSize()))
    {
        last_block_cycle = -1;
        return false;
    }

    const auto tm_blockers = get_blockers(agent, ball_inertia);
    if (std::find(tm_blockers.begin(), tm_blockers.end(), &wm.self()) == tm_blockers.end())
    {
        last_block_cycle = -1;
        return false;
    }

    const auto best_blocker_target = get_best_blocker(opp_min, ball_inertia, tm_blockers);
    if (best_blocker_target.first != &wm.self())
    {
        last_block_cycle = -1;
        return false;
    }
    Vector2D target_point = best_blocker_target.second;
    double safe_dist = 2;
    if (wm.self().pos().dist(target_point) > 15)
        safe_dist = 5;
    if (last_block_pos.isValid() && last_block_cycle > wm.time().cycle() - 5 && target_point.dist(last_block_pos) < safe_dist)
    {
        target_point = last_block_pos;
    }else{
        last_block_cycle = wm.time().cycle();
        last_block_pos = target_point;
    }

    dlog.addText(Logger::TEAM,
                 __FILE__ ": Bhv_BasicBlock target=(%.1f %.1f)",
                 target_point.x, target_point.y);

    agent->debugClient().addMessage("BasicBlock%.0f", 100.0);
    agent->debugClient().setTarget(target_point);

    if (!Body_GoToPoint(target_point,
                        0.5,
                        100,
                        -1,
                        100,
                        false,
                        25,
                        1.0,
                        false)
             .execute(agent))
    {
        Body_TurnToPoint(target_point).execute(agent);
    }

    if (wm.kickableOpponent() && wm.ball().distFromSelf() < 18.0)
    {
        agent->setNeckAction(new Neck_TurnToBall());
    }
    else
    {
        agent->setNeckAction(new Neck_TurnToBallOrScan(0));
    }

    return true;
}

std::vector<const AbstractPlayerObject *>
Bhv_BasicBlock::get_blockers(const PlayerAgent *agent, const Vector2D &ball_inertia)
{
    const WorldModel &wm = agent->world();
    double tm_defense_line_x = 0.0;
    for (int unum = 2; unum <= 5; ++unum)
    {
        const auto *defender = wm.ourPlayer(unum);
        if (defender && defender->unum() > 0)
        {
            tm_defense_line_x = std::min(tm_defense_line_x, defender->pos().x);
        }
    }

    std::vector<const AbstractPlayerObject *> tm_blockers;
    for (const auto *tm : wm.ourPlayers())
    {
        if (tm->unum() == Unum_Unknown
            || tm->isGhost() || tm->goalie() || tm->isTackling()
            || !tm->playerTypePtr() || tm->pos().dist(ball_inertia) > 40)
        {
            continue;
        }

        const Vector2D tm_home_pos = Strategy::i().getPosition(tm->unum());
        if (tm_home_pos.dist(ball_inertia) > 40
            || (tm->unum() <= 5 && ball_inertia.x > -30
                && ball_inertia.x > tm_home_pos.x + 10
                && ball_inertia.x > tm_defense_line_x + 10))
        {
            continue;
        }

        tm_blockers.push_back(tm);
        #ifdef DEBUG_BLOCK
        dlog.addText(Logger::BLOCK, "- tm %d is add as blocker", tm->unum());
        #endif
    }

    return tm_blockers;
}

std::pair<const AbstractPlayerObject *, Vector2D>
Bhv_BasicBlock::get_best_blocker(
    int opp_min, Vector2D ball_inertia,
    const std::vector<const AbstractPlayerObject *> &tm_blockers)
{
    const ServerParam &server = ServerParam::i();
    const double dribble_speed = 0.7;
    #ifdef DEBUG_BLOCK
    dlog.addText(Logger::BLOCK, "=====get best blocker=====");
    #endif
    for (int cycle = opp_min + 1; cycle <= opp_min + 40; cycle += 1)
    {
        AngleDeg dribble_dir = dribble_direction_detector(ball_inertia);
        ball_inertia += Vector2D::polar2vector(dribble_speed, dribble_dir);
        if (!InterceptPrediction::isBallWithinPitch(ball_inertia, server.pitchHalfLength(),
                                                    server.pitchHalfWidth(), server.ballSize()))
        {
            break;
        }

        #ifdef DEBUG_BLOCK
        dlog.addText(Logger::BLOCK, "## id=%d, ball_pos=(%.1f, %.1f)", cycle, ball_inertia.x, ball_inertia.y);
        dlog.addCircle(Logger::BLOCK, ball_inertia, 0.5, 255, 0, 0, false);
        char num[8];
        snprintf( num, 8, "%d", cycle );
        dlog.addMessage(Logger::BLOCK, ball_inertia + Vector2D(0, 1), num);
        #endif
        const AbstractPlayerObject *best_blocker = nullptr;
        int best_dash_step = 0;
        for (const auto *tm : tm_blockers)
        {
            const auto *player_type = tm->playerTypePtr();
            const Vector2D tm_pos = tm->inertiaPoint(cycle);
            const int dash_step = InterceptPrediction::dashCyclesToControl(
                tm_pos, ball_inertia,
                player_type->kickableArea(),
                [player_type]( double distance ) {
                    return player_type->cyclesToReachDistance( distance );
                } );
            if ( dash_step <= cycle )
            {
                // The world model lists self first, so candidate order cannot
                // break ties consistently across teammates.
                if (!best_blocker
                    || InterceptPrediction::hasBlockPriority(
                        dash_step, tm->unum(), best_dash_step, best_blocker->unum()))
                {
                    best_blocker = tm;
                    best_dash_step = dash_step;
                }
            }
            #ifdef DEBUG_BLOCK
            else
            {
                dlog.addText(Logger::BLOCK, "$$$$ tm=%d, block_step=%d, can not block", tm->unum(), dash_step);
            }
            #endif
        }
        if (best_blocker)
        {
            #ifdef DEBUG_BLOCK
            dlog.addText(Logger::BLOCK, "$$$$ tm=%d, block_step=%d, can block", best_blocker->unum(), best_dash_step);
            dlog.addCircle(Logger::BLOCK, ball_inertia, 0.5, 0, 0, 255, false);
            dlog.addLine(Logger::BLOCK, ball_inertia, best_blocker->inertiaPoint(cycle));
            #endif
            return {best_blocker, ball_inertia};
        }
    }

    return {nullptr, Vector2D::INVALIDATED};
}

AngleDeg Bhv_BasicBlock::dribble_direction_detector(Vector2D dribble_pos)
{
    static const auto directions = [] {
        std::array<Vector2D, 36> offsets;
        for (std::size_t i = 0; i < offsets.size(); ++i)
        {
            offsets[i] = Vector2D::polar2vector(10.0, AngleDeg(-180.0 + 10.0 * i));
        }
        return offsets;
    }();

    const ServerParam &server = ServerParam::i();
    std::size_t best_index = 0;
    double best_score = -1e9;
    for (std::size_t i = 0; i < directions.size(); ++i)
    {
        const Vector2D target = dribble_pos + directions[i];
        if (target.absX() > server.pitchHalfLength() || target.absY() > server.pitchHalfWidth())
        {
            continue;
        }

        const double score = -target.x + std::max(0.0, 40 - target.dist(Vector2D(-50, 0)));
        if (score > best_score)
        {
            best_score = score;
            best_index = i;
        }
    }
    return AngleDeg(-180.0 + 10.0 * best_index);
}
