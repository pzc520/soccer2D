//
// Created by nader on 2022-04-05.
//

#ifndef BHV_BASIC_BLOCK_H
#define BHV_BASIC_BLOCK_H

#include <rcsc/geom/vector_2d.h>
#include <rcsc/player/soccer_action.h>
#include <utility>
#include <vector>

namespace rcsc {
class AbstractPlayerObject;
}

class Bhv_BasicBlock
    : public rcsc::SoccerBehavior
{
public:
    Bhv_BasicBlock()
    {
    }

    bool execute(rcsc::PlayerAgent *agent);

private:
    std::vector<const rcsc::AbstractPlayerObject *> get_blockers(
        const rcsc::PlayerAgent *agent, const rcsc::Vector2D &ball_inertia);
    std::pair<const rcsc::AbstractPlayerObject *, rcsc::Vector2D> get_best_blocker(
        int opp_min, rcsc::Vector2D ball_inertia,
        const std::vector<const rcsc::AbstractPlayerObject *> &tm_blockers);
    rcsc::AngleDeg dribble_direction_detector(rcsc::Vector2D dribble_pos);
    static int last_block_cycle;
    static rcsc::Vector2D last_block_pos;
};

#endif
