#include "swarm_wifimessage.hh"

using namespace Stg;
using namespace std;

namespace swarm
{
    WifiMessage::WifiMessage() : WifiMessageBase()
    {
        type = MSG_UNDEFINED;
        state_robot = STATE_UNDEFINED_ROBOT;
        id_robot = -1;
        id_ds = -1;
        pose = Pose();
    }

    WifiMessage::WifiMessage(msg_type_t type, int id_robot, robot_state_t state_robot, int id_ds, Pose pose)
    {
        this->type = type;
        this->id_robot = id_robot;
        this->state_robot = state_robot;
        this->id_ds = id_ds;
        this->pose = pose;
    }

    WifiMessage::~WifiMessage()
    {
    }

    WifiMessage::WifiMessage(const WifiMessage& toCopy) : WifiMessageBase(toCopy)
    {
        type = toCopy.type;
        state_robot = toCopy.state_robot;
        id_robot = toCopy.id_robot;
        id_ds = toCopy.id_ds;
        pose = toCopy.pose;
    };

    WifiMessage& WifiMessage::operator=(const WifiMessage& toCopy)
    {
        WifiMessageBase::operator=(toCopy);

        type = toCopy.type;
        state_robot = toCopy.state_robot;
        id_robot = toCopy.id_robot;
        id_ds = toCopy.id_ds;
        pose = toCopy.pose;

        return *this;
    };

    WifiMessage* WifiMessage::Clone()
    {
        return new WifiMessage(*this);
    }

    string WifiMessage::ToString()
    {
        char output[500];
        sprintf(output, "---------------------------------\n msg type:    %d\n robot state: %d\n robot:       %d\n ds:          %d\n position:    (%.2f, %.2f, %.2f)\n---------------------------------\n", type, state_robot, id_robot, id_ds, pose.x, pose.y, pose.a);
        return string(output);
    }

    WifiMessageRobot::WifiMessageRobot(int id_robot, robot_state_t state_robot, Pose pose) : WifiMessage(MSG_ROBOT, id_robot, state_robot, -1, pose)
    {
    }

    WifiMessageDs::WifiMessageDs(int id_ds, Pose pose) : WifiMessage(MSG_DS, -1, STATE_UNDEFINED_ROBOT, id_ds, pose)
    {
    }
}
