#include "eae_wifimessage.hh"

using namespace Stg;
using namespace std;

namespace eae
{
    WifiMessage::WifiMessage() : WifiMessageBase()
    {
        type = MSG_UNDEFINED;
        state_robot = STATE_UNDEFINED_ROBOT;
        state_ds = STATE_UNDEFINED_DS;
        id_auction = -1;
        id_robot = -1;
        id_ds = -1;
        bid = BID_INV;
        pose = Pose();
        map = NULL;
    }

    WifiMessage::WifiMessage(msg_type_t type, int id_auction, int id_robot, robot_state_t state_robot, int id_ds, ds_state_t state_ds, double bid, Pose pose)
    {
        this->type = type;
        this->id_auction = id_auction;
        this->id_robot = id_robot;
        this->state_robot = state_robot;
        this->id_ds = id_ds;
        this->state_ds = state_ds;
        this->bid = bid;
        this->pose = pose;
        this->map = NULL;
    }

    WifiMessage::WifiMessage(GridMap* map)
    {
        this->type = MSG_MAP;
        this->id_auction = -1;
        this->id_robot = -1;
        this->state_robot = STATE_UNDEFINED_ROBOT;
        this->id_ds = -1;
        this->state_ds = STATE_UNDEFINED_DS;
        this->bid = BID_INV;
        this->pose = Pose();
        this->map = map;
    }

    WifiMessage::~WifiMessage()
    {
    }

    WifiMessage::WifiMessage(const WifiMessage& toCopy) : WifiMessageBase(toCopy)
    {
        type = toCopy.type;
        state_robot = toCopy.state_robot;
        state_ds = toCopy.state_ds;
        id_auction = toCopy.id_auction;
        id_robot = toCopy.id_robot;
        id_ds = toCopy.id_ds;
        bid = toCopy.bid;
        pose = toCopy.pose;
        map = toCopy.map;
    };

    WifiMessage& WifiMessage::operator=(const WifiMessage& toCopy)
    {
        WifiMessageBase::operator=(toCopy);

        type = toCopy.type;
        state_robot = toCopy.state_robot;
        state_ds = toCopy.state_ds;
        id_auction = toCopy.id_auction;
        id_robot = toCopy.id_robot;
        id_ds = toCopy.id_ds;
        bid = toCopy.bid;
        pose = toCopy.pose;
        map = toCopy.map;

        return *this;
    };

    WifiMessage* WifiMessage::Clone()
    {
        return new WifiMessage(*this);
    }

    string WifiMessage::ToString()
    {
        char output[500];
        sprintf(output, "---------------------------------\n msg type:    %d\n robot state: %d\n ds state:    %d\n auction:     %d\n robot:       %d\n ds:          %d\n bid:         %.2f\n position:    (%.2f, %.2f, %.2f)\n---------------------------------\n", type, state_robot, state_ds, id_auction, id_robot, id_ds, bid, pose.x, pose.y, pose.a);
        return string(output);
    }

    WifiMessageRobot::WifiMessageRobot(int id_robot, robot_state_t state_robot, Pose pose) : WifiMessage(MSG_ROBOT, -1, id_robot, state_robot, -1, STATE_UNDEFINED_DS, BID_INV, pose)
    {
    }

    WifiMessageDs::WifiMessageDs(int id_ds, ds_state_t state_ds, Pose pose) : WifiMessage(MSG_DS, -1, -1, STATE_UNDEFINED_ROBOT, id_ds, state_ds, BID_INV, pose)
    {
    }

    WifiMessageFrontierAuction::WifiMessageFrontierAuction(int id_auction, int id_robot, double bid, Pose pose) : WifiMessage(MSG_FRONTIER_AUCTION, id_auction, id_robot, STATE_UNDEFINED_ROBOT, -1, STATE_UNDEFINED_DS, bid, pose)
    {
    }

    WifiMessageDsAuction::WifiMessageDsAuction(int id_auction, int id_robot, robot_state_t state_robot, int id_ds, ds_state_t state_ds, double bid, Pose pose) : WifiMessage(MSG_DS_AUCTION, id_auction, id_robot, state_robot, id_ds, state_ds, bid, pose)
    {
    }

    WifiMessageMap::WifiMessageMap(GridMap* map) : WifiMessage(map)
    {
    }
}
