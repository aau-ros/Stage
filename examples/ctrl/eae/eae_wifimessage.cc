#include "eae.hh"

using namespace Stg;
using namespace std;

namespace eae
{
    WifiMessage::WifiMessage() : WifiMessageBase()
    {

    }

    WifiMessage::~WifiMessage()
    {

    }

    WifiMessage::WifiMessage(const WifiMessage& toCopy) : WifiMessageBase(toCopy)
    {
        //gpose = toCopy.gpose;
    };

    WifiMessage& WifiMessage::operator=(const WifiMessage& toCopy)
    {
        WifiMessageBase::operator=(toCopy);
        //gpose = toCopy.gpose;
        return *this;
    };

    WifiMessage* WifiMessage::Clone()
    {
        return new WifiMessage(*this);
    }


}
