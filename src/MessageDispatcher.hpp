#ifndef MESSAGE_DISPATCHER_H
#define MESSAGE_DISPATCHER_H
//------------------------------------------------------------------------
//
//  Name:   MessageDispatcher.hpp
//
//  Desc:   A message dispatcher. Manages messages of the type Message.
//          Instantiated as a singleton.
//
//  Author: Mat Buckland 2002 (fup@ai-junkie.com)
//
//------------------------------------------------------------------------

#include "includes/includes.hpp"
#include "Message.hpp"

class BaseEntity;

//to make code easier to read
const double SEND_MSG_IMMEDIATELY = 0.0f;
const int   NO_ADDITIONAL_INFO = 0;

//to make life easier...
#define Dispatch MessageDispatcher::Instance()

class MessageDispatcher
{
private:

    //a std::set is used as the container for the delayed messages
    //because of the benefit of automatic sorting and avoidance
    //of duplicates. Messages are sorted by their dispatch time.
    std::set<Message> PriorityQ;

    //this method is utilized by DispatchMessage or DispatchDelayedMessages.
    //This method calls the message handling member function of the receiving
    //entity, pReceiver, with the newly created telegram
    void Discharge(BaseEntity* pReceiver, const Message& msg);

    MessageDispatcher() {}

    //copy ctor and assignment should be private
    MessageDispatcher(const MessageDispatcher&);
    MessageDispatcher& operator=(const MessageDispatcher&) {}

public:

    //this class is a singleton
    static MessageDispatcher* Instance();

    //send a message to another agent. Receiving agent is referenced by ID.
    void DispatchMsg(double  delay,
                     int    sender,
                     int    receiver,
                     int    msg,
                     void*  ExtraInfo);

    //send out any delayed messages. This method is called each time through
    //the main game loop.
    void DispatchDelayedMsgs();
};

#endif
