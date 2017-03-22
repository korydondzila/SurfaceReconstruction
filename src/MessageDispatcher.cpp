#include "MessageDispatcher.hpp"
#include "BaseEntity.hpp"
#include "Scene.hpp"

using std::set;

//------------------------------ Instance -------------------------------------

MessageDispatcher* MessageDispatcher::Instance()
{
	static MessageDispatcher instance;
	return &instance;
}

//----------------------------- Dispatch ---------------------------------
//
//  see description in header
//------------------------------------------------------------------------
void MessageDispatcher::Discharge(BaseEntity* pReceiver,
	const Message& message)
{
	pReceiver->HandleMsg(message);
}

//---------------------------- DispatchMessage ---------------------------
//
//  given a message, a receiver, a sender and any time delay , this function
//  routes the message to the correct agent (if no delay) or stores
//  in the message queue to be dispatched at the correct time
//------------------------------------------------------------------------
void MessageDispatcher::DispatchMsg(double  delay,
	int    sender,
	int    receiver,
	int    msg,
	void*  ExtraInfo)
{
	// Negative receiver, send to all entities
	if (receiver < 0)
	{
		for (auto entity : *Scene::Instance()->Entities())
		{
			if (sender != entity.second->ID())
			{
                DispatchMsg(delay, sender, entity.second->ID(), msg, ExtraInfo);
			}
		}
	}
	else
	{
		BaseEntity* pReceiver = Scene::Instance()->GetEntityFromID(receiver);

		//make sure the receiver is valid
		if (pReceiver == NULL)
		{
			printf("\nWarning! No Receiver with ID of %d found\n", receiver);
			printf("Message sent by ID %d\n", sender);
			return;
		}

		//create the messsage
		Message message(0, sender, receiver, msg, ExtraInfo);

		//if there is no delay, route message immediately
		if (delay <= 0.0f)
		{
			//send the telegram to the recipient
			Discharge(pReceiver, message);
		}
		else //else calculate the time when the message should be dispatched
		{
			double currentTime = glutGet(GLUT_ELAPSED_TIME);
			message.DispatchTime = currentTime + delay;

			//and put it in the queue
			PriorityQ.insert(message);
		}
	}
}

//---------------------- DispatchDelayedMsgs -------------------------
//
//  This function dispatches any telegrams with a timestamp that has
//  expired. Any dispatched telegrams are removed from the queue
//------------------------------------------------------------------------
void MessageDispatcher::DispatchDelayedMsgs()
{
	//get current time
	double currentTime = glutGet(GLUT_ELAPSED_TIME);

	//now peek at the queue to see if any telegrams need dispatching.
	//remove all telegrams from the front of the queue that have gone
	//past their sell by date
	while (!PriorityQ.empty() &&
		(PriorityQ.begin()->DispatchTime < currentTime) &&
		(PriorityQ.begin()->DispatchTime > 0))
	{
		//read the telegram from the front of the queue
		const Message& telegram = *PriorityQ.begin();

		//find the recipient
		BaseEntity* pReceiver = Scene::Instance()->GetEntityFromID(telegram.Receiver);

		//send the telegram to the recipient
		Discharge(pReceiver, telegram);

		//remove it from the queue
		PriorityQ.erase(PriorityQ.begin());
	}
}
