/*
 * StateMachine.cpp
 *
 *  Created on: Sep 14, 2022
 *      Author: UOttawaRocketry
 */
#include <StateMachine.h>

//----------------------------------------------------------------------------
// StateMachine
//----------------------------------------------------------------------------
StateMachine::StateMachine(unsigned char maxStates, unsigned char initialState) :
	MAX_STATES(maxStates),
	m_currentState(initialState),
	m_newState(0),
	m_eventGenerated(0),
	m_pEventData(NULL)
{
	ASSERT(MAX_STATES < EVENT_IGNORED);
}

//----------------------------------------------------------------------------
// ExternalEvent
//----------------------------------------------------------------------------
void StateMachine::ExternalEvent(unsigned char newState, const EventData &pData)
{
	if (newState != EVENT_IGNORED)
		InternalEvent(newState, pData);
}

//----------------------------------------------------------------------------
// InternalEvent
//----------------------------------------------------------------------------
void StateMachine::InternalEvent(unsigned char newState, const EventData &pData)
{
	m_eventGenerated = 1;
	m_newState = newState;
	StateEngine(pData);
}


//----------------------------------------------------------------------------
// StateEngine
//----------------------------------------------------------------------------
void StateMachine::StateEngine(const EventData &pData)
{
    const StateMapRow *pStateMap = GetStateMap();
    if (pStateMap != nullptr)
        StateEngine(pStateMap);
    else
    {
        const StateMapRowEx *pStateMapEx = GetStateMapEx();
        ASSERT(pStateMapEx != nullptr);
        StateEngine(pStateMapEx, pData);
    }
}

//----------------------------------------------------------------------------
// StateEngine
//----------------------------------------------------------------------------
void StateMachine::StateEngine(const StateMapRow *const pStateMap)
{
    // While events are being generated keep executing states
    while (m_eventGenerated)
    {
        // Error check that the new state is valid before proceeding
        ASSERT(m_newState < MAX_STATES);

        // Get the pointer from the state map
        const StateBase *state = pStateMap[m_newState].State;

        // Event used up, reset the flag
        m_eventGenerated = 0;

        // Switch to the new current state
        SetCurrentState(m_newState);

        // Execute the state action passing in event data
        ASSERT(state != nullptr);

        // we don't want to execute the StateAction because the ExecuteCurrentState
        // method will do it state->InvokeStateAction(this, pDataTemp);
    }
}

//----------------------------------------------------------------------------
// StateEngine
//----------------------------------------------------------------------------
void StateMachine::StateEngine(const StateMapRowEx *const pStateMapEx, const EventData &data)
{
	const EventData* pDataTemp = NULL;

	// While events are being generated keep executing states
	while (m_eventGenerated)
	{
		// Error check that the new state is valid before proceeding
		ASSERT(m_newState < MAX_STATES);

		// Get the pointers from the state map
		const StateBase* state = pStateMapEx[m_newState].State;
		const GuardBase* guard = pStateMapEx[m_newState].Guard;
		const EntryBase* entry = pStateMapEx[m_newState].Entry;
		const ExitBase* exit = pStateMapEx[m_currentState].Exit;

		// Copy of event data pointer
		pDataTemp = m_pEventData;

		// Event data used up, reset the pointer
		m_pEventData = NULL;

		// Event used up, reset the flag
		m_eventGenerated = 0;

		// Execute the guard condition
		int guardResult = 1;
		if (guard != NULL)
			guardResult = guard->InvokeGuardCondition(this, pDataTemp);

		// If the guard condition succeeds
		if (guardResult == 1)
		{
			// Transitioning to a new state?
			if (m_newState != m_currentState)
			{
				// Execute the state exit action on current state before switching to new state
				if (exit != NULL)
					exit->InvokeExitAction(this);

				// Execute the state entry action on the new state
				if (entry != NULL)
					entry->InvokeEntryAction(this, pDataTemp);

				// Ensure exit/entry actions didn't call InternalEvent by accident
				ASSERT(m_eventGenerated == 0);
			}

			// Switch to the new current state
			SetCurrentState(m_newState);

			// Execute the state action passing in event data
			ASSERT(state != NULL);
			state->InvokeStateAction(this, pDataTemp);
		}
		if (pDataTemp)
		{
			delete pDataTemp;
			pDataTemp = NULL;
		}
	}
}

