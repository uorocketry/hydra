/*
 * StateMachine.h
 *
 *  Created on: Sep 14, 2022
 *      Author: noahs
 */

#ifndef INC_STATEMACHINE_H_
#define INC_STATEMACHINE_H_

#include <utils_assert.h>

class EventData
{
public:
	virtual ~EventData() {};
};

typedef EventData NoEventData;

struct StateStruct;

class StateMachine;

class StateBase
{
public:
	/// Called by the state machine engine to execute a state action. If a guard condition
	/// exists and it evaluates to false, the state action will not execute.
	/// @param[in] sm - A state machine instance.
	/// @param[in] data - The event data.
	virtual void InvokeStateAction(StateMachine* sm, const EventData* data) const = 0;
};


/// @brief StateAction takes three template arguments: A state machine class,
/// a state function event data type (derived from EventData) and a state machine
/// member function pointer.
template <class SM, class Data, void (SM::*Func)(const Data*)>
class StateAction : public StateBase
{
public:
	/// @see StateBase::InvokeStateAction
	virtual void InvokeStateAction(StateMachine* sm, const EventData* data) const
	{
		// Downcast the state machine and event data to the correct derived type
		SM* derivedSM = static_cast<SM*>(sm);

		// If this check fails, there is a mismatch between the STATE_DECLARE
		// event data type and the data type being sent to the state function.
		// For instance, given the following state defintion:
		//    STATE_DECLARE(MyStateMachine, MyStateFunction, MyEventData)
		// The following internal event transition is valid:
		//    InternalEvent(ST_MY_STATE_FUNCTION, new MyEventData());
		// This next internal event is not valid and causes the assert to fail:
		//    InternalEvent(ST_MY_STATE_FUNCTION, new OtherEventData());
		const Data* derivedData = dynamic_cast<const Data*>(data);

		// Call the state function
		(derivedSM->*Func)(derivedData);
	}
};

/// @brief Abstract guard base class that all guards classes inherit from.
class GuardBase
{
public:
	/// Called by the state machine engine to execute a guard condition action. If guard
	/// condition evaluates to TRUE the state action is executed. If FALSE, no state transition
	/// is performed.
	/// @param[in] sm - A state machine instance.
	/// @param[in] data - The event data.
	/// @return Returns TRUE if no guard condition or the guard condition evaluates to TRUE.
	virtual int InvokeGuardCondition(StateMachine* sm, const EventData* data) const = 0;
};

/// @brief GuardCondition takes three template arguments: A state machine class,
/// a state function event data type (derived from EventData) and a state machine
/// member function pointer.
template <class SM, class Data, int (SM::*Func)(const Data*)>
class GuardCondition : public GuardBase
{
public:
	virtual int InvokeGuardCondition(StateMachine* sm, const EventData* data) const
	{
		SM* derivedSM = static_cast<SM*>(sm);
		const Data* derivedData = dynamic_cast<const Data*>(data);

		// Call the guard function
		return (derivedSM->*Func)(derivedData);
	}
};

/// @brief Abstract entry base class that all entry classes inherit from.
class EntryBase
{
public:
	/// Called by the state machine engine to execute a state entry action. Called when
	/// entering a state.
	/// @param[in] sm - A state machine instance.
	/// @param[in] data - The event data.
	virtual void InvokeEntryAction(StateMachine* sm, const EventData* data) const = 0;
};

/// @brief EntryAction takes three template arguments: A state machine class,
/// a state function event data type (derived from EventData) and a state machine
/// member function pointer.
template <class SM, class Data, void (SM::*Func)(const Data*)>
class EntryAction : public EntryBase
{
public:
	virtual void InvokeEntryAction(StateMachine* sm, const EventData* data) const
	{
		SM* derivedSM = static_cast<SM*>(sm);
		const Data* derivedData = dynamic_cast<const Data*>(data);

		// Call the entry function
		(derivedSM->*Func)(derivedData);
	}
};

/// @brief Abstract exit base class that all exit classes inherit from.
class ExitBase
{
public:
	/// Called by the state machine engine to execute a state exit action. Called when
	/// leaving a state.
	/// @param[in] sm - A state machine instance.
	virtual void InvokeExitAction(StateMachine* sm) const = 0;
};

/// @brief ExitAction takes two template arguments: A state machine class and
/// a state machine member function pointer.
template <class SM, void (SM::*Func)(void)>
class ExitAction : public ExitBase
{
public:
	virtual void InvokeExitAction(StateMachine* sm) const
	{
		SM* derivedSM = static_cast<SM*>(sm);

		// Call the exit function
		(derivedSM->*Func)();
	}
};

/// @brief A structure to hold a single row within the state map.
struct StateMapRow
{
	const StateBase* const State;
};

struct StateMapRowEx
{
	const StateBase* const State;
	const GuardBase* const Guard;
	const EntryBase* const Entry;
	const ExitBase* const Exit;
};

class StateMachine
{
public:
	enum {EVENT_IGNORED = 0xFE, CANNOT_HAPPEN};
	StateMachine(unsigned char maxStates, unsigned char initialState = 0);
	virtual ~StateMachine() {}
	unsigned char GetCurrentState() {return m_currentState;}
	unsigned char GetMaxStates() {return MAX_STATES;}
protected:
	void ExternalEvent(unsigned char newState, const EventData &pData);
	void InternalEvent(unsigned char newState, const EventData &pData);
private:
	const unsigned char MAX_STATES;
	unsigned char m_currentState;
	unsigned char m_newState;
	int m_eventGenerated;
	const EventData* m_pEventData;
	virtual const StateMapRow* GetStateMap() = 0;
	virtual const StateMapRowEx* GetStateMapEx() = 0;
	void SetCurrentState(unsigned char newState) {m_currentState = newState;}
	void StateEngine(const EventData &pData);
    void StateEngine(const StateMapRow *pStateMap);
    void StateEngine(const StateMapRowEx *pStateMapEx, const EventData &data);
};

typedef void (StateMachine::*StateFunc)(EventData *);
struct StateStruct
{
	StateFunc pStateFunc;
};

#define STATE_DECLARE(stateMachine, stateName, eventData) \
	void ST_##stateName(const eventData*); \
	StateAction<stateMachine, eventData, &stateMachine::ST_##stateName> stateName;

#define STATE_DEFINE(stateMachine, stateName, eventData) \
	void stateMachine::ST_##stateName(const eventData* data)

#define GUARD_DECLARE(stateMachine, guardName, eventData) \
	int GD_##guardName(const eventData*); \
	GuardCondition<stateMachine, eventData, &stateMachine::GD_##guardName> guardName;

#define GUARD_DEFINE(stateMachine, guardName, eventData) \
	int stateMachine::GD_##guardName(const eventData* data)

#define ENTRY_DECLARE(stateMachine, entryName, eventData) \
	void EN_##entryName(const eventData*); \
	EntryAction<stateMachine, eventData, &stateMachine::EN_##entryName> entryName;

#define ENTRY_DEFINE(stateMachine, entryName, eventData) \
	void stateMachine::EN_##entryName(const eventData* data)

#define EXIT_DECLARE(stateMachine, exitName) \
	void EX_##exitName(void); \
	ExitAction<stateMachine, &stateMachine::EX_##exitName> exitName;

#define EXIT_DEFINE(stateMachine, exitName) \
	void stateMachine::EX_##exitName(void)

#define BEGIN_TRANSITION_MAP \
    static const unsigned char TRANSITIONS[] = {\

#define TRANSITION_MAP_ENTRY(entry)\
    entry,

#define END_TRANSITION_MAP \
    }\
	;\
	NoEventData _data;\
	ASSERT(GetCurrentState() < ST_MAX_STATES); \
    ExternalEvent(TRANSITIONS[GetCurrentState()], _data); \
	ASSERT((sizeof(TRANSITIONS)/sizeof(unsigned char)) == ST_MAX_STATES);

#define PARENT_TRANSITION(state) \
	if (GetCurrentState() >= ST_MAX_STATES && \
		GetCurrentState() < GetMaxStates()) { \
		ExternalEvent(state); \
		return; }

#define BEGIN_STATE_MAP \
	private:\
	virtual const StateMapRowEx* GetStateMapEx() { return NULL; }\
	virtual const StateMapRow* GetStateMap() {\
		static const StateMapRow STATE_MAP[] = {

#define STATE_MAP_ENTRY(stateName)\
	stateName,

#define END_STATE_MAP \
    }; \
	ASSERT((sizeof(STATE_MAP)/sizeof(StateMapRow)) == ST_MAX_STATES); \
	return &STATE_MAP[0]; }

#define BEGIN_STATE_MAP_EX \
	private:\
	virtual const StateMapRow* GetStateMap() { return NULL; }\
	virtual const StateMapRowEx* GetStateMapEx() {\
		static const StateMapRowEx STATE_MAP[] = {

#define STATE_MAP_ENTRY_EX(stateName)\
	{ stateName, 0, 0, 0 },

#define STATE_MAP_ENTRY_ALL_EX(stateName, guardName, entryName, exitName)\
	{ stateName, guardName, entryName, exitName },

#define END_STATE_MAP_EX \
    }; \
	ASSERT((sizeof(STATE_MAP)/sizeof(StateMapRowEx)) == ST_MAX_STATES); \
   return &STATE_MAP[0]; }

#endif /* INC_STATEMACHINE_H_ */
