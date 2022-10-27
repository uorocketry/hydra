/*
 * HotFire.h
 *
 *  Created on: Sep 14, 2022
 *      Author: noahs
 */

#ifndef INC_HOTFIRE_H_
#define INC_HOTFIRE_H_

#include <StateMachine.h>

struct UOSMData : public EventData
{
	// create a time here like std::chrono::time_point<std::chrono::steady_clock>

};

class HotFire : public StateMachine
{
public:
	HotFire();
	void ReadyEXT();
	void StartFillingEXT();
	void AbortEXT();
    void StopFillingEXT();
    void IgnitionEXT();
    void FinalVentingEXT();
    void DoneEXT();
    void ServoControlEXT();

    void updateHotFire(UOSMData &data);
private:
	// Might need to change some to a pointer of type EventData
 	enum E_States {
         ST_INIT = 0,
         ST_WAIT_FOR_INIT,
         ST_WAIT_FOR_READY,
         ST_WAIT_FOR_FILLING,
         ST_FILLING,
         ST_WAIT_FOR_IGNITION,
         ST_IGNITION,
         ST_FULL_BURN,
         ST_FINAL_VENTING,
         ST_DONE,
         ST_ABORT_FILLING,
         ST_ABORT_BURN,
         ST_SERVO_CONTROL, // Debugging state
         ST_MAX_STATES
 	};

    // Define the state machine state functions with event data type
    // Init
    STATE_DECLARE(HotFire, Init, UOSMData)
    EXIT_DECLARE(HotFire, ExitInit)
    // WaitForInit
    ENTRY_DECLARE(HotFire, EnterWaitForInit, UOSMData)
    STATE_DECLARE(HotFire, WaitForInit, UOSMData)
    EXIT_DECLARE(HotFire, ExitWaitForInit)
    // WaitForReady
    ENTRY_DECLARE(HotFire, EnterWaitForReady, UOSMData)
    STATE_DECLARE(HotFire, WaitForReady, UOSMData)
    EXIT_DECLARE(HotFire, ExitWaitForReady)
    // WaitForFilling
    ENTRY_DECLARE(HotFire, EnterWaitForFilling, UOSMData)
    STATE_DECLARE(HotFire, WaitForFilling, UOSMData)
    EXIT_DECLARE(HotFire, ExitWaitForFilling)
    // Filling
    ENTRY_DECLARE(HotFire, EnterFilling, UOSMData)
    STATE_DECLARE(HotFire, Filling, UOSMData)
    EXIT_DECLARE(HotFire, ExitFilling)
    // WaitForIgnition
    ENTRY_DECLARE(HotFire, EnterWaitForIgnition, UOSMData)
    STATE_DECLARE(HotFire, WaitForIgnition, UOSMData)
    EXIT_DECLARE(HotFire, ExitWaitForIgnition)
    // Ignition
    ENTRY_DECLARE(HotFire, EnterIgnition, UOSMData)
    STATE_DECLARE(HotFire, Ignition, UOSMData)
    EXIT_DECLARE(HotFire, ExitIgnition)
    // FullBurn
    ENTRY_DECLARE(HotFire, EnterFullBurn, UOSMData)
    STATE_DECLARE(HotFire, FullBurn, UOSMData)
    EXIT_DECLARE(HotFire, ExitFullBurn)
    // FinalVenting
    ENTRY_DECLARE(HotFire, EnterFinalVenting, UOSMData)
    STATE_DECLARE(HotFire, FinalVenting, UOSMData)
    EXIT_DECLARE(HotFire, ExitFinalVenting)
    // Done
    ENTRY_DECLARE(HotFire, EnterDone, UOSMData)
    STATE_DECLARE(HotFire, Done, UOSMData)
    // AbortFilling
    ENTRY_DECLARE(HotFire, EnterAbortFilling, UOSMData)
    STATE_DECLARE(HotFire, AbortFilling, UOSMData)
    // AbortBurn
    ENTRY_DECLARE(HotFire, EnterAbortBurn, UOSMData)
    STATE_DECLARE(HotFire, AbortBurn, UOSMData)
    // ServoControl
    ENTRY_DECLARE(HotFire, EnterServoControl, UOSMData)
    STATE_DECLARE(HotFire, ServoControl, UOSMData)

    // state map to define state function order
 	BEGIN_STATE_MAP_EX
		STATE_MAP_ENTRY_ALL_EX(&Init, 0, 0, &ExitInit)
		STATE_MAP_ENTRY_ALL_EX(&WaitForInit, 0, &EnterWaitForInit, &ExitWaitForInit)
		STATE_MAP_ENTRY_ALL_EX(&WaitForReady, 0, &EnterWaitForReady, &ExitWaitForReady)
		STATE_MAP_ENTRY_ALL_EX(&WaitForFilling, 0, &EnterWaitForFilling, &ExitWaitForFilling)
		STATE_MAP_ENTRY_ALL_EX(&Filling, 0, &EnterFilling, &ExitFilling)
		STATE_MAP_ENTRY_ALL_EX(&WaitForIgnition, 0, &EnterWaitForIgnition, &ExitWaitForIgnition)
		STATE_MAP_ENTRY_ALL_EX(&Ignition, 0, &EnterIgnition, &ExitIgnition)
		STATE_MAP_ENTRY_ALL_EX(&FullBurn, 0, &EnterFullBurn, &ExitFullBurn)
		STATE_MAP_ENTRY_ALL_EX(&FinalVenting, 0, &EnterFinalVenting, &ExitFinalVenting)
		STATE_MAP_ENTRY_ALL_EX(&Done, 0, &EnterDone, 0)
		STATE_MAP_ENTRY_ALL_EX(&AbortFilling, 0, &EnterAbortFilling, 0)
		STATE_MAP_ENTRY_ALL_EX(&AbortBurn, 0, &EnterAbortBurn, 0)
		STATE_MAP_ENTRY_ALL_EX(&ServoControl, 0, &EnterServoControl, 0)
 	END_STATE_MAP_EX
};
#endif /* INC_HOTFIRE_H_ */
