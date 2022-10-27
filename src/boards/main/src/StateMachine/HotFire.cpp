/*
 * HotFire.cpp
 *
 *  Created on: Sep 14, 2022
 *      Author: UOttawaRocketry
 */
#include <HotFire.h>
#include <utils_assert.h>
#include <driver_init.h>

HotFire::HotFire() : StateMachine(ST_MAX_STATES, 0)
{
	NoEventData data;
    InternalEvent(ST_INIT, data); // acts the same as enterNewState();
}

void HotFire::updateHotFire(UOSMData &data) {
	uint8_t update[] = "Update State Machine Data\r\n";
    ioU0->write(ioU0, (uint8_t *)&update, sizeof(update));
}

// StartFilling external event
void HotFire::ReadyEXT()
{
    BEGIN_TRANSITION_MAP                          // - Current State -
    TRANSITION_MAP_ENTRY(EVENT_IGNORED)           // ST_INIT
        TRANSITION_MAP_ENTRY(EVENT_IGNORED)       // ST_WAIT_FOR_INIT
        TRANSITION_MAP_ENTRY(ST_WAIT_FOR_FILLING) // ST_WAIT_FOR_READY
        TRANSITION_MAP_ENTRY(EVENT_IGNORED)       // ST_WAIT_FOR_FILLING
        TRANSITION_MAP_ENTRY(EVENT_IGNORED)       // ST_FILLING
        TRANSITION_MAP_ENTRY(EVENT_IGNORED)       // ST_WAIT_FOR_IGNITION
        TRANSITION_MAP_ENTRY(EVENT_IGNORED)       // ST_IGNITION
        TRANSITION_MAP_ENTRY(EVENT_IGNORED)       // ST_FULL_BURN
        TRANSITION_MAP_ENTRY(EVENT_IGNORED)       // ST_FINAL_VENTING
        TRANSITION_MAP_ENTRY(EVENT_IGNORED)       // ST_DONE
        TRANSITION_MAP_ENTRY(EVENT_IGNORED)       // ST_ABORT_FILLING
        TRANSITION_MAP_ENTRY(EVENT_IGNORED)       // ST_ABORT_BURN
        TRANSITION_MAP_ENTRY(EVENT_IGNORED)       // ST_SERVO_CONTROL
		END_TRANSITION_MAP
}

// StartFilling external event
void HotFire::StartFillingEXT()
{
    BEGIN_TRANSITION_MAP                    // - Current State -
    TRANSITION_MAP_ENTRY(EVENT_IGNORED)     // ST_INIT
        TRANSITION_MAP_ENTRY(EVENT_IGNORED) // ST_WAIT_FOR_INIT
        TRANSITION_MAP_ENTRY(EVENT_IGNORED) // ST_WAIT_FOR_READY
        TRANSITION_MAP_ENTRY(ST_FILLING)    // ST_WAIT_FOR_FILLING
        TRANSITION_MAP_ENTRY(EVENT_IGNORED) // ST_FILLING
        TRANSITION_MAP_ENTRY(EVENT_IGNORED) // ST_WAIT_FOR_IGNITION
        TRANSITION_MAP_ENTRY(EVENT_IGNORED) // ST_IGNITION
        TRANSITION_MAP_ENTRY(EVENT_IGNORED) // ST_FULL_BURN
        TRANSITION_MAP_ENTRY(EVENT_IGNORED) // ST_FINAL_VENTING
        TRANSITION_MAP_ENTRY(EVENT_IGNORED) // ST_DONE
        TRANSITION_MAP_ENTRY(ST_FILLING)    // ST_ABORT_FILLING
        TRANSITION_MAP_ENTRY(EVENT_IGNORED) // ST_ABORT_BURN
        TRANSITION_MAP_ENTRY(EVENT_IGNORED) // ST_SERVO_CONTROL
        END_TRANSITION_MAP
}

// Abort external event
void HotFire::AbortEXT()
{
    BEGIN_TRANSITION_MAP                       // - Current State -
    TRANSITION_MAP_ENTRY(EVENT_IGNORED)        // ST_INIT
        TRANSITION_MAP_ENTRY(EVENT_IGNORED)    // ST_WAIT_FOR_INIT
        TRANSITION_MAP_ENTRY(EVENT_IGNORED)    // ST_WAIT_FOR_READY
        TRANSITION_MAP_ENTRY(ST_ABORT_FILLING) // ST_WAIT_FOR_FILLING
        TRANSITION_MAP_ENTRY(ST_ABORT_FILLING) // ST_FILLING
        TRANSITION_MAP_ENTRY(ST_ABORT_FILLING) // ST_WAIT_FOR_IGNITION
        TRANSITION_MAP_ENTRY(ST_ABORT_BURN)    // ST_IGNITION
        TRANSITION_MAP_ENTRY(ST_ABORT_BURN)    // ST_FULL_BURN
        TRANSITION_MAP_ENTRY(EVENT_IGNORED)    // ST_FINAL_VENTING
        TRANSITION_MAP_ENTRY(EVENT_IGNORED)    // ST_DONE
        TRANSITION_MAP_ENTRY(EVENT_IGNORED)    // ST_ABORT_FILLING
        TRANSITION_MAP_ENTRY(EVENT_IGNORED)    // ST_ABORT_BURN
        TRANSITION_MAP_ENTRY(EVENT_IGNORED)    // ST_SERVO_CONTROL
        END_TRANSITION_MAP
}

// StopFilling external event
void HotFire::StopFillingEXT()
{
    BEGIN_TRANSITION_MAP                           // - Current State -
    TRANSITION_MAP_ENTRY(EVENT_IGNORED)            // ST_INIT
        TRANSITION_MAP_ENTRY(EVENT_IGNORED)        // ST_WAIT_FOR_INIT
        TRANSITION_MAP_ENTRY(EVENT_IGNORED)        // ST_WAIT_FOR_READY
        TRANSITION_MAP_ENTRY(EVENT_IGNORED)        // ST_WAIT_FOR_FILLING
        TRANSITION_MAP_ENTRY(ST_WAIT_FOR_IGNITION) // ST_FILLING
        TRANSITION_MAP_ENTRY(EVENT_IGNORED)        // ST_WAIT_FOR_IGNITION
        TRANSITION_MAP_ENTRY(EVENT_IGNORED)        // ST_IGNITION
        TRANSITION_MAP_ENTRY(EVENT_IGNORED)        // ST_FULL_BURN
        TRANSITION_MAP_ENTRY(EVENT_IGNORED)        // ST_FINAL_VENTING
        TRANSITION_MAP_ENTRY(EVENT_IGNORED)        // ST_DONE
        TRANSITION_MAP_ENTRY(EVENT_IGNORED)        // ST_ABORT_FILLING
        TRANSITION_MAP_ENTRY(EVENT_IGNORED)        // ST_ABORT_BURN
        TRANSITION_MAP_ENTRY(EVENT_IGNORED)        // ST_SERVO_CONTROL
        END_TRANSITION_MAP
}

// Ignition external event
void HotFire::IgnitionEXT()
{
    BEGIN_TRANSITION_MAP                    // - Current State -
    TRANSITION_MAP_ENTRY(EVENT_IGNORED)     // ST_INIT
        TRANSITION_MAP_ENTRY(EVENT_IGNORED) // ST_WAIT_FOR_INIT
        TRANSITION_MAP_ENTRY(EVENT_IGNORED) // ST_WAIT_FOR_READY
        TRANSITION_MAP_ENTRY(EVENT_IGNORED) // ST_WAIT_FOR_FILLING
        TRANSITION_MAP_ENTRY(EVENT_IGNORED) // ST_FILLING
        TRANSITION_MAP_ENTRY(ST_IGNITION)   // ST_WAIT_FOR_IGNITION
        TRANSITION_MAP_ENTRY(EVENT_IGNORED) // ST_IGNITION
        TRANSITION_MAP_ENTRY(EVENT_IGNORED) // ST_FULL_BURN
        TRANSITION_MAP_ENTRY(EVENT_IGNORED) // ST_FINAL_VENTING
        TRANSITION_MAP_ENTRY(EVENT_IGNORED) // ST_DONE
        TRANSITION_MAP_ENTRY(ST_IGNITION)   // ST_ABORT_FILLING
        TRANSITION_MAP_ENTRY(EVENT_IGNORED) // ST_ABORT_BURN
        TRANSITION_MAP_ENTRY(EVENT_IGNORED) // ST_SERVO_CONTROL
        END_TRANSITION_MAP
}

// FinalVenting external event
void HotFire::FinalVentingEXT()
{
    BEGIN_TRANSITION_MAP                       // - Current State -
    TRANSITION_MAP_ENTRY(EVENT_IGNORED)        // ST_INIT
        TRANSITION_MAP_ENTRY(EVENT_IGNORED)    // ST_WAIT_FOR_INIT
        TRANSITION_MAP_ENTRY(EVENT_IGNORED)    // ST_WAIT_FOR_READY
        TRANSITION_MAP_ENTRY(EVENT_IGNORED)    // ST_WAIT_FOR_FILLING
        TRANSITION_MAP_ENTRY(EVENT_IGNORED)    // ST_FILLING
        TRANSITION_MAP_ENTRY(EVENT_IGNORED)    // ST_WAIT_FOR_IGNITION
        TRANSITION_MAP_ENTRY(EVENT_IGNORED)    // ST_IGNITION
        TRANSITION_MAP_ENTRY(ST_FINAL_VENTING) // ST_FULL_BURN
        TRANSITION_MAP_ENTRY(EVENT_IGNORED)    // ST_FINAL_VENTING
        TRANSITION_MAP_ENTRY(EVENT_IGNORED)    // ST_DONE
        TRANSITION_MAP_ENTRY(ST_FINAL_VENTING) // ST_ABORT_FILLING
        TRANSITION_MAP_ENTRY(ST_FINAL_VENTING) // ST_ABORT_BURN
        TRANSITION_MAP_ENTRY(EVENT_IGNORED)    // ST_SERVO_CONTROL
        END_TRANSITION_MAP
}

// Done external event
void HotFire::DoneEXT()
{
    BEGIN_TRANSITION_MAP                    // - Current State -
    TRANSITION_MAP_ENTRY(EVENT_IGNORED)     // ST_INIT
        TRANSITION_MAP_ENTRY(EVENT_IGNORED) // ST_WAIT_FOR_INIT
        TRANSITION_MAP_ENTRY(EVENT_IGNORED) // ST_WAIT_FOR_READY
        TRANSITION_MAP_ENTRY(EVENT_IGNORED) // ST_WAIT_FOR_FILLING
        TRANSITION_MAP_ENTRY(EVENT_IGNORED) // ST_FILLING
        TRANSITION_MAP_ENTRY(EVENT_IGNORED) // ST_WAIT_FOR_IGNITION
        TRANSITION_MAP_ENTRY(EVENT_IGNORED) // ST_IGNITION
        TRANSITION_MAP_ENTRY(EVENT_IGNORED) // ST_FULL_BURN
        TRANSITION_MAP_ENTRY(ST_DONE)       // ST_FINAL_VENTING
        TRANSITION_MAP_ENTRY(EVENT_IGNORED) // ST_DONE
        TRANSITION_MAP_ENTRY(EVENT_IGNORED) // ST_ABORT_FILLING
        TRANSITION_MAP_ENTRY(EVENT_IGNORED) // ST_ABORT_BURN
        TRANSITION_MAP_ENTRY(EVENT_IGNORED) // ST_SERVO_CONTROL
        END_TRANSITION_MAP
}

// Done external event
// clang-format off
void HotFire::ServoControlEXT() {
    BEGIN_TRANSITION_MAP                       // - Current State -
    TRANSITION_MAP_ENTRY(ST_SERVO_CONTROL)     // ST_INIT
    TRANSITION_MAP_ENTRY(ST_SERVO_CONTROL)     // ST_WAIT_FOR_INIT
    TRANSITION_MAP_ENTRY(ST_SERVO_CONTROL)     // ST_WAIT_FOR_READY
    TRANSITION_MAP_ENTRY(ST_SERVO_CONTROL)     // ST_WAIT_FOR_FILLING
    TRANSITION_MAP_ENTRY(ST_SERVO_CONTROL)     // ST_FILLING
    TRANSITION_MAP_ENTRY(ST_SERVO_CONTROL)     // ST_WAIT_FOR_IGNITION
    TRANSITION_MAP_ENTRY(ST_SERVO_CONTROL)     // ST_IGNITION
    TRANSITION_MAP_ENTRY(ST_SERVO_CONTROL)     // ST_FULL_BURN
    TRANSITION_MAP_ENTRY(ST_SERVO_CONTROL)     // ST_FINAL_VENTING
    TRANSITION_MAP_ENTRY(ST_SERVO_CONTROL)     // ST_DONE
    TRANSITION_MAP_ENTRY(ST_SERVO_CONTROL)     // ST_ABORT_FILLING
    TRANSITION_MAP_ENTRY(ST_SERVO_CONTROL)     // ST_ABORT_BURN
    TRANSITION_MAP_ENTRY(ST_SERVO_CONTROL)     // ST_SERVO_CONTROL
    END_TRANSITION_MAP
} // clang-format on

// Code for each state. Do not put while in them. The right function according
// to the current state will be call in the main loop.

STATE_DEFINE(HotFire, Init, UOSMData)
{
    NoEventData eventData;
    os_sleep(1000);
    InternalEvent(ST_WAIT_FOR_INIT, eventData);
}

EXIT_DEFINE(HotFire, ExitInit)
{
	uint8_t exit[] = "Exit Init\r\n";
    ioU0->write(ioU0, (uint8_t *)&exit, sizeof(exit));
//    SPDLOG_LOGGER_INFO(logger, "HotFireSM::ExitInit");
}

ENTRY_DEFINE(HotFire, EnterWaitForInit, UOSMData)
{
	uint8_t enter[] = "Enter Wait For Init\r\n";
    ioU0->write(ioU0, (uint8_t *)&enter, sizeof(enter));
//    SPDLOG_LOGGER_INFO(logger, "HotFireSM::EnterWaitForInit");
//    enterNewState(ST_WAIT_FOR_INIT);
}

STATE_DEFINE(HotFire, WaitForInit, UOSMData)
{
//    interfaceData = updateInterface(&data, ST_WAIT_FOR_INIT);
    NoEventData eventData;
    os_sleep(1000);
    InternalEvent(ST_WAIT_FOR_READY, eventData);
//    if (interfaceData->isInitialized())
//    {
//        InternalEvent(ST_WAIT_FOR_READY, eventData);
//    }

    // showInfo(interfaceData);

//    interface->updateOutputs(interfaceData);
}

EXIT_DEFINE(HotFire, ExitWaitForInit)
{
	uint8_t exit[] = "Exit Wait For Init\r\n";
	ioU0->write(ioU0, (uint8_t *)&exit, sizeof(exit));
//    SPDLOG_LOGGER_INFO(logger, "HotFireSM::ExitWaitForInit");
}

ENTRY_DEFINE(HotFire, EnterWaitForReady, UOSMData)
{
	uint8_t enter[] = "Enter Wait For Ready\r\n";
    ioU0->write(ioU0, (uint8_t *)&enter, sizeof(enter));
//    SPDLOG_LOGGER_INFO(logger, "HotFireSM::EnterWaitForReady");
//    enterNewState(ST_WAIT_FOR_READY);
}

STATE_DEFINE(HotFire, WaitForReady, UOSMData)
{
//    interfaceData = updateInterface(&data, ST_WAIT_FOR_READY);

//    detectExternEvent(interfaceData);

//    interface->updateOutputs(interfaceData);
	NoEventData eventData;
    os_sleep(1000);
	InternalEvent(ST_WAIT_FOR_FILLING, eventData);
}

EXIT_DEFINE(HotFire, ExitWaitForReady)
{
	uint8_t exit[] = "Exit Wait For Ready\r\n";
	ioU0->write(ioU0, (uint8_t *)&exit, sizeof(exit));
//    SPDLOG_LOGGER_INFO(logger, "HotFireSM::ExitWaitForReady");
}

ENTRY_DEFINE(HotFire, EnterWaitForFilling, UOSMData)
{
	uint8_t enter[] = "Enter Wait For Filling\r\n";
	ioU0->write(ioU0, (uint8_t *)&enter, sizeof(enter));
//    SPDLOG_LOGGER_INFO(logger, "HotFireSM::EnterWaitForFilling");
//    enterNewState(ST_WAIT_FOR_FILLING);
}

STATE_DEFINE(HotFire, WaitForFilling, UOSMData)
{
	NoEventData eventData;
    os_sleep(1000);
	InternalEvent(ST_FILLING, eventData);
//    interfaceData = updateInterface(&data, ST_WAIT_FOR_FILLING);
//
//#if USE_GPIO == 1
//    GpioData &gpioData = interfaceData->gpioData;
//
//#if USE_VENT
//    gpioData.digitalOutputMap.insert({VENT_NAME, VENT_CLOSE});
//#endif
//
//#if USE_IGNITER
//    gpioData.digitalOutputMap.insert({IGNITER_NAME, IGNITER_OFF});
//#endif
//
//#if USE_PWM_MAIN
//    gpioData.dcOutputMap.insert({MAIN_NAME, MAIN_CLOSE});
//#endif
//
//#if USE_PWM_PINHOLE
//    gpioData.pwmOutputMap.insert({PINHOLE_NAME, PINHOLE_CLOSE});
//#endif
//
//#if USE_PWM_FILL
//    gpioData.pwmOutputMap.insert({FILL_NAME, FILL_CLOSE});
//#endif
//
//#endif

//    detectExternEvent(interfaceData);

//    interface->updateOutputs(interfaceData);
}

EXIT_DEFINE(HotFire, ExitWaitForFilling)
{
	uint8_t exit[] = "Exit Wait For Filling\r\n";
	ioU0->write(ioU0, (uint8_t *)&exit, sizeof(exit));
//    SPDLOG_LOGGER_INFO(logger, "HotFireSM::ExitWaitForFilling");
}

ENTRY_DEFINE(HotFire, EnterFilling, UOSMData)
{
	uint8_t enter[] = "Enter Filling\r\n";
    ioU0->write(ioU0, (uint8_t *)&enter, sizeof(enter));
//    SPDLOG_LOGGER_INFO(logger, "HotFireSM::EnterFilling");
//    enterNewState(ST_FILLING);
}

STATE_DEFINE(HotFire, Filling, UOSMData)
{
	NoEventData eventData;
    os_sleep(1000);
	InternalEvent(ST_WAIT_FOR_IGNITION, eventData);
//    interfaceData = updateInterface(&data, ST_FILLING);
//
//#if USE_GPIO == 1
//    GpioData &gpioData = interfaceData->gpioData;
//
//#if USE_VENT
//    gpioData.digitalOutputMap.insert({VENT_NAME, VENT_CLOSE});
//#endif
//
//#if USE_IGNITER
//    gpioData.digitalOutputMap.insert({IGNITER_NAME, IGNITER_OFF});
//#endif
//
//#if USE_PWM_MAIN
//    gpioData.dcOutputMap.insert({MAIN_NAME, MAIN_CLOSE});
//#endif
//
//#if USE_PWM_PINHOLE
//    gpioData.pwmOutputMap.insert({PINHOLE_NAME, PINHOLE_OPEN});
//#endif
//
//#if USE_PWM_FILL
//    gpioData.pwmOutputMap.insert({FILL_NAME, FILL_OPEN});
//#endif
//
//#endif

//    detectExternEvent(interfaceData);

//    interface->updateOutputs(interfaceData);
}

EXIT_DEFINE(HotFire, ExitFilling)
{
	uint8_t exit[] = "Exit Filling\r\n";
    ioU0->write(ioU0, (uint8_t *)&exit, sizeof(exit));
	//    SPDLOG_LOGGER_INFO(logger, "HotFireSM::ExitFilling");
}

ENTRY_DEFINE(HotFire, EnterWaitForIgnition, UOSMData)
{
	uint8_t enter[] = "Enter Wait For Ignition\r\n";
    ioU0->write(ioU0, (uint8_t *)&enter, sizeof(enter));
//    SPDLOG_LOGGER_INFO(logger, "HotFireSM::WaitForIgnition");
//    enterNewState(ST_WAIT_FOR_IGNITION);
}

STATE_DEFINE(HotFire, WaitForIgnition, UOSMData)
{
	NoEventData eventData;
    os_sleep(1000);
	InternalEvent(ST_IGNITION, eventData);
//    interfaceData = updateInterface(&data, ST_WAIT_FOR_IGNITION);
//
//#if USE_GPIO
//    GpioData &gpioData = interfaceData->gpioData;
//
//#if USE_VENT
//    gpioData.digitalOutputMap.insert({VENT_NAME, VENT_CLOSE});
//#endif
//
//#if USE_IGNITER
//    gpioData.digitalOutputMap.insert({IGNITER_NAME, IGNITER_OFF});
//#endif
//
//#if USE_PWM_MAIN
//    gpioData.dcOutputMap.insert({MAIN_NAME, MAIN_CLOSE});
//#endif
//
//#if USE_PWM_PINHOLE
//    gpioData.pwmOutputMap.insert({PINHOLE_NAME, PINHOLE_CLOSE});
//#endif
//
//#if USE_PWM_FILL
//    gpioData.pwmOutputMap.insert({FILL_NAME, FILL_CLOSE});
//#endif
//
//#endif

//    detectExternEvent(interfaceData);

//    interface->updateOutputs(interfaceData);
}

EXIT_DEFINE(HotFire, ExitWaitForIgnition)
{
	uint8_t exit[] = "Exit Wait For Ignition\r\n";
    ioU0->write(ioU0, (uint8_t *)&exit, sizeof(exit));
//    SPDLOG_LOGGER_INFO(logger, "HotFireSM::ExitWaitForIgnition");
}

ENTRY_DEFINE(HotFire, EnterIgnition, UOSMData)
{
	uint8_t enter[] = "Enter Ignition\r\n";
	ioU0->write(ioU0, (uint8_t *)&enter, sizeof(enter));
//    SPDLOG_LOGGER_INFO(logger, "HotFireSM::EnterIgnition");
//    enterNewState(ST_IGNITION);
}

STATE_DEFINE(HotFire, Ignition, UOSMData)
{
	NoEventData eventData;
    os_sleep(1000);
	InternalEvent(ST_FULL_BURN, eventData);
//    interfaceData = updateInterface(&data, ST_IGNITION);
//
//#if USE_GPIO
//    GpioData &gpioData = interfaceData->gpioData;
//
//#if USE_VENT
//    gpioData.digitalOutputMap.insert({VENT_NAME, VENT_CLOSE});
//#endif
//
//#if USE_IGNITER
//    gpioData.digitalOutputMap.insert({IGNITER_NAME, IGNITER_ON});
//#endif
//
//#if USE_PWM_MAIN
//    gpioData.dcOutputMap.insert({MAIN_NAME, MAIN_IGNITION});
//#endif
//
//#if USE_PWM_PINHOLE
//    gpioData.pwmOutputMap.insert({PINHOLE_NAME, PINHOLE_CLOSE});
//#endif
//
//#if USE_PWM_FILL
//    gpioData.pwmOutputMap.insert({FILL_NAME, FILL_CLOSE});
//#endif
//
//#endif
//    detectExternEvent(interfaceData);
//    switchStatesAfterTime((ST_FULL_BURN), duration_ms(5000), eventData);

//    interface->updateOutputs(interfaceData);
}

EXIT_DEFINE(HotFire, ExitIgnition)
{
	uint8_t exit[] = "Exit Ignition\r\n";
    ioU0->write(ioU0, (uint8_t *)&exit, sizeof(exit));
//    SPDLOG_LOGGER_INFO(logger, "HotFireSM::ExitIgnition");
}

ENTRY_DEFINE(HotFire, EnterFullBurn, UOSMData)
{
	uint8_t enter[] = "Enter Full Burn\r\n";
    ioU0->write(ioU0, (uint8_t *)&enter, sizeof(enter));
//    SPDLOG_LOGGER_INFO(logger, "HotFireSM::EnterFullBurn");
//    enterNewState(ST_FULL_BURN);
}

STATE_DEFINE(HotFire, FullBurn, UOSMData)
{
	NoEventData eventData;
    os_sleep(1000);
	InternalEvent(ST_FINAL_VENTING, eventData);
//    interfaceData = updateInterface(&data, ST_FULL_BURN);
//
//#if USE_GPIO
//    GpioData &gpioData = interfaceData->gpioData;
//
//#if USE_VENT
//    gpioData.digitalOutputMap.insert({VENT_NAME, VENT_CLOSE});
//#endif
//
//#if USE_IGNITER
//    gpioData.digitalOutputMap.insert({IGNITER_NAME, IGNITER_OFF});
//#endif
//
//#if USE_PWM_MAIN
//    gpioData.dcOutputMap.insert({MAIN_NAME, MAIN_OPEN});
//#endif
//
//#if USE_PWM_PINHOLE
//    gpioData.pwmOutputMap.insert({PINHOLE_NAME, PINHOLE_CLOSE});
//#endif
//
//#if USE_PWM_FILL
//    gpioData.pwmOutputMap.insert({FILL_NAME, FILL_CLOSE});
//#endif
//
//#endif

//    detectExternEvent(interfaceData);

//    interface->updateOutputs(interfaceData);
}

EXIT_DEFINE(HotFire, ExitFullBurn)
{
	uint8_t exit[] = "Exit Full Burn\r\n";
	ioU0->write(ioU0, (uint8_t *)&exit, sizeof(exit));
//    SPDLOG_LOGGER_INFO(logger, "HotFireSM::ExitFullBurn");
}

ENTRY_DEFINE(HotFire, EnterFinalVenting, UOSMData)
{
	uint8_t enter[] = "Enter Final Venting\r\n";
	ioU0->write(ioU0, (uint8_t *)&enter, sizeof(enter));
//    SPDLOG_LOGGER_INFO(logger, "HotFireSM::EnterFinalVenting");
//    enterNewState(ST_FINAL_VENTING);
}

STATE_DEFINE(HotFire, FinalVenting, UOSMData)
{
	NoEventData eventData; 
    os_sleep(1000);
	InternalEvent(ST_DONE, eventData);
//    interfaceData = updateInterface(&data, ST_FINAL_VENTING);
//
//#if USE_GPIO
//    GpioData &gpioData = interfaceData->gpioData;
//
//#if USE_VENT
//    gpioData.digitalOutputMap.insert({VENT_NAME, VENT_OPEN});
//#endif
//
//#if USE_IGNITER
//    gpioData.digitalOutputMap.insert({IGNITER_NAME, IGNITER_OFF});
//#endif
//
//#if USE_PWM_MAIN
//    gpioData.dcOutputMap.insert({MAIN_NAME, MAIN_CLOSE});
//#endif
//
//#if USE_PWM_PINHOLE
//    gpioData.pwmOutputMap.insert({PINHOLE_NAME, PINHOLE_OPEN});
//#endif
//
//#if USE_PWM_FILL
//    gpioData.pwmOutputMap.insert({FILL_NAME, FILL_CLOSE});
//#endif
//
//#endif

//    detectExternEvent(interfaceData);

//    interface->updateOutputs(interfaceData);
}

EXIT_DEFINE(HotFire, ExitFinalVenting)
{
    uint8_t exit[] = "Exit Final Venting\r\n";
	ioU0->write(ioU0, (uint8_t *)&exit, sizeof(exit));
//    SPDLOG_LOGGER_INFO(logger, "HotFireSM::ExitFinalVenting");
}

ENTRY_DEFINE(HotFire, EnterDone, UOSMData)
{
	uint8_t enter[] = "Enter Done\r\n";
    ioU0->write(ioU0, (uint8_t *)&enter, sizeof(enter));
//    SPDLOG_LOGGER_INFO(logger, "HotFireSM::EnterDone");
//    SPDLOG_LOGGER_INFO(logger, "Done.");
//    enterNewState(ST_DONE);
}

STATE_DEFINE(HotFire, Done, UOSMData)
{
	uint8_t enter[] = "Done\r\n";
    os_sleep(1000);
    ioU0->write(ioU0, (uint8_t *)&enter, sizeof(enter));
//    interfaceData = updateInterface(&data, ST_DONE);
//
//#if USE_GPIO
//    GpioData &gpioData = interfaceData->gpioData;
//
//#if USE_VENT
//    gpioData.digitalOutputMap.insert({VENT_NAME, VENT_CLOSE});
//#endif
//
//#if USE_IGNITER
//    gpioData.digitalOutputMap.insert({IGNITER_NAME, IGNITER_OFF});
//#endif
//
//#if USE_PWM_MAIN
//    gpioData.dcOutputMap.insert({MAIN_NAME, MAIN_OPEN});
//#endif
//
//#if USE_PWM_PINHOLE
//    gpioData.pwmOutputMap.insert({PINHOLE_NAME, PINHOLE_OPEN});
//#endif
//
//#if USE_PWM_FILL
//    gpioData.pwmOutputMap.insert({FILL_NAME, FILL_CLOSE});
//#endif
//
//#endif

//    detectExternEvent(interfaceData);

//    interface->updateOutputs(interfaceData);
}

ENTRY_DEFINE(HotFire, EnterAbortFilling, UOSMData)
{
	// uint8_t enter[] = "Enter Abort Filling";
	// HAL_UART_Transmit(&uart, enter, sizeof(enter), 10);
//    SPDLOG_LOGGER_INFO(logger, "HotFireSM::EnterAbortFilling");
//    enterNewState(ST_ABORT_FILLING);
}

STATE_DEFINE(HotFire, AbortFilling, UOSMData)
{
//    interfaceData = updateInterface(&data, ST_ABORT_FILLING);
//
//#if USE_GPIO
//    GpioData &gpioData = interfaceData->gpioData;
//
//#if USE_VENT
//    gpioData.digitalOutputMap.insert({VENT_NAME, VENT_CLOSE});
//#endif
//
//#if USE_IGNITER
//    gpioData.digitalOutputMap.insert({IGNITER_NAME, IGNITER_OFF});
//#endif
//
//#if USE_PWM_MAIN
//    gpioData.dcOutputMap.insert({MAIN_NAME, MAIN_CLOSE});
//#endif
//
//#if USE_PWM_PINHOLE
//    gpioData.pwmOutputMap.insert({PINHOLE_NAME, PINHOLE_CLOSE});
//#endif
//
//#if USE_PWM_FILL
//    gpioData.pwmOutputMap.insert({FILL_NAME, FILL_CLOSE});
//#endif
//
//#endif

//    detectExternEvent(interfaceData);

//    interface->updateOutputs(interfaceData);
}

ENTRY_DEFINE(HotFire, EnterAbortBurn, UOSMData)
{
	// uint8_t enter[] = "Enter Abort Burn";
	// HAL_UART_Transmit(&uart, enter, sizeof(enter), 10);
//    SPDLOG_LOGGER_INFO(logger, "HotFireSM::EnterAbortBurn");
//    enterNewState(ST_ABORT_BURN);
}

STATE_DEFINE(HotFire, AbortBurn, UOSMData)
{
//    interfaceData = updateInterface(&data, ST_ABORT_BURN);
//
//#if USE_GPIO
//    GpioData &gpioData = interfaceData->gpioData;
//
//#if USE_VENT
//    gpioData.digitalOutputMap.insert({VENT_NAME, VENT_CLOSE});
//#endif
//
//#if USE_IGNITER
//    gpioData.digitalOutputMap.insert({IGNITER_NAME, IGNITER_OFF});
//#endif
//
//#if USE_PWM_MAIN
//    gpioData.dcOutputMap.insert({MAIN_NAME, MAIN_CLOSE});
//#endif
//
//#if USE_PWM_PINHOLE
//    gpioData.pwmOutputMap.insert({PINHOLE_NAME, PINHOLE_CLOSE});
//#endif
//
//#if USE_PWM_FILL
//    gpioData.pwmOutputMap.insert({FILL_NAME, FILL_CLOSE});
//#endif
//
//#endif

//    detectExternEvent(interfaceData);

//    interface->updateOutputs(interfaceData);
}

ENTRY_DEFINE(HotFire, EnterServoControl, UOSMData)
{
	// uint8_t enter[] = "Enter Servo Control";
	// HAL_UART_Transmit(&uart, enter, sizeof(enter), 10);
//    SPDLOG_LOGGER_INFO(logger, "HotFireSM::EnterServoControl");
//    enterNewState(ST_SERVO_CONTROL);
}

STATE_DEFINE(HotFire, ServoControl, UOSMData)
{
//    interfaceData = updateInterface(&data, ST_SERVO_CONTROL);

//    detectConnectionTimeout(interfaceData);

//    eventType eventNbr = interfaceData->eventNumber;
//    bool dataRecieved = eventNbr > -1;
//
//    if (dataRecieved)
//    {
//#if USE_GPIO ---------------- USE_PWM_FILL
//         */== 1
//        GpioData &gpioData = interfaceData->gpioData;
//
//        /*
//         * GPIO event number, a 6 bit binary number where the
//         * 1st bit is the enable bit and the last 5 control
//         * whether the valves are open/closed.
//         * A '1' means to open the valve and a '0' to close it.
//         *
//         * 0 0 0 0 0 0
//         * | | | | | ^--------- Enable bit
//         * | | | | ^----------- USE_VENT
//         * | | | ^------------- USE_IGNITER
//         * | | ^--------------- USE_PWM_MAIN
//         * | ^----------------- USE_PWM_PINHOLE
//         * ^---
//
//        bool enabled = eventNbr > 0 && (eventNbr & EVENT_ENABLE_MASK);
//        if (enabled)
//        {
//
//#if USE_VENT == 1
//            {
//                bool open = (eventNbr & VENT_EVENT_ENABLE_MASK) > 0;
//
//                gpioData.digitalOutputMap.insert({VENT_NAME, open ? VENT_OPEN : VENT_CLOSE});
//
//                logValveStatus(VENT_NAME, open);
//            }
//#endif
//
//#if USE_IGNITER == 1
//            {
//                bool open = (eventNbr & IGNITER_EVENT_ENABLE_MASK) > 0;
//
//                gpioData.digitalOutputMap.insert({IGNITER_NAME, open ? IGNITER_ON : IGNITER_OFF});
//
//                logValveStatus(IGNITER_NAME, open);
//            }
//#endif
//
//#if USE_PWM_MAIN == 1
//            {
//                bool open = (eventNbr & MAIN_EVENT_ENABLE_MASK) > 0;
//
//                gpioData.dcOutputMap.insert({MAIN_NAME, open ? MAIN_OPEN : MAIN_CLOSE});
//
//                logValveStatus(MAIN_NAME, open);
//            }
//#endif
//
//#if USE_PWM_PINHOLE == 1
//            {
//                bool open = (eventNbr & PINHOLE_EVENT_ENABLE_MASK) > 0;
//
//                gpioData.pwmOutputMap.insert({PINHOLE_NAME, open ? PINHOLE_OPEN : PINHOLE_CLOSE});
//
//                logValveStatus(PINHOLE_NAME, open);
//            }
//#endif
//
//#if USE_PWM_FILL == 1
//            {
//                bool open = (eventNbr & FILL_EVENT_ENABLE_MASK) > 0;
//
//                gpioData.pwmOutputMap.insert({FILL_NAME, open ? FILL_OPEN : FILL_CLOSE});
//
//                logValveStatus(FILL_NAME, open);
//            }
//#endif
//        }
//        else
//        {
//            // Switch back to specified state
//            InternalEvent(eventNbr >> 1, data);
//        }
//#endif
//    }
//
//    interface->updateOutputs(interfaceData);
}


