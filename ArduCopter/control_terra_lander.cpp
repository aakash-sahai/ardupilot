/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Copter.h"

#define RELAY_PISTON_DISENGAGE     2
#define RELAY_ROVER_DISENGAGE      3
#define RELAY_SKYCRANE_DISENGAGE   0

#define RELAY_ON    0
#define RELAY_OFF   1

/*
 * control_terra_lender.pde - init and run calls for TerraLander flight mode
 *
 * There are two parts to Terra Lander, the high level decision making which controls which state we are in
 * and the lower implementation of the relays, waypoint or landing controllers within those states
 */

// terra_lander_init - initialise terra_lander controller
bool Copter::terra_lander_init(bool ignore_checks)
{
    if (position_ok() || ignore_checks) {
        terra_lander_state = TerraLander_FreeFall;
        terra_lander_state_complete = true;
        return true;
    } else {
        return false;
    }
}

uint8_t Copter::terra_lander_effective_mode()
{
  switch(terra_lander_state) {
    case TerraLander_FlyToRoverHome:
      return AUTO;
    case TerraLander_RoverDisengage:
    case TerraLander_RoverLand:
    case TerraLander_SkyCraneDisengage:
      return LOITER;
    case TerraLander_FlyToLanderHome:
    case TerraLander_Landing:
      return RTL;
    case TerraLander_Landed:
    case TerraLander_Standby:
    case TerraLander_ReadyForTakeoff:
    case TerraLander_InFlight:
    case TerraLander_PastApogee:
    case TerraLander_EjectFromPiston:
    case TerraLander_FreeFall:
    case TerraLander_Stabilize:
    default:
      return STABILIZE;
  }
  return STABILIZE;
}

// terra_lander_run - runs the Terra Lander controller
// should be called at 100hz or more
void Copter::terra_lander_run()
{
    // check if we need to move to next state
    if (terra_lander_state_complete) {
        terra_lander_state_complete = false;
        switch (terra_lander_state) {
        case TerraLander_Standby:
          terra_lander_state = TerraLander_ReadyForTakeoff;
          terra_lander_readyForTakeoff_start();
          break;
        case TerraLander_ReadyForTakeoff:
          terra_lander_state = TerraLander_InFlight;
          terra_lander_inFlight_start();
          break;
        case TerraLander_InFlight:
          terra_lander_state = TerraLander_PastApogee;
          terra_lander_pastApogee_start();
          break;
        case TerraLander_PastApogee:
          terra_lander_state = TerraLander_EjectFromPiston;
          terra_lander_ejectFromPiston_start();
          break;
        case TerraLander_EjectFromPiston:
          terra_lander_state = TerraLander_FreeFall;
          terra_lander_freeFall_start();
          break;
        case TerraLander_FreeFall:
          terra_lander_state = TerraLander_Stabilize;
          terra_lander_stabilize_start();
          break;
        case TerraLander_Stabilize:
          terra_lander_state = TerraLander_FlyToRoverHome;
          terra_lander_flyToRoverHome_start();
          break;
        case TerraLander_FlyToRoverHome:
          terra_lander_state = TerraLander_RoverDisengage;
          terra_lander_roverDisengage_start();
          break;
        case TerraLander_RoverDisengage:
          terra_lander_state = TerraLander_RoverLand;
          terra_lander_roverLand_start();
          break;
        case TerraLander_RoverLand:
          terra_lander_state = TerraLander_SkyCraneDisengage;
          terra_lander_skyCraneDisengage_start();
          break;
        case TerraLander_SkyCraneDisengage:
          terra_lander_state = TerraLander_FlyToLanderHome;
          terra_lander_flyToLanderHome_start();
          break;
        case TerraLander_FlyToLanderHome:
          terra_lander_state = TerraLander_Landing;
          terra_lander_landing_start();
          break;
        case TerraLander_Landing:
          terra_lander_state = TerraLander_Landed;
          terra_lander_landed_start();
          break;
        case TerraLander_Landed:
          break;
        }
    }

    // call the correct run function
    switch (terra_lander_state) {
    case TerraLander_Standby:
      terra_lander_standby_run();
      break;
    case TerraLander_ReadyForTakeoff:
      terra_lander_readyForTakeoff_run();
      break;
    case TerraLander_InFlight:
      terra_lander_inFlight_run();
      break;
    case TerraLander_PastApogee:
      terra_lander_pastApogee_run();
      break;
    case TerraLander_EjectFromPiston:
      terra_lander_ejectFromPiston_run();
      break;
    case TerraLander_FreeFall:
      terra_lander_freeFall_run();
      break;
    case TerraLander_Stabilize:
      terra_lander_stabilize_run();
      break;
    case TerraLander_FlyToRoverHome:
      terra_lander_flyToRoverHome_run();
      break;
    case TerraLander_RoverDisengage:
      terra_lander_roverDisengage_run();
      break;
    case TerraLander_RoverLand:
      terra_lander_roverLand_run();
      break;
    case TerraLander_SkyCraneDisengage:
      terra_lander_skyCraneDisengage_run();
      break;
    case TerraLander_FlyToLanderHome:
      terra_lander_flyToLanderHome_run();
      break;
    case TerraLander_Landing:
      terra_lander_landing_run();
      break;
    case TerraLander_Landed:
      terra_lander_landed_run();
      break;
    }
}

void Copter::terra_lander_standby_start()
{
  gcs_send_text_P(SEVERITY_HIGH,PSTR("TerraLander: Standby State\n"));
  /*
   * Invoke startup_ground(true) to do all the calibration during start and
   * set the land altitude.
   */
  startup_ground(true);
  terra_lander_alt = inertial_nav.get_altitude();
  terra_lander_max_alt = terra_lander_alt;
}


void Copter::terra_lander_readyForTakeoff_start()
{
  gcs_send_text_P(SEVERITY_HIGH,PSTR("TerraLander: Ready For Takeoff State\n"));
  /*
   * Nothing to do when starting readyForTakeoff state.
   */
}

void Copter::terra_lander_inFlight_start()
{
  gcs_send_text_P(SEVERITY_HIGH,PSTR("TerraLander: In Flight State\n"));
/*
 * No action to start
 */
}

void Copter::terra_lander_pastApogee_start()
{
  gcs_send_text_P(SEVERITY_HIGH,PSTR("TerraLander: Post Apogee State\n"));

/*
 * No action to start
 */
}

void Copter::terra_lander_ejectFromPiston_start() // COMPLETE
{
  gcs_send_text_P(SEVERITY_HIGH,PSTR("TerraLander: Eject From Piston State\n"));

/*
 * Turn on the RELAY_PISTON_DISENGAGE and start the timer to turn it off
 */
 ServoRelayEvents.do_set_relay(RELAY_PISTON_DISENGAGE, RELAY_ON);
 condition_start = millis();
 condition_value = g.tl_duration_burn;
}

void Copter::terra_lander_freeFall_start() // COMPLETE
{
  gcs_send_text_P(SEVERITY_HIGH,PSTR("TerraLander: Free Fall State\n"));
/*
 * Start the timer to let TerraLander free fall to let it free fall to steer away
 * from the descending Piston
 */
 condition_start = millis();
 condition_value = g.tl_delay_motor_arm;
}

void Copter::terra_lander_stabilize_start() // COMPLETE
{
  gcs_send_text_P(SEVERITY_HIGH,PSTR("TerraLander: Stabilize State\n"));
/*
 * Disable failsafe, arm and start the motors and start the ALT_HOLD mode
 */
 failsafe_disable();
 ahrs.set_correct_centrifugal(true);
 hal.util->set_soft_armed(true);
 enable_motor_output();
 motors.armed(true);
 althold_init(true);
 failsafe_enable();
}

void Copter::terra_lander_flyToRoverHome_start() // COMPLETE
{
  gcs_send_text_P(SEVERITY_HIGH,PSTR("TerraLander: Fly To Rover Home State\n"));
/*
 * Switch to auto_init to let the AUTO mode steer the lander towards the Rover Landing point
 */
 if (!auto_init(true)) {
   gcs_send_text_P(SEVERITY_HIGH,PSTR("TerraLander: Auto mode failed to initialize\n"));
 }
}

void Copter::terra_lander_roverDisengage_start() // COMPLETE
{
  gcs_send_text_P(SEVERITY_HIGH,PSTR("TerraLander: Rover Disengage State\n"));
/*
 * Start the LOITER mode
 * Turn on the RELAY_ROVER_DISENGAGE and start the timer to turn it off
 */
 loiter_init(true);
 ServoRelayEvents.do_set_relay(RELAY_ROVER_DISENGAGE, RELAY_ON);
 condition_start = millis();
 condition_value = g.tl_duration_burn;
}

void Copter::terra_lander_roverLand_start()  // COMPLETE
{
  gcs_send_text_P(SEVERITY_HIGH,PSTR("TerraLander: Free Fall State\n"));
/*
 * Start the g.tl_rover_land_timeout timer and let the Rover lower itself
 */
 condition_start = millis();
 condition_value = g.tl_rover_land_timeout;
}

void Copter::terra_lander_skyCraneDisengage_start()
{
  gcs_send_text_P(SEVERITY_HIGH,PSTR("TerraLander: Sky Crane Disengage State\n"));
/*
 * Turn on the SkyCrane Separation Relay (#3) and start the timer to turn it off
 */
 ServoRelayEvents.do_set_relay(RELAY_SKYCRANE_DISENGAGE, RELAY_ON);
 condition_start = millis();
 condition_value = g.tl_duration_burn;
}

void Copter::terra_lander_flyToLanderHome_start()
{
  gcs_send_text_P(SEVERITY_HIGH,PSTR("TerraLander: Fly To Lander Home State\n"));
/*
 * TBD: This state may not be needed if RTL works
 */
}

void Copter::terra_lander_landing_start()
{
  gcs_send_text_P(SEVERITY_HIGH,PSTR("TerraLander: Landing Start State\n"));
/*
 * Initiate return to home routine
 */
 rtl_init(true);
}

void Copter::terra_lander_landed_start()
{
  gcs_send_text_P(SEVERITY_HIGH,PSTR("TerraLander: Landed State\n"));
/*
 * Nothing to do
 */
}

void Copter::terra_lander_standby_run()
{
/*
 * TBD: Wait until the Lander is in nearly inverted position and then exit the state
 */
}

void Copter::terra_lander_readyForTakeoff_run()
{
/*
 * Wait until the Altitude increases by g.tl_alt_in_flight_delta+ cms for
 * g.tl_alt_integ_interval cycles indicating we are in flight
 */
 static int count = 0;
 float alt = inertial_nav.get_altitude();
 terra_lander_max_alt = max(alt, terra_lander_max_alt);
 float deltaAlt = alt - terra_lander_alt;
 if (deltaAlt > g.tl_alt_in_flight_delta) {
   count++;
 } else {
   count = 0;
 }
 if (count > g.tl_alt_integ_interval) {
   terra_lander_state_complete = true;
 }
}

void Copter::terra_lander_inFlight_run()
{
/*
 * Keep tab on maximum altitude until altitude decreases by g.tl_alt_past_apogee_delta+ cm
 * for g.tl_alt_integ_interval cycles indicating we are past the apogee
 */
 static int count = 0;
 float alt = inertial_nav.get_altitude();
 terra_lander_max_alt = max(alt, terra_lander_max_alt);
 float deltaAlt = terra_lander_max_alt - alt;
 if (deltaAlt > g.tl_alt_past_apogee_delta) {
   count++;
 } else {
   count = 0;
 }
 if (count > g.tl_alt_integ_interval) {
   terra_lander_state_complete = true;
 }
}

void Copter::terra_lander_pastApogee_run()
{
  static int count = 0;
  float alt = inertial_nav.get_altitude();
  float deltaAlt = alt - terra_lander_alt;
  if (deltaAlt > g.tl_alt_disengage_delta) {
    count++;
  } else {
    count = 0;
  }
  if (count > g.tl_alt_integ_interval) {
    if (gps.status() >= AP_GPS::GPS_OK_FIX_2D) {
      terra_lander_state_complete = true;
    }
  }
}

void Copter::terra_lander_ejectFromPiston_run()
{
/*
 * Wait for timer to expire to turn off Relay RELAY_PISTON_DISENGAGE.
 * TBD: Check the switch to make sure we have disengaged from piston
 */
 if (millis() - condition_start > (uint32_t)max(condition_value, 0)) {
   condition_value = 0;
   ServoRelayEvents.do_set_relay(RELAY_PISTON_DISENGAGE, RELAY_OFF);
   terra_lander_state_complete = true;
 }
}

void Copter::terra_lander_freeFall_run() // COMPLETE
{
/*
 * Wait for Motor start timer to expire.
 */
 if (millis() - condition_start > (uint32_t)max(condition_value, 0)) {
   condition_value = 0;
   terra_lander_state_complete = true;
 }
}

void Copter::terra_lander_stabilize_run() // COMPLETE
{
/*
 * Continue to perform the ALT_HOLD routine until GPS has at least a 3D lock
 */
 althold_run();
 if (gps.status() >= AP_GPS::GPS_OK_FIX_3D) {
   terra_lander_state_complete = true;
 }
}

void Copter::terra_lander_flyToRoverHome_run() // COMPLETE
{
/*
 * Let auto mode continue to navigate lander through the waypoints until the
 * last waypoint has been reached.
 */
 auto_run();
 if (mission.state() == AP_Mission::MISSION_COMPLETE) {
   terra_lander_state_complete = true;
 }
}

void Copter::terra_lander_roverDisengage_run() // COMPLETE
{
/*
 * Continue running the LOITER mode. Wait for timer to expire and turn off
 * RELAY_ROVER_DISENGAGE.
 */
 loiter_run();
 if (millis() - condition_start > (uint32_t)max(condition_value, 0)) {
   condition_value = 0;
   ServoRelayEvents.do_set_relay(RELAY_ROVER_DISENGAGE, RELAY_OFF);
   terra_lander_state_complete = true;
 }
}

void Copter::terra_lander_roverLand_run() // COMPLETE
{
/*
 * Continue running the LOITER mode.
 * For now, wait for g.tl_rover_land_timeout
 * TBD: Wait until touchdown switch is detected to be open
 */
 loiter_run();
 if (millis() - condition_start > (uint32_t)max(condition_value, 0)) {
   condition_value = 0;
   terra_lander_state_complete = true;
 }
}

void Copter::terra_lander_skyCraneDisengage_run() // COMPLETE
{
/*
 * Continue running the LOITER mode.
 * Wait for timer to expire and turn off RELAY_SKYCRANE_DISENGAGE
 */
 loiter_run();
 if (millis() - condition_start > (uint32_t)max(condition_value, 0)) {
   condition_value = 0;
   ServoRelayEvents.do_set_relay(RELAY_SKYCRANE_DISENGAGE, RELAY_OFF);
   terra_lander_state_complete = true;
 }
}

void Copter::terra_lander_flyToLanderHome_run()
{
/*
 * TBD: This state may not be needed if RTL works - simply mark it complete
 */
 terra_lander_state_complete = true;
}

void Copter::terra_lander_landing_run() // COMPLETE
{
/*
 * Continue the RTL routine until touchdown
 */
 rtl_run();
 if (ap.land_complete) {
   terra_lander_state_complete = true;
 }
}

void Copter::terra_lander_landed_run()  // COMPLETE
{
/*
 * Disarm the motors and stay in this state forever. Fanfare!
 */
 init_disarm_motors();
}
