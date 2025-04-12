#pragma once

//********** ELEVATOR CONSTANTS **********//

#define ELEV_PULLY_DIAM 1.25_in

#define ELEV_POSITION_HOME -0.03_tr    //home
#define ELEV_POSITION_L1 -0.85_tr    //L1
#define ELEV_POSITION_L2 -1.5_tr     //L2
#define ELEV_POSITION_L3 -3.15_tr       //L3
#define ELEV_POSITION_L4 -5.7_tr     //L4
#define ELEV_POSITION_ALG_FLOOR -3.3_tr       //ALgea Low -2.23 before now -2.4 , changed again now 2.45
#define ELEV_POSITION_CORAL_FLOOR -5.0_tr       //Algea High
#define ELEV_POSITION_BARGE -6_tr
#define ELEV_POSITION_L4_AUTO -5.8_tr
#define ELEV_POSITION_ALG_HIGH -1.65_tr

#define ELEV_TOLERANCE 0.05_tr


#define ELEV_HOMING_HIGH_SPEED 0.01
#define ELEV_HOMING_CREEP_SPEED 0.005

#define ELEV_MANUAL_SLOW_POWER 0.4
#define ELEV_MANUAL_SLOW_DOWN_POWER -0.1

#define ELEV_POS_HOME 1
#define ELEV_POS_L1 7
#define ELEV_POS_L2 4
#define ELEV_POS_L3 2
#define ELEV_POS_L4 3
#define ELEV_POS_L4_AUTO 10
//#define ELEV_POS_ALG_FLOOR 5
 //#define ELEV_POS_CORAL_FLOOR 6
#define ELEV_POS_ALG_HIGH 8
#define ELEV_POS_BARGE  9

// ********* PIVOT CONSTANTS **********//

#define PIVOT_POSITION_HOME 0_tr
#define PIVOT_POSITION_SCORE 0.837_tr
#define PIVOT_BARGE       0.4_tr
#define PIVOT_FLOOR_CORAL 1.6_tr
#define PIVOT_FLOOR_ALGAE 1.44_tr
#define PIVOT_CLOSE_HOME 0.1_tr

#define ALGAE_POS_HOME 1
#define ALGAE_POS_SCORE 2
#define ALGAE_POS_BARGE 3
#define ALGAE_POS_FLOOR_CORAL 4
#define ALGAE_POS_FLOOR_ALGAE 5
#define ALGAE_POS_CLOSE_HOME 6

#define PIVOT_HOME_POSITION    0
#define PIVOT_TOLERANCE        0.05
#define PIVOT_MANUAL_POWER     0.3
