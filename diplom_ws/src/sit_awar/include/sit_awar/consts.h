#pragma once

#include <cstdint>

//-----------------SIT AWAR CYCLE----------------------
const double delta_t = 0.1;

//-----------------OBJECT STATE HISTORY--------------
const int HISTORY_DEEP = 20;

//----------------------------KALMAN---------FILTER------------------------
const double FIL_MAX_OBJECT_DIST = 15; // 15 meters
const double FIL_MAX_OBJECT_AZIM = 0.034; // 2 degrees in radians
const double FIL_AZIM_SCALE_KOEFF = 15;
const double FIL_DIST_SCALE_KOEFF = 0.05;
const double FIL_COST_TRESHHOLD[2] = {1-0.3,1-0.3};
const uint16_t LIVE_TIME = 5;

//----------------------------HUNGARIAN-----AGGREGATION------------------------
const double AGG_MAX_OBJECT_DIST = 15; // 15 meters
const double AGG_MAX_OBJECT_AZIM = 0.034; // 2 degrees in radians

const double AGG_AZIM_SCALE_KOEFF = 15;
const double AGG_DIST_SCALE_KOEFF = 0.05;
const double AGG_CLASS_SCALE_KOEFF = 0.25;
const double AGG_TYPE_SCALE_KOEFF = 0.25;

const double AGG_COST_TRESHHOLD[4] = {1-0.3*0.36*0.25,1-0.3*0.5,1-0.3*0.36*0.25,1-0.3*0.5};

