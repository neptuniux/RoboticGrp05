//
// Created by daniel on 23.04.20.
//

#include "header.h"
#if SIMULATION
#include "../API/webots/webotsAPI.h"
#else
#include "../API/epuck/epuckAPI.h"
#endif

int stateLine(){
    printf("testPrintf");

}