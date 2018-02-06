#include "motionControl/control.h"

void control_iter() {
    if (!controlRunning) {
        return;
    }

    unsigned long now = HAL_GetTick();
    long delta = now - lastIter;
    
    if (delta >= DT) {
        lastIter = now;
        encoder_update();

        // DEBUG
        printf("%i:%i\n", encoder_left_getVal(), encoder_right_getVal());

        switch (currentControl) {
            case LEFT_VEL: {

                break;
            }
        
            case LEFT_POS: {

                break;
            }
        
            case RIGHT_VEL: {

                break;
            }
        
            case RIGHT_POS: {

                break;
            }
        
            case BOTH_VEL: {

                break;
            }
        
            case BOTH_POS: {

                break;
            }
        
            case DISTANCE: {

                break;
            }
        
            case ANGLE: {

                break;
            }
        
            case POLAR: {

                break;
            }

            default: {
                return;
            }
        }
    }
}

bool controlRunning        = false;
ControlType currentControl = BOTH_POS;
