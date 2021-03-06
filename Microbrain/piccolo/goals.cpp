
/*
 * goals.c
 */

#include "defines.h"

#include <xc.h>
#include <stdio.h>
#include <math.h>
#include <libpic30++.h>

#include "bus_interface.h"
#include "color.h"
#include "controller_presets.h"
#include "console.h"
#include "geometry.h"
#include "goals.h"
#include "goal_manager.h"
#include "gpio.h"
#include "routing.h"
#include "servos.h"
#include "wait.h"


typedef enum {
    YELLOW_MOONBASE,
    LEFT_MOONBASE,
    CENTER_MOONBASE,
    RIGHT_MOONBASE,
    BLUE_MOONBASE,
} t_moon_base;


// this class stores what modules the robot has picked
// storing sequence: front_up, front_arm, rear_up, rear_arm
class ModuleStatus {
public:
    ModuleStatus() { reset(); };
    void mark_front_up() { front_up = true;};
    void mark_front_mid() { front_mid = true;};
    void mark_rear_up() { rear_up = true;};
    void mark_rear_mid() { rear_mid = true;};
    void reset() { front_mid = rear_mid = front_up = rear_up = false;  };
    bool no_modules() {
        return !at_least_one();
    };
    bool at_least_one() {
        return front_up || front_mid || rear_up || rear_mid;
    }
    bool full(){
        return front_up && front_mid && rear_up && rear_mid;
    }
    bool half() {
        if(front_up && front_mid)
              return true;
        else if(rear_up && rear_mid)
              return true;
        else return false;
    }
    bool front_up, front_mid, rear_up, rear_mid;
};

static ModuleStatus module_status;


static GoalStart goal_start("start");
static GoalModule5 goal_module_5("module5");
static GoalModule3 goal_module_3("module3");
static GoalModule2 goal_module_2("module2");
static GoalRelease1 goal_release_1("release1");
static GoalDispenser1 goal_dispenser_1("dispenser1");
static GoalRelease2 goal_release_2("release2");
static GoalRelease3 goal_release_3("release3");

bool capture_module_from_dispenser(GraphNode * rest_point, GraphNode * capture_point,  t_side robot_side, bool up)
{
    //int delta_x, delta_y;
    //double capture_theta;
    bool done;

    if (robot_side == REAR_SIDE) {
        // wait if an automation is in progress at rear side
        int s;
        do {
            __delay_ms(200);
            s = automation_rear_status();
            printf("AUTOMATION_REAR_STATUS = %d\n",s);
        }
        while (s == AUTOMATION_STATUS_WORKING);
    }
    else {
        // wait if an automation is in progress at front side
        int s;
        do {
            __delay_ms(200);
            s = automation_front_status();
            printf("AUTOMATION_FRONT_STATUS = %d\n",s);
        }
        while (s == AUTOMATION_STATUS_WORKING);
    }

    arm_prepare_dispenser(robot_side);
    __delay_ms(1200);

    distance_slow.apply();
    // prima vai al dispenser con il braccio giu'
    heading_to(capture_point->pos, (robot_side == FRONT_SIDE) ? HEADING_FRONT : HEADING_BACK);
    forward_to_point(capture_point->pos);
    __delay_ms(200); // necessario per assicurarsi che arrivino i dati dal ServoController
    switch (wait(MotionEvent(), OmronSensorEvent(robot_side, true))) {
    case MOTOR_LOCKED:
        // abbiamo "forzato" sul dispenser, ma non e' scattato il sensore, che e' successo?
        done = false;
        break;
    case PATH_DONE:
        // percorso finito ma il sensore non si e' attivato, non c'e' il modulo
        // o forse l'odometria e' andata a donnine allegre :-(
        // Sarebbe il caso di avvicinarci??
        done = false;
        break;
    case OMRON_SENSOR:
        // ok, il sensore si e' attivato!! controlliamo l'odometria???
        done = true;
        break;
    default:
        break;
    }

    if (done) {
        __delay_ms(400);
    }
    motion_stop();
    __delay_ms(200);

 redo:
    distance_default.apply();
    forward_to_point(rest_point->pos);
    WaitResult r = wait(MotionEvent());
    // se c'e' un ostacolo riprova all'infinto, e' giusto??
    // Si'! E' giusto!!
    if (r != PATH_DONE)
        goto redo;

    if (!done) {
        distance_default.apply();
        servo_reset_automation(robot_side);
        return false;
    }

    bool module_present = (robot_side == FRONT_SIDE) ? omron_front_status() : omron_rear_status();

    if (module_present) {
        if (up) {
            // conserva il modulo in alto
            sucker(robot_side, true);
            arm_save(robot_side, true);
        }
        else {
            // conserva il modulo a meta'
            arm_save(robot_side, false);
        }
    }
    else {
        servo_reset_automation(robot_side);
    }

    distance_default.apply();
    return module_present;
}


void capture_from_dispenser(GraphNode * rest_point, GraphNode * capture_point)
{
    if (capture_module_from_dispenser(rest_point, capture_point, REAR_SIDE, true))
        module_status.mark_rear_up();

    if (capture_module_from_dispenser(rest_point, capture_point, REAR_SIDE, false))
        module_status.mark_rear_mid();

    if (capture_module_from_dispenser(rest_point, capture_point, FRONT_SIDE, true))
        module_status.mark_front_up();

    if (capture_module_from_dispenser(rest_point, capture_point, FRONT_SIDE, false))
        module_status.mark_front_mid();
}



// angle = angolo di scarica iniziale, per evitare alla funzione di ricalcolarlo ogni volta
success_t release_all(GraphNode * base_node, GraphNode * push_node, double angle)
{	
    double heading_angle = symmTifBlue(angle);
    double complementary_heading_angle = normalize_angle_degrees(heading_angle + 180);

    // CS! Aggiunto questo forward per assicurare la corretta posizione per il bump
    forward_to_point(push_node->pos);
    rotate_absolute(heading_angle);

    if (base_node == symmNodeifBlue(&Graph::BASE_1_Y)) {
        bump_and_set_x(symmXifBlue(DIM_H_BACK + 108), heading_angle);

    }
    else if (base_node == symmNodeifBlue(&Graph::BASE_2_Y)) {
        bump();
    }
    else if(base_node == symmNodeifBlue(&Graph::BASE_3_Y)){
	bump_and_set_x(symmXifBlue(1500 + 70 + DIM_H_BACK), heading_angle);	
    }
    Point safe_servo_rotate_point = localToGlobal(push_node->pos, TO_RADIANS(heading_angle), Point(80, 0));

    // CS! Aggiunto
    forward_to_point(push_node->pos);

    rotate_absolute(heading_angle);

    switch (wait(MotionEvent(), TimeoutEvent(3 * 4))) { // wait at most 3 seconds of timeout
    case PATH_DONE:
        motion_stop();
        break;
    case OBSTACLE:
        motion_stop();
        return FAIL;
    case TIMEOUT_EXPIRED:
        motion_stop();
        return FAIL;
    default:
        motion_stop();
        return FAIL;
    }


    // release object at rear mid
    if (module_status.rear_mid) {
        // CS! Rimosso per test
        //     distance_slow.apply();
        // redo:
        //     forward_to_point(base_node->pos);
        //     rotate_absolute(heading_angle);
        //     if (wait(MotionEvent()) != PATH_DONE) {
        //         goto redo;
        //         // infinite loop? ... well, here we should be in contact with the "moon base"
        //         // and we should move the base_node point to perform object release
        //     }
        //     distance_default.apply();
	
        // ********************************** START RELEASE REAR
        arm_release(REAR_SIDE);
        // wait for release completion
        do {
            __delay_ms(300);
        }
        while (automation_rear_status() != AUTOMATION_STATUS_NONE);

#define PUSH_ROTATION (color == YELLOW) ? -60 : 60
        distance_slow.apply();
        // PUSH MODULE 1
        forward_to_point(push_node->pos);
        rotate_relative(PUSH_ROTATION);
        rotate_absolute(heading_angle);

        // removed this! it should be more efficient
        //unchecked_wait(MotionEvent());
    }

    // release object at rear up
    if (module_status.rear_up) {
        // start changing the arm
        arm_change(REAR_SIDE);

        // CS! Rimosso x test
        // forward_to_point(base_node->pos);
        // rotate_absolute(heading_angle);
        // unchecked_wait(MotionEvent());
        // ok the unchecked_wait, the motion command implies to approach the "moon base":
        // no obstacle is there, but a motor-lock event could happen: well, skip it and go further!

        // wait for arm chage completion
        do {
            __delay_ms(300);
        }
        while (automation_rear_status() != AUTOMATION_STATUS_NONE);

        __delay_ms(500);
        // after the motion turn off the sucker
        sucker_rear(false);

        arm_release(REAR_SIDE);
        // wait for release completion
        do {
            __delay_ms(300);
        }
        while (automation_rear_status() != AUTOMATION_STATUS_NONE);

        // PUSH MODULE 2
        distance_slow.apply();
        // CS! Rimosso x test
        //forward_to_point(push_node->pos);
        rotate_relative(PUSH_ROTATION);
        rotate_absolute(heading_angle);
        unchecked_wait(MotionEvent());
    }

    if (module_status.rear_up || module_status.rear_mid) {
        // operations required to close the servos at rear side
        distance_default.apply();
    redo_rotate_point:
        forward_to_point(safe_servo_rotate_point);
        if (wait(MotionEvent()) != PATH_DONE)
            goto redo_rotate_point;
        servo_reset(REAR_SIDE);
        rotate_absolute(complementary_heading_angle);
        // CS! Aggiunto x test
        forward_to_point(push_node->pos);
    }
    else
        rotate_absolute(complementary_heading_angle);

    unchecked_wait(MotionEvent());

    if (module_status.front_mid) {
        // CS! Rimosso x test
        // forward_to_point(base_node->pos);
        // rotate_absolute(complementary_heading_angle);
        // unchecked_wait(MotionEvent());
        // ok the unchecked_wait, the motion command implies to approach the "moon base":
        // no obstacle is there, but a motor-lock event could happen: well, skip it and go further!

        // ********************************** START FRONT MID
        arm_release(FRONT_SIDE);
        // wait for release completion
        do {
            __delay_ms(300);
        }
        while (automation_front_status() != AUTOMATION_STATUS_NONE);

        // PUSH MODULE 3
        distance_slow.apply();
        // CS! Rimosso x test
        //forward_to_point(push_node->pos);
        rotate_relative(PUSH_ROTATION);
        rotate_absolute(complementary_heading_angle);

        // removed this! it should be more efficient
        //unchecked_wait(MotionEvent());
    }

    if (module_status.front_up) {
        arm_change(FRONT_SIDE);

        // CS! Rimosso x test
        // forward_to_point(base_node->pos);
        // rotate_absolute(complementary_heading_angle);
        // unchecked_wait(MotionEvent());
        // ok the unchecked_wait, the motion command implies to approach the "moon base":
        // no obstacle is there, but a motor-lock event could happen: well, skip it and go further!

        // wait for arm change completion
        do {
            __delay_ms(300);
        }
        while (automation_front_status() != AUTOMATION_STATUS_NONE);

        __delay_ms(500);
        sucker_front(false);
        arm_release(FRONT_SIDE);
        // wait for release completion
        do {
            __delay_ms(300);
        }
        while (automation_front_status() != AUTOMATION_STATUS_NONE);
    }

    if (module_status.front_up || module_status.front_mid) {
        // operations required to close the servos at front side
        distance_default.apply();
    redo_rotate_point_2:
        forward_to_point(safe_servo_rotate_point);
        if (wait(MotionEvent()) != PATH_DONE)
            goto redo_rotate_point_2;
        servo_reset(FRONT_SIDE);
    }

    module_status.reset();
    return DONE;
}

GoalStart::GoalStart(const char *name)
    : Goal(name)
{
}

int GoalStart::feasible()
{
    return 0; // Priorità massima
}

success_t GoalStart::execute()
{
    enable_obstacle_avoidance();
    set_obstacle_avoidance_nearonly(false);

    module_status.reset();

    // Primo cilindro
    printf("DIREZIONE CILINDRO 1");

    sucker_front(true);
    arm_capture(FRONT_SIDE, true);

    //__delay_ms(1000);

    GraphNode *p1 = symmNodeifBlue(&Graph::CAP_1_Y);

    GraphNode *p2 = symmNodeifBlue(&Graph::MODULE_1_Y);

    heading_to(p1->pos, HEADING_FRONT);
    forward_to_point(p1->pos);
    //distance_slow.apply();
    heading_to(p2->pos, HEADING_FRONT);
    forward_to_distance(140);
    distance_default.apply();

    module_status.mark_front_up();

    return DONE;

    // unchecked_wait(MotionEvent()); // CHECK!!!

    //  if (automation_front_status() == AUTOMATION_STATUS_WAITING) {
    //      // after the motion, we check if the capture started
    //      // if the automation is still in "WAITING", there is no module to capture
    //      // and we need to reset the automation itself
    //      servo_reset_automation(FRONT_SIDE);
    //  }
    //  else
    //      module_status.mark_front_up();

    // return DONE; // il goal start, se fallisce, non viene piu' schedulato, per tale motivo il return e' sempre DONE
}



//----------------------------------------------------------------------------

GoalModule5::GoalModule5(const char *name)
: Goal(name)
{
}

int GoalModule5::feasible()
{
    // Check che verifica che, davanti, abbiamo lo slot "mid" libero.
    // La execute e' infatti progettata per prendere il modulo "front_mid"
    if (module_status.front_mid)
        return IMPOSSIBLE;

    if (measurePathXY(symmNodeifBlue(&Graph::CAP_5_Y)->pos) == IMPOSSIBLE)
        return IMPOSSIBLE;
    else
        return 1;
}

success_t GoalModule5::execute()
{

     distance_default.apply();
     if (doPath(symmNodeifBlue(&Graph::CAP_5_Y)) == IMPOSSIBLE) {
         printf("\nDIREZIONE: SECONDO CILIDRO IMPOSSIBILE\n\n");
         return DONE; // CS! Non riproviamo piu' il goal
     }
     if (wait(MotionEvent(), BaumerSensorEvent(FRONT_SIDE, 0)) != PATH_DONE) {
         printf("\nOSTACOLO\n\n");
         return DONE; // CS! Non riproviamo piu' il goal
     }

     // wait if an automation is in progress at front side
     while (automation_front_status() == AUTOMATION_STATUS_WORKING) {
         __delay_ms(100);
     }

     // now you can start the automation at front side
     arm_capture(FRONT_SIDE, false);

     GraphNode *point = symmNodeifBlue(&Graph::MODULE_5_Y);
     heading_to(point->pos, HEADING_FRONT);
     distance_slow.apply();
     forward_to_distance(140);
     distance_default.apply();
     if (wait(MotionEvent()) != PATH_DONE) {
         // well... an obstacle here is quite improbabile (if not impossible!)
         // however we could have some problems that could block motors.
         // In this case, mark goal failed and reset automation
         printf("\nOSTACOLO\n\n");
         if (automation_front_status() == AUTOMATION_STATUS_WAITING) {
             // return fail only if the sensor didn't get the module.
             // Do not retry!
             servo_reset_automation(FRONT_SIDE);
             return DONE;
         }
         else {
             // here we are captuing the module, despite the motor lock
             goto goal_done;
         }
     }

     // we go here if the path has been done with success
     if (automation_front_status() == AUTOMATION_STATUS_WAITING) {
         // after the motion, we check if the capture started
         // if the automation is still in "WAITING", there is no module to capture
         // and we need to reset the automation itself.
         // Maybe the module is not here, so we mark the goal done in any case
         servo_reset_automation(FRONT_SIDE);
     }
     else {
     goal_done:
         module_status.mark_front_mid();
     }

     return DONE;
}



//----------------------------------------------------------------------------

GoalModule3::GoalModule3(const char *name)
: Goal(name)
{
}

int GoalModule3::feasible()
{
    // Check che verifica che, dietro,
    // abbiamo almeno uno slot libero.
    // La execute e' infatti progettata per prendere il modulo "rear_up"
    // (se diponibile), altrimenti "rear_mid"
    if (module_status.rear_mid)
        return IMPOSSIBLE;
    if (measurePathXY(symmNodeifBlue(&Graph::CAP_3_Y)->pos) == IMPOSSIBLE)
        return IMPOSSIBLE;
    else
        return 3;
}

success_t GoalModule3::execute()
{

     distance_default.apply();
     if (doPath(symmNodeifBlue(&Graph::CAP_3_Y), true) == IMPOSSIBLE) {
         printf("\nDIREZIONE: TERZO CILIDRO IMPOSSIBILE\n\n");
         return DONE; // CS! Non riproviamo piu' il goal
     }
     if (wait(MotionEvent(), BaumerSensorEvent(REAR_SIDE, 1)) != PATH_DONE) {
         printf("\nOSTACOLO\n\n");
         return DONE; // CS! Non riproviamo piu' il goal
     }

     // now you can start the automation at rear side
     if (!module_status.rear_up) {
         sucker_rear(true);
         arm_capture(REAR_SIDE, true);
     }
     else {
         arm_capture(REAR_SIDE, false);
     }

     GraphNode *point = symmNodeifBlue(&Graph::MODULE_3_Y);
     heading_to(point->pos, HEADING_BACK);
     distance_slow.apply();
     forward_to_distance(-150);
     distance_default.apply();
     if (wait(MotionEvent()) != PATH_DONE) {
         // well... an obstacle here is quite improbabile (if not impossible!)
         // however we could have some problems that could block motors.
         // In this case, mark goal failed and reset automation
         printf("\nOSTACOLO\n\n");
         if (automation_rear_status() == AUTOMATION_STATUS_WAITING) {
             // the sensor didn't get the module. Return DONE in any case.
             servo_reset_automation(REAR_SIDE);
             return DONE;
         }
         else {
             // here we are captuing the module, despite the motor lock
             goto goal_done;
         }
     }

     if (automation_rear_status() == AUTOMATION_STATUS_WAITING) {
         // after the motion, we check if the capture started
         // if the automation is still in "WAITING", there is no module to capture
         // and we need to reset the automation itself
         servo_reset_automation(REAR_SIDE);
     }
     else {
     goal_done:
         if (!module_status.rear_up) {
             module_status.mark_rear_up();
         }
         else {
             module_status.mark_rear_mid();
         }
     }
	
     return DONE;
}



//----------------------------------------------------------------------------

GoalModule2::GoalModule2(const char *name)
: Goal(name)
{
}

int GoalModule2::feasible()
{
    // Check che verifica che, dietro, abbiamo almeno uno slot libero.
    // La execute e' infatti progettata per prendere il modulo "rear_up"
    // (se diponibile), altrimenti "rear_mid"
    if (module_status.rear_mid)
        return IMPOSSIBLE;
    if (measurePathXY(symmNodeifBlue(&Graph::CAP_2_Y)->pos) == IMPOSSIBLE)
        return IMPOSSIBLE;
    else
        return 2;
}

success_t GoalModule2::execute()
{

     distance_default.apply();
     if (doPath(symmNodeifBlue(&Graph::CAP_2_Y), true) == IMPOSSIBLE) {
         printf("\nDIREZIONE: QUARTO CILIDRO IMPOSSIBILE\n\n");
         return DONE; // CS! Non riproviamo piu' il goal
     }
     if (wait(MotionEvent(), BaumerSensorEvent(REAR_SIDE, 1)) != PATH_DONE) {
         printf("\nOSTACOLO\n\n");
         return DONE; // CS! Non riproviamo piu' il goal
     }

     // wait if an automation is in progress at rear side
     while (automation_rear_status() == AUTOMATION_STATUS_WORKING) {
         __delay_ms(100);
     }
     // now you can start the automation at rear side
     if (!module_status.rear_up) {
         sucker_rear(true);
         arm_capture(REAR_SIDE, true);
     }
     else {
         arm_capture(REAR_SIDE, false);
     }
     __delay_ms(500);

     GraphNode *point = symmNodeifBlue(&Graph::MODULE_2_Y);
     heading_to(point->pos, HEADING_BACK);
     distance_slow.apply();
     forward_to_distance(-150);
     distance_default.apply();
     if (wait(MotionEvent()) != PATH_DONE) {
         printf("\nOSTACOLO\n\n");
         if (automation_rear_status() == AUTOMATION_STATUS_WAITING) {
             servo_reset_automation(REAR_SIDE);
             return DONE;
         }
         else {
             goto goal_done;
         }
     }

     if (automation_rear_status() == AUTOMATION_STATUS_WAITING) {
         // after the motion, we check if the capture started
         // if the automation is still in "WAITING", there is no module to capture
         // and we need to reset the automation itself
         servo_reset_automation(REAR_SIDE);
     }
     else {
     goal_done:
         if (!module_status.rear_up) {
             module_status.mark_rear_up();
         }
         else {
             module_status.mark_rear_mid();
         }
     }
	
     distance_default.apply();
 redo:
     forward_to_distance(150);
     if (wait(MotionEvent()) != PATH_DONE) {
         goto redo;
         // infinite loop? ... maybe yes, we must exit from the corner
         // and there is not another way to do it
     }

      __delay_ms(3000); //sincronizzazione grande-piccolo

     return DONE;
}


//----------------------------------------------------------------------------

GoalRelease1::GoalRelease1(const char *name)
: Goal(name)
{
}

int GoalRelease1::feasible()
{
    if (module_status.no_modules())
        return IMPOSSIBLE;
    else {
	if (measurePathXY(symmNodeifBlue(&Graph::BASE_1_Y)->pos) == IMPOSSIBLE)
            return IMPOSSIBLE;
        else
            return 4;
    }
}


success_t GoalRelease1::execute()
{


    GraphNode * base_node = symmNodeifBlue(&Graph::BASE_1_Y);
    GraphNode * push_node = symmNodeifBlue(&Graph::PUSH_BASE_1_Y);

    double angle = 0;

    distance_default.apply();
    if (doPath(base_node, true) == IMPOSSIBLE) {
        printf("\nDIREZIONE: BASE 1 IMPOSSIBILE\n\n");
        return FAIL;
    }
    if (wait(MotionEvent(), BaumerSensorEvent(REAR_SIDE, 1)) != PATH_DONE) {
        printf("\nOSTACOLO\n\n");
        return FAIL;
    }
    
    return release_all(base_node, push_node, angle);
}
//--------------------------------------------------------

GoalDispenser1::GoalDispenser1(const char *name)
: Goal(name)
{
}

int GoalDispenser1::feasible()
{
    // Se il robot ha almeno un modulo caricato" non puo' andare al dispenser.
    if (module_status.at_least_one())
        return IMPOSSIBLE;
    else {
        if (measurePathXY(symmNodeifBlue(&Graph::DISP_1_Y)->pos) == IMPOSSIBLE)	
            return IMPOSSIBLE;	
        else
            return 5;
    }
}

success_t GoalDispenser1::execute()
{
     //int j;
     __delay_ms(8000); //sincronizzazione grande-piccolo

     GraphNode * target = symmNodeifBlue(&Graph::DISP_1_Y);
     GraphNode * target_2 = symmNodeifBlue(&Graph::DISP_1_1_Y);
     GraphNode * dispenser_point = symmNodeifBlue(&Graph::__DISP_1_Y);

     distance_default.apply();
     if (doPath(target, true) == IMPOSSIBLE) {
         printf("\nDIREZIONE: DISPENSER IMPOSSIBILE\n\n");
         return FAIL;
     }
     if (wait(MotionEvent(), BaumerSensorEvent(REAR_SIDE, 1)) != PATH_DONE) {
         printf("\nOSTACOLO\n\n");
         return FAIL;
     }

     forward_to_point(target_2->pos);
     rotate_absolute(90);
     if (wait(MotionEvent()) != PATH_DONE) {
         // fail the goal! here the rotation does not succeded!
         return FAIL;
     }

     capture_from_dispenser(target_2, dispenser_point);

     return DONE;
}




GoalRelease2::GoalRelease2(const char *name)
: Goal(name)
{
}

int GoalRelease2::feasible()
{

    if(module_status.no_modules())
        return IMPOSSIBLE;
    else {
	if (measurePathXY(symmNodeifBlue(&Graph::BASE_2_Y)->pos) == IMPOSSIBLE)
            return IMPOSSIBLE;
        return 6;
    }
}


success_t GoalRelease2::execute()
{
    GraphNode * base_node = symmNodeifBlue(&Graph::BASE_2_Y);
    GraphNode * push_node = symmNodeifBlue(&Graph::PUSH_BASE_2_Y);

    double angle = -45;

    distance_default.apply();
    if (doPath(base_node, true) == IMPOSSIBLE) {
        printf("\nDIREZIONE: BASE 2 IMPOSSIBILE\n\n");
         return FAIL;
    }
    if (wait(MotionEvent(), BaumerSensorEvent(REAR_SIDE, 1)) != PATH_DONE) {
        printf("\nOSTACOLO\n\n");
        return FAIL;
    }
    return release_all(base_node, push_node, angle);
}

GoalRelease3::GoalRelease3(const char *name)
: Goal(name)
{
}

int GoalRelease3::feasible()
{
    if (module_status.no_modules())
            return IMPOSSIBLE;
    else {
	if (measurePathXY(symmNodeifBlue(&Graph::BASE_3_Y)->pos) == IMPOSSIBLE)
            return IMPOSSIBLE;
        else
            return 7;
    }
}

success_t GoalRelease3::execute()
{
    GraphNode * base_node = symmNodeifBlue(&Graph::BASE_3_Y);
    GraphNode * push_node = symmNodeifBlue(&Graph::PUSH_BASE_3_Y);

    double angle = 0;

    distance_default.apply();
    if (doPath(base_node) == IMPOSSIBLE) {
        printf("\nDIREZIONE: BASE 3 IMPOSSIBILE\n\n");
         return FAIL;
    }
    if (wait(MotionEvent(), BaumerSensorEvent(FRONT_SIDE, 0)) != PATH_DONE) {
        printf("\nOSTACOLO\n\n");
        return FAIL;
    }

    return release_all(base_node, push_node, angle);


}

void FunnyAction(void)
{
}


















