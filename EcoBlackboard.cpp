#include <stdio.h>
#include <time.h>
#include <math.h>
#include "EcoBlackboard.h"

/****************************
 *
 * Very basic C program to control the bot.
 * Keeping everything simple using simple C constructs
 * as we have to make this run on a microcontroller.
 *
 * Rob Siwicki @ ComparetheMarket.com
 *
 * *******************************/

// added these libs using the IDE import library
#if ARDUINO == 1
#include <EcoSensors.h>
#endif

int (EcoBlackboard::*sensors[NUMSNSRS])(int) = {NULL};

#if ARDUINO == 1
void agnt_take_reading(struct Wm* wm, enum acts act);
#endif

EcoBlackboard::EcoBlackboard()
{
  _currentOrientation = 0;
  _currentX = 0;
  _currentY = 0; 

  fdevopen( &_my_putc, 0);
  
  sensors[S0]   = &EcoBlackboard::_read_0_sonic_real;
  sensors[S90]  = &EcoBlackboard::_read_90_sonic_real;
  sensors[S180] = &EcoBlackboard::_read_180_sonic_real;
  sensors[S270] = &EcoBlackboard::_read_270_sonic_real;
  sensors[OR0]  = &EcoBlackboard::_read_orientation_is_0;
}

void EcoBlackboard::getAction(EcoSensors es, byte* &cmd) {
  _es = es;

  struct Map* beliefs;	
  beliefs = _new_map();

  // debug - need to add defensive code that means function poitners are all valid prior to entry into function
  struct Actn action = _collision_avoidance_str(beliefs, 0, 0, 0, TURN_PROBABILITY);// 0% prob of turn
//  printf("Action is 1  %d with magnitude 10 %d\n", action.c, action.m);

//  byte* cmd;

  if(action.c == MOVE) {
//    printf("forward %d cm\n", action.m);
    Serial.print("forward ");
    Serial.print(action.m);
    Serial.println(" cm");
    cmd = new byte[6];
    cmd[0] = MOVE_ROBOT;
    cmd[1] = (byte) 4;//sizeof action.m;
    cmd[2] = EcoSPI::getByte(0, action.m);
    cmd[3] = EcoSPI::getByte(1, action.m);
    cmd[4] = EcoSPI::getByte(2, action.m);
    cmd[5] = EcoSPI::getByte(3, action.m);
  } else if(action.c == TURN) {
//    printf("turn %d deg\n", action.m);
    Serial.print("turn ");
    Serial.print(action.m);
    Serial.println(" deg");
    cmd = new byte[6];
    cmd[0] = FACE_ROBOT;
    cmd[1] = (byte) 4;//sizeof action.m;
    cmd[2] = EcoSPI::getByte(0, action.m);
    cmd[3] = EcoSPI::getByte(1, action.m);
    cmd[4] = EcoSPI::getByte(2, action.m);
    cmd[5] = EcoSPI::getByte(3, action.m);
  } else if(action.c == NOP) {
//    printf("NOP\n");
    Serial.println("NOP");
    cmd = new byte[2];
    cmd[0] = END_OF_COMMANDS;
    cmd[1] = (byte) 0;//sizeof action.m;
  } else {
//    printf("dunno\n");
    Serial.println("dunno");
    cmd = new byte[2];
    cmd[0] = END_OF_COMMANDS;
    cmd[1] = (byte) 0;//sizeof action.m;

  }

  //return cmd;
}

int EcoBlackboard::_read_0_sonic_real(int opt) {  
  int r = _es.ultrasonic0; 
  return r;
}

int EcoBlackboard::_read_90_sonic_real(int opt) {  
  int r = _es.ultrasonic90; 
  return r;
}

int EcoBlackboard::_read_180_sonic_real(int opt) {  
  int r = _es.ultrasonic180; 
  return r;
}

int EcoBlackboard::_read_270_sonic_real(int opt) {  
  int r = _es.ultrasonic270; 
  return r;
}

int EcoBlackboard::_read_beyond_range(int opt) {
    return MAX_RANGE + 100;
}

int EcoBlackboard::_read_within_range(int opt) {
    return MAX_RANGE - 1;
}

int EcoBlackboard::_read_will_collide(int opt) {
    return COLLISION_THRESHOLD + STRIDE -1;
}

int EcoBlackboard::_read_orientation_is_0(int opt) {
    return 0;
}

int EcoBlackboard::_read_orientation_is_90(int opt) {
    return 90;
}

int EcoBlackboard::_read_orientation_is_180(int opt) {
    return 180;
}

int EcoBlackboard::_read_orientation_is_270(int opt) {
    return 270;
}

int EcoBlackboard::_read_at_range_100(int opt) {
    return 100;
}

int EcoBlackboard::_read_at_range_90(int opt) {
    return 90;
}

int EcoBlackboard::_read_at_range_80(int opt) {
    return 80;
}

int EcoBlackboard::_read_at_range_70(int opt) {
    return 70;
}

unsigned int EcoBlackboard::_pos(int n) {
	int const mask = n >> sizeof(int) * BITS -1;
	unsigned int r = (n + mask) ^ mask;
	return (unsigned) r;
}

struct Map* EcoBlackboard::_new_map() {
    struct Map* m = (struct Map*) malloc(sizeof(struct Map));
    m->size=0;
    m->maxdimx=0; // more efficient on insert
    m->maxdimy=0;
    return m;
}

struct Wm* EcoBlackboard::_new_wm() {
    struct Wm* wm = (struct Wm*) malloc(sizeof(struct Wm));
    wm->size=0;
    return wm;
}

void EcoBlackboard::_map_add_belief(struct Map* m, int x, int y, float temp, int obs) {
    
    struct Hn* point = (struct Hn*) malloc(sizeof(struct Hn));

    point->x=x;
    point->y=y;
#if ARDUINO == 1
    point->t=101;
#else    
    point->t= time(NULL);
#endif
    point->temp_1=temp;
    point->is_obstacle=obs;

    int s=m->size;

    m->points[s] = point; 
    m->maxdimx = x > m->maxdimx ? x : m->maxdimx;
    m->maxdimy = y > m->maxdimy ? y : m->maxdimy;
    m->size++;
}

void EcoBlackboard::_agnt_take_reading(struct Wm* wm, enum acts act) {

    struct LocF* fact = (struct LocF*) malloc(sizeof(struct LocF));
    
#if ARDUINO == 1
    fact->t0 = 101;
#else    
    fact->t0 = time(NULL);
#endif

    fact->s0 = ((this->*sensors[S0])(0));
    fact->s90 = ((this->*sensors[S90])(0));
    fact->s180 = ((this->*sensors[S180])(0));
    fact->s270 = ((this->*sensors[S270])(0));
    fact->ori = ((this->*sensors[OR0])(0));
    fact->precact = act;

//    printf("sensor reading %d\n", fact->s0);
    Serial.print("sensor reading ");
    Serial.println(fact->s0);

    wm->facts[wm->size++] = fact;
}

struct LocF* EcoBlackboard::_wm_get_latest_fact(struct Wm* wm) {
    return wm->facts[wm->size-1];
}

#if ARDUINO != 1
void EcoBlackboard::print_all_beliefs(struct Map* m) {
    
    int i;

    struct Hn* ps = m->points;
  
//    printf("val %d\n",m->points[0]->x);
    Serial.print("val ");
    Serial.println(m->points[0]->x);

    for(i=0;i<m->size; i++) {
	struct Hn* point = m->points[i];
	    
        //printf("data %d @\n", m->points[i]->x);
//        printf("data %d @\n", point->x);
      Serial.print("data ");
      Serial.println(point->x);
    } 
}
#endif

struct vct EcoBlackboard::_lvfp(int d, int m) {

	d = d % 360;

	struct vct v;
	v.x = 0;
	v.y = 0;

	if(d>=0&&d<=90) {
            double r = (90 - d) / (180/PI);
	    v.x = m * cos(r);
	    v.y = m * sin(r);
	}
	else if(d>=90&&d<=180) {
            double r = (d - 90) / (180/PI);
	    v.x = m * cos(r);
	    v.y = m * sin(r) * -1;
	}
	else if(d>=180&&d<=270) {
            double r = (270 - d) / (180/PI);
	    v.x = m * cos(r) * -1;
	    v.y = m * sin(r) * -1;
	}
	else if(d>=270&&d<=360) {
            double r = (d - 270) / (180/PI);
	    v.x = m * cos(r) * -1;
	    v.y = m * sin(r);
	}

	return v;
}

struct Actn EcoBlackboard::_collision_avoidance_str(struct Map* beliefMap, int curOr, int curXBel, int curYBel, int tProb) {

    int* vds =(int*) malloc(sizeof(int) * 4); 
    int svds = 0;

    int test = 10;

    struct Actn next;
    next.c = NOP;
    next.m = 0;

    curOr = (this->*sensors[OR0])(0);

    int COLLISION_INTERCEPT = COLLISION_THRESHOLD + STRIDE;

    if(((this->*sensors[S0])(0)) > COLLISION_INTERCEPT 
		    && ( ((this->*sensors[S0])(0)) < MAX_RANGE ||  ((this->*sensors[S180])(0)) < MAX_RANGE )) {
    	*(vds+svds) = 0;
    	svds++;
    }
   
    if(((this->*sensors[S90])(0)) > COLLISION_INTERCEPT 
		    && ( ((this->*sensors[S90])(0)) < MAX_RANGE ||  ((this->*sensors[S270])(0)) < MAX_RANGE )) {
	    //vds = (int*) realloc(vds, sizeof(int) * svds);
	    *(vds+svds) = 90;
	    svds++;
    }

    if(((this->*sensors[S180])(0)) > COLLISION_INTERCEPT
		    && ( ((this->*sensors[S180])(0)) < MAX_RANGE ||  ((this->*sensors[S0])(0)) < MAX_RANGE )) {
	    //vds = (int*) realloc(vds, sizeof(int) * svds);
	    *(vds+svds) = 180;
	    svds++;
    }
    
    if(((this->*sensors[S270])(0)) > COLLISION_INTERCEPT
		    && ( ((this->*sensors[S270])(0)) < MAX_RANGE ||  ((this->*sensors[S90])(0)) < MAX_RANGE )) {
	    //vds = (int*) realloc(vds, sizeof(int) * svds);
	    *(vds+svds) = 270;
	    svds++;
    }
//printf("svds:%d\n",svds);
Serial.print("svds:");
Serial.println(svds);
#if ARDUINO == 1
    if((random(1,101)) < tProb) {
        int index = random(0,svds);
#else
    if((rand() % 100) < tProb) {
    	int index = rand() % svds;
#endif	    
	next.c = TURN;
   	next.m = *(vds+index);
    } else {

	int i;

	// this is redundant
	for(i=0; i<svds;i++) {
		if(*(vds+i) == curOr) {
                    next.c = MOVE;
		    next.m = STRIDE;
		    break;
		}
	}
    }

    free(vds);
    
    return next;
}

struct Map* EcoBlackboard::_forward_chain_belief_map(struct Wm* wm) {

// need to plot all the obstacles as coordinates in the a' relative system
// at the moment they are curr v relative.

    struct LocF* earliestFact = wm->facts[0]; // get the first fact and chain forwards
    struct LocF* previousFact = wm->facts[wm->size-2];

    struct Map* beliefs = _new_map();

    struct vct curVct;
    curVct.x = 0;
    curVct.y = 0;

    int i;

    for(i=0;i<wm->size; i++) {
	struct LocF* currentFact = wm->facts[i];
	
	int delta0 = previousFact->s0 - currentFact->s0;
	int delta90 = previousFact->s90 - currentFact->s90;
	int delta180 = previousFact->s180 - currentFact->s180;
	int delta270 = previousFact->s270 - currentFact->s270;

	int dOr = previousFact->ori - currentFact->ori;

//	printf("precact %d\n", currentFact->precact);
        Serial.print("precact ");
        Serial.println(currentFact->precact);

        switch(currentFact->precact) {
		case TURN:
		    if(currentFact->s0 < MAX_RANGE) {
			    struct vct relVct = _lvfp(dOr,currentFact->s0);
			    _map_add_belief(beliefs, curVct.x+relVct.x, curVct.y+relVct.y, 1.0, OBS);
		    };
		   
		    if(currentFact->s90 < MAX_RANGE) {
			    struct vct relVct = _lvfp(dOr+90, currentFact->s90);
			    _map_add_belief(beliefs, curVct.x+relVct.x, curVct.y+relVct.y, 1.0, OBS);
		    };

		    if(currentFact->s180 < MAX_RANGE) {
			    struct vct relVct = _lvfp(dOr+180, currentFact->s180);
			    _map_add_belief(beliefs, curVct.x+relVct.x, curVct.y+relVct.y, 1.0, OBS);
		    };

		    if(currentFact->s270 < MAX_RANGE) {
			    struct vct relVct = _lvfp(dOr+270, currentFact->s270);
			    _map_add_belief(beliefs, curVct.x+relVct.x, curVct.y+relVct.y, 1.0, OBS);
		    };
		break;
		case MOVE:
		    // could add error correction here
		    if(_pos(delta0) < 1 && _pos(delta90) < 1 && _pos(delta180) < 1 && _pos(delta270) < 1) {
//		        printf("no movement detected post move command.\n");	
                        Serial.println("no movement detected post move command.");
			break;
		    }
		
		    struct vct dVct; 

		    // object should have moved closer on 0	    
		    // this means an increase in the delta
		    // e.g. if previous s0 100 and current s0 95 then we have moved forward 5
		    // at current bearing
		    // if delta0 is unavailable then we have to use delta 180 in which case
		    // the delta should be negative i.e. we have moved away from object
		    // by the amount of delta. Therefore new location vector is abs(delta)
		    // but need to add 180 degrees
                    Serial.print("delta0 ");
                    Serial.println(delta0);

//		    printf("delta0 %d\n", delta0);
		    if(delta0 > MEASURE_TOLERANCE) {
			  dVct  = _lvfp(currentFact->ori, delta0); 
//			  printf("Movement detected in forward sensor %d\n", delta0);
                          Serial.print("Movement detected in forward sensor ");
                          Serial.println(delta0);
		    }		    
		    else if(delta180 < MEASURE_TOLERANCE) {
			  dVct  = _lvfp(currentFact->ori + 180, _pos(delta180)); 
//			  printf("Movement detected in aft sensor %d\n", delta180);
                          Serial.print("Movement detected in aft sensor ");
                          Serial.println(delta180);
		    }		    
		    else {
//			printf("PANIC! In dev null. You shouldn't let agent turn 90 if there is no contact on 90 or 270. You really should add that to the collision strategy.\n");
                        Serial.println("PANIC! In dev null. You shouldn't let agent turn 90 if there is no contact on 90 or 270. You really should add that to the collision strategy.");
		    }		    

                    Serial.print("dVct.x is ");
                    Serial.println(dVct.x);
                    Serial.print("dVct.y is ");
                    Serial.println(dVct.y);

//		    printf("dVct.x is %d\n", dVct.x);	
//		    printf("dVct.y is %d\n", dVct.y);	

		    curVct.x += dVct.x;
		    curVct.y += dVct.y;
		    
		    if(currentFact->s0 < MAX_RANGE) {
			    struct vct relVct = _lvfp(dOr,currentFact->s0);
			    _map_add_belief(beliefs, curVct.x+relVct.x, curVct.y+relVct.y, 1.0, OBS);
		    };
		   
		    if(currentFact->s90 < MAX_RANGE) {
			    struct vct relVct = _lvfp(dOr+90, currentFact->s90);
			    _map_add_belief(beliefs, curVct.x+relVct.x, curVct.y+relVct.y, 1.0, OBS);
		    };

		    if(currentFact->s180 < MAX_RANGE) {
			    struct vct relVct = _lvfp(dOr+180, currentFact->s180);
			    _map_add_belief(beliefs, curVct.x+relVct.x, curVct.y+relVct.y, 1.0, OBS);
		    };

		    if(currentFact->s270 < MAX_RANGE) {
			    struct vct relVct = _lvfp(dOr+270, currentFact->s270);
			    _map_add_belief(beliefs, curVct.x+relVct.x, curVct.y+relVct.y, 1.0, OBS);
		    };
		break;
		case NOP:
		    if(currentFact->s0 < MAX_RANGE) {
			    struct vct relVct = _lvfp(dOr,currentFact->s0);
			    _map_add_belief(beliefs, curVct.x+relVct.x, curVct.y+relVct.y, 1.0, OBS);
		    };
		   
		    if(currentFact->s90 < MAX_RANGE) {
			    struct vct relVct = _lvfp(dOr+90, currentFact->s90);
			    _map_add_belief(beliefs, curVct.x+relVct.x, curVct.y+relVct.y, 1.0, OBS);
		    };

		    if(currentFact->s180 < MAX_RANGE) {
			    struct vct relVct = _lvfp(dOr+180, currentFact->s180);
			    _map_add_belief(beliefs, curVct.x+relVct.x, curVct.y+relVct.y, 1.0, OBS);
		    };

		    if(currentFact->s270 < MAX_RANGE) {
			    struct vct relVct = _lvfp(dOr+270, currentFact->s270);
			    _map_add_belief(beliefs, curVct.x+relVct.x, curVct.y+relVct.y, 1.0, OBS);
		    };
		break;
     
                Serial.println("no operation.");
//  		printf("no operation.\n");
		default:
                  Serial.println("Unknown command.");
//		   printf("Unknown command.\n");
		break;
	}	

        //printf("data %d @\n", m->points[i]->x);
        
        Serial.print("sensor reading in build map previous fact ");
        Serial.println(previousFact->s0);
        Serial.print("sensor reading in build map current fact ");
        Serial.println(currentFact->s0);
        Serial.print("current position x ");
        Serial.println(curVct.x);
        Serial.print("current position y ");
        Serial.println(curVct.y);

//        printf("sensor reading in build map previous fact %d \n", previousFact->s0);
//        printf("sensor reading in build map current fact %d \n", currentFact->s0);
//    	printf("current position x %d\n", curVct.x);
//    	printf("current position y %d\n", curVct.y);
    } 

    return beliefs;
}

#if ARDUINO == 1
int EcoBlackboard::_freeRam () {
  extern int __heap_start, *__brkval;
  int v; 
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval); 
}
#endif

void EcoBlackboard::_print_belief_map(struct Map* map) {
  // sprintf
  Serial.println("Printing belief map");
//  printf("Printing belief map\n");

  int i;

  for(i=0;i<map->size;i++) {
    struct Hn* point = map->points[i];
    Serial.print(point->x);
    Serial.print(",");
    Serial.print(point->y);
    Serial.print(",");
    Serial.println(point->is_obstacle);
    
//    printf("%d,%d,%d\n",point->x, point->y, point->is_obstacle);
  }
}

// dont want to use a fixed array as it limits the number of points
// currently memory is the constraint so use a sparse matrix.
//int EcoBlackboard::rundiagnostic() {
//  printf("---------------------------------------------\n");
//  printf("Running Diagnostic\n");
//  printf("---------------------------------------------\n");
//  printf("charbits are %d\n", BITS);
//  printf("pos should be positive %d\n", _pos(-10));
//  printf("size of LocF %d\n", sizeof(struct LocF));
//  printf("size of Hn %d\n", sizeof(struct Hn));
//  printf("---------------------------------------------\n");
//  printf("testing vectors\n");
//  printf("---------------------------------------------\n");
//  printf("vec test x is 1 %d\n", _lvfp(90,1).x);
//  printf("vec test y is 0 %d\n", _lvfp(90,1).y);
//  printf("vec test x is 2 %d\n", _lvfp(90,2).x);
//  printf("vec test y is 0 %d\n", _lvfp(90,2).y);
//  printf("vec test x is 0 %d\n", _lvfp(0,1).x);
//  printf("vec test y is 1 %d\n", _lvfp(0,1).y);
//  printf("vec test x is 0 %d\n", _lvfp(180,1).x);
//  printf("vec test y is -1 %d\n", _lvfp(180,1).y);
//  printf("vec test x is -1 %d\n", _lvfp(270,1).x);
//  printf("vec test y is 0 %d\n", _lvfp(270,1).y);
//		
//  struct vct dVct  = _lvfp(0, 10); 
//  printf("This should be  0 %d\n", dVct.x);
//  printf("This should be 10 %d\n", dVct.y);
//
//  printf("---------------------------------------------\n");
//  printf("Loading sensor array with mocks\n");
//  printf("---------------------------------------------\n");
//	
//  sensors[S0] = &EcoBlackboard::_read_within_range;
//  sensors[S90] = &EcoBlackboard::_read_within_range;
//  sensors[S180] = &EcoBlackboard::_read_within_range;
//  sensors[S270] = &EcoBlackboard::_read_within_range;
//  sensors[T0] = &EcoBlackboard::_read_within_range;
//  sensors[OR0] = &EcoBlackboard::_read_orientation_is_0;
//
//  printf("---------------------------------------------\n");
//  printf("Testing map\n");
//  printf("---------------------------------------------\n");
//	
//  struct Map* beliefs;	
//  beliefs = _new_map();
//
//  _map_add_belief(beliefs, 10,20, 1.0, 0);
//  _map_add_belief(beliefs, 100, 20, 1.0, 1);
//  _map_add_belief(beliefs, 90, 20, 1.0, 1);
//  _map_add_belief(beliefs, 100, 20, 1.0, 1);
//	
//  printf("belief size 4 %d\n",beliefs->size);
//
//  free(beliefs);
//	
//  printf("---------------------------------------------\n");
//  printf("Testing collision avoidance strategy\n");
//  printf("---------------------------------------------\n");
//  printf("Mocking sensors for collision at 0 90 180 100 pct probability of turn\n");
//	
//  beliefs = _new_map();
//
//  _map_add_belief(beliefs, 10,20, 1.0, 0);
//  _map_add_belief(beliefs, 100, 20, 1.0, 1);
//  _map_add_belief(beliefs, 90, 20, 1.0, 1);
//  _map_add_belief(beliefs, 100, 20, 1.0, 1);
//	
//  sensors[S0] = &EcoBlackboard::_read_will_collide;
//  sensors[S90] = &EcoBlackboard::_read_within_range;
//  sensors[S180] = &EcoBlackboard::_read_will_collide;
//  sensors[S270] = &EcoBlackboard::_read_will_collide;
//  sensors[OR0] = &EcoBlackboard::_read_orientation_is_0;
//	
//  struct Actn action = _collision_avoidance_str(beliefs, 0, 0, 0, 100); // 100% prob of turn
//
//  printf("Action is 0 %d with magnitude 90 %d\n", action.c, action.m);
//
//  sensors[S0] = &EcoBlackboard::_read_will_collide;
//  sensors[S90] = &EcoBlackboard::_read_will_collide;
//  sensors[S180] = &EcoBlackboard::_read_will_collide;
//  sensors[S270] = &EcoBlackboard::_read_within_range;
//  sensors[OR0] = &EcoBlackboard::_read_orientation_is_0;
//
//  action = _collision_avoidance_str(beliefs, 0, 0, 0, 100); // 100% prob of turn
//
//  printf("Action is 0 %d with magnitude 270 %d\n", action.c, action.m);
//
//  sensors[S0] = &EcoBlackboard::_read_will_collide;
//  sensors[S90] = &EcoBlackboard::_read_will_collide;
//  sensors[S180] = &EcoBlackboard::_read_will_collide;
//  sensors[S270] = &EcoBlackboard::_read_within_range;
//  sensors[OR0] = &EcoBlackboard::_read_orientation_is_270;
//
//  action = _collision_avoidance_str(beliefs, 0, 0, 0, 0); // 0% prob of turn
//
//  printf("Action is 1 %d with magnitude 10 %d\n", action.c, action.m);
//
//  sensors[S0] = &EcoBlackboard::_read_will_collide;
//  sensors[S90] = &EcoBlackboard::_read_beyond_range;
//  sensors[S180] = &EcoBlackboard::_read_within_range;
//  sensors[S270] = &EcoBlackboard::_read_beyond_range;
//  sensors[OR0] = &EcoBlackboard::_read_orientation_is_0;
//
//  action = _collision_avoidance_str(beliefs, 0, 0, 0, 100); // 0% prob of turn
//
//  printf("Action is 0  %d with magnitude 180 %d\n", action.c, action.m);
//
//  sensors[S0] = &EcoBlackboard::_read_will_collide;
//  sensors[S90] = &EcoBlackboard::_read_will_collide;
//  sensors[S180] = &EcoBlackboard::_read_will_collide;
//  sensors[S270] = &EcoBlackboard::_read_within_range;
//  sensors[OR0] = &EcoBlackboard::_read_orientation_is_270;
//
//  action = _collision_avoidance_str(beliefs, 0, 0, 0, 0); // 0% prob of turn
//
//  printf("Action is 1 %d with magnitude %d %d\n", action.c, STRIDE, action.m);
//
//  free(beliefs);
//	
//  printf("---------------------------------------------\n");
//  printf("Testing working memory\n");
//  printf("---------------------------------------------\n");
//
//  sensors[S0] = &EcoBlackboard::_read_will_collide;
//  sensors[S90] = &EcoBlackboard::_read_beyond_range;
//  sensors[S180] = &EcoBlackboard::_read_within_range;
//  sensors[S270] = &EcoBlackboard::_read_beyond_range;
//  sensors[OR0] = &EcoBlackboard::_read_orientation_is_0;
//	
//  struct Wm* wm = _new_wm();
//
//  _agnt_take_reading(wm, TURN);
//
//  struct LocF* lf = _wm_get_latest_fact(wm);
//  printf("Wm size %d\n", wm->size);
//  printf("latest clock reading %d\n", lf->t0);
//  printf("latest S0 reading %d\n", lf->s0);
//	
//  free(wm);
//
//  beliefs = _new_map();
//
//  sensors[S0] = &EcoBlackboard::_read_at_range_100;
//  sensors[S90] = &EcoBlackboard::_read_will_collide;
//  sensors[S180] = &EcoBlackboard::_read_will_collide;
//  sensors[S270] = &EcoBlackboard::_read_will_collide;
//  sensors[OR0] = &EcoBlackboard::_read_orientation_is_0;
//
//  _agnt_take_reading(wm,NOP);
//
//  _print_belief_map(beliefs);
//  //struct BeliefMap* m1 = forward_chain_belief_map(wm);
//  // seg fault
//  	
//  // force it to move forwards
//  action = _collision_avoidance_str(beliefs, 0, 0, 0, 0); // 0% prob of turn
//  printf("Action is 1  %d with magnitude 10 %d\n", action.c, action.m);
//
//  sensors[S0] = &EcoBlackboard::_read_at_range_90;
//
//  _agnt_take_reading(wm,action.c);
//
//  struct Map* m2 = _forward_chain_belief_map(wm);
//
//  _print_belief_map(m2);
//
//  action = _collision_avoidance_str(beliefs, 0, 0, 0, 0); // 0% prob of turn
//  printf("Action is 1  %d with magnitude 10 %d\n", action.c, action.m);
//
//  sensors[S0] = &EcoBlackboard::_read_at_range_80;
//
//  _agnt_take_reading(wm,action.c);
//
//  struct Map* m3 = _forward_chain_belief_map(wm);
//
//  _print_belief_map(m3);
//
//  action = _collision_avoidance_str(beliefs, 0, 0, 0, 0); // 0% prob of turn
//  printf("Action is 1  %d with magnitude 10 %d\n", action.c, action.m);
//
//  sensors[S0] = &EcoBlackboard::_read_at_range_70;
//
//  _agnt_take_reading(wm,action.c);
//
//  struct Map* m4 = _forward_chain_belief_map(wm);
//
//  _print_belief_map(m4);
//
//#if ARDUINO == 1
//  printf("Compiled for Arduino\n");
//#else
//  printf("gcc compiled\n");
//#endif
//
//  return 0;
//}


#if ARDUINO != 1
int main() {
  return rundiagnostic();
}
#else

int EcoBlackboard::_readLine(char str[]) {
  char c;
  int index =0;
	         
  while(true) {
    if(Serial.available() > 0){
      c = Serial.read();
      if(c!='\n') {
        str[index++] = c;
      } else
        str[index] = '\0';
        break;
      }
  }
  
  return index;
}

int EcoBlackboard::_my_putc( char c, FILE *t) {
  Serial.write( c );
}

#endif
