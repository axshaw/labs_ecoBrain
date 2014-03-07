#ifndef EcoBlackboard_h
#define EcoBlackboard_h

#include <EcoSPI.h>
#include <EcoSensors.h>
#include <time.h>

#define MAX_POINTS 1000
#define BITS 8
#define MAX_RANGE 300
#define PI 3.14159265
#define OBS 1
#define MEASURE_TOLERANCE 2
#define STRIDE 10
#define COLLISION_THRESHOLD 5
#define TURN_PROBABILITY 30
#define ARDUINO 1

#if ARDUINO == 1
typedef int time_t;
#endif

enum snsrs {S0,S90,S180,S270,OR0,T0,TEMP0,NUMSNSRS};
enum acts {TURN,MOVE,NOP,NUMACTS};

typedef enum acts (*funcStrategy) (struct Map*, int, int, int);

typedef int (*funcSensor) (int);

struct Actn {
    enum acts c;
    int m;
};

struct Wm {
    struct LocF* facts[MAX_POINTS];
    int size;
};

struct vct {
    int x;
    int y;
};

struct LocF {
    time_t t0;
    int s0; // MAX range is 300
    int s90;
    int s180;
    int s270;
    int temp_1;
    int ori;
    enum acts precact;
};	

struct Hn {
    int x;
    int y;
    time_t t;
    float temp_1;
    int is_obstacle; // consider uchar bit
};

struct Map {
    struct Hn* points[MAX_POINTS];
    int size;
    int maxdimx;
    int maxdimy;
};

class EcoBlackboard
{
  public:
    EcoBlackboard();
    void getAction(EcoSensors es, byte* &cmd);
    //int rundiagnostic();
  private:
    EcoSensors _es;
    int _currentOrientation;
    int _currentX;
    int _currentY;
    struct Map* _new_map();
    void _print_belief_map(struct Map* map);
    struct Map* _forward_chain_belief_map(struct Wm* wm);
    struct Actn _collision_avoidance_str(struct Map* beliefMap, int curOr, int curXBel, int curYBel, int tProb);
    struct vct _lvfp(int d, int m);
    struct LocF* _wm_get_latest_fact(struct Wm* wm);
    void _agnt_take_reading(struct Wm* wm, enum acts act);
    void _map_add_belief(struct Map* m, int x, int y, float temp, int obs);
    unsigned int _pos(int n);
    struct Wm* _new_wm();
    int _read_beyond_range(int opt);
    int _read_within_range(int opt);
    int _read_will_collide(int opt);
    int _read_orientation_is_0(int opt);
    int _read_orientation_is_90(int opt);
    int _read_orientation_is_180(int opt);
    int _read_orientation_is_270(int opt);
    int _read_at_range_100(int opt);
    int _read_at_range_90(int opt);
    int _read_at_range_80(int opt);
    int _read_at_range_70(int opt);
    int _read_0_sonic_real(int opt);
    int _read_90_sonic_real(int opt);
    int _read_180_sonic_real(int opt);
    int _read_270_sonic_real(int opt);
#if ARDUINO == 1
    int _freeRam();
    int _readLine(char str[]);
    static int _my_putc( char c, FILE *t);
#else
    void print_all_beliefs(struct Map* m);
#endif
};

#endif
