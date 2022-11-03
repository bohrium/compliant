#ifndef COOR_H
#define COOR_H

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
/*~~  Spatial Coordinates  ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

typedef struct {
    float x,y,z;
} coor_t ;

/*--  vector space structure  -----------------------------------------------*/

coor_t czero();
coor_t c_add(coor_t v, coor_t w);
coor_t c_sub(coor_t v, coor_t w);
coor_t scale(float c, coor_t v);

/*--  metric structure ------------------------------------------------------*/

float norm2(coor_t v);
float dist2(coor_t v, coor_t w);
float norm(coor_t v);
float dist(coor_t v, coor_t w);

/*--  networks  -------------------------------------------------------------*/

// introduce

//float nearest_neighbors(int len, coor_t* v, int idx, int k)
//{
//    for (int i = 0; i != len; ++i) {
//        int so_far = 0;
//        for (int j = 0; j != len; ++j) {
//            if (j==i) { continue; }
//        }
//    }
//}

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
/*~~  Dynamics  ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

typedef struct {
    int len;
    float* mass;
    coor_t* pos;
    coor_t* mom;

    coor_t* force_accum;
    //char which_force_accum; // <- TODO : leapfrog ring buffer
    //coor_t* force_accum[2];
} cloud_t ;

/*--  newton's laws  --------------------------------------------------------*/

void update_mom(float dt, cloud_t parts);
void update_pos(float dt, cloud_t parts);
void reset_forces(cloud_t parts);

/*--  formulas for force  ---------------------------------------------------*/

typedef struct {
    int len;
    float* stiffness;
    float* nat_len;
    int* src;
    int* dst;
} springs_t ;

void sim_springs(cloud_t parts, springs_t springs);
void sim_drag(cloud_t parts, float viscosity);

#endif//COOR_H
