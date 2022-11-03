#include "stdbool.h"
#include "stdio.h"
#include "stdlib.h"

#include "coor.h"

typedef char const* cstr_t ;


    float const T = 1000.  ;
    float const dt =  .01;

    float const viscosity = .001;

    int const nb_parts = 5;
    float mass[] = {1., 1., 1., 1., .1};
    coor_t pos[] =  {{1.,0.,0.}, {2.,0.,0.}, {3.,+.5,0.}, {3.,-.5,0.}, {1.,.5,0.}};
    coor_t mom[] =  {{0.,0.,0.}, {0.,0.,0.}, {0., 0.,0.}, {0., 0.,0.}, {0.,0.,0.}};
    coor_t facc[] = {{0.,0.,0.}, {0.,0.,0.}, {0., 0.,0.}, {0., 0.,0.}, {0.,0.,0.}};

    cloud_t const parts = {.len=nb_parts,
                           .mass=mass,
                           .pos=pos,
                           .mom=mom,
                           .force_accum=facc};

    int const nb_springs = 5;
    float stiffness[] = {.1,.1,.1,.1,.1};
    float nat_len[] = {1.,1.,1.,1.9,.5};
    int src[] = { 0, 1, 1, 2, 0 };
    int dst[] = { 1, 2, 3, 3, 4 };

    springs_t const springs = {.len=nb_springs,
                               .stiffness=stiffness,
                               .nat_len=nat_len,
                               .src=src,
                               .dst=dst};

void display(cloud_t parts, springs_t springs)
{
    int const H = 80;
    int const W = 160;
    float const xmin =  0.;
    float const xmax = +4.;
    float const ymin = -2.;
    float const ymax = +2.;
    char* canvas[H][W+1];
    for (int r = 0; r != H; ++r) {
        for (int c = 0; c != W; ++c) {
            canvas[r][c] = "\033[30m.";
        }
    }

    for (int i = 0; i != springs.len; ++i) {
        for (float t=0.; t<=1.0; t+=0.01) {
            coor_t dir = c_sub(parts.pos[springs.dst[i]],
                               parts.pos[springs.src[i]]);
            coor_t inter = c_add(parts.pos[springs.src[i]],
                                 scale(t, dir));
            int r = (inter.y - ymin)/(ymax - ymin) * H;
            int c = (inter.x - xmin)/(xmax - xmin) * W;

            if ( ! ( 0<=r && r<H && 0<=c && c<W ) ) { continue; }

            float X = fabs(dir.x);
            float Y = fabs(dir.y);
            bool sx = 0<=dir.x;
            bool sy = 0<=dir.y;

            if      (Y <= 0.2*X)           { canvas[r][c] = "\033[36m-"; }
            else if (X <= 0.2*Y)           { canvas[r][c] = "\033[36m|"; }
            else if (Y<=2.0*X && X<=1.5*Y) {
                if   ( sx == sy) { canvas[r][c] = "\033[36m\\"; }
                else             { canvas[r][c] = "\033[36m/"; }
            }
            else                            { canvas[r][c] = "\033[36m+"; }
        }
    }
    for (int i = 0; i != parts.len; ++i) {
        int r = (parts.pos[i].y - ymin)/(ymax - ymin) * H;
        int c = (parts.pos[i].x - xmin)/(xmax - xmin) * W;
        if ( ! ( 0<=r && r<H && 0<=c && c<W ) ) { continue; }
        canvas[r][c] = "\033[33mO";
    }

    for (int r = 0; r != H; ++r) {
        for (int c = 0; c != W; ++c) {
            printf("%s", canvas[r][c]);
        }
        printf("\n");
    }
    for (int r = 0; r != H+2; ++r) {
        printf("\033[1A");
    }
}


int main(int argc, cstr_t const* argv)
{
    printf("welcome!\n");
    printf("PRESS ENTER!\n");

    for (float t=0.; t<T; t+=dt) {
        if ( (int)t != (int)(t+dt) ) {
            printf("\033[36m");
            printf("woah!  %5.1f  :  %+.2f_%+.2f  %+.2f_%+.2f  %+.2f_%+.2f\n",
                   t,
                   parts.pos[0].x, parts.pos[0].y,
                   parts.pos[1].x, parts.pos[1].y,
                   parts.pos[2].x, parts.pos[2].y   );
            getchar();
            display(parts, springs);
        }
        update_pos(dt, parts);

        reset_forces(parts);
        sim_springs(parts, springs);
        sim_drag(parts, viscosity);
        update_mom(dt, parts);
    }

    printf("done!\n");
    return 0;
}
