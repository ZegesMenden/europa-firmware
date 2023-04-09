#include "core.h"
#include "pico/multicore.h"

#pragma once

namespace simulation {

    const float c6_thrust[] = { 0.0,0.3296616488367361,0.6293540568701328,0.9290464649035295,1.5537129265930751,2.178379388282621,2.8030458499721647,3.4277123116617085,
    4.052378773351253,4.677045235040802,5.70048635201434,6.768235764019432,7.835985176024525,8.903734588029609,9.83490685318774,10.604634208728305,
    11.374361564268872,12.144088919809437,12.913816275350005,13.683543630890572,12.615552102518523,11.059411615436026,9.244797988762558,7.430184362089087,
    6.680272731131041,6.025990358629265,5.878232151881686,5.730473945134105,5.582715738386525,5.434957531638944,5.340569236627502,5.26905376093156,
    5.197538285235617,5.126022809539674,5.054507333843732,4.982991858147789,4.9114763824518475,4.839960906755905,4.797417112904286,4.753176805051734,
    4.708936497199182,4.664696189346629,4.6204558814940775,4.576215573641525,4.531975265788973,4.4877349579364205,4.443494650083868,4.399254342231316,
    4.365430081490382,4.355909930676445,4.346389779862506,4.336869629048569,4.327349478234631,4.317829327420694,4.308309176606755,4.298789025792818,
    4.28926887497888,4.279748724164943,4.270228573351004,4.260708422537067,4.251188271723129,4.241668120909192,4.232147970095254,4.222627819281316,
    4.213107668467378,4.203587517653441,4.194067366839503,4.184547216025565,4.235673820177714,4.325644902504867,4.41561598483202,4.372214441596736,
    4.195440272799026,4.095964205108133,4.117925650400588,4.139887095693044,4.161848540985499,4.1838099862779545,4.20577143157041,4.227732876862866,
    4.249694322155322,4.271655767447777,4.293617212740233,4.315578658032688,4.337540103325144,4.359501548617599,4.365658028763392,4.361277865478106,
    4.35689770219282,4.352517538907535,4.348137375622249,4.343757212336964,4.339377049051678,4.334996885766392,4.330616722481107,4.326236559195821,
    4.321856395910536,4.31747623262525,4.313096069339964,4.308715906054679,4.304335742769393,4.299955579484108,4.2955754161988216,4.2911952529135355,
    4.28681508962825,4.282434926342964,4.278054763057679,4.274988648757993,4.274988648757993,4.274988648757993,4.274988648757993,4.274988648757993,
    4.274988648757993,4.274988648757993,4.274988648757993,4.274988648757993,4.274988648757993,4.274988648757993,4.274988648757993,4.274988648757993,
    4.274988648757993,4.274988648757993,4.239000215827132,4.14902913349998,4.059058051172826,4.005206795703052,4.0356351362338065,4.066063476764561,
    4.096491817295314,4.126920157826069,4.157348498356823,4.1828220493023505,4.188476441906988,4.194130834511625,4.199785227116262,4.2054396197209,
    4.211094012325536,4.216748404930174,4.222402797534811,4.228057190139448,4.233711582744085,4.239365975348723,4.24502036795336,4.250674760557997,
    4.256329153162635,4.261983545767271,4.267637938371909,4.273292330976546,4.277966227842351,4.282928859649616,4.28789149145688,4.292854123264144,
    4.297816755071408,4.302779386878672,4.307742018685936,4.3127046504932,4.317667282300464,4.322629914107728,4.327592545914992,4.332555177722257,
    4.337517809529521,4.342480441336785,4.347443073144049,4.352405704951313,4.357368336758578,4.362330968565841,4.367293600373105,4.368286126734564,
    4.368286126734564,4.368286126734564,4.368286126734564,4.368286126734564,4.368286126734564,4.368286126734564,4.368286126734564,4.368286126734564,
    4.368286126734564,4.368286126734564,4.368286126734564,4.368286126734564,4.368286126734564,4.368286126734564,4.368286126734564,4.139386241496504,
    2.994886815306201,1.9086322113992165,0.8007718757503225,0.0,0.0,0.0,0.0,0.0,
    0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};

    // ============================================================
    // ascent simulation

    struct ascent_sim_input {
        float position;
        float velocity;
        float acceleration;
        float mass;
        float time;
    } ascent_sim_input;

    struct ascent_sim_output {
        float energy;
        float time;
        float position;
        uint32_t time_taken;
    } ascent_sim_output;

    void run_ascent_sim() {

        uint64_t t_start = time_us_64();

        float pos = ascent_sim_input.position;
        float vel = ascent_sim_input.velocity;
        float acc = ascent_sim_input.acceleration;
        float mass = ascent_sim_input.mass;
        float time = ascent_sim_input.time;

        float aero_coeff = 0.0f;
        if ( vel != 0.0 ) { aero_coeff = ( (mass*acc) - clamp(int((time)*100.f), 0, 199) )/(vel*vel); }

        float dt = 0.01f;

        while ( time < 8.0f && pos > -1.0f ) {

            float force = -aero_coeff*vel*vel;
            force += c6_thrust[clamp(int((time)*100.f), 0, 199)];
            vel += (-9.816 + (force / mass)) * 0.01;
            pos += vel*0.01;
            time += dt;

            if ( vel < 0 ) { break; }

        }

        ascent_sim_output.position = pos;
        ascent_sim_output.energy = vel*vel*0.5*mass + mass*pos*9.816;
        ascent_sim_output.time = time;
        ascent_sim_output.time_taken = uint32_t(time_us_64() - t_start);

    }

    // ============================================================
    // landing burn simulation

    struct landing_sim_input {
        float position;
        float velocity;
        float acceleration;
        float mass;
        float burn_alt;
    } landing_sim_input;

    struct landing_sim_output {
        float position;
        float velocity;
        float work_done;
        float time;
        uint32_t time_taken;
    } landing_sim_output;

    void run_landing_sim() {

        uint64_t t_start = time_us_64();

        float pos = landing_sim_input.position;
        float vel = landing_sim_input.velocity;
        float acc = landing_sim_input.acceleration;
        float mass = landing_sim_input.mass;
        float burn_alt = landing_sim_input.burn_alt;
        float work = 0.0f;

        float aero_coeff = 0.0f;
        if ( vel != 0.0 ) { aero_coeff = (mass*acc)/(vel*vel); }

        float time = 0.0f;
        float dt = 0.01f;
        float time_burn_start = 0.0f;

        while ( time < 3.0f && pos > 0.0f ) {

            float force = aero_coeff*vel*vel;

            if ( pos < burn_alt ) {
                if ( time_burn_start == 0 ) { time_burn_start = time; }
                force += c6_thrust[clamp(int((time-time_burn_start)*100.f), 0, 199)];
            }

            vel += (-9.816 + (force / mass)) * 0.01;

            float vdt = vel * 0.01;

            work += force * vdt;
            pos += vdt;

            time += dt;

        }

        landing_sim_output.position = pos;
        landing_sim_output.velocity = vel;
        landing_sim_output.time = time;
        landing_sim_output.work_done = work;
        landing_sim_output.time_taken = uint32_t(time_us_64() - t_start);

    }

    // ============================================================
    // divert simulation

    struct divert_sim_input {
        float position;
        float velocity;
        float acceleration;
        float mass;
        float time_since_burn_start;
        float ang_coeff;
    } divert_sim_input;

    struct divert_sim_output {
        float position;
        float velocity;
        float work_done;
        float time;
        uint32_t time_taken;
    } divert_sim_output;

    void run_divert_sim() {

        uint64_t t_start = time_us_64();

        float pos = divert_sim_input.position;
        float vel = divert_sim_input.velocity;
        float acc = divert_sim_input.acceleration;
        float mass = divert_sim_input.mass;
        float time = divert_sim_input.time_since_burn_start;
        float angle_coeff = divert_sim_input.ang_coeff;
        float work = 0.0f;

        float aero_coeff = 0.0f;
        if ( vel != 0.0 ) { aero_coeff = (mass*acc)/(vel*vel); }

        float dt = 0.01f;
        float time_burn_start = 0.0f;

        while ( time < 2.0f && pos > 0.0f ) {

            float force = c6_thrust[clamp(int((time)*100.f), 0, 199)]*angle_coeff;
            force += aero_coeff*vel*vel;

            vel += (-9.816 + (force / mass)) * 0.01;

            float vdt = vel * 0.01;

            work += force * vdt;
            pos += vdt;

            time += dt;

        }

        divert_sim_output.position = pos;
        divert_sim_output.velocity = vel;
        divert_sim_output.time = time;
        divert_sim_output.work_done = work;
        divert_sim_output.time_taken = uint32_t(time_us_64() - t_start);

    }

}