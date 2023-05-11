#include "core.h"
#include "pico/multicore.h"

#pragma once

namespace simulation {

	// const float c6_thrust[] = { 0.0,0.3296616488367361,0.6293540568701328,0.9290464649035295,1.5537129265930751,2.178379388282621,2.8030458499721647,3.4277123116617085,
	// 4.052378773351253,4.677045235040802,5.70048635201434,6.768235764019432,7.835985176024525,8.903734588029609,9.83490685318774,10.604634208728305,
	// 11.374361564268872,12.144088919809437,12.913816275350005,13.683543630890572,12.615552102518523,11.059411615436026,9.244797988762558,7.430184362089087,
	// 6.680272731131041,6.025990358629265,5.878232151881686,5.730473945134105,5.582715738386525,5.434957531638944,5.340569236627502,5.26905376093156,
	// 5.197538285235617,5.126022809539674,5.054507333843732,4.982991858147789,4.9114763824518475,4.839960906755905,4.797417112904286,4.753176805051734,
	// 4.708936497199182,4.664696189346629,4.6204558814940775,4.576215573641525,4.531975265788973,4.4877349579364205,4.443494650083868,4.399254342231316,
	// 4.365430081490382,4.355909930676445,4.346389779862506,4.336869629048569,4.327349478234631,4.317829327420694,4.308309176606755,4.298789025792818,
	// 4.28926887497888,4.279748724164943,4.270228573351004,4.260708422537067,4.251188271723129,4.241668120909192,4.232147970095254,4.222627819281316,
	// 4.213107668467378,4.203587517653441,4.194067366839503,4.184547216025565,4.235673820177714,4.325644902504867,4.41561598483202,4.372214441596736,
	// 4.195440272799026,4.095964205108133,4.117925650400588,4.139887095693044,4.161848540985499,4.1838099862779545,4.20577143157041,4.227732876862866,
	// 4.249694322155322,4.271655767447777,4.293617212740233,4.315578658032688,4.337540103325144,4.359501548617599,4.365658028763392,4.361277865478106,
	// 4.35689770219282,4.352517538907535,4.348137375622249,4.343757212336964,4.339377049051678,4.334996885766392,4.330616722481107,4.326236559195821,
	// 4.321856395910536,4.31747623262525,4.313096069339964,4.308715906054679,4.304335742769393,4.299955579484108,4.2955754161988216,4.2911952529135355,
	// 4.28681508962825,4.282434926342964,4.278054763057679,4.274988648757993,4.274988648757993,4.274988648757993,4.274988648757993,4.274988648757993,
	// 4.274988648757993,4.274988648757993,4.274988648757993,4.274988648757993,4.274988648757993,4.274988648757993,4.274988648757993,4.274988648757993,
	// 4.274988648757993,4.274988648757993,4.239000215827132,4.14902913349998,4.059058051172826,4.005206795703052,4.0356351362338065,4.066063476764561,
	// 4.096491817295314,4.126920157826069,4.157348498356823,4.1828220493023505,4.188476441906988,4.194130834511625,4.199785227116262,4.2054396197209,
	// 4.211094012325536,4.216748404930174,4.222402797534811,4.228057190139448,4.233711582744085,4.239365975348723,4.24502036795336,4.250674760557997,
	// 4.256329153162635,4.261983545767271,4.267637938371909,4.273292330976546,4.277966227842351,4.282928859649616,4.28789149145688,4.292854123264144,
	// 4.297816755071408,4.302779386878672,4.307742018685936,4.3127046504932,4.317667282300464,4.322629914107728,4.327592545914992,4.332555177722257,
	// 4.337517809529521,4.342480441336785,4.347443073144049,4.352405704951313,4.357368336758578,4.362330968565841,4.367293600373105,4.368286126734564,
	// 4.368286126734564,4.368286126734564,4.368286126734564,4.368286126734564,4.368286126734564,4.368286126734564,4.368286126734564,4.368286126734564,
	// 4.368286126734564,4.368286126734564,4.368286126734564,4.368286126734564,4.368286126734564,4.368286126734564,4.368286126734564,4.139386241496504,
	// 2.994886815306201,1.9086322113992165,0.8007718757503225,0.0,0.0,0.0,0.0,0.0,
	// 0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};

	const float d12_thrust[] = {0.0,0.5896566578293295,1.1257081649469016,1.6617596720644745,2.1978111791820476,2.8341927625961945,3.871894651196643,4.909596539797095,5.9472984283975485,6.9850003169980015,8.022702205598454,
	9.060404094198907,10.173624378675838,11.362363059029285,12.551101739382732,13.73984041973618,14.928579100089626,16.117317780443074,17.306056460796533,18.605658559663237,19.952773552178474,
	21.299888544693715,22.647003537208953,23.99411852972419,25.299703103280766,26.542992048399242,27.786280993517718,29.029569938636193,30.272858883754672,28.728563141396947,26.64752939544121,
	23.41880499085432,19.676144357453115,17.48202050500016,15.962515038167613,14.443009571335075,13.897617714566763,13.352225857798434,12.806834001030104,12.346949585925739,12.111240289029016,
	11.875530992132294,11.639821695235572,11.40411239833885,11.168403101442127,11.004735205870048,10.913108711622646,10.821482217375246,10.729855723127846,10.638229228880446,10.546602734633044,
	10.454976240385644,10.363349746138244,10.271723251890844,10.180096757643442,10.088470263396042,10.063749674854192,10.029866436887673,9.995983198921152,9.962099960954633,9.928216722988113,
	9.894333485021594,9.860450247055075,9.826567009088555,9.792683771122036,9.758800533155515,9.724917295188996,9.691034057222478,9.657150819255957,9.623267581289438,9.589384343322918,
	9.555501105356399,9.521617867389878,9.504629505266815,9.491864862104796,9.47910021894278,9.46633557578076,9.453570932618742,9.440806289456724,9.428041646294705,9.415277003132687,
	9.402512359970668,9.389747716808651,9.376983073646633,9.364218430484614,9.351453787322596,9.338689144160577,9.325924500998559,9.31315985783654,9.302638120087591,9.291484212087129,
	9.280330304086668,9.269176396086205,9.258022488085745,9.246868580085282,9.23571467208482,9.224560764084359,9.213406856083896,9.202252948083435,9.191099040082973,9.17994513208251,
	9.16879122408205,9.157637316081587,9.146483408081126,9.135329500080664,9.124175592080201,9.11302168407974,9.101867776079278,9.092058955413549,9.081192168081515,9.07032538074948,
	9.059458593417446,9.04859180608541,9.037725018753376,9.026858231421341,9.015991444089305,9.005124656757271,8.994257869425237,8.983391082093203,8.972524294761167,8.961657507429132,
	8.950790720097098,8.939923932765064,8.929057145433028,8.918190358100993,8.907323570768959,8.896456783436923,8.877683052850708,8.85552063229858,8.833358211746452,8.811195791194322,
	8.789033370642194,8.766870950090066,8.744708529537938,8.72254610898581,8.700383688433682,8.678221267881554,8.656058847329426,8.633896426777298,8.61173400622517,8.589571585673042,
	8.567409165120914,8.545246744568786,8.523084324016658,8.50092190346453,8.495745825487305,8.494683548534878,8.493621271582452,8.492558994630027,8.4914967176776,8.490434440725174,
	8.48937216377275,8.488309886820323,8.487247609867897,8.486185332915472,8.485123055963046,8.48406077901062,8.482998502058194,8.481936225105768,8.480873948153343,8.1379842263724,
	6.427253586801124,4.716522947229848,3.5994517228859277,2.3582614736149083,1.117071224343891,0.0};

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
			// force += c6_thrust[clamp(int((time)*100.f), 0, 199)];
			force += d12_thrust[clamp(int((time)*100.f), 0, 166)];
			vel += (-9.816 + (force / mass)) * dt;
			pos += vel*dt;
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

		while ( time < 1.7f && pos > 0.0f ) {

			float force = aero_coeff*vel*vel;

			if ( pos < burn_alt ) {
				if ( time_burn_start == 0 ) { time_burn_start = time; }
				// force += c6_thrust[clamp(int((time-time_burn_start)*100.f), 0, 199)];
				force += d12_thrust[clamp(int((time)*100.f), 0, 166)];
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

		while ( time < 1.7f && pos > 0.0f ) {

			// float force = c6_thrust[clamp(int((time)*100.f), 0, 199)]*angle_coeff;
			float force = d12_thrust[clamp(int((time)*100.f), 0, 166)]*angle_coeff;
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