#define FLAG_NO_VALID_ALTITUDE    1
#define FLAG_RA_TRIGGERED_CLIMB   2
#define FLAG_RA_TRIGGERED_DESCEND 4

#include <math.h>

#include "user.h"

#include "simple_random.h"

typedef enum result_e {
    INHIBIT,
    CLEAR,
    COC,
    TA,
    RA_GENERIC, /* Used only internally */
    RA_CLIMB,
    RA_CLIMB_HIGH,
    RA_XING_CLIMB,
    RA_CLIMB_NOW,
    RA_DESCENT,
    RA_DESCENT_HIGH,	/*10*/
    RA_XING_DESCENT,
    RA_DESCENT_NOW,
    RA_MONITOR_VS,
    RA_MAINTAIN_VS,
    RA_XING_MAINTAIN_VS,
    RA_LEVELOFF
} result_t;


typedef struct plane_s {
    unsigned int identifier;    /* 24-bit TCAS identifier */
    int    baroalt;     /* [feet] in STD 29.92 */
    int    radioalt;    /* [feet] valid only for the own aircraft - not used for intruders. If not */
                        /*        avail, it must be set to 3000 or higher */
    int    vert_spd;    /* [feet/min] */
    float  slant_range; /* [feet] valid only for intruders */
    float  bearing;     /* [°] valid only for intruders */

    float delta_update_time; /*  [s] The time since the last call. It must be strictly > 0 */
    unsigned char flags;/* See constants defined above */

    /* The following fields are reserved for internal use, do not use.
     * Their values must be keep across calls. */
    unsigned char _hmdf_hits; /* init=0 */
    result_t _prev_decision; /* init=CLEAR */
    unsigned char _state; /* Must be initialized at 0  */
    float _closure_prev;  /* Must be initialized at -1 */
    float _heading_prev;  /* Must be initialized at `heading`  */
    float _hmdf_Rp;   /* MDF Predicted range, init=0 */
    float _hmdf_Vp;   /* MDF Predicted speed, init=0 */
    float _hmdf_Ap;   /* MDF Predicted acceleration, init=0 */
    float _hmdf_diff_ApAs; /* MDF Predicted acceleartion delta */
    float _hmdf_E;    /* MDF error, init=0 */
    float _hmdf_RPX;  /* Predicted position X */
    float _hmdf_RPY;  /* Predicted position Y */
    float _hmdf_VPX;  /* Predicted velocity X */
    float _hmdf_VPY;  /* Predicted velocity Y */
    float _hmdf_cov[2][2] /* Coveriance matrix, initialize at 0 */;
    float _hmdf_dAs;  /* Maneuvre filtered hmdf_diff_ApAs  */
} plane_t;

static result_t acas_bench(plane_t *own_aircraft, plane_t *intruders, int nr_intruders);
static void initialize_acf(plane_t *acf);

#if defined(ACAS_TO_HARDEN) && !defined(TO_HARDEN_FUNC)
result_t __attribute__((annotate("to_harden"))) results[ITERATIONS] = {0};
crc __attribute__((annotate("to_harden"))) crc_acas;
#else
result_t results[ITERATIONS] = {0};
crc crc_acas;
#endif

MEASURE_GLOBAL_VARIABLES()

/* ************************************************************************ */
/*                              CONSTANTS                                   */
/* ************************************************************************ */

#ifndef M_PI
    /* This is not actually defined by the ANSI C standard */
    #define M_PI 3.14159265358979323846
#endif

#define M_TO_FEET(x) (3.28084 * x)

#define ARBITRARY_LARGE_TAU 1e9
#define STATE_WAS_ACTIVE 1
#define TAB_TA 0
#define TAB_RA 1

#define SIGN(x) ((x) == 0 ? 0 : ((x) > 0 ? 1 : -1))
#define ABS(x) ((x) >= 0 ? (x) : -(x) )
#define SAFE_DEN(x) ((x == 0.0) ? (1e-37): (x))
#define SAFE_DEN_INT(x) ((x == 0) ? (1): (x))

#define MAX(a,b) ((a) > (b) ? (a) : (b))
#define MIN(a,b) ((a) < (b) ? (a) : (b))

const int TAU[2][7] = {
    {20, 25, 30, 40, 45, 48, 48},
    {-1, 15, 20, 25, 30, 35, 35}
};

const int TVTHR[7] = {-1, 15, 20, 25, 30, 35, 35};

const float DMOD[2][7] = {
    {0.30, 0.33, 0.48, 0.75, 1.00, 1.30, 1.30},
    {-1,   0.20, 0.35, 0.55, 0.80, 1.10, 1.10}
};

const int ZTHR[2][7] = {
    {850, 850, 850, 850, 850, 850, 1200},
    {-1,  600, 600, 600, 600, 700, 800 },
};

const int ALIM[7] = {-1, 300, 300, 350, 400, 600, 700};

#define RA_SENSE_INITIAL_G   0.25F
#define RA_SENSE_INITIAL_SEC 5.0F
#define RA_SENSE_INITIAL_FPM 1500
#define RA_SENSE_SUBSEQ_G    0.35F
#define RA_SENSE_SUBSEQ_SEC  2.5F
#define RA_SENSE_SUBSEQ_FPM  2500

#define HMDF_MAX_HITS 8
#define HMDF_ALPHA_PRECISION 0.01f
const unsigned short HMDF_a[9] = {1000, 1000, 830, 700, 600, 460, 400, 400, 400};
const unsigned short HMDF_b[9] = {0,    1000, 500, 300, 200, 110, 100, 100, 100};
const unsigned short HMDF_c[9] = {0,       0, 160,  70,  35,  13,  10,  10,  10};

const float HMDF_BBT_a[9] = {1.0, 1.0, 0.83, 0.70, 0.60, 0.46, 0.39, 0.31, 0.278};
const float HMDF_BBT_b[9] = {0.0, 1.0, 0.50, 0.30, 0.20, 0.11, 0.07, 0.05, 0.0453};

#define MANEUVRE_ALPHA 0.1

/* ************************************************************************ */
/*                              FUNCTIONS                                   */
/* ************************************************************************ */
static unsigned int table_index(int baro_alt, int radio_alt) {
    if (radio_alt < 2600) {
        if (radio_alt < 1000) {
            return 0;
        }
        else if (radio_alt < 2350) {
            return 1;
        }
        else {
            return 2;
        }
    } else {
        if (baro_alt < 5000) {
            return 2;
        }
        else if (baro_alt < 10000) {
            return 3;
        }
        else if (baro_alt < 20000) {
            return 4;
        }
        else if (baro_alt < 42000) {
            return 5;
        }
        else {
            return 6;
        }
    }
}

static int quantize_altitude(int altitude) {    /* Altitude of intruders is quantized at 25-feet */
    int remainder = altitude % 25;
    altitude = altitude - remainder;
    if (remainder >= 13) {
        altitude = altitude + 25;
    }
    return altitude;
}

static int estimate_intruder_alt(const plane_t *own_aircraft, const plane_t *intruder) {
    return intruder->baroalt - (own_aircraft->baroalt - own_aircraft->radioalt);
}

static float time_to_go_CPA(plane_t *intruder, float dmod) {
    float slant_range = intruder->slant_range;
    float closure_speed = 0;

    if(intruder->_closure_prev < 0 ) {
        /* First point, we need to save the current distance to compute the CPA */
        intruder->_closure_prev = slant_range;
    }

    closure_speed = (intruder->_closure_prev - slant_range) / intruder->delta_update_time;

    if (closure_speed <=0) {
        return ARBITRARY_LARGE_TAU;
    }

    return (slant_range-dmod) / SAFE_DEN(closure_speed);
}

static unsigned char altitude_test(const plane_t *own_aircraft, const plane_t *intruder, int type, int idx_table) {
    int projected_alt_time, closure_rate;
    int own_vs = own_aircraft->vert_spd;
    int int_vs = intruder->vert_spd;
    int alt_diff = ABS(own_aircraft->baroalt - intruder->baroalt);
    
    if (alt_diff <= ZTHR[type][idx_table]) {
        return 1;
    }

    closure_rate = ABS(own_vs - int_vs) * 60;  /* feet/s */
    projected_alt_time = alt_diff / SAFE_DEN_INT(closure_rate);

    if (own_vs < 600 || (SIGN(own_vs) == SIGN(int_vs) && (ABS(own_vs) < ABS(int_vs)))) {
        if (projected_alt_time < TVTHR[idx_table]) {
            return 1;
        }
    } else {
        if (projected_alt_time < TAU[type][idx_table]) {
            return 1;
        }
    }

    return 0;
}

static void hmdf_reset(plane_t *acf) {
    acf->_hmdf_Rp = 0;
    acf->_hmdf_Vp = 0;
    acf->_hmdf_Ap = 0;
    acf->_hmdf_diff_ApAs = 0;
    acf->_hmdf_E = 0;
    acf->_hmdf_RPX = 0;
    acf->_hmdf_RPY = 0;
    acf->_hmdf_VPX = 0;
    acf->_hmdf_VPY= 0 ;
    acf->_hmdf_cov[0][0] = acf->_hmdf_cov[0][1] = 0;
    acf->_hmdf_cov[1][0] = acf->_hmdf_cov[1][1] = 0;
    acf->_hmdf_dAs = 0;
    acf->_hmdf_hits = 0;
}

static float hmdf_parabolic_range_est(const plane_t *own_aircraft, plane_t *intruder, float *As, float *Vs, float *Rs) {
    float Re, curr_hit_ratio;
    unsigned char int_curr_hit_ratio;

    Re = intruder->slant_range - intruder->_hmdf_Rp;

    /* To compute how many hits are missing, we compute the ratio between own_aircraft delta time and the
       other aircraft dela time. If 1, no hits are missing. */
    curr_hit_ratio = intruder->delta_update_time / own_aircraft->delta_update_time;
    if (curr_hit_ratio <= 1.f) {
        /* No missing hits */
        intruder->_hmdf_hits++;
        if (intruder->_hmdf_hits > HMDF_MAX_HITS) {
            intruder->_hmdf_hits = HMDF_MAX_HITS;
        }
    } else {
        /* At least one hit is missing */
        int_curr_hit_ratio = (unsigned char)ceil(curr_hit_ratio);
        if (intruder->_hmdf_hits > int_curr_hit_ratio) {
            intruder->_hmdf_hits = intruder->_hmdf_hits - int_curr_hit_ratio;
        } else {
            intruder->_hmdf_hits = 0;
        }
    }

    *As = intruder->_hmdf_Ap + HMDF_c[intruder->_hmdf_hits]/1000. * Re / (intruder->delta_update_time * intruder->delta_update_time);
    *Vs = intruder->_hmdf_Vp + HMDF_b[intruder->_hmdf_hits]/1000. * Re / intruder->delta_update_time;
    *Rs = intruder->_hmdf_Rp + HMDF_a[intruder->_hmdf_hits]/1000. * Re;

    intruder->_hmdf_diff_ApAs = intruder->_hmdf_Ap - *As;
    intruder->_hmdf_Ap = *As;
    intruder->_hmdf_Vp = *Vs + intruder->delta_update_time * *As;
    intruder->_hmdf_Rp = *Rs + intruder->delta_update_time * *Vs + intruder->delta_update_time * intruder->delta_update_time * *As / 2.f;

    return Re;
}

static float hdmf_accuracy_check(plane_t *intruder, float Re) {
    float sigma, threshold;
    intruder->_hmdf_E = HMDF_ALPHA_PRECISION * intruder->_hmdf_E + (1-HMDF_ALPHA_PRECISION) * Re * Re;
    sigma = sqrt(intruder->_hmdf_E);
    threshold = sigma > 35.f ? 1.5f * sigma / 35.f : 1.5f;
    
    if (intruder->_hmdf_Ap <= threshold) {
        return 0;   /* Invalid */
    }

    return threshold;   /* Accuracy ok */
}

static float hmdf_predict_bearing_distance(plane_t *intruder) {
    const float sigma_theta = 0.0087;
    const float Q = 0.01;
    const float T = intruder->delta_update_time;
    float d_xm, d_ym, range, xrange, RBX, RBY, VBX, VBY;
    float slant_range = intruder->slant_range;
    float theta = intruder->bearing * M_PI / 180;
    float s_theta = sin(theta);
    float c_theta = cos(theta);
    float C_mu, alpha_x, beta_x;

    d_xm = slant_range * s_theta - intruder->_hmdf_RPX;
    d_ym = slant_range * c_theta - intruder->_hmdf_RPY;

    range  = d_xm * s_theta + d_ym * c_theta;
    xrange = d_ym * s_theta - d_xm * c_theta;

    /* Compute alpha and beta smoothing parameters */
    if(intruder->_hmdf_hits <= 1) {
        intruder->_hmdf_cov[0][0] = slant_range * slant_range * sigma_theta * sigma_theta;
        intruder->_hmdf_cov[0][1] = slant_range * slant_range * sigma_theta * sigma_theta / T;
        intruder->_hmdf_cov[1][0] = intruder->_hmdf_cov[0][0] / T;
        intruder->_hmdf_cov[1][1] = intruder->_hmdf_cov[0][1] * 2 / T;
    } else {
        intruder->_hmdf_cov[0][0] = intruder->_hmdf_cov[0][0] + 2 * T * intruder->_hmdf_cov[0][1] + T * T * intruder->_hmdf_cov[1][1] + T * T * T * T * Q / 4;
        intruder->_hmdf_cov[0][1] = intruder->_hmdf_cov[0][1] + T * intruder->_hmdf_cov[1][1] + T * T * T * Q / 2;
        intruder->_hmdf_cov[1][0] = intruder->_hmdf_cov[1][0] + T * intruder->_hmdf_cov[1][1] + T * T * T * Q / 2;
        intruder->_hmdf_cov[1][1] = intruder->_hmdf_cov[1][1] + T * T * Q;
    }

    C_mu = intruder->_hmdf_cov[0][0] + slant_range * slant_range * sigma_theta * sigma_theta;

    alpha_x = C_mu > 0 ? intruder->_hmdf_cov[0][0] / SAFE_DEN(C_mu) : 0;
    beta_x  = C_mu > 0 ? intruder->_hmdf_cov[1][0] / SAFE_DEN(C_mu) : 0;

    if (intruder->_hmdf_hits >= 3) {
        intruder->_hmdf_cov[0][0] = (1-alpha_x) * intruder->_hmdf_cov[0][0];
        intruder->_hmdf_cov[0][1] = (1-alpha_x) * intruder->_hmdf_cov[0][1];
        intruder->_hmdf_cov[1][0] = (1-alpha_x) * intruder->_hmdf_cov[1][0];
        intruder->_hmdf_cov[1][1] = intruder->_hmdf_cov[1][1] - beta_x * intruder->_hmdf_cov[0][1];
    }


    RBX = intruder->_hmdf_RPX + HMDF_BBT_a[intruder->_hmdf_hits] * range * s_theta - alpha_x * xrange * c_theta;
    RBY = intruder->_hmdf_RPY + HMDF_BBT_a[intruder->_hmdf_hits] * range * c_theta - alpha_x * xrange * s_theta;
    VBX = intruder->_hmdf_VPX + HMDF_BBT_b[intruder->_hmdf_hits] / SAFE_DEN(T * range * s_theta) - beta_x / SAFE_DEN(T * xrange * c_theta);
    VBY = intruder->_hmdf_VPY + HMDF_BBT_b[intruder->_hmdf_hits] / SAFE_DEN(T * range * c_theta) - beta_x / SAFE_DEN(T * xrange * s_theta);
    
    /* Update for the next round */
    intruder->_hmdf_VPX = VBX;
    intruder->_hmdf_VPY = VBY;
    intruder->_hmdf_RPX = RBX + T * VBX;
    intruder->_hmdf_RPY = RBY + T * VBY;

    /* Compute and return the distance */

    return ABS(RBX*VBY-RBY*VBX) / SAFE_DEN(sqrt(VBX * VBX + VBY * VBY));
}

static float get_threshold(int sensitivity_level, float ttg_cpa) {
    if (ttg_cpa >= 25) {
        return 0;
    }

    if (sensitivity_level <= 2) {
        if (ttg_cpa >= 11)
            return 0;
        else
            return 2500;
    } else if (sensitivity_level == 3) {
        if (ttg_cpa >= 14)
            return 0;
        else
            return 3500;
    } else if (sensitivity_level == 4) {
        if (ttg_cpa >= 16)
            return 0;
        else if (ttg_cpa >= 10)
            return 5500;
        else
            return 4500;
    } else if (sensitivity_level == 5) {
        if (ttg_cpa >= 19)
            return 0;
        else if (ttg_cpa >= 17)
            return 6000;
        else if (ttg_cpa >= 8)
            return 5500;
        else
            return 5000;
    } else {
        if (ttg_cpa >= 22)
            return 9000;
        else if (ttg_cpa >= 20)
            return 8500;
        else if (ttg_cpa >= 16)
            return 8000;
        else if (ttg_cpa >= 11)
            return 7000;
        else
            return 6000;
    }
}

static unsigned char is_manoeuvring_filter_1(plane_t *intruder) {
    intruder->_hmdf_dAs = (1-MANEUVRE_ALPHA) * intruder->_hmdf_dAs + MANEUVRE_ALPHA * intruder->_hmdf_diff_ApAs;
    return intruder->_hmdf_dAs < 0;
}

static unsigned char is_manoeuvring_filter_2(plane_t *intruder, float threshold_accuracy) {
    return -(intruder->_hmdf_diff_ApAs - intruder->_hmdf_Ap) < -threshold_accuracy;
}

static unsigned char is_manoeuvring_filter_3_4(plane_t *intruder, float threshold_accuracy, const float *As, const float *Vs, const float *Rs) {
    float Vys, Rys, Rxs, Xp, Yp, Rp2, Trv;
    /*float Vyp, Vp2, Ap2; */
    Vys = sqrt(*Rs * *As + (*Vs * *Vs));
    Rys = *Rs * *Vs / Vys;
    Rxs = sqrt(*Rs* *Rs - Rys * Rys);

    Xp = Rxs;
    /*Vyp = Vys;*/
    Yp = Rys + Vys * intruder->delta_update_time;

    Rp2 = sqrt(Xp*Xp + Yp*Yp);
    /*Vp2 = Yp * Vyp / Rp2;*/
    /*Ap2 = Xp * Xp * Vyp * Vyp / (Rp2*Rp2*Rp2);*/

    threshold_accuracy = threshold_accuracy / 1.5f *  35.f; /* Sigma */
    if (threshold_accuracy < 35) {
        threshold_accuracy = 35;
    }

    Trv = -3 * threshold_accuracy;
    if (intruder->slant_range - Rp2 < Trv) {
        return 1;
    }

    if (*As < -threshold_accuracy) {
        return 1;
    }

    return 0;

}

static unsigned char is_manoeuvring_filter_5(float R_HMD, float B_HMD) {
    return B_HMD < 0.5 * R_HMD;
}


static unsigned char is_manoeuvring(plane_t *intruder, float threshold_accuracy, float R_HMD, float B_HMD, const float *As, const float *Vs, const float *Rs) {
    if (is_manoeuvring_filter_1(intruder) != 0) {
        return 1;
    }

    if (is_manoeuvring_filter_2(intruder, threshold_accuracy) != 0) {
        return 1;
    }

    if (is_manoeuvring_filter_3_4(intruder, threshold_accuracy, As, Vs, Rs) != 0) {
        return 1;
    }

    if (is_manoeuvring_filter_5(R_HMD, B_HMD) != 0) {
        return 1;
    }

    return 0;
}

/* Returns 1 if ihibited */
static unsigned char hmdf_test(const plane_t *own_aircraft, plane_t *intruder, int idx_table, float ttg_cpa) {

    float Re, R_HMD, B_HMD, smaller_HMD, threshold, As, Vs, Rs, threshold_accuracy;

    /* Step 1 - Parabolic range estimator */
    Re = hmdf_parabolic_range_est(own_aircraft, intruder, &As, &Vs, &Rs);

    /* STEP 2 - accuracy check */
    threshold_accuracy = hdmf_accuracy_check(intruder, Re);
    if (threshold_accuracy == 0) {
        return 0;   /* Not sufficient accuracy to run HMDF */
    }

    /* STEP 3 - Compute the projected range HMD */
    R_HMD = sqrt(Rs * Rs - (Rs*Vs*Rs*Vs) / SAFE_DEN((Rs * As + Vs*Vs)));
    B_HMD = hmdf_predict_bearing_distance(intruder);

    smaller_HMD = (R_HMD > B_HMD) ? B_HMD : R_HMD;

    threshold = get_threshold(idx_table, ttg_cpa);

    if (smaller_HMD <= threshold) {
        return 0;   /* Too close, don't inhibit */
    }

    /* Check if maneuvring */
    if (is_manoeuvring(intruder, threshold_accuracy, R_HMD, B_HMD, &As, &Vs, &Rs)) {
        return 0;   /* Maneuvring */
    }

    return 1;   /* Filtered RA */
}

static result_t test_ra(const plane_t *own_aircraft, plane_t *intruder, int idx_table, float *ttg_cpa) {

    unsigned char result_hdmf;

    if (intruder->flags & FLAG_NO_VALID_ALTITUDE) {
        return CLEAR;   /* NO RAs for non-reporting altitude ACFs */
    }

    /* Range test is performed with RA values if the target has no valid altitude */
    *ttg_cpa = time_to_go_CPA(intruder, DMOD[TAB_RA][idx_table]);

    /* Execute the HDMF filter. NOTE: we have to run this even if there are blocking */
    /* conditions afterwards because it updates the internal state of the filter */
    result_hdmf = hmdf_test(own_aircraft, intruder, idx_table, *ttg_cpa);

    if (*ttg_cpa >= TAU[TAB_RA][idx_table]) {
        return CLEAR;
    }

    if(altitude_test(own_aircraft, intruder, TAB_RA, idx_table)) {
        if (result_hdmf) {
            return CLEAR;
        } else if( own_aircraft->vert_spd > 0 && own_aircraft->radioalt < 1100 ) {
        	return TA; /* ALL RAs - climb inhibition case */
        } else if( own_aircraft->vert_spd < 0 && own_aircraft->radioalt < 900 ) {
        	return TA; /* ALL RAs - descent inhibition case */
        } else {
            return RA_GENERIC;
        }
    }

    return CLEAR;
}

static result_t test_ta(const plane_t *own_aircraft, plane_t *intruder, int idx_table) {

    unsigned char alt_reporting;
    float ttg_cpa;

    alt_reporting = !(intruder->flags & FLAG_NO_VALID_ALTITUDE);

    /* Range test is performed with RA values if the target has no valid altitude */
    ttg_cpa = time_to_go_CPA(intruder, DMOD[alt_reporting ? TAB_TA : TAB_RA][idx_table]);

    if (ttg_cpa >= TAU[TAB_TA][idx_table]) {
        return CLEAR;
    }

    if (alt_reporting) {
        if(altitude_test(own_aircraft, intruder, TAB_TA, idx_table)) {
            return TA;
        }
    } else {
        if (own_aircraft->baroalt < 15500) {
            return TA;
        }
    }

    return CLEAR;
}

static float ra_sense_param(const plane_t *own_aircraft, const plane_t *intruder, float ttg_cpa, int target_vs, float target_G, int initial_delay) {

    /* The predicted separation is computed as the sum of three components:
     * 1) The current separation + the separation if no action is taken;
     * 2) The current separation during the phase transition to the target V/S (depend on acceleration);
     * 3) The separation when reached the target V/S to ttg_cpa.*/

    float initial_separation = intruder->baroalt - own_aircraft->baroalt;   /* [feet] */
    float initial_vs = intruder->vert_spd - own_aircraft->vert_spd;         /* [feet/min] */

    float no_action_sep; /* Separation achieved if no action [feet] */

    int vs_diff_sign;
    float time_to_reach_fpm, sec_to_target_fpm, dist_start_to_target_fpm, dist_steady_vs;

    initial_vs = initial_vs / 60;   /* [feet/s] */
    no_action_sep = initial_vs * ttg_cpa;

    ttg_cpa = ttg_cpa - initial_delay;  /* 5 sec delay */

    if (ttg_cpa <= 0) {
        /* Bad news, too near the target, let's use the previous decision
         * Better not to disprut the pilot in the last seconds. */
        return -1;
    }

    /* This is the time to reach the target FPM +/-1500 (or +/-2500) */
    time_to_reach_fpm = (target_vs-own_aircraft->vert_spd) / (M_TO_FEET(target_G * 9.81) * 60); /* [s] */
    time_to_reach_fpm = ABS(time_to_reach_fpm);

    /* The we compute the vertical distance during the 0-target_fpm stage */
    sec_to_target_fpm = MIN(time_to_reach_fpm, ttg_cpa);  /* Cannot be lower than ttg_cpa [s] */
    vs_diff_sign = ABS(target_vs-own_aircraft->vert_spd) / SAFE_DEN_INT(target_vs-own_aircraft->vert_spd);
    dist_start_to_target_fpm = vs_diff_sign * M_TO_FEET(target_G * 9.81) * sec_to_target_fpm * sec_to_target_fpm / 2;

    /* If there is more time, the V/S is assumed fixed at target_fpm 
     * it is computed as 0 otherwise */

    if (ttg_cpa - sec_to_target_fpm > 0) {
        dist_steady_vs = target_vs / 60 * (ttg_cpa - sec_to_target_fpm);
    } else {
        dist_steady_vs = 0;
    }


    return ABS(initial_separation + no_action_sep - dist_start_to_target_fpm - dist_steady_vs);

}

static result_t ra_sense(const plane_t *own_aircraft, plane_t *intruder, float ttg_cpa, int idx_table) {

    result_t current_decision;
    unsigned char was_climb, was_descent;
    unsigned char no_level_off = 0;
    float stay_zero = ra_sense_param(own_aircraft, intruder, ttg_cpa, 0, RA_SENSE_INITIAL_G, RA_SENSE_INITIAL_SEC);
    int curr_alim = ALIM[idx_table];

    float clb_sep, des_sep;

    if (stay_zero < 0) {    /* Too close to target to change */
        return intruder->_prev_decision;
    }

    clb_sep = ra_sense_param(own_aircraft, intruder, ttg_cpa, RA_SENSE_INITIAL_FPM, RA_SENSE_INITIAL_G, RA_SENSE_INITIAL_SEC);
    des_sep = ra_sense_param(own_aircraft, intruder, ttg_cpa, -RA_SENSE_INITIAL_FPM, RA_SENSE_INITIAL_G, RA_SENSE_INITIAL_SEC);

    
    if(own_aircraft->vert_spd < -300 && des_sep > stay_zero) {
        no_level_off = 1;
    }

    if(own_aircraft->vert_spd > 300 && clb_sep > stay_zero) {
        no_level_off = 1;
    }

    if ((own_aircraft->_prev_decision == CLEAR || 
        own_aircraft->_prev_decision == RA_LEVELOFF) &&
        stay_zero > ALIM[idx_table] &&
        no_level_off == 0) {
        /* 7.1 extension: if leveling off at zero V/S is sufficient to achieve separation, then
         * enforce level off. This can be selected only as the first RA */
        intruder->_prev_decision = RA_LEVELOFF;
        return RA_LEVELOFF;
    }

    /* TCAS always prefers the non-crossing response if the achieved separation is
       at least ALIM. Otherwise, it selects the best one */

    if (own_aircraft->baroalt > intruder->baroalt) {
        if (clb_sep >= curr_alim) {
            current_decision = RA_CLIMB;
        } else {
            current_decision =  clb_sep > des_sep ? RA_CLIMB : RA_XING_DESCENT;
            if (ABS(own_aircraft->baroalt - intruder->baroalt) < 100 && current_decision == RA_XING_DESCENT) {
                current_decision = RA_DESCENT;  /* Do not trigger crossing if within 100ft of altitude */
            }
        }
    } else {
        if (des_sep >= curr_alim) {
            current_decision = RA_DESCENT;
        } else {
            current_decision = clb_sep > des_sep ? RA_XING_CLIMB : RA_DESCENT;
            if (ABS(own_aircraft->baroalt - intruder->baroalt) < 100 && current_decision == RA_XING_CLIMB) {
                current_decision = RA_CLIMB;  /* Do not trigger crossing if within 100ft of altitude */
            }
        }
    }

    /* If we are reversing the decision, check if don't do that can still guarantee the ALIM value */
    was_climb = intruder->_prev_decision == RA_CLIMB || intruder->_prev_decision == RA_CLIMB_HIGH || intruder->_prev_decision == RA_XING_CLIMB;
    if (was_climb && (current_decision == RA_DESCENT || current_decision == RA_XING_DESCENT)) {
        if (clb_sep >= curr_alim) {
            current_decision = intruder->_prev_decision == RA_XING_CLIMB ? RA_XING_CLIMB : RA_CLIMB;
        }
    }
    was_descent = intruder->_prev_decision == RA_DESCENT || intruder->_prev_decision == RA_DESCENT_HIGH || intruder->_prev_decision == RA_XING_DESCENT;
    if (was_descent && (current_decision == RA_CLIMB || current_decision == RA_XING_CLIMB)) {
        if (des_sep >= curr_alim) {
            current_decision = intruder->_prev_decision == RA_XING_DESCENT ? RA_XING_DESCENT : RA_DESCENT;
        }
    }

    if (clb_sep < curr_alim && des_sep < curr_alim && intruder->_prev_decision != CLEAR) {
        /* If ALIM is not achieved in any sense, and we already
         * triggered an RA, then we need to strength the advisory */

        clb_sep = ra_sense_param(own_aircraft, intruder, ttg_cpa, RA_SENSE_SUBSEQ_FPM, RA_SENSE_INITIAL_G, RA_SENSE_INITIAL_SEC);
        des_sep = ra_sense_param(own_aircraft, intruder, ttg_cpa, -RA_SENSE_SUBSEQ_FPM, RA_SENSE_SUBSEQ_G, RA_SENSE_SUBSEQ_SEC);

        current_decision = clb_sep > des_sep ? RA_CLIMB_HIGH : RA_DESCENT_HIGH;

    }

    return current_decision;
}

static result_t acas_single(const plane_t *own_aircraft, plane_t *intruder) {
    int int_agl, idx_table;
    float ttg_cpa;
    result_t result, result_ra;

    intruder->baroalt = quantize_altitude(intruder->baroalt);
    
    /* Estimate the intruder altitude AGL */
    int_agl = estimate_intruder_alt(own_aircraft, intruder);
    
    /* Inhibition system for the intruder: hysteresis at 380+-20ft. */
    if (intruder->_state & STATE_WAS_ACTIVE) {   /* Prev state = active */
        if (int_agl <= 400) {
            intruder->_state = intruder->_state & ~STATE_WAS_ACTIVE;
            /* We didn't run the HMDF filter if we come up here, so we need to reset */
            /* the internal state */
            
            hmdf_reset(intruder);
            return INHIBIT;
        }
    } else { /* Prev state = inactive */
        if (int_agl >= 600) {
            intruder->_state = intruder->_state | STATE_WAS_ACTIVE;
        } else {
            /* We didn't run the HMDF filter if we come up here, so we need to reset */
            /* the internal state */
            
            hmdf_reset(intruder);
            return INHIBIT;
        }   
    }
    
    /* Ok, if we are here, then it's not inhibited */
    idx_table = table_index(own_aircraft->baroalt, own_aircraft->radioalt);

    result = test_ta(own_aircraft, intruder, idx_table);

    if (result != CLEAR && (intruder->flags & FLAG_NO_VALID_ALTITUDE) == 0) {
        result_ra = test_ra(own_aircraft, intruder, idx_table, &ttg_cpa);
	
        if (result_ra != CLEAR) {
        	result = result_ra;
        }
    } else {
        /* We didn't run the HMDF filter if we come up here, so we need to reset */
        /* the internal state */
        hmdf_reset(intruder);
    }
    if (result == RA_GENERIC) {
        result = ra_sense(own_aircraft, intruder, ttg_cpa, idx_table);
    }

    /* Time to record the current slant range as previous. */
    /* We cannot do this in test_ta or test_ra because both */
    /* uses the old value*/
    intruder->_closure_prev = intruder->slant_range;

    return result;
}

static result_t get_final_ra(const plane_t *own_aircraft, result_t current_dec, int voting_ra, unsigned char is_clb_xing, unsigned char is_des_xing) {
    result_t overall_res;
    result_t prev_dec = own_aircraft->_prev_decision;


	if (current_dec == TA) {
		return TA;
	}

    /* Select the best RA in case of multiple aircrafts, with a slight preference on climbing */
    if (voting_ra >= 100) {
        overall_res = RA_CLIMB_HIGH;
    } else if (voting_ra > 0) {
        overall_res = is_clb_xing ? RA_XING_CLIMB : RA_CLIMB;
    } else if (voting_ra <= -100) {
        overall_res = RA_DESCENT_HIGH;
    } else if (voting_ra < 0) {
        overall_res = is_des_xing ? RA_XING_DESCENT : RA_DESCENT;
    } else if (current_dec == RA_LEVELOFF) {
        overall_res = RA_LEVELOFF;
    } else if (current_dec != CLEAR && current_dec != COC) {
        overall_res = RA_CLIMB; /* Here it's problematic, but we prefer to climb */
    } else if (current_dec == COC) {
        overall_res = COC;
    } else {
        overall_res = CLEAR;
    }

    /* Reverse RA */
    if(prev_dec == RA_CLIMB_HIGH || prev_dec == RA_CLIMB || prev_dec == RA_XING_CLIMB) {
        if(overall_res == RA_XING_DESCENT || overall_res == RA_DESCENT || overall_res == RA_DESCENT_HIGH) {
            overall_res = RA_DESCENT_NOW;
        }
    }
    if(prev_dec == RA_DESCENT_HIGH || prev_dec == RA_DESCENT || prev_dec == RA_XING_DESCENT) {
        if(overall_res == RA_XING_CLIMB || overall_res == RA_CLIMB || overall_res == RA_CLIMB_HIGH) {
            overall_res = RA_CLIMB_NOW;
        }
    }

    /* Maintain */
    if(overall_res == RA_CLIMB_NOW || overall_res == RA_CLIMB || overall_res == RA_XING_CLIMB) {
        if (own_aircraft->vert_spd > 1500) {
            overall_res = overall_res == RA_XING_CLIMB ? RA_XING_MAINTAIN_VS : RA_MAINTAIN_VS;
        }
    }
    if(overall_res == RA_CLIMB_HIGH) {
        if (own_aircraft->vert_spd > 2500) {
            overall_res = RA_MAINTAIN_VS;
        }
    }
    if(overall_res == RA_DESCENT_NOW || overall_res == RA_DESCENT || overall_res == RA_XING_DESCENT) {
        if (own_aircraft->vert_spd < -1500) {
            overall_res = overall_res == RA_XING_DESCENT ? RA_XING_MAINTAIN_VS : RA_MAINTAIN_VS;
        }
    }
    if(overall_res == RA_DESCENT_HIGH) {
        if (own_aircraft->vert_spd < -2500) {
            overall_res = RA_MAINTAIN_VS;
        }
    }
    if(overall_res == RA_LEVELOFF) {
        if (own_aircraft->vert_spd < 300 && own_aircraft->vert_spd > -300) {
            overall_res = RA_MONITOR_VS;
        }
    }
    return overall_res;
}

static unsigned char check_inhibit(plane_t *own_aircraft) {
    /* My plane inhibition system: there is a hysteresis at 500+-100ft which disables all
     * the TCAS RA and TA alerts. */

    if (own_aircraft->_state & STATE_WAS_ACTIVE) {   /* Prev state = active */
        if (own_aircraft->radioalt <= 400) {
            own_aircraft->_state = own_aircraft->_state & ~STATE_WAS_ACTIVE;
            return 1;
        }
    } else { /* Prev state = inactive */
        if (own_aircraft->radioalt >= 600) {
            own_aircraft->_state = own_aircraft->_state | STATE_WAS_ACTIVE;
        } else {
            return 1;
        }   
    }
    return 0;
}

static result_t acas_bench(plane_t *own_aircraft, plane_t *intruders, int nr_intruders) {


    unsigned char is_clb_xing, is_des_xing;
    int i, voting_ra;
    result_t res, overall_res;

    if(check_inhibit(own_aircraft)) {
        return INHIBIT;
    }

    overall_res = CLEAR;

    is_clb_xing = is_des_xing = 0;
    voting_ra = 0;

    for(i=0; i<nr_intruders; i++) {
        plane_t* curr_intruder = intruders + i;

        res = acas_single(own_aircraft, curr_intruder);

        switch(res) {
        case CLEAR:
            if(curr_intruder->_prev_decision != CLEAR &&
               curr_intruder->_prev_decision != TA &&
               curr_intruder->_prev_decision != COC) {
                overall_res = COC;
            }
        break;
        case INHIBIT:
        	/* Do nothing */
        break;
        case RA_XING_CLIMB:
            is_clb_xing = 1;
            overall_res = RA_CLIMB; /* it doesn't matter, just not CLEAR */
            /* Fallthrough */
        case RA_CLIMB:
            voting_ra++;
            overall_res = RA_CLIMB;
        break;
        case RA_CLIMB_HIGH:
            voting_ra += 100;
            overall_res = RA_CLIMB_HIGH; /* it doesn't matter, just not CLEAR */
        break;
        case RA_XING_DESCENT:
            is_des_xing = 1;
            overall_res = RA_DESCENT; /* it doesn't matter, just not CLEAR */
            /* Fallthrough */
        case RA_DESCENT:
            voting_ra--;
            overall_res = RA_DESCENT;
        break;
        case RA_DESCENT_HIGH:
            overall_res = RA_DESCENT_HIGH; /* it doesn't matter, just not CLEAR */
            voting_ra -= 100;
        break;
        case RA_LEVELOFF:
            overall_res = RA_LEVELOFF;
        break;
        case TA:
        	overall_res = TA;
        break;
        default:
            /* This shouldn't happen */
            overall_res = CLEAR;
        break;
        }

        curr_intruder->_prev_decision = overall_res;
    }

	overall_res = get_final_ra(own_aircraft, overall_res, voting_ra, is_clb_xing, is_des_xing);

    own_aircraft->_prev_decision = overall_res;

    return overall_res;
}

static void initialize_acf(plane_t *acf) {
    acf->identifier = 0;
    acf->baroalt = 0;
    acf->radioalt = 3000;
    acf->vert_spd = 0;
    acf->slant_range = 0;
    acf->bearing = 0;
    acf->flags = 0;

    acf->delta_update_time = 0;

    acf->_state = 0;
    acf->_closure_prev = -1;
    acf->_heading_prev = 0;
    acf->_prev_decision = CLEAR;

    hmdf_reset(acf);

}

static void random_set(plane_t *acf) {
    double r = random_get();
	acf->baroalt  = r * (35000 - acf->baroalt * r);
    r = random_get();
	acf->radioalt = acf->baroalt < 10000 ? r * (5000 - acf->radioalt * r) : 9999;
    r = random_get();
    acf->vert_spd = r * (10000 - acf->vert_spd * r) - 5000;
    r = random_get();
    acf->slant_range = r * (20000 - acf->slant_range * r);
    r = random_get();
    acf->bearing = r * (360 - acf->bearing * r);
    acf->delta_update_time = 1;
}

int i;
plane_t own_acf = {0};
plane_t intruders[NR_PLANES] ={0};

void init_acas() {
	initialize_acf(&own_acf);

	for(i=0; i<NR_PLANES; i++) {
		initialize_acf(&intruders[i]);
	}
}

void acas_update_init_conditions() {
    random_set(&own_acf);
    for(i=0; i<NR_PLANES; i++) {
        random_set(&intruders[i]);
    }
}

crc acas(SemaphoreHandle_t xMutexRandom) {
    MEASURE_START(acas_result);

    for(int j=0; j<ITERATIONS;j++) {
	    xSemaphoreTake(xMutexRandom, TIMEOUT_QUEUES);
        acas_update_init_conditions();
        xSemaphoreGive(xMutexRandom);

        results[j] = acas_bench(&own_acf, intruders, NR_PLANES);
    }
    MEASURE_STOP(acas_result);
    crc_acas = crc32(results, sizeof(results));

    return crc_acas;
}


