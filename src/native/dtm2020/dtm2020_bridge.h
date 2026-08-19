#ifndef AOCS_DTM2020_BRIDGE_H
#define AOCS_DTM2020_BRIDGE_H

#ifdef __cplusplus
extern "C" {
#endif

void dtm2020_initialize(const char *coefficient_file, int path_length, int *status);

void dtm2020_evaluate(double day_of_year,
                      double f107_sfu,
                      double f107_81d_sfu,
                      double kp_delayed_3h,
                      double kp_mean_24h,
                      double altitude_km,
                      double local_solar_time_rad,
                      double latitude_rad,
                      double longitude_rad,
                      double *rho_g_cm3,
                      double *uncertainty_percent,
                      double *temperature_local_K,
                      double *temperature_exospheric_K,
                      double species_g_cm3[6],
                      int *status);

#ifdef __cplusplus
}
#endif

#endif
