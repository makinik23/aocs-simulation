module dtm2020_c_bridge
    use, intrinsic :: iso_c_binding
    implicit none

    logical :: model_initialized = .false.

contains

    subroutine dtm2020_initialize(coefficient_file, path_length, status) &
            bind(C, name="dtm2020_initialize")
        character(kind=c_char), intent(in) :: coefficient_file(*)
        integer(c_int), value, intent(in) :: path_length
        integer(c_int), intent(out) :: status

        character(len=:), allocatable :: filename
        integer :: file_unit, io_status, index
        real :: pi_common, two_pi_common, degrees_to_radians_common, sard_common
        common /cons/ pi_common, two_pi_common, degrees_to_radians_common, sard_common

        status = 0_c_int
        if (path_length <= 0_c_int) then
            status = 10_c_int
            return
        end if

        allocate(character(len=path_length) :: filename)
        do index = 1, path_length
            filename(index:index) = coefficient_file(index)
        end do

        open(newunit=file_unit, file=filename, status="old", action="read", &
             iostat=io_status)
        if (io_status /= 0) then
            status = int(io_status, c_int)
            return
        end if

        pi_common = acos(-1.0)
        two_pi_common = 2.0 * pi_common
        degrees_to_radians_common = pi_common / 180.0
        sard_common = 0.0

        call lecdtm(file_unit)
        close(file_unit)
        model_initialized = .true.
    end subroutine dtm2020_initialize

    subroutine dtm2020_evaluate(day_of_year, f107_sfu, f107_81d_sfu, &
            kp_delayed_3h, kp_mean_24h, altitude_km, local_solar_time_rad, &
            latitude_rad, longitude_rad, rho_g_cm3, uncertainty_percent, &
            temperature_local_K, temperature_exospheric_K, species_g_cm3, &
            status) bind(C, name="dtm2020_evaluate")
        real(c_double), value, intent(in) :: day_of_year
        real(c_double), value, intent(in) :: f107_sfu, f107_81d_sfu
        real(c_double), value, intent(in) :: kp_delayed_3h, kp_mean_24h
        real(c_double), value, intent(in) :: altitude_km
        real(c_double), value, intent(in) :: local_solar_time_rad
        real(c_double), value, intent(in) :: latitude_rad, longitude_rad
        real(c_double), intent(out) :: rho_g_cm3, uncertainty_percent
        real(c_double), intent(out) :: temperature_local_K
        real(c_double), intent(out) :: temperature_exospheric_K
        real(c_double), intent(out) :: species_g_cm3(6)
        integer(c_int), intent(out) :: status

        real(c_float) :: f(2), fbar(2), akp(4), species_single(6)
        real(c_float) :: day_single, altitude_single, local_time_single
        real(c_float) :: latitude_single, longitude_single
        real(c_float) :: rho_single, uncertainty_single
        real(c_float) :: temperature_local_single, temperature_exospheric_single
        real(c_float) :: mean_molecular_mass_single
        real(c_float), parameter :: pi_single = acos(-1.0_c_float)

        rho_g_cm3 = 0.0_c_double
        uncertainty_percent = 0.0_c_double
        temperature_local_K = 0.0_c_double
        temperature_exospheric_K = 0.0_c_double
        species_g_cm3 = 0.0_c_double
        status = 0_c_int

        if (.not. model_initialized) then
            status = 20_c_int
            return
        end if
        if (altitude_km < 120.0_c_double .or. altitude_km > 1500.0_c_double) then
            status = 21_c_int
            return
        end if

        day_single = real(day_of_year, c_float)
        altitude_single = real(altitude_km, c_float)
        local_time_single = real(local_solar_time_rad, c_float)
        latitude_single = real(latitude_rad, c_float)
        longitude_single = real(longitude_rad, c_float)
        f = [real(f107_sfu, c_float), 0.0_c_float]
        fbar = [real(f107_81d_sfu, c_float), 0.0_c_float]
        akp = [real(kp_delayed_3h, c_float), 0.0_c_float, &
               real(kp_mean_24h, c_float), 0.0_c_float]

        call dtm3(day_single, f, fbar, akp, altitude_single, &
                  local_time_single, latitude_single, longitude_single, &
                  temperature_local_single, temperature_exospheric_single, &
                  rho_single, species_single, mean_molecular_mass_single)

        call sigma_function(latitude_single * 180.0_c_float / pi_single, &
                            local_time_single * 12.0_c_float / pi_single, &
                            day_single, altitude_single, fbar(1), akp(1), &
                            uncertainty_single)

        rho_g_cm3 = real(rho_single, c_double)
        uncertainty_percent = real(uncertainty_single, c_double)
        temperature_local_K = real(temperature_local_single, c_double)
        temperature_exospheric_K = real(temperature_exospheric_single, c_double)
        species_g_cm3 = real(species_single, c_double)
    end subroutine dtm2020_evaluate

end module dtm2020_c_bridge
