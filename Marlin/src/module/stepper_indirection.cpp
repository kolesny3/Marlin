/**
 * Marlin 3D Printer Firmware
 * Copyright (C) 2016 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

/**
 * stepper_indirection.cpp
 *
 * Stepper motor driver indirection to allow some stepper functions to
 * be done via SPI/I2c instead of direct pin manipulation.
 *
 * Part of Marlin
 *
 * Copyright (c) 2015 Dominik Wenger
 */

#include "stepper_indirection.h"

#include "../inc/MarlinConfig.h"

//
// TMC26X Driver objects and inits
//
#if HAS_DRIVER(TMC26X)
  #include <SPI.h>

  #ifdef STM32F7
    #include "../HAL/HAL_STM32F7/TMC2660.h"
  #else
    #include <TMC26XStepper.h>
  #endif

  #define _TMC26X_DEFINE(ST) TMC26XStepper stepper##ST(200, ST##_CS_PIN, ST##_STEP_PIN, ST##_DIR_PIN, ST##_MAX_CURRENT, ST##_SENSE_RESISTOR)

  #if AXIS_DRIVER_TYPE(X, TMC26X)
    _TMC26X_DEFINE(X);
  #endif
  #if AXIS_DRIVER_TYPE(X2, TMC26X)
    _TMC26X_DEFINE(X2);
  #endif
  #if AXIS_DRIVER_TYPE(Y, TMC26X)
    _TMC26X_DEFINE(Y);
  #endif
  #if AXIS_DRIVER_TYPE(Y2, TMC26X)
    _TMC26X_DEFINE(Y2);
  #endif
  #if AXIS_DRIVER_TYPE(Z, TMC26X)
    _TMC26X_DEFINE(Z);
  #endif
  #if AXIS_DRIVER_TYPE(Z2, TMC26X)
    _TMC26X_DEFINE(Z2);
  #endif
  #if AXIS_DRIVER_TYPE(E0, TMC26X)
    _TMC26X_DEFINE(E0);
  #endif
  #if AXIS_DRIVER_TYPE(E1, TMC26X)
    _TMC26X_DEFINE(E1);
  #endif
  #if AXIS_DRIVER_TYPE(E2, TMC26X)
    _TMC26X_DEFINE(E2);
  #endif
  #if AXIS_DRIVER_TYPE(E3, TMC26X)
    _TMC26X_DEFINE(E3);
  #endif
  #if AXIS_DRIVER_TYPE(E4, TMC26X)
    _TMC26X_DEFINE(E4);
  #endif

  #define _TMC26X_INIT(A) do{ \
    stepper##A.setMicrosteps(A##_MICROSTEPS); \
    stepper##A.start(); \
  }while(0)

  void tmc26x_init_to_defaults() {
    #if AXIS_DRIVER_TYPE(X, TMC26X)
      _TMC26X_INIT(X);
    #endif
    #if AXIS_DRIVER_TYPE(X2, TMC26X)
      _TMC26X_INIT(X2);
    #endif
    #if AXIS_DRIVER_TYPE(Y, TMC26X)
      _TMC26X_INIT(Y);
    #endif
    #if AXIS_DRIVER_TYPE(Y2, TMC26X)
      _TMC26X_INIT(Y2);
    #endif
    #if AXIS_DRIVER_TYPE(Z, TMC26X)
      _TMC26X_INIT(Z);
    #endif
    #if AXIS_DRIVER_TYPE(Z2, TMC26X)
      _TMC26X_INIT(Z2);
    #endif
    #if AXIS_DRIVER_TYPE(E0, TMC26X)
      _TMC26X_INIT(E0);
    #endif
    #if AXIS_DRIVER_TYPE(E1, TMC26X)
      _TMC26X_INIT(E1);
    #endif
    #if AXIS_DRIVER_TYPE(E2, TMC26X)
      _TMC26X_INIT(E2);
    #endif
    #if AXIS_DRIVER_TYPE(E3, TMC26X)
      _TMC26X_INIT(E3);
    #endif
    #if AXIS_DRIVER_TYPE(E4, TMC26X)
      _TMC26X_INIT(E4);
    #endif
  }
#endif // TMC26X

#if HAS_TRINAMIC
  #if AXIS_IS_TMC(X)
    static const char TMC_X_LABEL[] PROGMEM = MSG_A;
  #endif
  #if AXIS_IS_TMC(X2)
    static const char TMC_X2_LABEL[] PROGMEM = MSG_X2;
  #endif
  #if AXIS_IS_TMC(Y)
    static const char TMC_Y_LABEL[] PROGMEM = MSG_B;
  #endif
  #if AXIS_IS_TMC(Y2)
    static const char TMC_Y2_LABEL[] PROGMEM = MSG_Y2;
  #endif
  #if AXIS_IS_TMC(Z)
    static const char TMC_Z_LABEL[] PROGMEM = MSG_C;
  #endif
  #if AXIS_IS_TMC(Z2)
    static const char TMC_Z2_LABEL[] PROGMEM = MSG_Z2;
  #endif
  #if AXIS_IS_TMC(E0)
    static const char TMC_E0_LABEL[] PROGMEM = MSG_E;
  #endif
  #if AXIS_IS_TMC(E1)
    static const char TMC_E1_LABEL[] PROGMEM = MSG_E1;
  #endif
  #if AXIS_IS_TMC(E2)
    static const char TMC_E2_LABEL[] PROGMEM = MSG_E2;
  #endif
  #if AXIS_IS_TMC(E3)
    static const char TMC_E3_LABEL[] PROGMEM = MSG_E3;
  #endif
  #if AXIS_IS_TMC(E4)
    static const char TMC_E4_LABEL[] PROGMEM = MSG_E4;
  #endif

  #define _TMC_INIT(ST, SPMM) tmc_init(stepper##ST, ST##_CURRENT, ST##_MICROSTEPS, ST##_HYBRID_THRESHOLD, SPMM)
#endif

//
// TMC2130 Driver objects and inits
//
#if HAS_DRIVER(TMC2130)

  #include <SPI.h>
  #include "planner.h"
  #include "../core/enum.h"

  #if ENABLED(TMC_USE_SW_SPI)
    #define _TMC2130_DEFINE(ST) TMCMarlin<TMC2130Stepper> stepper##ST(TMC_##ST##_LABEL, ST##_CS_PIN, R_SENSE, TMC_SW_MOSI, TMC_SW_MISO, TMC_SW_SCK)
  #else
    #define _TMC2130_DEFINE(ST) TMCMarlin<TMC2130Stepper> stepper##ST(TMC_##ST##_LABEL, ST##_CS_PIN, R_SENSE)
  #endif
  // Stepper objects of TMC2130 steppers used
  #if AXIS_DRIVER_TYPE(X, TMC2130)
    _TMC2130_DEFINE(X);
  #endif
  #if AXIS_DRIVER_TYPE(X2, TMC2130)
    _TMC2130_DEFINE(X2);
  #endif
  #if AXIS_DRIVER_TYPE(Y, TMC2130)
    _TMC2130_DEFINE(Y);
  #endif
  #if AXIS_DRIVER_TYPE(Y2, TMC2130)
    _TMC2130_DEFINE(Y2);
  #endif
  #if AXIS_DRIVER_TYPE(Z, TMC2130)
    _TMC2130_DEFINE(Z);
  #endif
  #if AXIS_DRIVER_TYPE(Z2, TMC2130)
    _TMC2130_DEFINE(Z2);
  #endif
  #if AXIS_DRIVER_TYPE(E0, TMC2130)
    _TMC2130_DEFINE(E0);
  #endif
  #if AXIS_DRIVER_TYPE(E1, TMC2130)
    _TMC2130_DEFINE(E1);
  #endif
  #if AXIS_DRIVER_TYPE(E2, TMC2130)
    _TMC2130_DEFINE(E2);
  #endif
  #if AXIS_DRIVER_TYPE(E3, TMC2130)
    _TMC2130_DEFINE(E3);
  #endif
  #if AXIS_DRIVER_TYPE(E4, TMC2130)
    _TMC2130_DEFINE(E4);
  #endif

  void tmc_init(TMC2130Stepper &st, const uint16_t mA, const uint16_t microsteps, const uint32_t thrs, const float spmm) {
    #if DISABLED(STEALTHCHOP) || DISABLED(HYBRID_THRESHOLD)
      UNUSED(thrs);
      UNUSED(spmm);
    #endif
    st.begin();
    st.rms_current(mA, HOLD_MULTIPLIER);
    st.microsteps(microsteps);

    int8_t timings[] = CHOPPER_TIMING; // Default 4, -2, 1

    CHOPCONF_t chopconf;
    chopconf.tbl = 1;
    chopconf.toff = timings[0];
    chopconf.intpol = INTERPOLATE;
    chopconf.hend = timings[1] + 3;
    chopconf.hstrt = timings[2] - 1;
    st.CHOPCONF(chopconf.sr);

    st.iholddelay(10);
    st.TPOWERDOWN(128); // ~2s until driver lowers to hold current

    #if ENABLED(ADAPTIVE_CURRENT)
      COOLCONF_t coolconf;
      coolconf.semin = INCREASE_CURRENT_THRS;
      coolconf.semax = REDUCE_CURRENT_THRS;
      st.COOLCONF(coolconf.sr);
    #endif

    #if ENABLED(STEALTHCHOP)
      st.en_pwm_mode(true);

      PWMCONF_t pwmconf;
      pwmconf.pwm_freq = 0b01; // f_pwm = 2/683 f_clk
      pwmconf.pwm_autoscale = true;
      pwmconf.pwm_grad = 5;
      pwmconf.pwm_ampl = 180;
      st.PWMCONF(pwmconf.sr);

      #if ENABLED(HYBRID_THRESHOLD)
        st.TPWMTHRS(12650000UL*microsteps/(256*thrs*spmm));
      #else
        UNUSED(thrs);
        UNUSED(spmm);
      #endif
    #endif
    st.GSTAT(); // Clear GSTAT
  }
#endif // TMC2130

//
// TMC2208 Driver objects and inits
//
#if HAS_DRIVER(TMC2208)

  #include <SoftwareSerial.h>
  #include <HardwareSerial.h>
  #include "planner.h"

  #define _TMC2208_DEFINE_HARDWARE(ST) TMCMarlin<TMC2208Stepper> stepper##ST(TMC_##ST##_LABEL, &ST##_HARDWARE_SERIAL, R_SENSE)
  #define _TMC2208_DEFINE_SOFTWARE(ST) TMCMarlin<TMC2208Stepper> stepper##ST(TMC_##ST##_LABEL, ST##_SERIAL_RX_PIN, ST##_SERIAL_TX_PIN, R_SENSE, ST##_SERIAL_RX_PIN > -1)

  // Stepper objects of TMC2208 steppers used
  #if AXIS_DRIVER_TYPE(X, TMC2208)
    #ifdef X_HARDWARE_SERIAL
      _TMC2208_DEFINE_HARDWARE(X);
    #else
      _TMC2208_DEFINE_SOFTWARE(X);
    #endif
  #endif
  #if AXIS_DRIVER_TYPE(X2, TMC2208)
    #ifdef X2_HARDWARE_SERIAL
      _TMC2208_DEFINE_HARDWARE(X2);
    #else
      _TMC2208_DEFINE_SOFTWARE(X2);
    #endif
  #endif
  #if AXIS_DRIVER_TYPE(Y, TMC2208)
    #ifdef Y_HARDWARE_SERIAL
      _TMC2208_DEFINE_HARDWARE(Y);
    #else
      _TMC2208_DEFINE_SOFTWARE(Y);
    #endif
  #endif
  #if AXIS_DRIVER_TYPE(Y2, TMC2208)
    #ifdef Y2_HARDWARE_SERIAL
      _TMC2208_DEFINE_HARDWARE(Y2);
    #else
      _TMC2208_DEFINE_SOFTWARE(Y2);
    #endif
  #endif
  #if AXIS_DRIVER_TYPE(Z, TMC2208)
    #ifdef Z_HARDWARE_SERIAL
      _TMC2208_DEFINE_HARDWARE(Z);
    #else
      _TMC2208_DEFINE_SOFTWARE(Z);
    #endif
  #endif
  #if AXIS_DRIVER_TYPE(Z2, TMC2208)
    #ifdef Z2_HARDWARE_SERIAL
      _TMC2208_DEFINE_HARDWARE(Z2);
    #else
      _TMC2208_DEFINE_SOFTWARE(Z2);
    #endif
  #endif
  #if AXIS_DRIVER_TYPE(E0, TMC2208)
    #ifdef E0_HARDWARE_SERIAL
      _TMC2208_DEFINE_HARDWARE(E0);
    #else
      _TMC2208_DEFINE_SOFTWARE(E0);
    #endif
  #endif
  #if AXIS_DRIVER_TYPE(E1, TMC2208)
    #ifdef E1_HARDWARE_SERIAL
      _TMC2208_DEFINE_HARDWARE(E1);
    #else
      _TMC2208_DEFINE_SOFTWARE(E1);
    #endif
  #endif
  #if AXIS_DRIVER_TYPE(E2, TMC2208)
    #ifdef E2_HARDWARE_SERIAL
      _TMC2208_DEFINE_HARDWARE(E2);
    #else
      _TMC2208_DEFINE_SOFTWARE(E2);
    #endif
  #endif
  #if AXIS_DRIVER_TYPE(E3, TMC2208)
    #ifdef E3_HARDWARE_SERIAL
      _TMC2208_DEFINE_HARDWARE(E3);
    #else
      _TMC2208_DEFINE_SOFTWARE(E3);
    #endif
  #endif
  #if AXIS_DRIVER_TYPE(E4, TMC2208)
    #ifdef E4_HARDWARE_SERIAL
      _TMC2208_DEFINE_HARDWARE(E4);
    #else
      _TMC2208_DEFINE_SOFTWARE(E4);
    #endif
  #endif

  void tmc2208_serial_begin() {
    #if AXIS_DRIVER_TYPE(X, TMC2208)
      #ifdef X_HARDWARE_SERIAL
        X_HARDWARE_SERIAL.begin(115200);
      #else
        stepperX.beginSerial(115200);
      #endif
    #endif
    #if AXIS_DRIVER_TYPE(X2, TMC2208)
      #ifdef X2_HARDWARE_SERIAL
        X2_HARDWARE_SERIAL.begin(115200);
      #else
        stepperX2.beginSerial(115200);
      #endif
    #endif
    #if AXIS_DRIVER_TYPE(Y, TMC2208)
      #ifdef Y_HARDWARE_SERIAL
        Y_HARDWARE_SERIAL.begin(115200);
      #else
        stepperY.beginSerial(115200);
      #endif
    #endif
    #if AXIS_DRIVER_TYPE(Y2, TMC2208)
      #ifdef Y2_HARDWARE_SERIAL
        Y2_HARDWARE_SERIAL.begin(115200);
      #else
        stepperY2.beginSerial(115200);
      #endif
    #endif
    #if AXIS_DRIVER_TYPE(Z, TMC2208)
      #ifdef Z_HARDWARE_SERIAL
        Z_HARDWARE_SERIAL.begin(115200);
      #else
        stepperZ.beginSerial(115200);
      #endif
    #endif
    #if AXIS_DRIVER_TYPE(Z2, TMC2208)
      #ifdef Z2_HARDWARE_SERIAL
        Z2_HARDWARE_SERIAL.begin(115200);
      #else
        stepperZ2.beginSerial(115200);
      #endif
    #endif
    #if AXIS_DRIVER_TYPE(E0, TMC2208)
      #ifdef E0_HARDWARE_SERIAL
        E0_HARDWARE_SERIAL.begin(115200);
      #else
        stepperE0.beginSerial(115200);
      #endif
    #endif
    #if AXIS_DRIVER_TYPE(E1, TMC2208)
      #ifdef E1_HARDWARE_SERIAL
        E1_HARDWARE_SERIAL.begin(115200);
      #else
        stepperE1.beginSerial(115200);
      #endif
    #endif
    #if AXIS_DRIVER_TYPE(E2, TMC2208)
      #ifdef E2_HARDWARE_SERIAL
        E2_HARDWARE_SERIAL.begin(115200);
      #else
        stepperE2.beginSerial(115200);
      #endif
    #endif
    #if AXIS_DRIVER_TYPE(E3, TMC2208)
      #ifdef E3_HARDWARE_SERIAL
        E3_HARDWARE_SERIAL.begin(115200);
      #else
        stepperE3.beginSerial(115200);
      #endif
    #endif
    #if AXIS_DRIVER_TYPE(E4, TMC2208)
      #ifdef E4_HARDWARE_SERIAL
        E4_HARDWARE_SERIAL.begin(115200);
      #else
        stepperE4.beginSerial(115200);
      #endif
    #endif
  }

  void tmc_init(TMC2208Stepper &st, const uint16_t mA, const uint16_t microsteps, const uint32_t thrs, const float spmm) {
    st.rms_current(mA, HOLD_MULTIPLIER);
    st.microsteps(microsteps);

    int8_t timings[] = CHOPPER_TIMING; // Default 4, -2, 1

    TMC2208_n::GCONF_t gconf;
    gconf.pdn_disable = true; // Use UART
    gconf.mstep_reg_select = true; // Select microsteps with UART
    gconf.i_scale_analog = false;

    TMC2208_n::CHOPCONF_t chopconf;
    chopconf.tbl = 0b01; // blank_time = 24
    chopconf.toff = timings[0];
    chopconf.intpol = INTERPOLATE;
    chopconf.hend = timings[1] + 3;
    chopconf.hstrt = timings[2] - 1;
    st.CHOPCONF(chopconf.sr);

    st.iholddelay(10);
    st.TPOWERDOWN(128); // ~2s until driver lowers to hold current
    #if ENABLED(STEALTHCHOP)
      gconf.en_spreadcycle = false;

      TMC2208_n::PWMCONF_t pwmconf;
      pwmconf.pwm_lim = 12;
      pwmconf.pwm_reg = 8;
      pwmconf.pwm_autograd = true;
      pwmconf.pwm_autoscale = true;
      pwmconf.pwm_freq = 0b01;
      pwmconf.pwm_grad = 14;
      pwmconf.pwm_ofs = 36;
      st.PWMCONF(pwmconf.sr);
      #if ENABLED(HYBRID_THRESHOLD)
        st.TPWMTHRS(12650000UL*microsteps/(256*thrs*spmm));
      #else
        UNUSED(thrs);
        UNUSED(spmm);
      #endif
    #else
      gconf.en_spreadcycle = true;
    #endif
    st.GCONF(gconf.sr);
    st.GSTAT(0b111); // Clear
    delay(200);
  }
#endif // TMC2208

//
// TMC2660 Driver objects and inits
//
#if HAS_DRIVER(TMC2660)

  #include <SPI.h>
  #include "planner.h"
  #include "../core/enum.h"

  #if ENABLED(TMC_USE_SW_SPI)
    #define _TMC2660_DEFINE(ST) TMCMarlin<TMC2660Stepper> stepper##ST(TMC_##ST##_LABEL, ST##_CS_PIN, R_SENSE, TMC_SW_MOSI, TMC_SW_MISO, TMC_SW_SCK)
  #else
    #define _TMC2660_DEFINE(ST) TMCMarlin<TMC2660Stepper> stepper##ST(TMC_##ST##_LABEL, ST##_CS_PIN, R_SENSE)
  #endif

  // Stepper objects of TMC2660 steppers used
  #if AXIS_DRIVER_TYPE(X, TMC2660)
    _TMC2660_DEFINE(X);
  #endif
  #if AXIS_DRIVER_TYPE(X2, TMC2660)
    _TMC2660_DEFINE(X2);
  #endif
  #if AXIS_DRIVER_TYPE(Y, TMC2660)
    _TMC2660_DEFINE(Y);
  #endif
  #if AXIS_DRIVER_TYPE(Y2, TMC2660)
    _TMC2660_DEFINE(Y2);
  #endif
  #if AXIS_DRIVER_TYPE(Z, TMC2660)
    _TMC2660_DEFINE(Z);
  #endif
  #if AXIS_DRIVER_TYPE(Z2, TMC2660)
    _TMC2660_DEFINE(Z2);
  #endif
  #if AXIS_DRIVER_TYPE(E0, TMC2660)
    _TMC2660_DEFINE(E0);
  #endif
  #if AXIS_DRIVER_TYPE(E1, TMC2660)
    _TMC2660_DEFINE(E1);
  #endif
  #if AXIS_DRIVER_TYPE(E2, TMC2660)
    _TMC2660_DEFINE(E2);
  #endif
  #if AXIS_DRIVER_TYPE(E3, TMC2660)
    _TMC2660_DEFINE(E3);
  #endif
  #if AXIS_DRIVER_TYPE(E4, TMC2660)
    _TMC2660_DEFINE(E4);
  #endif

  void tmc_init(TMC2660Stepper &st, const uint16_t mA, const uint16_t microsteps, const uint32_t, const float) {
    st.begin();
    st.rms_current(mA);
    st.microsteps(microsteps);
    st.intpol(INTERPOLATE);
    st.diss2g(true); // Disable short to ground protection. Too many false readings?

    int8_t timings[] = CHOPPER_TIMING; // Default 4, -2, 1

    TMC2660_n::CHOPCONF_t chopconf;
    chopconf.tbl = 1;
    chopconf.toff = timings[0];
    chopconf.hend = timings[1] + 3;
    chopconf.hstrt = timings[2] - 1;
    st.CHOPCONF(chopconf.sr);

    #if ENABLED(ADAPTIVE_CURRENT)
      SMARTEN_t smarten;
      smarten.semin = INCREASE_CURRENT_THRS;
      smarten.semax = REDUCE_CURRENT_THRS;
      st.SMARTEN(smarten.sr);
    #endif
  }
#endif // TMC2660

//
// TMC5160 Driver objects and inits
//
#if HAS_DRIVER(TMC5160)

  #include <SPI.h>
  #include "planner.h"
  #include "../core/enum.h"

  #if ENABLED(TMC_USE_SW_SPI)
    #define _TMC5160_DEFINE(ST) TMCMarlin<TMC5160Stepper> stepper##ST(TMC_##ST##_LABEL, ST##_CS_PIN, R_SENSE, TMC_SW_MOSI, TMC_SW_MISO, TMC_SW_SCK)
  #else
    #define _TMC5160_DEFINE(ST) TMCMarlin<TMC5160Stepper> stepper##ST(TMC_##ST##_LABEL, ST##_CS_PIN, R_SENSE)
  #endif
  // Stepper objects of TMC5160 steppers used
  #if AXIS_DRIVER_TYPE(X, TMC5160)
    _TMC5160_DEFINE(X);
  #endif
  #if AXIS_DRIVER_TYPE(X2, TMC5160)
    _TMC5160_DEFINE(X2);
  #endif
  #if AXIS_DRIVER_TYPE(Y, TMC5160)
    _TMC5160_DEFINE(Y);
  #endif
  #if AXIS_DRIVER_TYPE(Y2, TMC5160)
    _TMC5160_DEFINE(Y2);
  #endif
  #if AXIS_DRIVER_TYPE(Z, TMC5160)
    _TMC5160_DEFINE(Z);
  #endif
  #if AXIS_DRIVER_TYPE(Z2, TMC5160)
    _TMC5160_DEFINE(Z2);
  #endif
  #if AXIS_DRIVER_TYPE(E0, TMC5160)
    _TMC5160_DEFINE(E0);
  #endif
  #if AXIS_DRIVER_TYPE(E1, TMC5160)
    _TMC5160_DEFINE(E1);
  #endif
  #if AXIS_DRIVER_TYPE(E2, TMC5160)
    _TMC5160_DEFINE(E2);
  #endif
  #if AXIS_DRIVER_TYPE(E3, TMC5160)
    _TMC5160_DEFINE(E3);
  #endif
  #if AXIS_DRIVER_TYPE(E4, TMC5160)
    _TMC5160_DEFINE(E4);
  #endif

  void tmc_init(TMC5160Stepper &st, const uint16_t mA, const uint16_t microsteps, const uint32_t thrs, const float spmm) {
    #if DISABLED(STEALTHCHOP) || DISABLED(HYBRID_THRESHOLD)
      UNUSED(thrs);
      UNUSED(spmm);
    #endif
    st.begin();
    st.rms_current(mA, HOLD_MULTIPLIER);
    st.microsteps(microsteps);

    int8_t timings[] = CHOPPER_TIMING; // Default 4, -2, 1

    CHOPCONF_t chopconf;
    chopconf.tbl = 1;
    chopconf.toff = timings[0];
    chopconf.intpol = INTERPOLATE;
    chopconf.hend = timings[1] + 3;
    chopconf.hstrt = timings[2] - 1;
    st.CHOPCONF(chopconf.sr);

    st.iholddelay(10);
    st.TPOWERDOWN(128); // ~2s until driver lowers to hold current

    #if ENABLED(ADAPTIVE_CURRENT)
      COOLCONF_t coolconf;
      coolconf.semin = INCREASE_CURRENT_THRS;
      coolconf.semax = REDUCE_CURRENT_THRS;
      st.COOLCONF(coolconf.sr);
    #endif

    #if ENABLED(STEALTHCHOP)
      st.en_pwm_mode(true);

      PWMCONF_t pwmconf;
      pwmconf.pwm_freq = 0b01; // f_pwm = 2/683 f_clk
      pwmconf.pwm_autoscale = true;
      pwmconf.pwm_grad = 5;
      pwmconf.pwm_ampl = 180;
      st.PWMCONF(pwmconf.sr);

      #if ENABLED(HYBRID_THRESHOLD)
        st.TPWMTHRS(12650000UL*microsteps/(256*thrs*spmm));
      #else
        UNUSED(thrs);
        UNUSED(spmm);
      #endif
    #endif
    st.GSTAT(); // Clear GSTAT
  }
#endif // HAVE_TMC5160

void restore_stepper_drivers() {
  #if AXIS_IS_TMC(X)
    stepperX.push();
  #endif
  #if AXIS_IS_TMC(X2)
    stepperX2.push();
  #endif
  #if AXIS_IS_TMC(Y)
    stepperY.push();
  #endif
  #if AXIS_IS_TMC(Y2)
    stepperY2.push();
  #endif
  #if AXIS_IS_TMC(Z)
    stepperZ.push();
  #endif
  #if AXIS_IS_TMC(Z2)
    stepperZ2.push();
  #endif
  #if AXIS_IS_TMC(E0)
    stepperE0.push();
  #endif
  #if AXIS_IS_TMC(E1)
    stepperE1.push();
  #endif
  #if AXIS_IS_TMC(E2)
    stepperE2.push();
  #endif
  #if AXIS_IS_TMC(E3)
    stepperE3.push();
  #endif
  #if AXIS_IS_TMC(E4)
    stepperE4.push();
  #endif
}

void reset_stepper_drivers() {
  #if HAS_DRIVER(TMC26X)
    tmc26x_init_to_defaults();
  #endif
  #if ENABLED(HAVE_L6470DRIVER)
    L6470_init_to_defaults();
  #endif

  #if AXIS_IS_TMC(X)
    _TMC_INIT(X, planner.axis_steps_per_mm[X_AXIS]);
  #endif
  #if AXIS_IS_TMC(X2)
    _TMC_INIT(X2, planner.axis_steps_per_mm[X_AXIS]);
  #endif
  #if AXIS_IS_TMC(Y)
    _TMC_INIT(Y, planner.axis_steps_per_mm[Y_AXIS]);
  #endif
  #if AXIS_IS_TMC(Y2)
    _TMC_INIT(Y2, planner.axis_steps_per_mm[Y_AXIS]);
  #endif
  #if AXIS_IS_TMC(Z)
    _TMC_INIT(Z, planner.axis_steps_per_mm[Z_AXIS]);
  #endif
  #if AXIS_IS_TMC(Z2)
    _TMC_INIT(Z2, planner.axis_steps_per_mm[Z_AXIS]);
  #endif
  #if AXIS_IS_TMC(E0)
    _TMC_INIT(E0, planner.axis_steps_per_mm[E_AXIS]);
  #endif
  #if AXIS_IS_TMC(E1)
    { constexpr int extruder = 1; _TMC_INIT(E1, planner.axis_steps_per_mm[E_AXIS_N]); }
  #endif
  #if AXIS_IS_TMC(E2)
    { constexpr int extruder = 2; _TMC_INIT(E2, planner.axis_steps_per_mm[E_AXIS_N]); }
  #endif
  #if AXIS_IS_TMC(E3)
    { constexpr int extruder = 3; _TMC_INIT(E3, planner.axis_steps_per_mm[E_AXIS_N]); }
  #endif
  #if AXIS_IS_TMC(E4)
    { constexpr int extruder = 4; _TMC_INIT(E4, planner.axis_steps_per_mm[E_AXIS_N]); }
  #endif

  #if ENABLED(SENSORLESS_HOMING)
    #define TMC_INIT_SGT(P,Q) stepper##Q.sgt(P##_HOMING_SENSITIVITY);
    #if X_SENSORLESS
      #if X_HAS_STALLGUARD
        stepperX.sgt(X_HOMING_SENSITIVITY);
      #endif
      #if X2_HAS_STALLGUARD
        stepperX2.sgt(X_HOMING_SENSITIVITY);
      #endif
    #endif
    #if Y_SENSORLESS
      #if Y_HAS_STALLGUARD
        stepperY.sgt(Y_HOMING_SENSITIVITY);
      #endif
      #if Y2_HAS_STALLGUARD
        stepperY2.sgt(Y_HOMING_SENSITIVITY);
      #endif
    #endif
    #if Z_SENSORLESS
      #if Z_HAS_STALLGUARD
        stepperZ.sgt(Z_HOMING_SENSITIVITY);
      #endif
      #if Z2_HAS_STALLGUARD
        stepperZ2.sgt(Z_HOMING_SENSITIVITY);
      #endif
    #endif
  #endif
  #ifdef TMC_ADV
    TMC_ADV()
  #endif
}

//
// L6470 Driver objects and inits
//
#if HAS_DRIVER(L6470)

  #include <SPI.h>
  #include <L6470.h>

  #define _L6470_DEFINE(ST) L6470 stepper##ST(ST##_ENABLE_PIN)

  // L6470 Stepper objects
  #if AXIS_DRIVER_TYPE(X, L6470)
    _L6470_DEFINE(X);
  #endif
  #if AXIS_DRIVER_TYPE(X2, L6470)
    _L6470_DEFINE(X2);
  #endif
  #if AXIS_DRIVER_TYPE(Y, L6470)
    _L6470_DEFINE(Y);
  #endif
  #if AXIS_DRIVER_TYPE(Y2, L6470)
    _L6470_DEFINE(Y2);
  #endif
  #if AXIS_DRIVER_TYPE(Z, L6470)
    _L6470_DEFINE(Z);
  #endif
  #if AXIS_DRIVER_TYPE(Z2, L6470)
    _L6470_DEFINE(Z2);
  #endif
  #if AXIS_DRIVER_TYPE(E0, L6470)
    _L6470_DEFINE(E0);
  #endif
  #if AXIS_DRIVER_TYPE(E1, L6470)
    _L6470_DEFINE(E1);
  #endif
  #if AXIS_DRIVER_TYPE(E2, L6470)
    _L6470_DEFINE(E2);
  #endif
  #if AXIS_DRIVER_TYPE(E3, L6470)
    _L6470_DEFINE(E3);
  #endif
  #if AXIS_DRIVER_TYPE(E4, L6470)
    _L6470_DEFINE(E4);
  #endif

  #define _L6470_INIT(A) do{ \
    stepper##A.init(); \
    stepper##A.softFree(); \
    stepper##A.setMicroSteps(A##_MICROSTEPS); \
    stepper##A.setOverCurrent(A##_OVERCURRENT); \
    stepper##A.setStallCurrent(A##_STALLCURRENT); \
  }while(0)

  void L6470_init_to_defaults() {
    #if AXIS_DRIVER_TYPE(X, L6470)
      _L6470_INIT(X);
    #endif
    #if AXIS_DRIVER_TYPE(X2, L6470)
      _L6470_INIT(X2);
    #endif
    #if AXIS_DRIVER_TYPE(Y, L6470)
      _L6470_INIT(Y);
    #endif
    #if AXIS_DRIVER_TYPE(Y2, L6470)
      _L6470_INIT(Y2);
    #endif
    #if AXIS_DRIVER_TYPE(Z, L6470)
      _L6470_INIT(Z);
    #endif
    #if AXIS_DRIVER_TYPE(Z2, L6470)
      _L6470_INIT(Z2);
    #endif
    #if AXIS_DRIVER_TYPE(E0, L6470)
      _L6470_INIT(E0);
    #endif
    #if AXIS_DRIVER_TYPE(E1, L6470)
      _L6470_INIT(E1);
    #endif
    #if AXIS_DRIVER_TYPE(E2, L6470)
      _L6470_INIT(E2);
    #endif
    #if AXIS_DRIVER_TYPE(E3, L6470)
      _L6470_INIT(E3);
    #endif
    #if AXIS_DRIVER_TYPE(E4, L6470)
      _L6470_INIT(E4);
    #endif
  }

#endif // L6470
