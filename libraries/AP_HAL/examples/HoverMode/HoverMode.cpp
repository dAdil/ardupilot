/*
  Tail-sitter Hover Mode Implementation
  Author: Damian Adil
  2016
 */

#include <AP_HAL/AP_HAL.h>      // Hardware abstraction layer
#include <AP_AHRS/AP_AHRS.h>    // Attitude heading and reference system
#include <AP_ADC/AP_ADC.h>      // Analog-digital converter
#include <DataFlash/DataFlash.h>// Memory chip library


const AP_HAL::HAL& hal = AP_HAL::get_HAL();

// AHRS Declarations
AP_InertialSensor ins;
Compass Compasss;
AP_GPS gps;
AP_Baro baro;
AP_SerialManager serial_manager;

AP_AHRS_DCM ahrs(ins, baro, gps);


// Gains
Vector3f P = Vector3f( 2.20 , 4.00 , 1.60 ); 				//( 2.20 , 4.00 , 1.60 );
Vector3f I = Vector3f( 0.00 , 0.00 , 0.00 ); 				//( 0.00 , 0.00 , 0.00 );
Vector3f D = Vector3f( 1.00 , 1.30 , 1.50 ); 				//( 1.00 , 1.30 , 1.50 );
Vector3f PID_alt = Vector3f(  1.00 , 0.10 , 0.00 ); 		//( 1.00 , 0.10 , 0.00 );
Vector3f TrimMoment = Vector3f( 0 , 0.11 , 0.01 ); 				//( 0.00 , 0.07 , 0.00 );
Vector3f TrimOrient = Vector3f( 0 , 0    , 0 ) * M_PI / 180;	//( 0.00 , 0.00 , 0.00 );


// ================================================================================================
// Variable declarations
// ================================================================================================

// Constants for gyroscope filters.
// a=0.029 | 5Hz Lag
// a=0.010 | 1.75Hz Lag (To 90%)
Vector3f a = Vector3f( 0.070 , 0.070 , 0.070 ); // ** ( 0.07 , 0.07 , 0.07 )

Vector3f moment_p;
float    fx_d;

float elevL;
float elevR;
float throttleL;
float throttleR;
int16_t pwm_elevL;
int16_t pwm_elevR;
int16_t pwm_motorL;
int16_t pwm_motorR;

uint8_t counter;

Vector3f    phiThetaPsi;    // Orientation
Quaternion  quat_a;         // Quaternion orientation
Quaternion  quat_v;
Vector3f    pqr;            // "Gyro" or orientation rates.
Vector3f    pqrOld;         // Previous pqr values used for filtering inertia

float fx_trim = 8.5f;
float d_alt = 1.8;
float t_alt = 0;
float a_alt = 0.01;
float    alt;
float    altERR;
float    altINT;
float    altOld;


float heading;
float target_heading = 0;
float target_theta;
float target_psi;
float yaw_rate_input;
float max_heading_rate = 10 * M_PI / 180;
Matrix3f dcm_matrix;

Vector3f    eul_d = Vector3f(0,0,0);          // Desired eulers
Quaternion  quat_d;         // Desired quaternion
Quaternion  quat_c;         // Actual quaternion conjugate
Quaternion  quat_e;         // Quaternion error
Quaternion  quat_b;

// Body component error from quaternion subtraction
Vector3f errPhiThetaPsi;
Vector3f errPhiThetaPsiINT = Vector3f(0,0,0);


Vector3f errPhiThetaPsiDER = Vector3f(0,0,0);

// Data flash declarations
// =======================
DataFlash_Class dataflash{"DF Test 0.1"};

#define LOG_TEST_MSG 1
static uint16_t log_num;

struct PACKED log_Test {
    LOG_PACKET_HEADER;
    float   t;
    float   phi,the,psi;
    float   ERR, INT, DER, Pz;
    float   sL, sR, tL, tR;
    uint16_t Switch;
    float   z;
    float   fx;
};

static const struct LogStructure log_structure[] = {
    LOG_COMMON_STRUCTURES,
    { LOG_TEST_MSG, sizeof(log_Test),
    "Data", "ffffffffffffHff",        "t,phi,the,psi,ERR,INT,"
    		"DER,Pz,sL,sR,tL,tR,Switch,z,fx" }
};

// My functions
// ===========================================================
Quaternion quatMultiply(Quaternion q, Quaternion r) {
    Quaternion t;
    Quaternion u;

    t.q1 = r.q1*q.q1 - r.q2*q.q2 - r.q3*q.q3 - r.q4*q.q4;
    t.q2 = r.q1*q.q2 + r.q2*q.q1 - r.q3*q.q4 + r.q4*q.q3;
    t.q3 = r.q1*q.q3 + r.q2*q.q4 + r.q3*q.q1 - r.q4*q.q2;
    t.q4 = r.q1*q.q4 - r.q2*q.q3 + r.q3*q.q2 + r.q4*q.q1;

    u.q1 = t.q1 / pow((pow(t.q1,2) + pow(t.q2,2) + pow(t.q3,2) + pow(t.q4,2)),0.5);
    u.q2 = t.q2 / pow((pow(t.q1,2) + pow(t.q2,2) + pow(t.q3,2) + pow(t.q4,2)),0.5);
    u.q3 = t.q3 / pow((pow(t.q1,2) + pow(t.q2,2) + pow(t.q3,2) + pow(t.q4,2)),0.5);
    u.q4 = t.q4 / pow((pow(t.q1,2) + pow(t.q2,2) + pow(t.q3,2) + pow(t.q4,2)),0.5);

    return u;
}

int intLimit (int value, int upper, int lower) {
    if (value > upper) {
        value = upper;
    }
    if (value < lower) {
        value = lower;
    }
    return value;
}

float floLimit (float value, float upper, float lower) {
    if (value > upper) {
        value = upper;
    }
    if (value < lower) {
        value = lower;
    }
    return value;
}

static void setup_uart(AP_HAL::UARTDriver *uart, const char *name)
{
    if (uart == NULL) {
        // that UART doesn't exist on this platform
        return;
    }
    uart->begin(57600);
}


// Arduino functions
// ===================================================================
void setup (void)
{


    // Set the orientation to be 90 degrees of pitch.
    AP_Param::set_object_value(&ahrs,ahrs.var_info,"ORIENTATION", 24);
    // Pitch 90 deg COMMENTED OUT FOR CONVENTIONAL
    AP_Param::set_object_value(&Compasss,Compasss.var_info,"ORIENT", 24);

    AP_Param::set_object_value(&Compasss,Compasss.var_info,"EXTERNAL",1);
    AP_Param::set_object_value(&Compasss,Compasss.var_info,"LEARN",0);
    AP_Param::set_object_value(&Compasss,Compasss.var_info,"USE", 1);
    // true if used for DCM yaw
    AP_Param::set_object_value(&Compasss,Compasss.var_info,"AUTODEC",0);

    // Initialise sensors
    ins.init(400);

    baro.init();
    baro.calibrate();

    ahrs.init();
    serial_manager.init();

    if( Compasss.init() ) {
        Compasss.set_and_save_offsets(0, -289, -423, 307);
        ahrs.set_compass(&Compasss);
    } else {
    }

    gps.init(NULL, serial_manager);

    // Enable the rc channels
    for (uint8_t i=0; i<14; i++) {
        hal.rcout->enable_ch(i);
    }

    setup_uart(hal.uartC, "uartC"); // telemetry 1

    // Memory card initialisation
    // ==========================
    dataflash.Init(log_structure, ARRAY_SIZE(log_structure));

    hal.console->println("Dataflash Log Test 1.0");

    // Test
    hal.scheduler->delay(20);
    dataflash.ShowDeviceInfo(hal.console);

    if (dataflash.NeedPrep()) {
        hal.console->println("Preparing dataflash...");
        dataflash.Prep();
    }

    // We start to write some info (sequentialy) starting from page 1
    // This is similar to what we will do...
    dataflash.StartNewLog();
    log_num = dataflash.find_last_log();
    hal.console->printf("Using log number %u\n", log_num);
}


void loop (void)
{
    static uint32_t last_read;
    static uint32_t last_controlled;
    static uint32_t last_print;
    uint32_t now = AP_HAL::micros();
    static uint32_t timeOld;
    static uint32_t timeNew;

    // Read AHRS data at 400Hz.
    if (now - last_read > 2500 /* 2.5ms = 400Hz*/) {
        last_read = now;

        pqrOld = pqr;

        // Update the AHRS system.
        timeOld = timeNew;
        ahrs.update();
        timeNew = AP_HAL::micros();


        baro.accumulate();
        baro.update();

        altOld = alt;
        alt = a_alt * baro.get_altitude() + (1-a_alt) * altOld;
        altERR = d_alt - (alt + t_alt);

        yaw_rate_input = hal.rcin->read(3);
        if (yaw_rate_input > 1600 || yaw_rate_input < 1400) {
        	target_heading = target_heading + ((((yaw_rate_input-1000.0f)/500.0f)-1.0f) * max_heading_rate)*(timeNew-timeOld)/1000000.0f;
        }

        target_theta = - (1.0f * hal.rcin->read(1) - 1500.0f) / 500.0f * (13.0f * M_PI / 180.0f);
        target_psi   = - (1.0f * hal.rcin->read(0) - 1500.0f) / 500.0f * (13.0f * M_PI / 180.0f);

        // Update the orientation values and filter the rates.
        phiThetaPsi.y   =  ahrs.pitch;
        phiThetaPsi.z   =  ahrs.roll;
        dcm_matrix.from_euler(phiThetaPsi.z, phiThetaPsi.y, 0);
        Compasss.read();
        heading = Compasss.calculate_heading(dcm_matrix);
        phiThetaPsi.x = -heading;

        quat_a.from_euler(phiThetaPsi.x,phiThetaPsi.y,phiThetaPsi.z);

        phiThetaPsi.x = quat_a.get_euler_roll();
        phiThetaPsi.y = quat_a.get_euler_pitch();
        phiThetaPsi.z = quat_a.get_euler_yaw();

        pqr.x = a.x *-ahrs.get_gyro().z + (1-a.x)*pqrOld.x;
        pqr.y = a.y * ahrs.get_gyro().y + (1-a.y)*pqrOld.y;
        pqr.z = a.z * ahrs.get_gyro().x + (1-a.z)*pqrOld.z;

        // Get the desired orientation in LVLHquat_d.from_euler(eul_d.x,eul_d.y,eul_d.z);

        // Calculate the orientation error
        quat_b.from_euler(phiThetaPsi.x,phiThetaPsi.y,phiThetaPsi.z);
        quat_c = quat_b;
        quat_c.q2 = -quat_c.q2;
        quat_c.q3 = -quat_c.q3;
        quat_c.q4 = -quat_c.q4;

        quat_d.from_euler(target_heading,target_theta,target_psi);

        quat_e = quatMultiply(quat_c,quat_d);

        errPhiThetaPsi.x = 2 * acosf(quat_e.q1) * quat_e.q2 / sinf(acosf(quat_e.q1)) + TrimOrient.x;
        errPhiThetaPsi.y = 2 * acosf(quat_e.q1) * quat_e.q3 / sinf(acosf(quat_e.q1)) + TrimOrient.y;
        errPhiThetaPsi.z = 2 * acosf(quat_e.q1) * quat_e.q4 / sinf(acosf(quat_e.q1)) + TrimOrient.z;

        if (hal.rcin->read(4) < 1500) {
            errPhiThetaPsiINT = errPhiThetaPsiINT + errPhiThetaPsi * (timeNew - timeOld)/1000000;
            altINT = altINT + altERR * (timeNew - timeOld ) / 1000000;
        } else {
        	// Reset the integral terms
            errPhiThetaPsiINT = Vector3f(0,0,0);
            altINT = 0;
        }


        errPhiThetaPsiDER =  - pqr;
    }



    // Calculate the responses and write to controllers at 100Hz.
    if (now - last_controlled > 10000 /* 10ms = 100Hz*/) {
        last_controlled = now;

        P.z = ((hal.rcin->read(5) - 1500)/500) + 4.5; // **
        //P.z = 0;

        moment_p.x = 0.1524 * (P.x * errPhiThetaPsi.x +I.x * errPhiThetaPsiINT.x + D.x * errPhiThetaPsiDER.x) + TrimMoment.x;
        moment_p.y = 0.3700 * (P.y * errPhiThetaPsi.y +I.y * errPhiThetaPsiINT.y + D.y * errPhiThetaPsiDER.y) + TrimMoment.y;
        moment_p.z = 0.1524 * (P.z * errPhiThetaPsi.z +I.z * errPhiThetaPsiINT.z + D.z * errPhiThetaPsiDER.z) + TrimMoment.z;

        //Rate controller implementation: Comment out
        //float K = 4.5;
        //moment_p.z = 0.3700 * (((errPhiThetaPsi.z * K) - pqr.z)*P.z) + TrimMoment.z;

        moment_p.x = floLimit(moment_p.x, 0.2, -0.2);

        //fx_d = (hal.rcin->read(2) - 1000.0f) / 1000.0f * 16.0f;
        fx_d = altERR * PID_alt.x + altINT * PID_alt.y + fx_trim;

        // Turn moment controls into physical controls.
        elevL =-1.0f * atan2f(moment_p.y/0.3700 - moment_p.x/0.1524, fx_d + moment_p.z/0.1524);
        elevR =-1.0f * atan2f(moment_p.y/0.3700 + moment_p.x/0.1524, fx_d - moment_p.z/0.1524);
        throttleL = (fx_d + moment_p.z/0.1524) / (2.0f * cosf(elevL));
        throttleR = (fx_d - moment_p.z/0.1524) / (2.0f * cosf(elevR));

        // Turn physical controls into pwm values.
        // Minus sign can occur on servos due to servo reversing.
        pwm_elevL   =  ( elevL * 180 / M_PI * 10 ) + 1500;
        pwm_elevR   =  ( elevR * 180 / M_PI * 10 ) + 1500;
        pwm_motorL  = 100 * throttleL + 1000;
        pwm_motorR  = 100 * throttleR + 1000;

        pwm_elevL   = intLimit(pwm_elevL,2100,900);
        pwm_elevR   = intLimit(pwm_elevR,2100,900);
        pwm_motorL  = intLimit(pwm_motorL,2000,1000);
        pwm_motorR  = intLimit(pwm_motorR,2000,1000);


        // Write the values to the servos and ESCs.
        uint16_t switchValue = hal.rcin->read(4);
        if (switchValue < 1250) {
            hal.rcout->write(0, pwm_elevL);
            hal.rcout->write(1, pwm_elevR);
            hal.rcout->write(2, pwm_motorL);
            hal.rcout->write(3, pwm_motorR);
            //hal.rcout->write(2, 1000);
            //hal.rcout->write(3, 1000);
        } else {
            hal.rcout->write(0, 1500);
            hal.rcout->write(1, 1500);
            hal.rcout->write(2, 1000);
            hal.rcout->write(3, 1000);
            pwm_elevL = 1500;
            pwm_elevR = 1500;
            pwm_motorL = 1000;
            pwm_motorR = 1000;
        }

        if (switchValue > 1750) {
            t_alt = 0.5 - alt;
        }


    }


    // Calculate the responses and write to telemetry at 10Hz.
    if (now - last_print > 100000) {
        last_print = now;

        hal.console->printf("%6.1f | %6.1f | %6.1f | %6.1f\n",ToDeg(errPhiThetaPsi.x),ToDeg(errPhiThetaPsi.y),ToDeg(errPhiThetaPsi.z),ToDeg(target_theta));

        //
        struct log_Test pkt = {
            LOG_PACKET_HEADER_INIT(LOG_TEST_MSG),
            t       : (AP_HAL::micros()/1000000.0f),
            phi     : (phiThetaPsi.x * 180 / M_PI),
            the     : (phiThetaPsi.y * 180 / M_PI),
            psi     : (phiThetaPsi.z * 180 / M_PI),
            ERR     : 0.370 * (P.y * errPhiThetaPsi.y),
            INT     : 0.370 * (I.y * errPhiThetaPsiINT.y),
            DER     : 0.370 * (D.y * errPhiThetaPsiDER.y),
            Pz      : P.z,
            sL      : (elevL * 180 / M_PI),
            sR      : (elevR * 180 / M_PI),
            tL      : throttleL,
            tR      : throttleR,
            Switch  : (hal.rcin->read(4)),
            z       : altERR,
            fx      : fx_d,
        };
        dataflash.WriteBlock(&pkt, sizeof(pkt));
    }



}


AP_HAL_MAIN();
