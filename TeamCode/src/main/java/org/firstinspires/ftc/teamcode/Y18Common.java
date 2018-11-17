/**
 * Put all common hardware configuration here
 */

//Port 2 = drive motors, port 3 = attachment servos + motors


package org.firstinspires.ftc.teamcode;

import java.util.Locale;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.lynx.LynxI2cColorRangeSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DigitalChannelController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

//import com.qualcomm.robotcore.util.Range;


/**
 * Common codes
 */
public class Y18Common extends OpMode {

    /// Main timer
    ElapsedTime timer_ = new ElapsedTime();

    /// Motors
    DcMotor motorRF_;
    DcMotor motorRB_;
    DcMotor motorLF_;
    DcMotor motorLB_;


    static final boolean USE_LIFT = true;
    DcMotor motorLift_;
    double power_lift_ = 0;
    static final double LIFT_UP_POWER = -1.0;
    static final double LIFT_DOWN_POWER = 1.0;
    static final double  LIFT_ENC_COUNT_PER_GLYPH = 750;    // 750 for 6inch, new NeverRest 3.7 motor
    //static final double  MAX_LIFT_ENC_COUNT = LIFT_ENC_COUNT_PER_GLYPH*3+200;    // 3 glyphs + extra room
    static final double  MAX_LIFT_ENC_COUNT = 15000;        //  TBD
    static final double LANDING_ENC_CNT = 12500;            //TBD
    static final double  MIN_LIFT_ENC_COUNT = 0;            // min lift encoder count to prevent reverse
    static final double  LIFT_DIP_ENC_COUNT = 100;          // max lift encoder dip allowed; auto-hold OFF if set to 0
    static final double  MIN_LIFT_HOLD_POWER = 0.05;        // min power to hold the lift
    static final double  MIN_LIFT_POWER = 0.0;


    /// Servo Variables
    static final double  CR_SERVO_STOP = 0.5;

    ///Intake servos
    boolean USE_INTAKE_SERVOS = true;
    Servo servo_intake_r_;              //right intake servo
    Servo servo_intake_l_;              //left intake servo
    double INTAKE_POWER_BRAKE = CR_SERVO_STOP;
    //continuous, controlled by y button

    ///Dumping servos
    boolean USE_SERVO_DUMP = true;
    double DUMP_UP = 0.45;
    double DUMP_DOWN = 0;
    Servo servo_dump_;
    double servo_dump_init_ = DUMP_UP;
    //180, controlled by b button


    ///Servo for the intake system winch
    boolean USE_INTAKE_WINCH = true;
    Servo servo_winch_;
    double WINCH_POWER_BRAKE = CR_SERVO_STOP;
    //cont. servo, controlled by right (down) and left(up) bumpers
    double WINCH_UP_POWER = 1;
    double WINCH_DOWN_POWER = 0;

    ///Team marker servo
    boolean USE_SERVO_MARKER = true;
    Servo servo_marker_;
    /// TODO: Test + fix angles
    double MARKER_UP_POS_ = 0.7;
    double MARKER_DROP_POS_ = 0.0;

    //Front linear slide
    boolean USE_MINERALS_LIFT = true;
    double power_minerals_lift = 0.0;
    DcMotor motorMineralsLift_;
    //controls the linear slide at the front of the robot
    //uses the left joystick y axis for up and down.

    ///Lift pin servo
    Servo  servo_lift_pin_;                                 // Servo for lift pin
    double servo_lift_pin_pos_;                            // Lift pin servo position
    static final double  LIFT_PIN_STOP = CR_SERVO_STOP;
    static final double  LIFT_PIN_PULL = 0.0;

    ///Intake extension (continuous servo)
    boolean USE_SERVO_EXTENSION = true;
    Servo servo_extension_;
    double SERVO_EXTENSION_STOP = CR_SERVO_STOP;
    static final double EXTENSION_OUT = 1.0;
    static final double EXTENSION_IN = 0.0;



    /// IMU
    static final boolean USE_IMU  = true;           // use IMU sensor to turn
    BNO055IMU imu_;                         // Adafruit or RevHub IMU
    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

    Orientation imu_angles_;
    double heading_ = 0.0;                                  // current heading


    static final boolean USE_MR_RANGE = false;       // sensor removed bcz it may cause crash, 2018/01/06
    ModernRoboticsI2cRangeSensor mr_range_;          // MR range sensor
    double dist_mr_range_ = 0.0;

    /// Joy sticks
    static final float JOYSTICK_DEAD_ZONE = 0.1f;
    int a1_cnt_;                 // number of times pad1/A is pressed
    int b1_cnt_;                 // number of times pad1/B is pressed
    int x1_cnt_;                 // number of times pad1/X is pressed
    int y1_cnt_;                 // number of times pad1/Y is pressed
    int lb1_cnt_;                // number of times pad1/left_bumper is pressed
    int rb1_cnt_;                // number of times pad1/right_bumper is pressed
    int lsb1_cnt_;               // number of times pad2/left_joystick is pressed
    int rsb1_cnt_;               // number of times pad2/right_joystick is pressed

    int a2_cnt_;                 // number of times pad2/A is pressed
    int b2_cnt_;                 // number of times pad2/B is pressed
    int x2_cnt_;                 // number of times pad2/X is pressed
    int y2_cnt_;                 // number of times pad2/Y is pressed
    int lb2_cnt_;                // number of times pad2/left_bumper is pressed
    int rb2_cnt_;                // number of times pad2/right_bumper is pressed
    int lsb2_cnt_;               // number of times pad2/left_joystick is pressed
    int rsb2_cnt_;               // number of times pad2/right_joystick is pressed

    double last_button_time_;
    double last_button_time2_;

    /// Other variables
    double curr_time_;

    /// Battery voltage
    double battery_voltage_=0.0;     // battery voltage


    ///  Constructor
    public Y18Common() {
    }


    ///  Code to run when the op mode is initialized goes here
    @Override public void init() {
        /// Use the hardwareMap to get the dc motors and servos by name.
        motorLF_ = hardwareMap.dcMotor.get("motor1");
        motorLB_ = hardwareMap.dcMotor.get("motor2");
        motorRF_ = hardwareMap.dcMotor.get("motor3");
        motorRB_ = hardwareMap.dcMotor.get("motor4");

        // Reverse motor, required for RevHub
        motorLF_.setDirection(DcMotor.Direction.REVERSE);
        motorLB_.setDirection(DcMotor.Direction.REVERSE);
        motorRF_.setDirection(DcMotor.Direction.REVERSE);
        motorRB_.setDirection(DcMotor.Direction.REVERSE);

        motorLF_.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLB_.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRF_.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRB_.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if(USE_MINERALS_LIFT) {
            motorMineralsLift_ = hardwareMap.dcMotor.get("motorMineralsLift_");
            motorMineralsLift_.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        if(USE_INTAKE_SERVOS) {
            servo_intake_l_ = hardwareMap.servo.get("servo_intake_l_");
            servo_intake_r_ = hardwareMap.servo.get("servo_intake_r_");
            servo_intake_l_.setPosition(INTAKE_POWER_BRAKE);
            servo_intake_r_.setPosition(INTAKE_POWER_BRAKE);
        }

        if(USE_INTAKE_WINCH) {
            servo_winch_ = hardwareMap.servo.get("servo_winch_");
            servo_winch_.setPosition(WINCH_POWER_BRAKE);
        }

        if(USE_SERVO_DUMP) {
            servo_dump_ = hardwareMap.servo.get("servo_dump_");
            servo_dump_.setPosition(servo_dump_init_);
        }

        if (USE_SERVO_MARKER) {
            servo_marker_ = hardwareMap.servo.get("servo_marker_");
            servo_marker_.setPosition(MARKER_UP_POS_);
        }

        if(USE_LIFT) {
            motorLift_ = hardwareMap.dcMotor.get("motor6");
            motorLift_.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            ///TODO: Fix stuck init, reset lift position
            servo_lift_pin_ = hardwareMap.servo.get("servo_lift_pin_");
            servo_lift_pin_pos_ = LIFT_PIN_STOP ;
        }

        if (USE_SERVO_EXTENSION) {
            servo_extension_ = hardwareMap.servo.get("servo_extension_");
            servo_extension_.setPosition(SERVO_EXTENSION_STOP);
        }

        power_lift_ = 0.0;

        if( USE_IMU ) {
            parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
            parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
            parameters.loggingEnabled      = true;
            parameters.loggingTag          = "IMU";
            parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

            imu_ = hardwareMap.get(BNO055IMU.class, "imu");
            imu_.initialize(parameters);
            //while( !imu_.isSystemCalibrated() ) { }    // Error: cause OpMode stuck in init(), and robot to stop
            imu_.startAccelerationIntegration(new Position(), new Velocity(), 1000);
        }

        if( USE_MR_RANGE ) {
            mr_range_ = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "mr_range");
        }


        /// Set joystick deadzone, any value below this threshold value will be considered as 0; moved from init() to init_loop() to aovid crash
        if(gamepad1!=null) gamepad1.setJoystickDeadzone( JOYSTICK_DEAD_ZONE );
        if(gamepad2!=null) gamepad2.setJoystickDeadzone( JOYSTICK_DEAD_ZONE );


        curr_time_ = 0.0;
        last_button_time_ = 0.0;
        a1_cnt_ = 0;
        b1_cnt_ = 0;
        x1_cnt_ = 0;
        y1_cnt_ = 0;
        lb1_cnt_ = 0;
        rb1_cnt_ = 0;
        lsb1_cnt_ = 0;
        rsb1_cnt_ = 0;

        last_button_time2_ = 0.0;
        a2_cnt_ = 0;
        b2_cnt_ = 0;
        x2_cnt_ = 0;
        y2_cnt_ = 0;
        lb2_cnt_ = 0;
        rb2_cnt_ = 0;
        lsb2_cnt_ = 0;
        rsb2_cnt_ = 0;

    }


    /// This method will be called once before entering loop()
    @Override public void init_loop() {
        timer_.reset();
        curr_time_ = 0.0;
    }

    /// This method will be called repeatedly in a loop
    @Override public void loop () {
    }


    /// Code to run when the op mode is first disabled goes here
    @Override public void stop () {
    }

    /// Return true if it's RED team
    boolean isRedTeam() {
        //return ( START_AS_RED ? (!color_flipped_) : color_flipped_ );
        return true;
    }

    /// Return current robot heading based on gyro/IMU reading
    double getHeading() {
        heading_ = 0;
        if( USE_IMU && imu_!=null ) {
            imu_angles_  = imu_.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);  // acquiring angles are expensive, keep it minimal
            heading_ = AngleUnit.DEGREES.normalize(imu_angles_.firstAngle);
        }
        return heading_;
    }

    /// Return heading diff for a given init heading
    double getHeadingDiff(double init_h) {
        double curr_h = getHeading();
        double diff_h = init_h - curr_h;
        if( diff_h>=360.0 ) diff_h -= 360.0;
        else if( diff_h<=-360.0 ) diff_h += 360.0;
        return diff_h ;
    }

    /// Computes the current battery voltage
    // Copy from /robotcontroller/external/samples/ConceptTelemetry.java
    double getBatteryVoltage(boolean first_reading) {
        double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > 0) {
                result = Math.min(result, voltage);
                if(first_reading) break;    // return the first valid reading (>0.0) to minimize overhead
            }
        }
        return result;
    }

    int numVoltageSensors() {
        int num = 0;
        for( VoltageSensor sensor : hardwareMap.voltageSensor ) {
            ++num;
        }
        return num;
    }

    /// Update distance readings for all available range sensors, and return the reading for MR range
    double getDistances() {
        dist_mr_range_= 0.0;
        if( USE_MR_RANGE && mr_range_!=null ) {
            dist_mr_range_ = mr_range_.getDistance(DistanceUnit.CM);
            if( Double.isNaN(dist_mr_range_) ) dist_mr_range_ = 5.0;  // ultrasonic => optical
        }
        return dist_mr_range_;
    }

}

