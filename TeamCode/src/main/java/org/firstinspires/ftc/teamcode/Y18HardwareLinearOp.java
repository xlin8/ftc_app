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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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

/**
 * Hardware used by autonomous
 */
public class Y18HardwareLinearOp extends LinearOpMode {
    // Main timer
    ElapsedTime timer_ = new ElapsedTime();
    double currTime_ = 0;

    // Motors
    DcMotor motorRF_;
    DcMotor motorRB_;
    DcMotor motorLF_;
    DcMotor motorLB_;

    // Lift motor
    DcMotor motorLift_;
    static final boolean USE_LIFT = true;
    static final double LIFT_UP_POWER = -1.0;
    static final double LIFT_DOWN_POWER = 1.0;
    static final double LIFT_LANDING_ENC_CNT = 13500;           //
    static final double MIN_LIFT_HOLD_POWER = 0.05;        // min power to hold the lift
    static final double MIN_LIFT_POWER = 0.0;
    double liftPower_ = 0;

    // Lift pin servo
    Servo  servoLiftPin_;                                 // Servo for lift pin
    static final double  LIFT_PIN_STOP = 1.0;
    static final double  LIFT_PIN_PULL = 1.0 - LIFT_PIN_STOP;
    double servoLiftPinPos_ = LIFT_PIN_STOP;             // Lift pin servo position

    // Servo Variables
    static final double CR_SERVO_STOP = 0.5;

    //Intake servos
    Servo servoIntakeR_;              // right intake servo
    Servo servoIntakeL_;              // left intake servo
    static final boolean USE_INTAKE_SERVOS = true;
    static final double INTAKE_POWER_BRAKE = CR_SERVO_STOP;

    // Dumping servos
    Servo servoDump_;
    static final boolean USE_SERVO_DUMP = true;
    static final double DUMP_UP = 0.45;
    static final double DUMP_DOWN = 0;
    static final double DUNP_INIT_POSITION = DUMP_UP;

    // Team marker servo
    Servo servoMarker_;
    static final boolean USE_SERVO_MARKER = true;
    static final double MARKER_UP_POS_ = 0.7;
    static final double MARKER_DROP_POS_ = 0.0;

    // Front linear slide
    DcMotor motorMineralLift_;
    static final boolean USE_MINERAL_LIFT = false;

    // IMU
    BNO055IMU imu_;                                        // Adafruit or RevHub IMU
    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
    static final boolean USE_IMU = true;                  // use IMU sensor to turn
    Orientation imuAngles_;
    double heading_ = 0.0;                                 // current heading

    ModernRoboticsI2cRangeSensor mrRange_;                 // MR range sensor
    static final boolean USE_MR_RANGE = false;             // sensor removed bcz it may cause crash, 2018/01/06
    double mrRangeDist_ = 0.0;

    @Override
    public void runOpMode() { 
        // Do nothing
    }

    //  Code to run when the op mode is initialized goes here
    public void initialize() {
        // Use the hardwareMap to get the dc motors and servos by name.
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

        motorLF_.setPower(0);
        motorLB_.setPower(0);
        motorRF_.setPower(0);
        motorRB_.setPower(0);

        if (USE_MINERAL_LIFT) {
            motorMineralLift_ = hardwareMap.dcMotor.get("motorMineralsLift_");
            motorMineralLift_.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        if (USE_INTAKE_SERVOS) {
            servoIntakeL_ = hardwareMap.servo.get("servo_intake_l_");
            servoIntakeR_ = hardwareMap.servo.get("servo_intake_r_");
            servoIntakeL_.setPosition(INTAKE_POWER_BRAKE);
            servoIntakeR_.setPosition(INTAKE_POWER_BRAKE);
        }

        if (USE_SERVO_DUMP) {
            servoDump_ = hardwareMap.servo.get("servo_dump_");
            servoDump_.setPosition(DUNP_INIT_POSITION);
        }

        if (USE_SERVO_MARKER) {
            servoMarker_ = hardwareMap.servo.get("servo_marker_");
            servoMarker_.setPosition(MARKER_UP_POS_);
        }

        if (USE_LIFT) {
            motorLift_ = hardwareMap.dcMotor.get("motor6");
            motorLift_.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            liftPower_ = 0.0;

            servoLiftPin_ = hardwareMap.servo.get("servo_lift_pin_");
            servoLiftPinPos_ = LIFT_PIN_STOP;
            servoLiftPin_.setPosition(servoLiftPinPos_);
        }

        if (USE_IMU) {
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

        if (USE_MR_RANGE) {
            mrRange_ = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "mr_range");
        }

        timer_.reset();
        currTime_ = 0.0;
    }

    // Return true if it's RED team
    boolean isRedTeam() {
        // return (START_AS_RED ? (!color_flipped_) : color_flipped_);
        return true;
    }

    // Return current robot heading based on gyro/IMU reading
    double getHeading() {
        heading_ = 0;
        if (USE_IMU && imu_!=null) {
            imuAngles_  = imu_.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);  // acquiring angles are expensive, keep it minimal
            heading_ = AngleUnit.DEGREES.normalize(imuAngles_.firstAngle);
        }

        return heading_;
    }

    // Return heading diff for a given init heading
    double getHeadingDiff(double init_h) {
        double curr_h = getHeading();
        double diff_h = init_h - curr_h;
        if (diff_h >= 360.0) diff_h -= 360.0;
        else if (diff_h <= -360.0) diff_h += 360.0;
        return diff_h ;
    }

    // Computes the current battery voltage
    // Copy from /robotcontroller/external/samples/ConceptTelemetry.java
    double getBatteryVoltage(boolean first_reading) {
        double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > 0) {
                result = Math.min(result, voltage);
                if (first_reading) break;    // return the first valid reading (>0.0) to minimize overhead
            }
        }

        return result;
    }

    int numVoltageSensors() {
        int num = 0;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            ++num;
        }

        return num;
    }

    // Update distance readings for all available range sensors, and return the reading for MR range
    double getDistances() {
        mrRangeDist_= 0.0;
        if (USE_MR_RANGE && mrRange_!=null) {
            mrRangeDist_ = mrRange_.getDistance(DistanceUnit.CM);
            if( Double.isNaN(mrRangeDist_) ) mrRangeDist_ = 5.0;  // ultrasonic => optical
        }
        return mrRangeDist_;
    }
}
