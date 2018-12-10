/*
 * This is Program for Autonomous Run for FTC 2018-2019 
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.lang.Math;
import java.util.List;
import java.util.Locale;

///  Autonomous Run for League Meet0
@Autonomous(name="Y18AutoLinearOp", group="GG")
// @Disabled
public class Y18AutoLinearOp extends Y18HardwareLinearOp
{
    /// Drive modes
    static final int DRIVE_STOP = 0;                        // stop
    static final int DRIVE_RESET_ENC_DONE = 1;              // wait till encoder reset
    static final int DRIVE_FORWARD = 2;                     // go forward till timeout
    static final int DRIVE_FORWARD_ENC = 3;                 // go forward for a given distance
    static final int DRIVE_FORWARD_ENC_SLOW = 4;            // slowly go forward for a given distance
    static final int DRIVE_BACKWARD = 5;                    // go backward till timeout
    static final int DRIVE_BACKWARD_ENC = 6;                // go backward for a given distance
    static final int DRIVE_BACKWARD_ENC_SLOW = 7;           // slowly go backward for a given distance
    static final int DRIVE_TURN_LEFT = 8;                   // turn left till timeout
    static final int DRIVE_TURN_TO_LEFT = 9;                // turn left to a specified heading based on gyro
    static final int DRIVE_TURN_LEFT_ENC = 10;              // turn left for a given degree based on encoders
    static final int DRIVE_TURN_RIGHT = 11;                 // turn right till timeout
    static final int DRIVE_TURN_TO_RIGHT = 12;              // turn right to a specified heading based on gyro
    static final int DRIVE_TURN_RIGHT_ENC = 13;             // turn right for a given degree based on encoders
    static final int DRIVE_FORWARD_ENC_TO_WALL = 14;        // go forward to approach to the wall
    static final int DRIVE_BACKWARD_ENC_TO_WALL = 15;       // go backward to approach to the wall
    static final int DRIVE_CHECK_DISTANCE = 16;
    static final int DRIVE_WAIT_TILL = 17;                  // wait till a time is reached
    static final int DRIVE_STATE_JUMP = 18;                 // state jump
    static final int DRIVE_SHIFT_GEAR = 19;                 // shift gears to make robot move faster/slower by changing drivePowerFactor_
    static final int DRIVE_LANDING = 20;                    //for landing the robot
    static final int DRIVE_PULL_PIN = 21;
    static final int DRIVE_DROP_MARKER = 22;
    static final int DRIVE_MINERAL_DETECTION = 23;
    static final int DRIVE_MODE_NUM = 24;                   // total number of modes

    /// General settings for AutoRun
    static final double  AUTO_RUN_TIME = 60.0;              // 60 sec for testing/debugging
    static final double  AUTO_RESET_ENC_TIME = 1.00;        // period for reseting encoders when entering DRIVE_RESET_ENC_DONE mode
    static final boolean USE_ENC_FOR_DRIVE = true;          // use encoder for accurate movement

    /// Power for drive and turn, need be tuned for each robot
    static final double DRIVE_ENC_WHEEL_POWER = 0.50;       // default driving power using encoder; ~29sec for 6*3m, for 2017       //was 0.4 Aditi Dec 9th 2018
    static final double DRIVE_ENC_SLOW_WHEEL_POWER = 0.15;  // power for encoder based slow drive for STOP_WHITE
    static final double DRIVE_ENC_TURN_WHEEL_POWER = 0.20;  // default turning power using encoder, 2017/09/08          //was 0.12 Aditi Dec 9th 2018
    static final double DRIVE_WHEEL_POWER = 0.20;           // default driving power
    static final double TURN_WHEEL_POWER = 0.20;            // default turning power
    static final double TURN_SLOW_WHEEL_POWER = 0.15;       // slow turning power
    static final double SLOW_TURN_DEGREE = 10.0;            // degree to enable slow turn; 10.0 degree by dflt; 0, disabled

    static final double ENC_DIST_SCALE = 2000.0/1.00;       // 2000 ticks <=> 1.00 meters, 2017/09/06
    static final double ENC_DEG_SCALE = 2000.0/150;         // 2000 ticks <=> ~150 for MW6 on mat, 2017/10/15

    static final double  AUTO_CORRECT_HEADING_GAIN = 0.012;  // previously 0.025; power percentage to be adjusted for each degree of heading error; 1-degree error => 5% power diff // dflt for 0.40, 2017/09/08
    static final double  AUTO_CORRECT_MAX_HEADING_ERROR = 40;  // power percentage to be adjusted for each degree of heading error; 1-degree error => 5% power diff
    static final boolean AUTO_CORRECT_HEADING = true;       // automatically correct the last heading for DRIVE_FORWARD mode
    static final double  MAX_HEADING_CORRECTION = 0.95;     // max heading correction; 0, not used

    static final double MAX_WALL_DISTANCE = 17.5;           // max distance to wall; distance higher than this will trigger re-alignment
    static final double MAX_DISTANCE_RANGE = 99.9;          // any distance larger than this value will be consider as invalid reading
    static final double SEE_WALL_DISTANCE = 100.0;          // wall distance to stop

    // Mineral detector
    VuforiaLocalizer vuforia_;
    TFObjectDetector tfod_;
    static final boolean USE_MINERAL_DETECTION = true;
    static final boolean DETECT_GOLD_MINERAL_BEFORE_START = false;
    static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    static final String VUFORIA_KEY = "ATbVhA//////AAAAGQpUcoBny0Xdi+FFWntcC3w9C63+hv3ccdXKcUsUhNYtbt8IbpT9SQ+VsthWIyix0rrzYP8KYaSYY5na+nufoGmLQo8vE8CPWmUj8eZcdlM9k4mi8ge0T2uzuoKZmcllal8cM3hRxo1JBFVtavCrgulnZxQ8hMsbzZuA+dZDGQTOEOCCH8ZHuh6wrIUygVerHfrXXlpeIAQvXzBiYrVPetr3zu3ROn6rno75mQ0KCM8Qp87BGS4Orx+GwxL8FlO+EXA3aSBvDh7+a57co5212MkGIRUceXxAd+BfoFjiWg3SbJpVbDM7TcDApVR88jlqeEDmbc/ODajLjEKziycgihi1rpq1lOBys2oJ68qdVrtO";
    static final int MAX_TIMES_DETECT_GOLD_MINERAL = 3;
    static final int GOLD_MINERAL_AT_LEFT = 0;
    static final int GOLD_MINERAL_AT_CENTER = 1;
    static final int GOLD_MINERAL_AT_RIGHT = 2;
    static final int GOLD_MINERAL_AT_UNKNOWN = 3;
    static final double MIN_MINERAL_HEIGHT_RATIO = 0.6;
    static final double MIN_MINERAL_HEIGHT_WIDTH_RATIO = 0.75;
    static final double MAX_MINERAL_BOTTOM_DIFF_RATIO = 0.4;

    // Control state machine
    int currStateId_ = -1;                                  // current state ID
    double currStateStartTime_ = 0.0;                       // current state start time
    double currStateStartHeading_ = 0.0;                    // current state start heading
    double currStateEncCnt_ = 0.0;                          // current state targeted encoder count

    double initWaitTime_ = 0.0;                             // Additional wait time before starting state machine

    boolean turnSlow_ = false;                              // turn slow if true

    int numDistOk_ = 0;                                     // numer of times for which distance is OK
    int numDistFar_ = 0;                                    // numer of times for which distance is too far

    double targetHeading_ = 0.0;                            // target heading for current state
    double lastTurnToHeading_ = -1.0;                       // last TURN_TO heading
    double headingError_ = 0.0;                             // current heading error
    double adjustedHeadingError_ = 0.0;                     // adjusted heading error
    double headingCorrection_ = 0.0;                        // heading correction

    double drivePowerFactor_ = 1.0;                         // factor for drive power

    // Detect gold mineral position
    int goldPosition_ = GOLD_MINERAL_AT_UNKNOWN;
    boolean isGuessedGoldPosition_ = false;

    String tripName_ = "DepotTrip";                          // trip name

    @Override
    public void runOpMode() {
        initialize();

        // Wait for the game to begin
        telemetry.addData(">", "Press Play to start autonomous " + "(Gold_at="+String.valueOf(goldPosition_)+" is_guessed="+String.valueOf(isGuessedGoldPosition_)+")");
        telemetry.update();
        waitForStart();

        initializeWhenStart();

        while (opModeIsActive()) {
            driveRobort();
        }

        cleanUpAtEndOfRun();
    }

    /// Autonomous mode specific initialization
    @Override
    public void initialize() {
        super.initialize();

        if (USE_MINERAL_DETECTION) {
            initializeVuforia();

            if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
                initializeTfod();

                if (DETECT_GOLD_MINERAL_BEFORE_START) {
                    // Detect gold mineral position before pushing the START buton
                    if (tfod_ != null) tfod_.activate();

                    detectGoldMineralPosition(timer_.time(), 20.0);
                }
            } else {
                telemetry.addData("Warn", "This device is not compatible with TFOD");
            }
        }

        if (USE_ENC_FOR_DRIVE) {
            resetDriveEncoders();
        }

        if (USE_LIFT) {
            motorLift_.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        numDistOk_ = 0;
        numDistFar_ = 0;

        targetHeading_ = 0.0;
    }

    void initializeWhenStart() {
        currTime_ = 0.0;

        if (USE_LIFT) motorLift_.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Activate Tensor Flow Object Detection.
        if (DETECT_GOLD_MINERAL_BEFORE_START == false) {
           if (tfod_ != null) tfod_.activate();
        }
    }

    void cleanUpAtEndOfRun() {
       if (tfod_ != null) {
            tfod_.shutdown();
       }
    }

    void driveRobort() {
        double time_t = timer_.time();
        currTime_ = time_t;

        int drive_mode = DRIVE_STOP;

        // power for wheels
        double power_lf = 0;
        double power_rf = 0;
        double power_lb = 0;
        double power_rb = 0;

        if (time_t >= AUTO_RUN_TIME) {
            // timeout, stop robot and sweeper
        } else {
            drive_mode = getDriveModeRed(time_t);

            power_lf = getLeftFrontPower(drive_mode);
            power_rf = getRightFrontPower(drive_mode);

            double h_err = 0.0;    // heading error
            if (AUTO_CORRECT_HEADING) {
                switch (drive_mode) {
                    case DRIVE_FORWARD_ENC_TO_WALL:
                    case DRIVE_BACKWARD_ENC_TO_WALL:
                    case DRIVE_FORWARD_ENC:
                    case DRIVE_BACKWARD_ENC:
                    case DRIVE_FORWARD_ENC_SLOW:
                    case DRIVE_BACKWARD_ENC_SLOW:
                        // Get the heading error from IMU
                        h_err = getHeadingError();  // expensive heading reading
                        if (Math.abs(h_err) > AUTO_CORRECT_MAX_HEADING_ERROR) {  // prevent incorrect heading error causing robot to spin
                            h_err = 0.0;
                        }
                        break;
                    default:
                        break;
                }
            }

            // Adjust wheel power to correct heading or follow wall
            if (Math.abs(h_err) > 0.0) {
                if (drive_mode == DRIVE_BACKWARD_ENC ||
                        drive_mode == DRIVE_BACKWARD_ENC_SLOW ||
                        drive_mode == DRIVE_BACKWARD_ENC_TO_WALL) {
                    // flip heading error for backward drive
                    h_err *= -1;
                }

                adjustedHeadingError_ = h_err;

                headingCorrection_ = h_err * AUTO_CORRECT_HEADING_GAIN;
                if (MAX_HEADING_CORRECTION > 0.0) {             // clip the correction to ensure that motor is not reversed to avoid big swing
                    headingCorrection_ = Range.clip(headingCorrection_, -MAX_HEADING_CORRECTION, MAX_HEADING_CORRECTION);
                }

                power_lf *= (1 + headingCorrection_);  // h_err>0, bias to left, turn right by increasing power for left wheels
                power_rf *= (1 - headingCorrection_);
            }

            power_lb = power_lf;
            power_rb = power_rf;
        }

        /// Set power values for all wheel motors
        if (drivePowerFactor_ != 1.0) {
            power_lf *= drivePowerFactor_;
            power_rf *= drivePowerFactor_;
            power_lb *= drivePowerFactor_;
            power_rb *= drivePowerFactor_;
        }

        power_lf = Range.clip(power_lf, -1, 1);
        power_rf = Range.clip(power_rf, -1, 1);
        power_lb = Range.clip(power_lb, -1, 1);
        power_rb = Range.clip(power_rb, -1, 1);

        motorLF_.setPower(power_lf);
        motorRF_.setPower(power_rf);
        motorLB_.setPower(power_lb);
        motorRB_.setPower(power_rb);

        boolean show_msg_flag = false;
        boolean debug_drive_motor_flag = true;

        if (show_msg_flag) {
            telemetry.addData("Trip", String.valueOf(tripName_)+", TimeLeft:"+String.format("%.2f",AUTO_RUN_TIME-time_t));
            telemetry.addData("CurrState ID", String.valueOf(currStateId_) + " mode=%d", drive_mode);
            telemetry.addData("Heading", String.format("Start=%.2f/Curr=%.2f/Target=%.2f; Error=%.2f/AHE=%.2f/Corr=%.2f; DriveFactor=%.2f", currStateStartHeading_, heading_, targetHeading_, headingError_, adjustedHeadingError_, headingCorrection_, drivePowerFactor_));

            if (debug_drive_motor_flag && USE_ENC_FOR_DRIVE) {
                int lf_enc = motorLF_.getCurrentPosition();
                int lr_enc = motorLB_.getCurrentPosition();
                int rf_enc = motorRF_.getCurrentPosition();
                int rr_enc = motorRB_.getCurrentPosition();
                telemetry.addData("Current EncPos", "Expect=" + String.valueOf(currStateEncCnt_)+ ", LF="+String.valueOf(lf_enc)+", LR="+String.valueOf(lr_enc)+"; RF="+String.valueOf(rf_enc)+", RR="+String.valueOf(rr_enc));
            }

            telemetry.update();
        }
    }

    /// Return degree which robot has turned. Must be very careful with the boundary and the range of gyro reading.
    //  Assume: heading_ and curr_state_start_h_ are valid.
    //  For MR I2C Gyro, heading range (-inf, int). No special boundary handling is needed.
    double getDegreeTurned() {
        return Math.abs(heading_ - currStateStartHeading_);
    }

    /// Return heading error and update heading_ & heading_error_
    double getHeadingError() {
        headingError_ = 0;
        if (AUTO_CORRECT_HEADING) {
            headingError_ = getHeading() - targetHeading_;
            if (headingError_ > (360 - AUTO_CORRECT_MAX_HEADING_ERROR) &&
                headingError_ < (360 + AUTO_CORRECT_MAX_HEADING_ERROR)) {
                headingError_ = headingError_ - 360;
            } else if (headingError_ > (-360 - AUTO_CORRECT_MAX_HEADING_ERROR) &&
                       headingError_ < (-360 + AUTO_CORRECT_MAX_HEADING_ERROR)) {
                headingError_ = headingError_ + 360;
            }
        }
        return headingError_;
    }

    // Determine the power for left front wheel based on the driving mode
    private double getLeftFrontPower(int mode) {
        switch (mode) {
            case DRIVE_STOP:
                return 0.0;
            case DRIVE_FORWARD:
                return DRIVE_WHEEL_POWER;
            case DRIVE_BACKWARD:
                return (-1.0 * DRIVE_WHEEL_POWER);
            case DRIVE_TURN_LEFT:
            case DRIVE_TURN_TO_LEFT:
                if (turnSlow_) return (-1.0 * TURN_SLOW_WHEEL_POWER);
                return (-1.0 * TURN_WHEEL_POWER);
            case DRIVE_TURN_RIGHT:
            case DRIVE_TURN_TO_RIGHT:
                if (turnSlow_) return TURN_SLOW_WHEEL_POWER;
                return TURN_WHEEL_POWER;
            case DRIVE_RESET_ENC_DONE:
                return 0.0;
            case DRIVE_FORWARD_ENC:
                return DRIVE_ENC_WHEEL_POWER;
            case DRIVE_FORWARD_ENC_SLOW:
            case DRIVE_FORWARD_ENC_TO_WALL:
                return DRIVE_ENC_SLOW_WHEEL_POWER;
            case DRIVE_BACKWARD_ENC:
                return (-1.0 * DRIVE_ENC_WHEEL_POWER);
            case DRIVE_BACKWARD_ENC_SLOW:
            case DRIVE_BACKWARD_ENC_TO_WALL:
                return (-1.0 * DRIVE_ENC_SLOW_WHEEL_POWER);
            case DRIVE_TURN_LEFT_ENC:
                return (-1.0 * DRIVE_ENC_TURN_WHEEL_POWER);
            case DRIVE_TURN_RIGHT_ENC:
                return DRIVE_ENC_TURN_WHEEL_POWER;
            case DRIVE_CHECK_DISTANCE:
                return 0.0;
            default:
                break;
        }

        return 0.0;
    }

    // Determine the power for right front wheel based on the driving mode
    double getRightFrontPower(int mode) {
        switch (mode) {
            case DRIVE_STOP:
                return 0.0;
            case DRIVE_FORWARD:
                return (-1.0 * DRIVE_WHEEL_POWER);
            case DRIVE_BACKWARD:
                return DRIVE_WHEEL_POWER;
            case DRIVE_TURN_LEFT:
            case DRIVE_TURN_TO_LEFT:
                if (turnSlow_) return (-1.0 * TURN_SLOW_WHEEL_POWER);
                return TURN_WHEEL_POWER;
            case DRIVE_TURN_RIGHT:
            case DRIVE_TURN_TO_RIGHT:
                if (turnSlow_) return TURN_SLOW_WHEEL_POWER;
                return TURN_WHEEL_POWER;
            case DRIVE_RESET_ENC_DONE:
                return 0.0;
            case DRIVE_FORWARD_ENC:
                return (-1.0 * DRIVE_ENC_WHEEL_POWER);
            case DRIVE_FORWARD_ENC_SLOW:
            case DRIVE_FORWARD_ENC_TO_WALL:
                return (-1.0 * DRIVE_ENC_SLOW_WHEEL_POWER);
            case DRIVE_BACKWARD_ENC:
                return DRIVE_ENC_WHEEL_POWER;
            case DRIVE_BACKWARD_ENC_SLOW:
            case DRIVE_BACKWARD_ENC_TO_WALL:
                return DRIVE_ENC_SLOW_WHEEL_POWER;
            case DRIVE_TURN_LEFT_ENC:
                return (-1.0 * DRIVE_ENC_TURN_WHEEL_POWER);
            case DRIVE_TURN_RIGHT_ENC:
                return DRIVE_ENC_TURN_WHEEL_POWER;
            case DRIVE_CHECK_DISTANCE:
                return 0.0;
            default:
                break;
        }

        return 0.0;
    }

    /// Go to next state, and update starting conditions
    int gotoNextState(double [] states,          // states
                      double time,               // current time
                      boolean reset_encoders) {  // reset encoders if true
        int num_states = states.length / 2;
        currStateStartTime_ = time;

        currStateStartHeading_ = heading_;
        currStateEncCnt_ = 0.0;

        ++currStateId_;

        int mode = DRIVE_STOP;

        if (currStateId_ >= num_states) {
            currStateId_ = num_states;
        } else {
            if (currStateId_ < 0 ) currStateId_ = 0;

            mode = (int)states[(currStateId_ * 2) + 1];

            if (AUTO_CORRECT_HEADING) {
                double degree = states[currStateId_ * 2];
                switch (mode) {
                    case DRIVE_TURN_TO_LEFT:
                    case DRIVE_TURN_TO_RIGHT:
                        targetHeading_ = degree;      // a specific heading
                        break;
                    case DRIVE_TURN_LEFT_ENC:
                        targetHeading_ += degree;     // left, heading increase
                        break;
                    case DRIVE_TURN_RIGHT_ENC:
                        targetHeading_ -= degree;     // turn right, heading decrease
                        break;
                    default:
                        break;
                }
            }

            if (mode==DRIVE_CHECK_DISTANCE) { // reset counters
                numDistOk_ = 0;
                numDistFar_ = 0;
            }
        }

        if (reset_encoders && USE_ENC_FOR_DRIVE) {  // reset encoders for other states
            resetDriveEncoders();
        }

        return mode;
    }

    /// Generalized function to determine the robot drive mode based on time
    int getDriveMode(double [] states,
                     double time) {
        int num_states = states.length / 2;

        if (currStateId_ >= num_states) {
            if (USE_ENC_FOR_DRIVE) {
                runUsingEncoders() ;   // use encoders for time based drive. Required for SDK 2.2.
            }

            return DRIVE_STOP;
        }

        if (currStateId_ < 0 ) {
            currStateId_ = -1;

            if (USE_ENC_FOR_DRIVE) {
                runUsingEncoders() ;   // use encoders for time based drive. Required for SDK 2.2.
            }

            if (time < initWaitTime_) return DRIVE_STOP;      // delayed start

            // Go to first state
            return gotoNextState(states, time, true);    // go to first state, currStateId_: -1 => 0
        }

        int mode = (int)(states[(currStateId_ * 2) + 1]);
        switch (mode) {
            case DRIVE_TURN_TO_LEFT:
            case DRIVE_TURN_TO_RIGHT:
                return getDriveModeWhenAtDriveTurn(mode, states, time);
            case DRIVE_WAIT_TILL:
                return getDriveModeWhenAtDriveWaitTill(states, time);
            case DRIVE_CHECK_DISTANCE:
                return getDriveModeWhenAtDriveCheckDistance(states, time);
            case DRIVE_STATE_JUMP:
                return getDriveModeWhenAtDriverStateJump(states, time);
            case DRIVE_SHIFT_GEAR:
                return getDriveModeWhenAtDriveShiftGear(states, time);
            case DRIVE_LANDING:
                return getDriveModeWhenAtDriveLandingMode(states, time);
            case DRIVE_PULL_PIN:
                return getDriveModeWhenAtDrivePullPin(states, time);
            case DRIVE_DROP_MARKER:
                return getDriveModeWhenAtDriveDropMarker(states, time);
            case DRIVE_MINERAL_DETECTION:
                return getDriveModeWhenAtDriveMineralDetection(states);
            default:
                if (USE_ENC_FOR_DRIVE) {
                   switch (mode) {
                       case DRIVE_RESET_ENC_DONE:
                       case DRIVE_TURN_LEFT_ENC:
                       case DRIVE_TURN_RIGHT_ENC:
                       case DRIVE_FORWARD_ENC:
                       case DRIVE_BACKWARD_ENC:
                       case DRIVE_FORWARD_ENC_SLOW:
                       case DRIVE_BACKWARD_ENC_SLOW:
                       case DRIVE_FORWARD_ENC_TO_WALL:
                       case DRIVE_BACKWARD_ENC_TO_WALL:
                          return getDriveModeWhenAtDriveUsingEncoder(mode, states, time);
                       default:
                          break;
                   }
                }

                // Time based drive modes: STOP/FORWARD/BACKWARD/TURN_LEFT/TURN_RIGHT
                return getDriveModeWhenAtTimingBasedMode(mode, states, time);
        }

        // return DRIVE_STOP;
    }

    int getDriveModeWhenAtDriveUsingEncoder(int curr_mode,
                                            double [] states,
                                            double time) {
        // Use encoder to control driving
        if (curr_mode == DRIVE_RESET_ENC_DONE) {   // wait till encoder is reset
            if (haveDriveEncodersReset()) {        // reset done, go to next state
                return gotoNextState(states, time, false);
            }

	        // stay in RESET_ENC state, wait for reseting to complete
            if (AUTO_RESET_ENC_TIME > 0 &&
                time < (currStateStartTime_ + AUTO_RESET_ENC_TIME)) {  // auto-reset encoders
                resetDriveEncoders();
            }

            return  DRIVE_RESET_ENC_DONE;
        }

        // driving using encoders
        double tg_enc_cnt = Math.abs(states[currStateId_ * 2]);
        currStateEncCnt_ = tg_enc_cnt;

        if (tg_enc_cnt == 0.0) {
            return gotoNextState(states, time, /*reset_encoders*/false);
        }

        switch (curr_mode) {
            case DRIVE_BACKWARD_ENC:
            case DRIVE_FORWARD_ENC:
	        case DRIVE_BACKWARD_ENC_SLOW:
	        case DRIVE_FORWARD_ENC_SLOW:
	        case DRIVE_BACKWARD_ENC_TO_WALL:
            case DRIVE_FORWARD_ENC_TO_WALL:
                if (tg_enc_cnt < 10.0) {   // treat it as meters, convert it to encoder count
                    tg_enc_cnt = tg_enc_cnt * ENC_DIST_SCALE;
                }
                break;
	        case DRIVE_TURN_LEFT_ENC:
	        case DRIVE_TURN_RIGHT_ENC:
                if (tg_enc_cnt < 360.0) {   // treat it as degrees
                    tg_enc_cnt = tg_enc_cnt * ENC_DEG_SCALE ;
                }
                break;
	        default:
                break;
        }

        runUsingEncoders();

        if (curr_mode == DRIVE_BACKWARD_ENC_TO_WALL || curr_mode == DRIVE_FORWARD_ENC_TO_WALL) {
            if (mrRange_ != null) {
                double dis = mrRange_.getDistance(DistanceUnit.CM);
                if (dis > 0.0) {
                    if (dis > SEE_WALL_DISTANCE) {
                        return gotoNextState(states, time, true);
                    }
                }
            }
        }

        if (haveDriveEncodersReached(tg_enc_cnt, tg_enc_cnt)) {  // reset encoders and go to next state
            return gotoNextState(states, time, true);
        }

        return curr_mode;
    }

    int getDriveModeWhenAtDriveTurn(int curr_mode,
                                    double [] states,
                                    double time) {

        if (USE_ENC_FOR_DRIVE) {  // disable encoders for gryo based turning
            runUsingEncoders() ;  // use encoders for time based drive. Required for SDK 2.2.
        }

        double cur_heading = getHeading();

        if (states[currStateId_ * 2] < 0.0 ) {   // set lastTurnToHeading_ to current heading
            lastTurnToHeading_ = cur_heading;
            return gotoNextState(states, time, true);
        }

        double tg_deg = states[currStateId_ * 2];
        if (tg_deg != lastTurnToHeading_) lastTurnToHeading_ = tg_deg ;

        double to_turn_deg = Math.abs( heading_-tg_deg );
        if ((curr_mode == DRIVE_TURN_TO_LEFT && heading_ >= tg_deg) ||   // MR Gryo, turn left, heading increase
            (curr_mode == DRIVE_TURN_TO_RIGHT && heading_ <= tg_deg)) {  // MR Gyro, turn right, heading decrease
            return gotoNextState(states, time, true);
        }

        if (SLOW_TURN_DEGREE > 0.0 && to_turn_deg < SLOW_TURN_DEGREE) {
            turnSlow_ = true;
        } else {
            turnSlow_ = false;
        }

        return curr_mode;
    }

    int getDriveModeWhenAtDriveWaitTill(double [] states,
                                        double time) {
        if (USE_ENC_FOR_DRIVE) {
            runUsingEncoders() ;   // use encoders for time based drive. Required for SDK 2.2.
        }

        double t = states[currStateId_ * 2] ;   // wait till this time
        if (t < 0.0) t = AUTO_RUN_TIME+t;       // negative means minus N seconds

        if (time < t) {
           return DRIVE_WAIT_TILL;
        }

        return gotoNextState(states, time, true);
    }

    int getDriveModeWhenAtDriveCheckDistance(double [] states,
                                             double time) {
        // use range sensor for distance checking if available
        double dist = mrRange_.getDistance(DistanceUnit.CM);
        if (dist > MAX_DISTANCE_RANGE) {
           // invalid reading, ignored
        } else if (dist > MAX_WALL_DISTANCE) {
           numDistFar_ ++;
        } else {
           numDistOk_ ++;
        }

        double period = Math.abs( states[currStateId_ * 2] );   // time limit
        if (time < (currStateStartTime_+period)) {  // timeout
           return DRIVE_CHECK_DISTANCE;
        }

        // if (numDistOk_ >= numDistFar_) { // distance is OK, skip re-alignment
        //   currStateId_ += 8;
        // }

        return gotoNextState(states, time, true);
    }

    int getDriveModeWhenAtDriverStateJump(double [] states,
                                          double time) {
        int jump = (int) ( states[currStateId_ * 2 ] );   // jump
        currStateId_ += jump;
        return gotoNextState(states, time, true);
    }

    int getDriveModeWhenAtDriveShiftGear(double [] states,
                                         double time) {
        double f = states[currStateId_ * 2];
        if (f > 0.33 && f <= 3.0) drivePowerFactor_ = f;
        return gotoNextState(states, time, true);
    }

    int getDriveModeWhenAtDriveLandingMode(double [] states,
                                           double time) {
        if (motorLift_ != null) {
            double lift_enc = motorLift_.getCurrentPosition();
            boolean finish_landing_flag = (Math.abs(lift_enc) >= LIFT_LANDING_ENC_CNT);

            double lift_power = 0.0;
            if (finish_landing_flag == false) lift_power = LIFT_UP_POWER;
            lift_power = Range.clip(lift_power, -1, 1);
            motorLift_.setPower(lift_power);

            if (finish_landing_flag == false) return DRIVE_LANDING;
        }

        return gotoNextState(states, time, true);
    }

    int getDriveModeWhenAtDrivePullPin(double [] states,
                                       double time) {
        //use servo to pull pin out of hanger
        double period = Math.abs( states[currStateId_ * 2] );
        if ((time - currStateStartTime_) <= period){
            servoLiftPin_.setPosition(LIFT_PIN_PULL);
            return DRIVE_PULL_PIN;
        }

        return gotoNextState(states, time, true);
    }

    int getDriveModeWhenAtDriveDropMarker(double [] states,
                                          double time) {
        double period = Math.abs(states[currStateId_ * 2]);
        if ((time - currStateStartTime_) <= period) {
            servoMarker_.setPosition(MARKER_DROP_POS_);
            return DRIVE_DROP_MARKER;
        }

        return gotoNextState(states, time, true);
    }

    int getDriveModeWhenAtDriveMineralDetection(double [] states) {
        double period = Math.abs(states[currStateId_ * 2]);
        detectGoldMineralPosition(currStateStartTime_, period);

        return gotoNextState(states, time, true);
    }

    int getDriveModeWhenAtTimingBasedMode(int curr_mode,
                                          double [] states,
                                          double time) {
        if (USE_ENC_FOR_DRIVE) {
            runUsingEncoders() ;   // use encoders for time based drive. Required for SDK 2.2.
        }

        double period = Math.abs(states[currStateId_ * 2]);   // time limit

        if (time < (currStateStartTime_+period)) return curr_mode;

        return gotoNextState(states, time, true);
    }

    /// Define red trip
    int getDriveModeRed(double t) {
        double [] CraterTrip = {
                0.1, DRIVE_STOP,
                // 2.5, DRIVE_MINERAL_DETECTION,
                1.0, DRIVE_LANDING,
                1.7, DRIVE_PULL_PIN,
                0.1, DRIVE_RESET_ENC_DONE,
                1.0, DRIVE_FORWARD_ENC,
                0.1, DRIVE_RESET_ENC_DONE,
                0.4, DRIVE_BACKWARD_ENC,
                0.1, DRIVE_RESET_ENC_DONE,              //added from here...
                80, DRIVE_TURN_LEFT_ENC,
                0.1, DRIVE_RESET_ENC_DONE,
                1.4, DRIVE_FORWARD_ENC,
                0.1, DRIVE_RESET_ENC_DONE,
                45, DRIVE_TURN_LEFT_ENC,
                0.1, DRIVE_RESET_ENC_DONE,
                1.0, DRIVE_FORWARD_ENC,
                2.0, DRIVE_DROP_MARKER,
                0.1, DRIVE_RESET_ENC_DONE,
                2.5, DRIVE_BACKWARD_ENC,
                0.1, DRIVE_RESET_ENC_DONE,              // ... to here by Aditi Dec 9th, 2018                60.0, DRIVE_STOP
        };

        double [] DepotTripCenter = {
                0.1, DRIVE_STOP,
                // 2.5, DRIVE_MINERAL_DETECTION,
              // 1.0, DRIVE_LANDING,                          todo : need to un-comment
              //1.5, DRIVE_PULL_PIN,                         todo : landing and pull pin
                0.1, DRIVE_RESET_ENC_DONE,

                1.7, DRIVE_FORWARD_ENC,
                1.0, DRIVE_DROP_MARKER,
                0.1, DRIVE_RESET_ENC_DONE,                                 // added by Aditi Dec 9th, 2018
                // 0.05, DRIVE_BACKWARD_ENC,
                45, DRIVE_TURN_RIGHT_ENC,

                1.5, DRIVE_SHIFT_GEAR,
                0.1, DRIVE_RESET_ENC_DONE,
                2.3, DRIVE_BACKWARD_ENC,

                60.0, DRIVE_STOP
        };
        double [] DepotTripLeft = {        //left
                0.1, DRIVE_STOP,
                // 2.5, DRIVE_MINERAL_DETECTION,
                // 1.0, DRIVE_LANDING,                          todo : need to un-comment
                //1.5, DRIVE_PULL_PIN,                         todo : landing and pull pin
                0.1, DRIVE_RESET_ENC_DONE,

                0.2, DRIVE_FORWARD_ENC,
                0.1, DRIVE_RESET_ENC_DONE,

                40, DRIVE_TURN_LEFT_ENC,
                0.1, DRIVE_RESET_ENC_DONE,
                0.7, DRIVE_FORWARD_ENC,
                0.1, DRIVE_RESET_ENC_DONE,
                53, DRIVE_TURN_RIGHT_ENC,
                0.1, DRIVE_RESET_ENC_DONE,
                0.6, DRIVE_FORWARD_ENC,
                0.1, DRIVE_RESET_ENC_DONE,
                0.1, DRIVE_RESET_ENC_DONE,
                // 0.05, DRIVE_BACKWARD_ENC,
                32, DRIVE_TURN_RIGHT_ENC,
                1.0, DRIVE_DROP_MARKER,

                1.5, DRIVE_SHIFT_GEAR,                     //speeds up by 1.5x reg.
                0.1, DRIVE_RESET_ENC_DONE,
                1.75, DRIVE_BACKWARD_ENC,

                60.0, DRIVE_STOP
        };
        double [] DepotTripRight = {
                0.1, DRIVE_STOP,
                // 2.5, DRIVE_MINERAL_DETECTION,
                // 1.0, DRIVE_LANDING,                          todo : need to un-comment
                //1.5, DRIVE_PULL_PIN,                         todo : landing and pull pin
                0.1, DRIVE_RESET_ENC_DONE,

                1.7, DRIVE_FORWARD_ENC,
                1.0, DRIVE_DROP_MARKER,
                0.1, DRIVE_RESET_ENC_DONE,
                // 0.05, DRIVE_BACKWARD_ENC,
                45, DRIVE_TURN_RIGHT_ENC,
                0.1, DRIVE_RESET_ENC_DONE,
                2.3, DRIVE_BACKWARD_ENC,

                60.0, DRIVE_STOP
        };
        double [] DepotTrip = {
                0.1, DRIVE_STOP,
                // 2.5, DRIVE_MINERAL_DETECTION,
                // 1.0, DRIVE_LANDING,                          todo : need to un-comment
                //1.5, DRIVE_PULL_PIN,                         todo : landing and pull pin
                0.1, DRIVE_RESET_ENC_DONE,

                1.7, DRIVE_FORWARD_ENC,
                1.0, DRIVE_DROP_MARKER,
                0.1, DRIVE_RESET_ENC_DONE,
                // 0.05, DRIVE_BACKWARD_ENC,
                45, DRIVE_TURN_RIGHT_ENC,
                0.1, DRIVE_RESET_ENC_DONE,
                2.3, DRIVE_BACKWARD_ENC,

                60.0, DRIVE_STOP
        };

        /// Run red trips
        if (tripName_ == "DepotTrip") {
            return getDriveMode(DepotTrip, t);
        } else {
            return getDriveMode(CraterTrip, t);
        }
    }

    /// Define blue trip
    // int getDriveModeBlue(double t) {
    // }

    /// Access the left encoder's count.
    int getLFEncoderPos() {
        if (motorLF_ != null)  return motorLF_.getCurrentPosition();
        return 0;
    }

    int getLBEncoderPos() {
        if (motorLB_ != null) return motorLB_.getCurrentPosition();
        return 0;
    }

    /// Access the right encoder's count.
    int getRFEncoderPos() {
        if (motorRF_ != null) return motorRF_.getCurrentPosition();
        return 0;
    }

    int getRBEncoderPos() {
        if (motorRB_ != null) return motorRB_.getCurrentPosition();
        return 0;
    }

    /// Indicate whether the left drive encoder has been completely reset.
    boolean isleftEncodersReset() {
        if (getLFEncoderPos()==0 && getLBEncoderPos()==0) return true;
        return false;
    }

    /// Indicate whether the left drive encoder has been completely reset.
    boolean isRightEncodersReset() {
        if (getRFEncoderPos()==0 && getRBEncoderPos()==0) return true;
        return false;
    }

    /// Indicate whether the encoders have been completely reset.
    boolean haveDriveEncodersReset() {
        if (isleftEncodersReset() && isRightEncodersReset()) return true;
        return false;
    }

    /// Indicate whether the left front or left rear drive motor's encoder has reached a value.
    boolean isLeftEncodersReached(double p_count) {
        boolean l_return = false;
        if (motorLF_ != null) {
            if (Math.abs(motorLF_.getCurrentPosition ()) >= p_count) return true;
        }

        if (motorLB_ != null) {
            if (Math.abs(motorLB_.getCurrentPosition() ) >= p_count ) return true;
        }

        return false;
    }

    /// Indicate whether the right front or right rear drive motor's encoder has reached a value.
    boolean isRightEncodersReached (double p_count) {
        if (motorRF_ != null) {
            if (Math.abs(motorRF_.getCurrentPosition ()) >= p_count) return true;
        }

        if (motorRB_ != null) {
            if (Math.abs(motorRB_.getCurrentPosition() ) >= p_count) return true;
        }

        return false;
    }

    /// Indicate whether the drive motors' encoders have reached a value.
    boolean haveDriveEncodersReached(double p_left_count, double p_right_count) {
        if (isLeftEncodersReached(p_left_count) && isRightEncodersReached(p_right_count)) return true;
        return false;
    }

    /// Reset the left drive wheel encoder.
    void resetLeftEncoders() {
        if (motorLF_ != null) {
            motorLF_.setMode (DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        if (motorLB_ != null) {
            motorLB_.setMode (DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }

    /// Reset the right drive wheel encoder.
    void resetRightEncoders () {
        if (motorRF_ != null) {
            motorRF_.setMode ( DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        if (motorRB_ != null) {
            motorRB_.setMode ( DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }

    /// Reset both drive wheel encoders.
    void resetDriveEncoders() {
        resetLeftEncoders();
        resetRightEncoders();
    }

    /// Set the left drive wheel encoder to run, if the mode is appropriate.
    void useLeftEncoders () {
        if (motorLF_ != null) {
            motorLF_.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        if (motorLB_ != null) {
            motorLB_.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    /// Set the right drive wheel encoder to run, if the mode is appropriate.
    void useRightEncoders() {
        if (motorRF_ != null) {
            motorRF_.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        if (motorRB_ != null) {
            motorRB_.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    /// Set both drive wheel encoders to run, if the mode is appropriate.
    void runUsingEncoders() {
        useLeftEncoders ();
        useRightEncoders ();
    }

    void initializeVuforia() {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.FRONT;

        vuforia_ = ClassFactory.getInstance().createVuforia(parameters);
    }

    void initializeTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod_ = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia_);
        tfod_.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }

    void detectGoldMineralPosition(double start_time, double max_used_time) {
        int num_det_times = 0;
        int [] det_gold_cnt = {0, 0, 0};

        if (tfod_ != null) {
            telemetry.addData(">", "Detect mineral");
            telemetry.update();

            while (num_det_times < MAX_TIMES_DETECT_GOLD_MINERAL) {
                switch (detectGoldMineralsByTfod()) {
                    case GOLD_MINERAL_AT_LEFT:
                        ++det_gold_cnt[0];
                        ++num_det_times;
                        break;
                    case GOLD_MINERAL_AT_CENTER:
                        ++det_gold_cnt[1];
                        ++num_det_times;
                        break;
                    case GOLD_MINERAL_AT_RIGHT:
                        ++det_gold_cnt[2];
                        ++num_det_times;
                        break;
                    default:
                        break;
                }

                if ((timer_.time() - start_time) > max_used_time) break;
            }
        }

        if (det_gold_cnt[0] > det_gold_cnt[1]) {
            if (det_gold_cnt[0] > det_gold_cnt[2]) {
                goldPosition_ = GOLD_MINERAL_AT_LEFT;
            } else if (det_gold_cnt[2] > det_gold_cnt[0]) {
                goldPosition_ = GOLD_MINERAL_AT_RIGHT;
            } else {
                // We do not know where it is. Assume Left
                goldPosition_ = GOLD_MINERAL_AT_LEFT;
                isGuessedGoldPosition_ = true;
            }
        } else if (det_gold_cnt[0] == det_gold_cnt[1]) {
            if (det_gold_cnt[2] > det_gold_cnt[0]) {
                goldPosition_ = GOLD_MINERAL_AT_RIGHT;
            } else {
                // We do not know where it is. Assume Left.
                goldPosition_ = GOLD_MINERAL_AT_LEFT;
                isGuessedGoldPosition_ = true;
            }
        } else {
            if (det_gold_cnt[1] > det_gold_cnt[2]) {
                goldPosition_ = GOLD_MINERAL_AT_CENTER;
            } else if (det_gold_cnt[2] > det_gold_cnt[1]) {
                goldPosition_ = GOLD_MINERAL_AT_RIGHT;
            } else {
                // We do not know where it is. Assume center
                goldPosition_ = GOLD_MINERAL_AT_CENTER;
                isGuessedGoldPosition_ = true;
            }
        }

        telemetry.addData("Gold at", "%d (num_det_cnt=%d, is_guessed="+String.valueOf(isGuessedGoldPosition_)+", used_time=%.2f)",
                goldPosition_, num_det_times, (timer_.time() - currStateStartTime_));
        telemetry.update();
    }

    int detectGoldMineralsByTfod() {
        if (tfod_ == null) return GOLD_MINERAL_AT_UNKNOWN;

        List<Recognition> updatedRecognitions = tfod_.getUpdatedRecognitions();
        if (updatedRecognitions == null) return GOLD_MINERAL_AT_UNKNOWN;

        int num_detect_obj = updatedRecognitions.size();
        // telemetry.addData("# Object Detected", num_detect_obj);

        if (num_detect_obj >= 2) {
            int[] mineral_y = {-1, -1, -1};
            int[] mineral_x = {-1, -1, -1};
            int[] mineral_height = {-1, -1, -1};
            int[] mineral_width = {-1, -1, -1};
            boolean[] is_gold_mineral_flag = {false, false, false};
            int num_bottom_minerals = 0;
            for (Recognition recognition : updatedRecognitions) {
                int curr_x = (int) recognition.getLeft();
                int curr_y = (int) recognition.getBottom();
                int curr_height = (int) recognition.getHeight();
                int curr_width = (int) recognition.getWidth();
                boolean curr_is_gold_flag = false;
                if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) curr_is_gold_flag = true;

                if (num_bottom_minerals < 3) {
                    mineral_x[num_bottom_minerals] = curr_x;
                    mineral_y[num_bottom_minerals] = curr_y;
                    mineral_height[num_bottom_minerals] = curr_height;
                    mineral_width[num_bottom_minerals] = curr_width;
                    is_gold_mineral_flag[num_bottom_minerals] = curr_is_gold_flag;
                    ++num_bottom_minerals;
                } else {
                    int top_y = mineral_y[0];
                    int top_id = 0;
                    for (int i = 1; i < 3; ++i) {
                        if (mineral_y[i] < top_y) {
                            top_y = mineral_y[i];
                            top_id = i;
                        }
                    }

                    if (curr_y > top_y) {
                        mineral_x[top_id] = curr_x;
                        mineral_y[top_id] = curr_y;
                        mineral_height[top_id] = curr_height;
                        mineral_width[top_id] = curr_width;
                        is_gold_mineral_flag[top_id] = curr_is_gold_flag;
                    }
                }
            }

            // Sort mineral array in dcreasing order of Y
            for (int i = 0; i < num_bottom_minerals - 1; ++i) {
                for (int j = i; j < num_bottom_minerals - 1; ++j) {
                    if (mineral_y[j] < mineral_y[j + 1]) {
                        int tmp = mineral_x[j];
                        mineral_x[j] = mineral_x[j + 1];
                        mineral_x[j + 1] = tmp;
                        tmp = mineral_y[j];
                        mineral_y[j] = mineral_y[j + 1];
                        mineral_y[j + 1] = tmp;
                        tmp = mineral_height[j];
                        mineral_height[j] = mineral_height[j + 1];
                        mineral_height[j + 1] = tmp;
                        boolean tmp_bool = is_gold_mineral_flag[j];
                        is_gold_mineral_flag[j] = is_gold_mineral_flag[j + 1];
                        is_gold_mineral_flag[j + 1] = tmp_bool;
                    }
                }
            }

            if (is_gold_mineral_flag[0] == false || is_gold_mineral_flag[1] == false) {
                boolean has_two_mineral_in_bottem_flag = true;

                if (num_bottom_minerals == 3) {
                    if (mineral_height[2] >= mineral_width[2]) {
                        if (((double) mineral_width[2] / (double) mineral_height[2]) < MIN_MINERAL_HEIGHT_WIDTH_RATIO)
                            num_bottom_minerals = 2;
                    } else {
                        if (((double) mineral_height[2] / (double) mineral_width[2]) < MIN_MINERAL_HEIGHT_WIDTH_RATIO)
                            num_bottom_minerals = 2;
                    }

                    if (num_bottom_minerals == 3) {
                        int diff_y = mineral_y[1] - mineral_y[2];
                        if (diff_y >= mineral_height[1]) num_bottom_minerals = 2;
                        else if (((double) diff_y / (double) mineral_height[1]) > MAX_MINERAL_BOTTOM_DIFF_RATIO)
                            num_bottom_minerals = 2;

                        if (num_bottom_minerals == 3) {
                            if (mineral_height[2] > mineral_height[1]) {
                                if (((double) mineral_height[1] / (double) mineral_height[2]) >= MIN_MINERAL_HEIGHT_RATIO)
                                    has_two_mineral_in_bottem_flag = false;
                            } else {
                                if (((double) mineral_height[2] / (double) mineral_height[1]) >= MIN_MINERAL_HEIGHT_RATIO)
                                    has_two_mineral_in_bottem_flag = false;
                            }
                        }
                    }
                }

                if (has_two_mineral_in_bottem_flag == true) {
                    int diff_y = mineral_y[0] - mineral_y[1];
                    if (diff_y >= mineral_height[0]) has_two_mineral_in_bottem_flag = false;
                    else if (((double) diff_y / (double) mineral_height[0]) > MAX_MINERAL_BOTTOM_DIFF_RATIO)
                        has_two_mineral_in_bottem_flag = false;

                    /*
                    if (mineral_height[0] > mineral_height[1]) {
                        if (((double)mineral_height[1] / (double)mineral_height[0]) < MIN_MINERAL_HEIGHT_RATIO)
                            has_two_mineral_in_bottem_flag = false;
                    } else {
                        if (((double)mineral_height[0] / (double)mineral_height[1]) < MIN_MINERAL_HEIGHT_RATIO)
                            has_two_mineral_in_bottem_flag = false;
                    }
                    */

                    if (has_two_mineral_in_bottem_flag == true) {
                        if (is_gold_mineral_flag[0] == false && is_gold_mineral_flag[1] == false) {
                            // telemetry.addData("Gold Mineral Position", "Left (%d, %d, %d) (%d, %d, %d)",
                            //        mineral_x[0], mineral_y[0], mineral_height[0], mineral_x[1], mineral_y[1], mineral_height[1]);
                            return GOLD_MINERAL_AT_LEFT;
                        } else {
                            boolean is_in_center_flag = false;
                            if (is_gold_mineral_flag[0] == true) {
                                if (mineral_x[0] < mineral_x[1]) is_in_center_flag = true;
                            } else {
                                if (mineral_x[0] > mineral_x[1]) is_in_center_flag = true;
                            }

                            if (is_in_center_flag == true) {
                                // telemetry.addData("Gold Mineral Position", "Center (%d, %d, %d) (%d, %d, %d)",
                                //        mineral_x[0], mineral_y[0], mineral_height[0], mineral_x[1], mineral_y[1], mineral_height[1]);
                                return GOLD_MINERAL_AT_CENTER;
                            } else {
                                // telemetry.addData("Gold Mineral Position", "Right (%d, %d, %d) (%d, %d, %d)",
                                //        mineral_x[0], mineral_y[0], mineral_height[0], mineral_x[1], mineral_y[1], mineral_height[1]);
                                return GOLD_MINERAL_AT_RIGHT;
                            }
                        }
                    }
                }
            }
        }

        return GOLD_MINERAL_AT_UNKNOWN;
    }
}
