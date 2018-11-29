/*
 * This is Program for Autonomous Run for FTC 2017-2018 Relic Recovery season
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.lang.Math;
import java.util.List;
import java.util.Locale;


/**
 * Created by aliris on 10/13/2018.
 */

///  Autonomous Run for League Meet0
@Autonomous(name="Y18Auto", group="GG")
//@Disabled
public class Y18Auto extends Y18Common
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
    static final int DRIVE_SHIFT_GEAR = 19;                 // shift gears to make robot move faster/slower by changing drive_power_factor_

    static final int DRIVE_LANDING = 20;                    //for landing the robot

    static final int DRIVE_PULL_PIN = 21;

    static final int DRIVE_DROP_MARKER = 22;

    static final int DRIVE_MINERAL_DETECTION = 23;

    static final int DRIVE_MODE_NUM = 24;                   // total number of modes

//*****************************************************************************************************************

    /// General settings for AutoRun
    static final double  AUTO_RUN_TIME = 60.0;              // 60 sec for testing/debugging
    static final double  AUTO_RESET_ENC_TIME = 1.00;        // period for reseting encoders when entering DRIVE_RESET_ENC_DONE mode
    static final boolean START_AS_RED = true;               // true for RED team; false for BLUE team
    static final boolean FLIP_COLOR_BY_X1 = true;           // allow user to press Pad1/X to flip team color within 1sec if true for debugging
    static final boolean FLIP_ROBOT_FOR_BLUE = false;       // flip robot front/end for BLUE
    static final boolean USE_IMU_FOR_TURN = true;           // use IMU sensor to turn
    static final boolean USE_ENC_FOR_DRIVE = true;          // use encoder for accurate movement

    /// Power for drive and turn, need be tuned for each robot
    static final double DRIVE_ENC_WHEEL_POWER = 0.40;       // default driving power using encoder; ~29sec for 6*3m, for 2017
    static final double DRIVE_ENC_SLOW_WHEEL_POWER = 0.15;  // power for encoder based slow drive for STOP_WHITE
    static final double DRIVE_ENC_TURN_WHEEL_POWER = 0.12;  // default turning power using encoder, 2017/09/08
    static final double DRIVE_ENC_POWER_RATIO = DRIVE_ENC_SLOW_WHEEL_POWER/DRIVE_ENC_WHEEL_POWER ;  // ratio between low and high
    static final double DRIVE_WHEEL_POWER = 0.20;           // default driving power
    static final double TURN_WHEEL_POWER = 0.20;            // default turning power
    static final double TURN_SLOW_WHEEL_POWER = 0.15;       // slow turning power
    static final double SLOW_TURN_DEGREE = 10.0;            // degree to enable slow turn; 10.0 degree by dflt; 0, disabled

    final static double ENC_DIST_SCALE = 2000.0/1.00;       // 2000 ticks <=> 1.00 meters, 2017/09/06
    final static double ENC_DEG_SCALE = 2000.0/150;         // 2000 ticks <=> ~150 for MW6 on mat, 2017/10/15

    static final double  AUTO_CORRECT_HEADING_GAIN = 0.012;  // previously 0.025; power percentage to be adjusted for each degree of heading error; 1-degree error => 5% power diff // dflt for 0.40, 2017/09/08
    static final double  AUTO_CORRECT_MAX_HEADING_ERROR = 40;  // power percentage to be adjusted for each degree of heading error; 1-degree error => 5% power diff
    static final boolean AUTO_CORRECT_HEADING = true;       // automatically correct the last heading for DRIVE_FORWARD mode
    static final double  MAX_HEADING_CORRECTION = 0.95;     // max heading correction; 0, not used

    /// Autonomous specific hardware
    //2018-2019
    VuforiaLocalizer vuforia;
    static final String VUFORIA_KEY = "ATbVhA//////AAAAGQpUcoBny0Xdi+FFWntcC3w9C63+hv3ccdXKcUsUhNYtbt8IbpT9SQ+VsthWIyix0rrzYP8KYaSYY5na+nufoGmLQo8vE8CPWmUj8eZcdlM9k4mi8ge0T2uzuoKZmcllal8cM3hRxo1JBFVtavCrgulnZxQ8hMsbzZuA+dZDGQTOEOCCH8ZHuh6wrIUygVerHfrXXlpeIAQvXzBiYrVPetr3zu3ROn6rno75mQ0KCM8Qp87BGS4Orx+GwxL8FlO+EXA3aSBvDh7+a57co5212MkGIRUceXxAd+BfoFjiWg3SbJpVbDM7TcDApVR88jlqeEDmbc/ODajLjEKziycgihi1rpq1lOBys2oJ68qdVrtO";

    TFObjectDetector tfod;
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";


    /// Autonomous Specific Variables

    static final double  WALL_DISTANCE_MAX = 17.5;          // max distance to wall; distance higher than this will trigger re-alignment
    static final double  MAX_RANGE  = 99.9;                 // any distance larger than this value will be considered as invalid reading
    static final double  MIN_WALL_DIS_RED = 20.0;           // wall distance to stop for RED/ENC_TO_WALL
    static final double  MIN_WALL_DIS_BLUE = 25.0;          // wall distance to stop for BLUE/ENC_TO_WALL
    static final double  MAX_WALL_DIS_RED1 = 100.0;         // wall distance to stop for RED position1
    static final double  MAX_WALL_DIS_BLUE1 = 100.0;        // wall distance to stop for BLUE position1


    /// Variables for Autonomous Mode
    double init_wait_time_ = 0.01;                           // additional wait time; default 1 sec for debugging
    boolean dflt_start_pos_ = true;                         // default starting position if true
    boolean color_flipped_ = false;                         // false for RED team, true for BLUE team


    int curr_state_id_ = -1;                                // current state ID
    double curr_state_value_ = 0.0;                         // current state value for distance, timeout, etc.
    int curr_state_drive_mode_ = -1;                        // current state drive mode
    double curr_state_start_t_ = 0.0;                       // current state start time
    double curr_state_start_h_ = 0.0;                       // current state start heading
    double curr_state_enc_cnt_ = 0.0;                       // current state targeted encoder count

    double heading_ = 0.0;                                  // current heading
    double target_heading_ = 0.0;                           // target heading for current state
    double heading_error_ = 0.0;                            // current heading error
    double adjusted_heading_error_ = 0.0;                   // adjusted heading error
    double heading_correction_ = 0.0;                       // heading correction
    double last_turn_to_heading_ = -1.0;                    // last TURN_TO heading

    boolean turn_slow_ = false;                             // turn slow if true

    int num_dist_ok_ = 0;                                   // numer of times for which distance is OK
    int num_dist_far_ = 0;                                  // numer of times for which distance is too far

    String trip_name_ = "Default";                          // trip name

    int target_lift_enc_ = -1;                              // targeted lift encoder position
    double drive_power_factor_ = 1.0;                       // factor for drive power

    int lf_encoder_max_ = 0;                                // LF motor encoder max position for debugging drive motors
    int lr_encoder_max_ = 0;
    int rf_encoder_max_ = 0;
    int rr_encoder_max_ = 0;


    // Constructor
    public Y18Auto() {
    }

    /// Autonomous mode specific initialization
    @Override public void init() {
        super.init();

        // initVuforia();

        // if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
        //    initTfod();
        //} else {
        //   telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        //}

        if( USE_ENC_FOR_DRIVE ) {
            reset_drive_encoders();
        }
        target_lift_enc_ = -1;
        if( USE_LIFT ) { motorLift_.setMode ( DcMotor.RunMode.STOP_AND_RESET_ENCODER); }

        heading_ = 0.0;
        target_heading_ = 0.0;

        num_dist_ok_ = 0;
        num_dist_far_ = 0;

    }

    @Override
    public void start() {
    }

    @Override public void init_loop() {
        super.init_loop();

        curr_time_ = 0.0;

        if(USE_LIFT) motorLift_.setMode ( DcMotor.RunMode.RUN_USING_ENCODER );
    }

    @Override public void loop() {
        double time_t = timer_.time();
        curr_time_ = time_t;
        heading_error_ = 0.0;
        adjusted_heading_error_ = 0.0;
        heading_correction_ = 0.0;

        power_lift_ = 0.0;

        if( time_t<=1.0 ) {
            /// Allow user to flip team color for easy testing
            if( FLIP_COLOR_BY_X1 && gamepad1.x ) {
                color_flipped_ = true;
            }

            /// Choose different test trip for debugging, only for RED dflt starting position
            if( gamepad1.a && !gamepad1.b ) {
                trip_name_ = "TestEncDrive";  // encoder based driving including heading correction
            } else if( !gamepad1.a && gamepad1.b ) {
                trip_name_ = "TestSimpleTrip";   // simple test trip
            } else if( gamepad1.a && gamepad1.b ) {
                trip_name_ = "TestCalDist";    // calibrate distance
            }
        }

        int drive_mode = DRIVE_STOP;
        if( (START_AS_RED && !color_flipped_) || (!START_AS_RED && color_flipped_) ) {
            if( dflt_start_pos_ ) {
                drive_mode = getDriveModeRed(time_t);
            }
        } else {
            if( dflt_start_pos_ ) {
                drive_mode = getDriveModeBlue(time_t);
            }
        }

        double power_lf = 0, power_rf = 0, power_lb = 0, power_rb = 0;   // power for wheels


        if( time_t>=AUTO_RUN_TIME ) {
            // timeout, stop robot and sweeper
        } else {

            power_lf = getLeftFrontPower( drive_mode );
            power_rf = getRightFrontPower( drive_mode );



            double h_err = 0.0;    // heading error
            if( AUTO_CORRECT_HEADING &&
                    ( drive_mode==DRIVE_FORWARD_ENC_TO_WALL || drive_mode==DRIVE_BACKWARD_ENC_TO_WALL ||
                            drive_mode==DRIVE_FORWARD_ENC || drive_mode==DRIVE_BACKWARD_ENC ||
                            drive_mode==DRIVE_FORWARD_ENC_SLOW || drive_mode==DRIVE_BACKWARD_ENC_SLOW ) ) {
                // Get the heading error from IMU
                h_err = getHeadingError();  // expensive heading reading
                if( Math.abs(h_err)>AUTO_CORRECT_MAX_HEADING_ERROR ) {  // prevent incorrect heading error causing robot to spin
                    h_err=0.0;
                }

            }

            /// Adjust wheel power to correct heading or follow wall
            if( Math.abs(h_err)>0.0 ) {
                if( FLIP_ROBOT_FOR_BLUE && !isRedTeam() ) {
                    // left/right flipped when swapping the front/back
                    h_err *= -1;
                }
                if( drive_mode==DRIVE_BACKWARD_ENC ||
                        drive_mode==DRIVE_BACKWARD_ENC_SLOW ||
                        drive_mode==DRIVE_BACKWARD_ENC_TO_WALL ) {
                    // flip heading error for backward drive
                    h_err *= -1;
                }

                adjusted_heading_error_ = h_err;

                double corr = h_err*AUTO_CORRECT_HEADING_GAIN;
                if( MAX_HEADING_CORRECTION>0.0 ) {             // clip the correction to ensure that motor is not reversed to avoid big swing
                    corr = Range.clip(corr, -MAX_HEADING_CORRECTION, MAX_HEADING_CORRECTION);
                }
                heading_correction_ = corr;
                power_lf *= ( 1 + corr );  // h_err>0, bias to left, turn right by increasing power for left wheels
                power_rf *= ( 1 - corr );
            }


            if( FLIP_ROBOT_FOR_BLUE && !isRedTeam() ) {
                // flip power for all driving modes
                if( drive_mode==DRIVE_FORWARD || drive_mode==DRIVE_BACKWARD ||
                        drive_mode==DRIVE_FORWARD_ENC || drive_mode==DRIVE_BACKWARD_ENC ||
                        drive_mode==DRIVE_FORWARD_ENC_SLOW || drive_mode==DRIVE_BACKWARD_ENC_SLOW ||
                        drive_mode==DRIVE_FORWARD_ENC_TO_WALL|| drive_mode==DRIVE_BACKWARD_ENC_TO_WALL )
                {
                    power_lf *= -1;
                    power_rf *= -1;
                }
            }

            power_lb = power_lf;
            power_rb = power_rf;
        }

        /// Set power values for all wheel motors
        if( drive_power_factor_!=1.0 ) {
            power_lf *= drive_power_factor_;
            power_rf *= drive_power_factor_;
            power_lb *= drive_power_factor_;
            power_rb *= drive_power_factor_;
        }

        power_lf = Range.clip(power_lf, -1, 1);
        power_rf = Range.clip(power_rf, -1, 1);
        power_lb = Range.clip(power_lb, -1, 1);
        power_rb = Range.clip(power_rb, -1, 1);

        motorLF_.setPower(power_lf);
        motorRF_.setPower(power_rf);
        motorLB_.setPower(power_lb);
        motorRB_.setPower(power_rb);


        ///TODO: Servo ranges???????

        if( USE_LIFT ) {
            /*
            int max_lift_enc_diff = 50;
            if( target_lift_enc_>=0 ) {
                int lift_enc = motorLift_.getCurrentPosition();
                int lift_enc_diff = lift_enc-target_lift_enc_;
                if( lift_enc_diff<0 ) { // go up
                    if( Math.abs(lift_enc_diff)<max_lift_enc_diff ) {
                        power_lift_ = MIN_LIFT_HOLD_POWER;
                    } else {
                        power_lift_ = LIFT_UP_POWER;
                    }
                } else if( lift_enc_diff>0 ) {  // go down
                    if( Math.abs(lift_enc_diff)<max_lift_enc_diff ) {
                        power_lift_ = 0;  // let it drip
                    } else {
                        power_lift_ = LIFT_DOWN_POWER;
                    }
                }
            }
            */
            if(drive_mode == DRIVE_LANDING) {
                power_lift_ = LIFT_UP_POWER;
            } else {
                power_lift_ = 0.0;
            }
            power_lift_ = Range.clip(power_lift_, -1, 1);
            motorLift_.setPower( power_lift_ );
        }


        /// Set power values for all servos




        /// Print messages
        boolean no_msg = false;              // set true to disable all messages to minimize lagging
        boolean debug_enc_drive = false;     // wheel motor power and encoder position
        boolean debug_enc_lift = false;     // lift motor encoder position
        boolean debug_imu = false;          // expensive, big over-turning
        boolean debug_drive_motors = false;  // print out motor encoder position
        boolean show_mr_range = false;

        if( !no_msg ) {
            telemetry.addData("GiftGears", "Trip:"+String.valueOf(trip_name_)+"Team:"+(isRedTeam()?"RED":"BLUE")+", DfltStart:"+String.valueOf(dflt_start_pos_)+", TimeLeft:"+String.format("%.2f",AUTO_RUN_TIME-time_t));
            telemetry.addData("CurrState", "Id:" + String.valueOf(curr_state_id_)+", Mode:"+String.valueOf(curr_state_drive_mode_)+", StartTime:"+String.format("%.2f", curr_state_start_t_)+", Heading:"+ String.format("Start=%.2f/Curr=%.2f/Target=%.2f; Error=%.2f/AHE=%.2f/Corr=%.2f; DriveFactor=%.2f",curr_state_start_h_,heading_,target_heading_,heading_error_,adjusted_heading_error_,heading_correction_,drive_power_factor_));

            if(debug_enc_drive) {
                telemetry.addData("Wheel Power",  ": LF="+String.valueOf(motorLF_.getPower())+", RF="+String.valueOf(motorRF_.getPower())+", LB="+String.valueOf(motorLB_.getPower())+",RB="+String.valueOf(motorRB_.getPower()));
                if(USE_ENC_FOR_DRIVE) telemetry.addData("EncPos",  ": LF="+String.valueOf(motorLF_.getCurrentPosition())+", RF="+String.valueOf(motorRF_.getCurrentPosition())+", LB="+String.valueOf(motorLB_.getCurrentPosition())+",RB="+String.valueOf(motorRB_.getCurrentPosition()));
            }
            if(debug_enc_lift && USE_LIFT) {
                telemetry.addData("Lift",  ": power="+String.valueOf(motorLift_.getPower())+", pos="+String.valueOf(motorLift_.getCurrentPosition()));
            }

            if( show_mr_range && mr_range_!=null ) {
                telemetry.addData("MRRangeSensor", String.format("ultra/opt/dist=%4d/%.2f/%.2f",mr_range_.rawUltrasonic(),mr_range_.cmOptical(),mr_range_.getDistance(DistanceUnit.CM)));
            }

            if(debug_drive_motors && USE_ENC_FOR_DRIVE) {
                int lf_enc = motorLF_.getCurrentPosition();
                int lr_enc = motorLB_.getCurrentPosition();
                int rf_enc = motorRF_.getCurrentPosition();
                int rr_enc = motorRB_.getCurrentPosition();
                telemetry.addData("Current EncPos", ": LF="+String.valueOf(lf_enc)+", LR="+String.valueOf(lr_enc)+"; RF="+String.valueOf(rf_enc)+", RR="+String.valueOf(rr_enc));

                if(Math.abs(lf_enc)>Math.abs(lf_encoder_max_))  lf_encoder_max_=lf_enc;
                if(Math.abs(lr_enc)>Math.abs(lr_encoder_max_))  lr_encoder_max_=lr_enc;
                if(Math.abs(rf_enc)>Math.abs(rf_encoder_max_))  rf_encoder_max_=rf_enc;
                if(Math.abs(rr_enc)>Math.abs(rr_encoder_max_))  rr_encoder_max_=rr_enc;
                telemetry.addData("Max EncPos", ": LF="+String.valueOf(lf_encoder_max_)+", LR="+String.valueOf(lr_encoder_max_)+"; RF="+String.valueOf(rf_encoder_max_)+", RR="+String.valueOf(rr_encoder_max_));

                telemetry.addData("IMU", "heading: " + String.format("%.2f", getHeading()));
            }

            if(debug_imu) {
                if(USE_IMU_FOR_TURN && imu_!=null) telemetry.addData("IMU", "status="+String.valueOf(imu_.getSystemStatus())+", calib="+String.valueOf(imu_.getCalibrationStatus())+", heading="+formatAngle(imu_angles_.angleUnit,imu_angles_.firstAngle)+", roll="+formatAngle(imu_angles_.angleUnit,imu_angles_.secondAngle)+", pitch="+formatAngle(imu_angles_.angleUnit,imu_angles_.thirdAngle));
            }
        }
    }

    // Code to run when the op mode is first disabled goes here
    @Override public void stop() {
    }

    /// Return true if it's RED team
    boolean isRedTeam() {
        return ( START_AS_RED ? (!color_flipped_) : color_flipped_ );
    }

    /// Return current robot heading based on gyro/IMU reading


    /// Return degree which robot has turned. Must be very careful with the boundary and the range of gyro reading.
    //  Assume: heading_ and curr_state_start_h_ are valid.
    //  For MR I2C Gyro, heading range (-inf, int). No special boundary handling is needed.
    double getDegreeTurned() {
        return Math.abs(heading_ - curr_state_start_h_);
    }

    /// Return heading error and update heading_ & heading_error_
    double getHeadingError() {
        heading_error_ = 0;
        if( AUTO_CORRECT_HEADING ) {
            heading_error_ = getHeading()-target_heading_;
            if( heading_error_>(360-AUTO_CORRECT_MAX_HEADING_ERROR) && heading_error_<(360+AUTO_CORRECT_MAX_HEADING_ERROR) ) {
                heading_error_ = heading_error_ - 360;
            } else if( heading_error_>(-360-AUTO_CORRECT_MAX_HEADING_ERROR) && heading_error_<(-360+AUTO_CORRECT_MAX_HEADING_ERROR) ) {
                heading_error_ = heading_error_ + 360;
            }
        }
        return heading_error_;
    }

    /// Determine the power for left front wheel based on the driving mode
    double getLeftFrontPower(int mode) {
        double pwr = 0;
        switch( mode ) {
            case DRIVE_STOP:
                break;
            case DRIVE_FORWARD:
                pwr = DRIVE_WHEEL_POWER;
                break;
            case DRIVE_BACKWARD:
                pwr = -1.0*DRIVE_WHEEL_POWER;
                break;
            case DRIVE_TURN_LEFT:
            case DRIVE_TURN_TO_LEFT:
                if( turn_slow_ ) {
                    pwr = -1.0*TURN_SLOW_WHEEL_POWER;
                } else {
                    pwr = -1.0*TURN_WHEEL_POWER;
                }
                break;
            case DRIVE_TURN_RIGHT:
            case DRIVE_TURN_TO_RIGHT:
                if( turn_slow_ ) {
                    pwr = TURN_SLOW_WHEEL_POWER;
                } else {
                    pwr = TURN_WHEEL_POWER;
                }
                break;
            case DRIVE_RESET_ENC_DONE:
                pwr = 0.0;
                break;
            case DRIVE_FORWARD_ENC:
            case DRIVE_FORWARD_ENC_SLOW:
            case DRIVE_FORWARD_ENC_TO_WALL:
                pwr = DRIVE_ENC_SLOW_WHEEL_POWER;
                break;
            case DRIVE_BACKWARD_ENC:
                pwr = -1.0*DRIVE_ENC_WHEEL_POWER;
                break;
            case DRIVE_BACKWARD_ENC_SLOW:
            case DRIVE_BACKWARD_ENC_TO_WALL:
                pwr = -1.0*DRIVE_ENC_SLOW_WHEEL_POWER;
                break;
            case DRIVE_TURN_LEFT_ENC:
                pwr = -1.0*DRIVE_ENC_TURN_WHEEL_POWER;
                break;
            case DRIVE_TURN_RIGHT_ENC:
                pwr = DRIVE_ENC_TURN_WHEEL_POWER;
                break;
            case DRIVE_CHECK_DISTANCE:
                pwr = 0.0;
                break;
            default:
                pwr = 0.0;
                break;
        }

        return pwr;
    }

    /// Determine the power for right front wheel based on the driving mode
    double getRightFrontPower(int mode) {
        double pwr = 0;
        switch( mode ) {
            case DRIVE_STOP:
                break;
            case DRIVE_FORWARD:
                pwr = -1.0*DRIVE_WHEEL_POWER;
                break;
            case DRIVE_BACKWARD:
                pwr = DRIVE_WHEEL_POWER;
                break;
            case DRIVE_TURN_LEFT:
            case DRIVE_TURN_TO_LEFT:
                if( turn_slow_ ) {
                    pwr = -1.0*TURN_SLOW_WHEEL_POWER;
                } else {
                    pwr = TURN_WHEEL_POWER;
                }
                break;
            case DRIVE_TURN_RIGHT:
            case DRIVE_TURN_TO_RIGHT:
                if( turn_slow_ ) {
                    pwr = TURN_SLOW_WHEEL_POWER;
                } else {
                    pwr = TURN_WHEEL_POWER;
                }
                break;
            case DRIVE_RESET_ENC_DONE:
                pwr = 0.0;
                break;
            case DRIVE_FORWARD_ENC:
            case DRIVE_FORWARD_ENC_SLOW:
            case DRIVE_FORWARD_ENC_TO_WALL:
                pwr = -1.0*DRIVE_ENC_SLOW_WHEEL_POWER;
                break;
            case DRIVE_BACKWARD_ENC:
                pwr = DRIVE_ENC_WHEEL_POWER;
                break;
            case DRIVE_BACKWARD_ENC_SLOW:
            case DRIVE_BACKWARD_ENC_TO_WALL:
                pwr = DRIVE_ENC_SLOW_WHEEL_POWER;
                break;
            case DRIVE_TURN_LEFT_ENC:
                pwr = -1.0*DRIVE_ENC_TURN_WHEEL_POWER;
                break;
            case DRIVE_TURN_RIGHT_ENC:
                pwr = DRIVE_ENC_TURN_WHEEL_POWER;
                break;
            case DRIVE_CHECK_DISTANCE:
                pwr = 0.0;
                break;
            default:
                pwr = 0.0;
                break;
        }

        return pwr;
    }

    /// Generalized function to determine the robot drive mode based on time
    int getDriveMode(double[] states, double time) {
        int tot_state = states.length/2;
        return getDriveModeGyroEncTime(tot_state, states, time);
    }

    /// Go to next state, and update starting conditions
    void gotoNextState(
            int tot_state,            // total number of states
            double[] states,          // states
            double time,              // current time
            boolean reset_encoders)   // reset encoders if true
    {
        curr_state_id_ ++;
        curr_state_start_t_ = time;

        curr_state_start_h_ = heading_;
        curr_state_enc_cnt_ = 0.0;
        int mode = DRIVE_STOP;
        if( curr_state_id_<tot_state ) {
            mode = (int)( states[ curr_state_id_*2 +1 ] );
            curr_state_value_ = states[ curr_state_id_*2 ];
        }
        curr_state_drive_mode_ = mode;
        if( AUTO_CORRECT_HEADING && curr_state_id_<tot_state ) {
            double deg = states[ curr_state_id_*2 ];
            if( mode==DRIVE_TURN_TO_LEFT || mode==DRIVE_TURN_TO_RIGHT ) {
                target_heading_ = deg;      // a specific heading
            } else if( mode==DRIVE_TURN_LEFT_ENC ) {
                target_heading_ += deg;     // left, heading increase
            } else if( mode==DRIVE_TURN_RIGHT_ENC ) {
                target_heading_ -= deg;     // turn right, heading decrease
            }
        }

        if( mode==DRIVE_CHECK_DISTANCE ) { // reset counters
            num_dist_ok_ = 0;
            num_dist_far_ = 0;
        }

        if( reset_encoders && USE_ENC_FOR_DRIVE ) {  // reset encoders for other states
            reset_drive_encoders();
        }
    }

    /// Generalized function to determine the robot drive mode based on time
    int getDriveModeGyroEncTime(int num_state, double[] states, double time) {
        int mode = DRIVE_STOP;
        if( curr_state_id_ >= num_state ) {
            // Should not happen, DRIVE_STOP
            if( USE_ENC_FOR_DRIVE ) {
                run_using_encoders() ;   // use encoders for time based drive. Required for SDK 2.2.
            }
        } else if( curr_state_id_>=0 && curr_state_id_<num_state ) {
            int m = (int)(states[ curr_state_id_*2 +1 ]);

            if( m==DRIVE_WAIT_TILL ) {
                if( USE_ENC_FOR_DRIVE ) {
                    run_using_encoders() ;   // use encoders for time based drive. Required for SDK 2.2.
                }
                double t =  states[ curr_state_id_*2 ] ;   // wait till this time
                if( t<0.0 ) { t = AUTO_RUN_TIME+t; }       // negative means minus N seconds
                if( time>=t ) {  // time reached
                    gotoNextState( num_state, states, time, true );
                } else {   // keep waiting
                    mode = m;
                }
            } else if ( m==DRIVE_TURN_TO_LEFT || m==DRIVE_TURN_TO_RIGHT ) {
                if( USE_ENC_FOR_DRIVE ) {  // disable encoders for gryo based turning
                    run_using_encoders() ;   // use encoders for time based drive. Required for SDK 2.2.
                }
                getHeading();

                if( states[ curr_state_id_*2 ] == -1.0 ) {   // use "-1, DRIVE_TURN_TO_LEFT" to set last_turn_to_heading_ to current heading
                    last_turn_to_heading_ = heading_;
                    gotoNextState( num_state, states, time, true );
                } else {
                    double tg_deg = states[ curr_state_id_*2 ];
                    if( tg_deg != last_turn_to_heading_ ) last_turn_to_heading_ = tg_deg ;
                    double to_turn_deg = Math.abs( heading_-tg_deg );
                    if( (m == DRIVE_TURN_TO_LEFT && heading_ >= tg_deg ) ||    // MR Gryo, turn left, heading increase
                            (m == DRIVE_TURN_TO_RIGHT && heading_ <= tg_deg) )     // MR Gyro, turn right, heading decrease
                    {
                        gotoNextState( num_state, states, time, true );
                    } else {
                        if(SLOW_TURN_DEGREE>0.0 && to_turn_deg<SLOW_TURN_DEGREE) {
                            turn_slow_ = true;
                        }
                        mode = m;
                    }
                }
            } else if ( USE_ENC_FOR_DRIVE && (
                    (m==DRIVE_RESET_ENC_DONE) ||
                            (m==DRIVE_TURN_LEFT_ENC) || (m==DRIVE_TURN_RIGHT_ENC) ||
                            (m==DRIVE_FORWARD_ENC) || (m==DRIVE_BACKWARD_ENC) ||
                            (m==DRIVE_FORWARD_ENC_SLOW) || (m==DRIVE_BACKWARD_ENC_SLOW) ||
                            (m==DRIVE_FORWARD_ENC_TO_WALL) || (m==DRIVE_BACKWARD_ENC_TO_WALL)
            ) ) {
                // Use encoder to control driving
                if( m==DRIVE_RESET_ENC_DONE ) {   // wait till encoder is reset
                    if( have_drive_encoders_reset() ) {  // reset done, go to next state
                        gotoNextState( num_state, states, time, false );
                    } else { // stay in RESET_ENC state, wait for reseting to complete
                        if( AUTO_RESET_ENC_TIME>0 && time<(curr_state_start_t_+AUTO_RESET_ENC_TIME) ) {  // auto-reset encoders
                            reset_drive_encoders();
                        }
                        mode = m ;
                    }
                } else {  // driving using encoders
                    double tg_enc_cnt = Math.abs(states[ curr_state_id_*2 ]);
                    if( tg_enc_cnt==0.0 ) {
                        gotoNextState( num_state, states, time, /*reset_encoders*/false );
                    }
                    if( (m==DRIVE_BACKWARD_ENC) || (m==DRIVE_FORWARD_ENC ) ||
                            (m==DRIVE_BACKWARD_ENC_SLOW) || (m==DRIVE_FORWARD_ENC_SLOW) ||
                            (m==DRIVE_BACKWARD_ENC_TO_WALL) || (m==DRIVE_FORWARD_ENC_TO_WALL)) {
                        if( tg_enc_cnt<10.0 ) {   // treat it as meters, convert it to encoder count
                            tg_enc_cnt = tg_enc_cnt * ENC_DIST_SCALE;
                        }
                    } else if( (m==DRIVE_TURN_LEFT_ENC) || (m==DRIVE_TURN_RIGHT_ENC) ) {
                        if( tg_enc_cnt<360.0 ) {   // treat it as degrees
                            tg_enc_cnt = tg_enc_cnt * ENC_DEG_SCALE ;
                        }
                    }
                    run_using_encoders();


                    double dis = 0.0;
                    boolean wall_seen = false;
                    if( (m==DRIVE_BACKWARD_ENC_TO_WALL || m==DRIVE_FORWARD_ENC_TO_WALL) ) {
                        if( mr_range_!=null ) {
                            dis = mr_range_.getDistance(DistanceUnit.CM);
                            if(dis>0.0) {
                                if(isRedTeam() && dis>MAX_WALL_DIS_RED1) {
                                    wall_seen = true;
                                } else if( !isRedTeam() && dis>MAX_WALL_DIS_BLUE1 ) {
                                    wall_seen = true;
                                }
                            }
                        }
                    }
                    if( wall_seen ) {
                        reset_drive_encoders();
                        gotoNextState(num_state, states, time, false);
                    } else if( have_drive_encoders_reached( tg_enc_cnt, tg_enc_cnt ) ) {  // reset encoders and go to next state
                        reset_drive_encoders();
                        gotoNextState( num_state, states, time, false );
                    } else { // keep driving
                        curr_state_enc_cnt_ = tg_enc_cnt;
                        mode = m;
                    }
                }
            } else if( m==DRIVE_CHECK_DISTANCE ) {
                // use range sensor for distance checking if available
                double dist = mr_range_.getDistance(DistanceUnit.CM);
                if( dist>MAX_RANGE ) {
                    // invalid reading, ignored
                } else if( dist>WALL_DISTANCE_MAX ) {
                    num_dist_far_ ++;
                } else {
                    num_dist_ok_ ++;
                }
                double period = Math.abs( states[ curr_state_id_*2 ] );   // time limit
                if( time>=(curr_state_start_t_+period) ) {  // timeout
                    if( num_dist_ok_>=num_dist_far_ ) {   // distance is OK, skip re-alignment
                        curr_state_id_ += 8;
                    }
                    gotoNextState( num_state, states, time, true );
                } else { // keep checking
                    mode = m;
                }
            } else if (m == DRIVE_PULL_PIN ){
                //use servo to pull pin out of hanger
                double time_lift_pin_ = time;
                if (time_lift_pin_ - curr_state_start_t_ <= 3){
                    servo_lift_pin_.setPosition(LIFT_PIN_PULL);
                    mode = m;
                } else {
                    servo_lift_pin_.setPosition(LIFT_PIN_STOP);
                    gotoNextState( num_state, states, time, true );
                }
            } else if (m == DRIVE_DROP_MARKER) {
                double time_drop_marker_ = time;
                if (time_drop_marker_ - curr_state_start_t_<=3) {
                    servo_marker_.setPosition(MARKER_DROP_POS_);
                    mode = m;
                } else {
                    gotoNextState( num_state, states, time, true );
                }
            }

            else if( m == DRIVE_STATE_JUMP ) {
                int jump = (int) ( states[ curr_state_id_*2 ] );   // jump
                if( jump==1 ) {
                    // automatically goto NEXT state
                } else if( jump>1 ) {
                    curr_state_id_ += jump-1 ;
                } else if ( jump<-1000 ) {
                    // -1001, last step; -1002 second last
                    jump += 1000;
                    curr_state_id_ = num_state + jump - 1 ;
                } else if ( jump<0 ) {
                    // jump back
                    curr_state_id_ += jump-1 ;
                }
                gotoNextState( num_state, states, time, true );
            }
            else if( m == DRIVE_SHIFT_GEAR ) {
                double f = states[ curr_state_id_*2 ];
                if( f>0.33 && f<=3.0 ) {
                    drive_power_factor_ = f;
                }
                gotoNextState( num_state, states, time, true );
            }
            else if( m == DRIVE_MINERAL_DETECTION ) {

                /** Wait for the game to begin */
                telemetry.addData(">", "Detect mineral");
                telemetry.update();

                /** Activate Tensor Flow Object Detection. */

                if (tfod != null) {
                    tfod.activate();

                    boolean det_done_flag=false;
                    while (det_done_flag==false) {
                        // getUpdatedRecognitions() will return null if no new information is available since
                        // the last time that call was made.
                        List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                        if (updatedRecognitions != null) {
                            telemetry.addData("# Object Detected = ", updatedRecognitions.size());
                            if (updatedRecognitions.size() == 3) {
                                int goldMineralX = -1;
                                int silverMineral1X = -1;
                                int silverMineral2X = -1;
                                for (Recognition recognition : updatedRecognitions) {
                                    if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                        goldMineralX = (int) recognition.getLeft();
                                    } else if (silverMineral1X == -1) {
                                        silverMineral1X = (int) recognition.getLeft();
                                    } else {
                                        silverMineral2X = (int) recognition.getLeft();
                                    }
                                }

                                if (goldMineralX != -1 && silverMineral1X != -1 && silverMineral2X != -1) {
                                    det_done_flag = true;

                                    if (goldMineralX < silverMineral1X && goldMineralX < silverMineral2X) {
                                        telemetry.addData("Gold Mineral Position", "Left");
                                    } else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
                                        telemetry.addData("Gold Mineral Position", "Right");
                                    } else {
                                        telemetry.addData("Gold Mineral Position", "Center");
                                    }
                                } else {
                                    telemetry.addData("Gold mineral position", "Unknown");
                                }
                            }

                            telemetry.update();
                        }

                        double period = Math.abs( states[ curr_state_id_*2 ] );   // time limit
                        if( time>=(curr_state_start_t_+period) ) {  // timeout
                            det_done_flag = true;
                        }
                    }

                    tfod.shutdown();
                }

                gotoNextState( num_state, states, time, true );
            }
            else if (m == DRIVE_LANDING) {
                double lift_enc = motorLift_.getCurrentPosition();
                if ( Math.abs(lift_enc) >= LANDING_ENC_CNT) {
                    gotoNextState( num_state, states, time, true );
                } else { // keep checking
                    mode = m;
                }
            }
            else {
                // Time based drive modes: STOP/FORWARD/BACKWARD/TURN_LEFT/TURN_RIGHT
                if( USE_ENC_FOR_DRIVE ) {
                    run_using_encoders() ;   // use encoders for time based drive. Required for SDK 2.2.
                }
                double period = Math.abs( states[ curr_state_id_*2 ] );   // time limit
                if( time>=(curr_state_start_t_+period) ) {  // timeout
                    gotoNextState( num_state, states, time, true );
                } else { // keep going
                    mode = m;
                }
            }
        } else {
            // curr_state_id_=-1; hanlde initial wait time
            if( USE_ENC_FOR_DRIVE ) {
                run_using_encoders() ;   // use encoders for time based drive. Required for SDK 2.2.
            }

            if( time<init_wait_time_ ) {
                mode = DRIVE_STOP;      // delayed start
            } else {   // initial wait time passes, go to first state
                double period = states[0];
                mode = (int)(states[1]);
                gotoNextState( num_state, states, time, true );    // go to first state, curr_state_id_: -1 => 0
            }
        }

        return mode;
    }



    ///////TRIPS///////***************************************************************************************************************************************
    /// Define red trip
    int getDriveModeRed(double t) {

        double[] testTrip1 = {
                // Test/calibrate distance for encoder based drive
                1.0, DRIVE_STOP,
                2.9, DRIVE_FORWARD_ENC,      // 1.0m
                10.0, DRIVE_STOP,
                3.0, DRIVE_BACKWARD_ENC,     // 1.0m
                10.0, DRIVE_STOP,
        } ;

        double[] testTrip1A = {
                // Test/calibrate distance for encoder based drive
                1.0, DRIVE_STOP,
                3.0, DRIVE_FORWARD_ENC,      // 3.0m
                1.0, DRIVE_STOP,
                3.0, DRIVE_BACKWARD_ENC,     // 3.0m
                3.0, DRIVE_STOP,
                3.0, DRIVE_FORWARD_ENC,      // 3.0m
                1.0, DRIVE_STOP,
                3.0, DRIVE_BACKWARD_ENC,     // 3.0m
                3.0, DRIVE_STOP,
                3.0, DRIVE_FORWARD_ENC,      // 3.0m
                1.0, DRIVE_STOP,
                3.0, DRIVE_BACKWARD_ENC,     // 3.0m
                10.0, DRIVE_STOP,
        } ;

        double[] testTrip1B = {
                // Test/calibrate distance for encoder based drive
                1.0, DRIVE_STOP,
                0.6, DRIVE_FORWARD_ENC,      // 3.0m
                1.0, DRIVE_STOP,
                3.0, DRIVE_BACKWARD_ENC,     // 3.0m
                3.0, DRIVE_STOP,
                3.0, DRIVE_FORWARD_ENC,      // 3.0m
                1.0, DRIVE_STOP,
                3.0, DRIVE_BACKWARD_ENC,     // 3.0m
                3.0, DRIVE_STOP,
                3.0, DRIVE_FORWARD_ENC,      // 3.0m
                1.0, DRIVE_STOP,
                3.0, DRIVE_BACKWARD_ENC,     // 3.0m
                10.0, DRIVE_STOP,
        } ;

        double[] testTrip2 = {
                // Test encoder base drive and turn
                1.0, DRIVE_STOP,

                0.1, DRIVE_RESET_ENC_DONE,
                2000, DRIVE_FORWARD_ENC,
                3.0, DRIVE_STOP,
                0.1, DRIVE_RESET_ENC_DONE,
                2000, DRIVE_BACKWARD_ENC,
                3.0, DRIVE_STOP,

                0.1, DRIVE_RESET_ENC_DONE,
                2000, DRIVE_TURN_LEFT_ENC,
                3.0, DRIVE_STOP,
                0.1, DRIVE_RESET_ENC_DONE,
                2000, DRIVE_TURN_RIGHT_ENC,
                3.0, DRIVE_STOP,

                0.1, DRIVE_RESET_ENC_DONE,
                1.0, DRIVE_FORWARD_ENC,      // 1.0m
                3.0, DRIVE_STOP,
                0.1, DRIVE_RESET_ENC_DONE,
                1.0, DRIVE_BACKWARD_ENC,     // 1.0m
                3.0, DRIVE_STOP,

                0.1, DRIVE_RESET_ENC_DONE,
                90,  DRIVE_TURN_RIGHT_ENC,
                3.0, DRIVE_STOP,
                0.1, DRIVE_RESET_ENC_DONE,
                90,  DRIVE_TURN_LEFT_ENC,
                3.0, DRIVE_STOP,
                0.1, DRIVE_RESET_ENC_DONE,
                180, DRIVE_TURN_RIGHT_ENC,
                1.0, DRIVE_STOP,
                9.0, DRIVE_STOP
        };

        double[] testTrip3 = {
                // Test simple trip
                1.0, DRIVE_STOP,
                0.1, DRIVE_RESET_ENC_DONE,
                0.6, DRIVE_FORWARD_ENC,
                0.1, DRIVE_RESET_ENC_DONE,
                45,  DRIVE_TURN_LEFT_ENC,
                0.1, DRIVE_RESET_ENC_DONE,
                1.7, DRIVE_FORWARD_ENC,
                0.1, DRIVE_RESET_ENC_DONE,
                45,  DRIVE_TURN_LEFT_ENC,
                0.1, DRIVE_RESET_ENC_DONE,
                1.2, DRIVE_BACKWARD_ENC,
                0.1, DRIVE_RESET_ENC_DONE,
                90,  DRIVE_TURN_RIGHT_ENC,
                0.1, DRIVE_RESET_ENC_DONE,
                1.8, DRIVE_BACKWARD_ENC,
                10.0, DRIVE_STOP,
        } ;

        double[] testTrip8 = {
                // Test CHECK_DISTANCE using range sensor
                1.0, DRIVE_STOP,                       //
                0.1, DRIVE_CHECK_DISTANCE,             // check if robot is too far
                0.01,DRIVE_RESET_ENC_DONE,             // adjust robot:  turn right, back up 10cm, turn left, stop at white
                45,  DRIVE_TURN_RIGHT_ENC,
                0.01,DRIVE_RESET_ENC_DONE,
                0.10,DRIVE_BACKWARD_ENC,               // input backup distance?
                0.01,DRIVE_RESET_ENC_DONE,
                45,  DRIVE_TURN_LEFT_ENC,
                0.01,DRIVE_RESET_ENC_DONE,
                10.0, DRIVE_STOP
        };

        double[] testTrip9 = {
                // Test driving to wall using range sensor
                1.1, DRIVE_STOP,//
                0.1, DRIVE_RESET_ENC_DONE,
                1.0, DRIVE_FORWARD_ENC_TO_WALL,        // stop at wall with heading correction
                0.1, DRIVE_RESET_ENC_DONE,
                0.5, DRIVE_BACKWARD_ENC,
                0.1, DRIVE_RESET_ENC_DONE,
                90,  DRIVE_TURN_RIGHT_ENC,
                0.1, DRIVE_RESET_ENC_DONE,
                1.0, DRIVE_BACKWARD_ENC_TO_WALL,        // stop at wall with heading correction
                0.1, DRIVE_RESET_ENC_DONE,
                1.0, DRIVE_STOP,
                10.0, DRIVE_STOP
        };


        double[] testTrip11 = {
                // Test heading correction on IMU heading boundary 180/-180
                1.0, DRIVE_STOP,
                0.1, DRIVE_RESET_ENC_DONE,
                145, DRIVE_TURN_LEFT_ENC,
                0.1, DRIVE_RESET_ENC_DONE,
                3.0, DRIVE_STOP,
                45,  DRIVE_TURN_LEFT_ENC,     // target 190, IMU -170
                0.1, DRIVE_RESET_ENC_DONE,
                3.0, DRIVE_STOP,
                10,  DRIVE_TURN_RIGHT_ENC,
                0.1, DRIVE_RESET_ENC_DONE,
                3.0, DRIVE_STOP,
                1.1, DRIVE_FORWARD_ENC,       // check heading correction
                0.1, DRIVE_RESET_ENC_DONE,
                1.0, DRIVE_STOP,
                10.0, DRIVE_STOP
        };

        double [] testTensorFlow = {
                0.1, DRIVE_STOP,
                //1.0, DRIVE_LANDING,
                //1.7, DRIVE_PULL_PIN,
                //0.1, DRIVE_RESET_ENC_DONE,
                5.0, DRIVE_MINERAL_DETECTION,
                //1.0, DRIVE_FORWARD_ENC,
                60.0, DRIVE_STOP
        };


        double [] CraterTrip1 /*dummy made*/ = {
                0.1, DRIVE_STOP,
                1.0, DRIVE_LANDING,
                1.7, DRIVE_PULL_PIN,
                0.1, DRIVE_RESET_ENC_DONE,
                1.0, DRIVE_FORWARD_ENC,
                60.0, DRIVE_STOP
        };

        double [] CraterTrip1QT = {
                60.0, DRIVE_STOP
        };

        double [] CraterTrip1OR /*dummy made*/ = {
                0.1, DRIVE_STOP,
                1.0, DRIVE_LANDING,
                1.7, DRIVE_PULL_PIN,
                0.1, DRIVE_RESET_ENC_DONE,
                1.0, DRIVE_FORWARD_ENC,
                60.0, DRIVE_STOP
        };

        double [] DepotTrip2 /*dummy made*/ = {
                0.1, DRIVE_STOP,
                1.0, DRIVE_LANDING,
                1.7, DRIVE_PULL_PIN,
                0.1, DRIVE_RESET_ENC_DONE,
                1.5, DRIVE_FORWARD_ENC, ///TODO: Change for depot position
                2.0, DRIVE_DROP_MARKER,
                60.0, DRIVE_STOP
        };

        double [] DepotTrip2QT = {
                60.0, DRIVE_STOP
        };

        double [] DepotTrip2SQ = {
                60.0, DRIVE_STOP
        };

        double [] DepotTrip2OR = {
                60.0, DRIVE_STOP
        };


        /// Run red trips or test trips
        if( trip_name_ == "PosCrater1") {
            return getDriveMode(CraterTrip1, t);
        } else if (trip_name_ == "PosDepot2") {
            return getDriveMode(DepotTrip2, t);
        } else if( trip_name_ == "PosCrater1OR") {
            return getDriveMode(CraterTrip1OR, t);
        } else if( trip_name_ == "PosDepot2OR") {
            return getDriveMode(DepotTrip2OR, t);
        } else if( trip_name_ == "PosCrater1QT") {
            return getDriveMode(CraterTrip1QT, t);
        } else if( trip_name_ == "PosDepot2QT") {
            return getDriveMode(DepotTrip2QT, t);
        } else if( trip_name_ == "PosDepot2SQ") {
            return getDriveMode(DepotTrip2SQ, t);
        } else if( trip_name_ == "TestEncDrive") {
            return getDriveMode(testTrip2, t);
        } else if( trip_name_ == "TestSimpleTrip") {
            return getDriveMode(testTrip3, t);
        } else if( trip_name_ == "TestCalDist") {
            return getDriveMode(testTrip1B, t);
        } else if (trip_name_ == "TestTensorFlow"){
            return getDriveMode(testTensorFlow, t);
        } else {
            return getDriveMode(CraterTrip1, t);
        }
    }

    /// Define blue trip
    int getDriveModeBlue(double t) {

        double [] blueTrip1 = {
                60.0, DRIVE_STOP
        };

        double [] blueTrip1QT = {
                60.0, DRIVE_STOP
        };

        double [] blueTrip1OR = {
                60.0, DRIVE_STOP
        };

        double [] blueTrip1SQ = {
                60.0, DRIVE_STOP
        };

        double [] blueTrip2 = {
                60.0, DRIVE_STOP
        };

        double [] blueTrip2QT = {
                60.0, DRIVE_STOP
        };

        double [] blueTrip2SQ = {
                60.0, DRIVE_STOP
        };

        double [] blueTrip2OR = {
                60.0, DRIVE_STOP
        };


        if( trip_name_ == "BluePos1") {
            return getDriveMode(blueTrip1, t);
        } else if( trip_name_ == "BluePos1OR") {
            return getDriveMode(blueTrip1OR, t);
        } else if( trip_name_ == "BluePos2OR") {
            return getDriveMode(blueTrip2OR, t);
        } else if( trip_name_ == "BluePos1QT") {
            return getDriveMode(blueTrip1QT, t);
        } else if( trip_name_ == "BluePos2") {
            return getDriveMode(blueTrip2QT, t);
        } else if( trip_name_ == "BluePos2SQ") {
            return getDriveMode(blueTrip2SQ, t);
        } else {
            return getDriveMode(blueTrip1, t);
        }
    }


    /****************************************************************************
     * Begin of Encoder related codes
     ****************************************************************************/

    /// Access the left encoder's count.
    int getLFEncoderPos() {
        int l_return = 0;
        if( motorLF_ != null ) {
            l_return = motorLF_.getCurrentPosition();
        }
        return l_return;
    }
    int getLBEncoderPos() {
        int l_return = 0;
        if( motorLB_ != null ) {
            l_return = motorLB_.getCurrentPosition();
        }
        return l_return;
    }

    /// Access the right encoder's count.
    int getRFEncoderPos() {
        int l_return = 0;
        if( motorRF_ != null ) {
            l_return = motorRF_.getCurrentPosition();
        }
        return l_return;
    }
    int getRBEncoderPos() {
        int l_return = 0;
        if( motorRB_ != null ) {
            l_return = motorRB_.getCurrentPosition();
        }
        return l_return;
    }

    /// Indicate whether the left drive encoder has been completely reset.
    boolean isleftEncodersReset() {
        boolean l_return = false;
        if( getLFEncoderPos()==0 && getLBEncoderPos()==0 ) { l_return = true; }
        return l_return;
    }

    /// Indicate whether the left drive encoder has been completely reset.
    boolean isRightEncodersReset () {
        boolean l_return = false;
        if( getRFEncoderPos()==0 && getRBEncoderPos()==0 ) { l_return = true; }
        return l_return;
    }

    /// Indicate whether the encoders have been completely reset.
    boolean have_drive_encoders_reset () {
        boolean l_return = false;
        if( isleftEncodersReset() && isRightEncodersReset() ) {
            l_return = true;
        }
        return l_return;
    }

    /// Indicate whether the left front or left rear drive motor's encoder has reached a value.
    boolean isLeftEncodersReached (double p_count) {
        boolean l_return = false;
        if( motorLF_ != null ) {
            if (Math.abs (motorLF_.getCurrentPosition ()) >= p_count) {
                l_return = true;
            }
        }
        if( !l_return && motorLB_!=null) {
            if( Math.abs( motorLB_.getCurrentPosition() ) >= p_count ) {
                l_return = true;
            }
        }
        return l_return;
    }

    /// Indicate whether the right front or right rear drive motor's encoder has reached a value.
    boolean isRightEncodersReached (double p_count) {
        boolean l_return = false;
        if (motorRF_ != null) {
            if (Math.abs (motorRF_.getCurrentPosition ()) >= p_count) {
                l_return = true;
            }
        }
        if( !l_return && motorRB_!=null) {
            if( Math.abs( motorRB_.getCurrentPosition() ) >= p_count ) {
                l_return = true;
            }
        }
        return l_return;
    }

    /// Indicate whether the drive motors' encoders have reached a value.
    boolean have_drive_encoders_reached(double p_left_count, double p_right_count) {
        boolean l_return = false;
        if( isLeftEncodersReached(p_left_count) && isRightEncodersReached(p_right_count) ) {
            l_return = true;
        }
        return l_return;
    }

    /// Reset the left drive wheel encoder.
    public void resetLeftEncoders () {
        if (motorLF_ != null) {
            motorLF_.setMode ( DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        if (motorLB_ != null) {
            motorLB_.setMode ( DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }

    /// Reset the right drive wheel encoder.
    public void resetRightEncoders () {
        if (motorRF_ != null) {
            motorRF_.setMode ( DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        if (motorRB_ != null) {
            motorRB_.setMode ( DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }

    /// Reset both drive wheel encoders.
    public void reset_drive_encoders () {
        resetLeftEncoders ();
        resetRightEncoders ();
    }

    /// Set the left drive wheel encoder to run, if the mode is appropriate.
    public void useLeftEncoders () {
        if (motorLF_ != null) {
            motorLF_.setMode ( DcMotor.RunMode.RUN_USING_ENCODER);
        }
        if (motorLB_ != null) {
            motorLB_.setMode ( DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    /// Set the right drive wheel encoder to run, if the mode is appropriate.
    public void useRightEncoders () {
        if (motorRF_ != null) {
            motorRF_.setMode ( DcMotor.RunMode.RUN_USING_ENCODER);
        }
        if (motorRB_ != null) {
            motorRB_.setMode ( DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    /// Set both drive wheel encoders to run, if the mode is appropriate.
    public void run_using_encoders () {
        useLeftEncoders ();
        useRightEncoders ();
    }

    /// Set the left drive wheel encoder to run, if the mode is appropriate.
    public void useNoLeftEncoders () {
        if (motorLF_ != null) {
            if (motorLF_.getMode () == DcMotor.RunMode.STOP_AND_RESET_ENCODER) {
                motorLF_.setMode ( DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
        }
        if (motorLB_ != null) {
            if (motorLB_.getMode () == DcMotor.RunMode.STOP_AND_RESET_ENCODER) {
                motorLB_.setMode ( DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
        }
    }

    /// Set the right drive wheel encoder to run, if the mode is appropriate.
    public void useNoRightEncoders () {
        if (motorRF_ != null) {
            if (motorRF_.getMode () == DcMotor.RunMode.STOP_AND_RESET_ENCODER) {
                motorRF_.setMode ( DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
        }
        if (motorRB_ != null) {
            if (motorRB_.getMode () == DcMotor.RunMode.STOP_AND_RESET_ENCODER) {
                motorRB_.setMode ( DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
        }
    }

    /// Set all drive wheel encoders to run, if the mode is appropriate.
    //  Motors must be put in DcMotor.RunMode.RUN_WITHOUT_ENCODERS first, then safely disable encoders
    //  WARNING: RUN_WITHOUT_ENCODERS is deprecated. NOT working!
    public void run_without_drive_encoders() {
        useNoLeftEncoders ();
        useNoRightEncoders ();
    }

    /****************************************************************************
     * End of Encoder related codes
     ****************************************************************************/

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

    // Return values for a given Adafruit RGB connected to a specific port
    String getMuxRgbValues(int port){
        String s = "INVALID";
        return s;
    }


    /// Progressively scale up the heading error for a better heading correction for a given drive mode
    double scaleUpHeadingError(double err, boolean slow_drive) {
        double[] scaleUpperBound  = {0.0, 3.0, 5.0, 7.0, 10.0};    // heading error in degree

        // Original factor for gear ratio (2:3), 2016/12/01
        double[] scaleFactor      = {3.0, 2.5, 2.0, 1.5,  1.0};    // scale up factor; 0~3, 3.0X; 3~5, 2.5X; 5~7, 2.0X; 7~10, 1.5X; >10, 1.0X
        double[] scaleFactorSlow  = {2.0, 1.7, 1.4, 1.2,  1.0};    // smaller scale up factors for slow drive mode if needed

        // New factors for high gear ratio (3:2), 2017/02/20; lower factors for high speed
        //double[] scaleFactor      = {2.0, 1.7, 1.4, 1.2,  1.0};    // lower scale up factors for fast drive mode
        //double[] scaleFactorSlow  = {2.0, 1.7, 1.4, 1.2,  1.0};    // smaller scale up factors for slow drive mode if needed

        // Progressive scaling for smooth heading correction
        //  E.g. err=2 => adj_err=2*3.0=6; 3X
        //  E.g. err=4 => adj_err=1*2.5+3*3.0=11.5; 2.88X
        //  E.g. err=6 => adj_err=1*2.0+2*2.5+3*3.0=16.0; 2.67X
        //  E.g. err=8 => adj_err=1*1.5+2*2.0+2*2.5+3*3.0=19.5; 2.44X
        //  E.g. err=11 => adj_err=1*1.0+3*1.5+2*2.0+2*2.5+3*3.0=23.5; 2.13X
        double abs_err = Math.abs(err);
        double new_err = 0.0;
        for( int i=scaleUpperBound.length-1; i>=0; --i ) {
            double ub = scaleUpperBound[i];
            double f = scaleFactor[i];
            if( slow_drive ) {
                f = scaleFactorSlow[i];
            }
            if( abs_err>ub ) {
                new_err += (abs_err-ub)*f;
                abs_err = ub;
            }
        }

        if( err<0.0 ) new_err *= -1.0;
        return new_err;
    }

    void initVuforia() {
        // int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        // parameters.vuforiaLicenseKey = "ATbVhA//////AAAAGQpUcoBny0Xdi+FFWntcC3w9C63+hv3ccdXKcUsUhNYtbt8IbpT9SQ+VsthWIyix0rrzYP8KYaSYY5na+nufoGmLQo8vE8CPWmUj8eZcdlM9k4mi8ge0T2uzuoKZmcllal8cM3hRxo1JBFVtavCrgulnZxQ8hMsbzZuA+dZDGQTOEOCCH8ZHuh6wrIUygVerHfrXXlpeIAQvXzBiYrVPetr3zu3ROn6rno75mQ0KCM8Qp87BGS4Orx+GwxL8FlO+EXA3aSBvDh7+a57co5212MkGIRUceXxAd+BfoFjiWg3SbJpVbDM7TcDApVR88jlqeEDmbc/ODajLjEKziycgihi1rpq1lOBys2oJ68qdVrtO";
        // parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        // this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }

}

