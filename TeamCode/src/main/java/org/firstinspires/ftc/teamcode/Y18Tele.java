/*
 * This is Program for TeleOp Run for FTC 2017-2018 Relic Recovery season
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


//y for intake servos - in, out, and off - cont. servos - done and compiles
//b for dumping servo, down and up - 180 servo - done and compiles
//right(down) and left(up) bumpers for winch (servo) that lifts up/down the entire pickup mechanism - not using toggles because they give analog value...
//left toggle for linear slide (intake)
//right toggle for Extention linear slide (intake)  todo : program
// D-pad for landing lin-slide (up/down)
//a for lowest mineral linear slide position
//x for highest mineral linear slide position
//right toggle for extension intake linear slide


/**
 * Created by aliris on 10/13/2018.
 */

/// TeleOp Run for League Meet0
@TeleOp(name="Y18Tele", group="GG")
//@Disabled
public class Y18Tele extends Y18Common
{

    /// Other variables
    static final double  MIN_BUTTON_INTERVAL = 0.3;


    boolean DEBUG_DRIVE_MOTORS = false;

    static final double  QUICK_RESET_TIME = 3.0;            // allow 3 sec for grabber to reset, incl. lift, claw, and pusher


    boolean USE_MECANUM_WHEELS = true;
    boolean USE_MECANUM_FOR_SIDEWALK_ONLY = true;
    static final double MECANUM_RIGHT_BACK_SCALE = 1.0;    // compensate the unbalanced robot at right back wheel

    boolean USE_ENCODER_FOR_TELEOP = true;
    double ENCODER_MAX_DRIVE_POWER = 0.8;   // 2017/12/01
    double ENCODER_MAX_ROTATE_POWER = 0.6;
    double ENCODER_MAX_SIDEWALK_POWER = 0.5;   // 2018/02/09, cap Mecanum wheel sidewalk power at 0.5

    double RIGHT_POWER_RATIO = 1.00;

    static final boolean USE_LOW_SEN_DRIVE = true;   // use low sensitivity drive mode for relic and balancing
    boolean low_sen_drive_ = true;

    ///TODO: PUT IN COMMON

    //Intake servo variables (continuous servo)
    double INTAKE_POWER_IN = 1;
    double INTAKE_POWER_OUT = 0;

    //Winch
    double motor_winch_enc = 0.0;

    //DCmotor - lift for the linear slide at the front, called motorMineralsLift_
    double MINERALS_LIFT_UP_POWER = 0.75;
    double MINERALS_LIFT_DOWN_POWER = -0.75;
    static final double MAX_MINERALS_LIFT_ENC_COUNT = 1500;
    static final double MIN_MINERALS_LIFT_ENC_COUNT = 0;
    double MINERALS_LIFT_UP_POS = 1150;
    double MINERALS_LIFT_DOWN_POS = 300;
    double MINERALS_LIFT_UP_HOLD_POWER = 0.05;
    double MINERALS_DUMP_RANGE = 50;

    boolean mineral_lift_auto_up_flag_ = false;
    boolean mineral_lift_auto_down_flag_ = false;
    int a2_prev_cnt_ = 0;
    int x2_prev_cnt_ = 0;

    ///  Constructor
    public Y18Tele() {
    }


    ///  Code to run when the op mode is initialized goes here
    @Override public void init() {
        super.init();

        /// TODO: add tele-op specific hardware here
        if( USE_ENCODER_FOR_TELEOP ) {
            motorLF_.setMode ( DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorLB_.setMode ( DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorRF_.setMode ( DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorRB_.setMode ( DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            //motorLF_.setMaxSpeed(4000);  // unavailable for SDK V3.10

            if(USE_LIFT) motorLift_.setMode ( DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        if( USE_MECANUM_WHEELS ) {
            motorLF_.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            motorLB_.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            motorRF_.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            motorRB_.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }

        if(USE_MINERALS_LIFT){
            motorMineralsLift_.setMode ( DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        if (USE_INTAKE_WINCH) {
            motorWinch_.setMode ( DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }


    }

    @Override public void init_loop() {
        super.init_loop();

        if( USE_ENCODER_FOR_TELEOP ) {
            motorLF_.setMode ( DcMotor.RunMode.RUN_USING_ENCODER);
            motorLB_.setMode ( DcMotor.RunMode.RUN_USING_ENCODER);
            motorRF_.setMode ( DcMotor.RunMode.RUN_USING_ENCODER);
            motorRB_.setMode ( DcMotor.RunMode.RUN_USING_ENCODER);

            if(USE_LIFT) motorLift_.setMode ( DcMotor.RunMode.RUN_USING_ENCODER );
        }

        if(USE_MINERALS_LIFT){
            motorMineralsLift_.setMode ( DcMotor.RunMode.RUN_USING_ENCODER);
        }

        if (USE_INTAKE_WINCH) {
            motorWinch_.setMode (DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    @Override public void loop () {
        curr_time_ = timer_.time();
        servo_lift_pin_pos_ = LIFT_PIN_STOP;

        if (USE_LOW_SEN_DRIVE) {
            low_sen_drive_ = (rsb1_cnt_ % 2) == 1;
        }


        power_lift_ = 0.0;

        double power_lf = 0, power_lb = 0, power_rf = 0, power_rb = 0;
        double power_sweeper = 0;
        double lsy = 0, lsx = 0, rsy = 0, rsx = 0;
        double drive_power_f = 1.0;
        //if( lb1_cnt_%2 != 0 ) drive_power_f = 0.75;  // 25% slowdown for dflt, for shelfing and balancing

        // rsy: right_stick_y ranges from -1 to 1, where -1 is full up, and 1 is full down
        // rsx: right_stick_x ranges from -1 to 1, where -1 is full left and 1 is full right
        rsy = -gamepad1.right_stick_y;
        rsx = gamepad1.right_stick_x;

        if ((curr_time_ - last_button_time_) > MIN_BUTTON_INTERVAL) {
            if (gamepad1.x) {
                x1_cnt_++;
                last_button_time_ = curr_time_;
            } else if (gamepad1.y) {
                y1_cnt_++;
                last_button_time_ = curr_time_;
            } else if (gamepad1.a) {
                a1_cnt_++;
                last_button_time_ = curr_time_;
            } else if (gamepad1.b) {
                b1_cnt_++;
                last_button_time_ = curr_time_;
            } else if (gamepad1.left_bumper) {
                lb1_cnt_++;
                last_button_time_ = curr_time_;
            } else if (gamepad1.right_bumper) {
                rb1_cnt_++;
                last_button_time_ = curr_time_;
            } else if (gamepad1.left_stick_button) {
                lsb1_cnt_++;
                last_button_time_ = curr_time_;
            } else if (gamepad1.right_stick_button) {
                rsb1_cnt_++;
                last_button_time_ = curr_time_;
            }
        }
        if ((curr_time_ - last_button_time2_) > MIN_BUTTON_INTERVAL) {
            if (gamepad2.x) {
                x2_cnt_++;
                last_button_time2_ = curr_time_;
            } else if (gamepad2.y) {
                y2_cnt_++;
                last_button_time2_ = curr_time_;
            } else if (gamepad2.a) {
                a2_cnt_++;
                last_button_time2_ = curr_time_;
            } else if (gamepad2.b) {
                b2_cnt_++;
                last_button_time2_ = curr_time_;
            } else if (gamepad2.left_bumper) {
                lb2_cnt_++;
                last_button_time2_ = curr_time_;
            } else if (gamepad2.right_bumper) {
                rb2_cnt_++;
                last_button_time2_ = curr_time_;
            } else if (gamepad2.left_stick_button) {
                lsb2_cnt_++;
                last_button_time2_ = curr_time_;
            } else if (gamepad2.right_stick_button) {
                rsb2_cnt_++;
                last_button_time2_ = curr_time_;
            }
        }


        if (Math.abs(rsx) > 0.1) {
            /// Rotate/spin the robot

            // Use right_stick_x to rotate the robot; left => turn left(CCW), right => turn right(CW)
            // Ignore it if the value is too small to avoid mis-operation.
            // right_stick_y is not used at this time
            power_rf = rsx;
            power_lf = rsx;

            // clip the power_rf/power_lf values so that the values never exceed +/- 1.0
            power_rf = Range.clip(power_rf, -1, 1);
            power_lf = Range.clip(power_lf, -1, 1);

            // scale the joystick value to make it easier to control the robot more precisely at slower speeds.
            power_rf = (double) scaleRotatePower(power_rf);
            power_lf = (double) scaleRotatePower(power_lf);

            power_rb = power_rf;
            power_lb = power_lf;


        } else {
            ///  Use left stick to move/turn the robot to a specific direction at a given speed

            lsy = -gamepad1.left_stick_y;   // throttle
            lsx = gamepad1.left_stick_x;    // direction; lsx>0, turn right <=> RF<LF, left wheels turn faster

            power_lf = lsx + lsy;
            power_rf = lsx - lsy;

            // clip the power_rf/power_lf values so that the values never exceed +/- 1.0
            power_rf = Range.clip(power_rf, -1, 1);
            power_lf = Range.clip(power_lf, -1, 1);

            // scale the joystick value to make it easier to control the robot more precisely at slower speeds.
            power_rf = (double) scaleDrivePower(power_rf, drive_power_f);
            power_lf = (double) scaleDrivePower(power_lf, drive_power_f);

            if (USE_LOW_SEN_DRIVE && low_sen_drive_) {
                power_rf = (double) scaleDrivePowerLowSensitivity(power_rf,/*drive_power_f*/1.0);
                power_lf = (double) scaleDrivePowerLowSensitivity(power_lf,/*drive_power_f*/1.0);
            }

            power_rf = Range.clip(power_rf, -1, 1);
            power_lf = Range.clip(power_lf, -1, 1);

            /// LR is same as LF, RR is same as RF
            power_lb = power_lf;
            power_rb = power_rf;

        }

        if (DEBUG_DRIVE_MOTORS) {
            boolean test_drive = false;
            if (gamepad1.left_bumper) {
                power_lf = 1.0;
                test_drive = true;
            } else if (gamepad1.right_bumper) {
                power_lf = -1.0;
                test_drive = true;
            } else if (Math.abs(gamepad1.left_trigger) > 0.1) {
                power_lf = gamepad1.left_trigger;
                test_drive = true;
            } else if (Math.abs(gamepad1.right_trigger) > 0.1) {
                power_lf = -gamepad1.right_trigger;
                test_drive = true;
            }
            if (test_drive) {
                power_lf = Range.clip(power_lf, -1, 1);
                power_lb = power_lf;
                power_rf = power_rb = -power_lf * RIGHT_POWER_RATIO;
            }
        }

        /// Use Mecanum wheels by right joystick
        boolean drive_sidewalk = false;
        if (USE_MECANUM_WHEELS && Math.abs(rsx) > JOYSTICK_DEAD_ZONE) {
            lsx = gamepad1.right_stick_x;    // direction; lsx>0, turn right <=> RF<LF, left wheels turn faster
            lsy = -gamepad1.right_stick_y;   // throttle
            if (USE_MECANUM_FOR_SIDEWALK_ONLY) lsy = 0;
            drive_sidewalk = true;

            power_lf = lsy + lsx;
            power_lb = lsy - lsx;
            power_rf = -lsy + lsx;
            power_rb = -lsy - lsx;

            power_lf = Range.clip(power_lf, -1, 1);
            power_lb = Range.clip(power_lb, -1, 1);
            power_rf = Range.clip(power_rf, -1, 1);
            power_rb = Range.clip(power_rb, -1, 1);

            power_lf = (double) scaleDrivePower(power_lf, drive_power_f);
            power_lb = (double) scaleDrivePower(power_lb, drive_power_f);
            power_rf = (double) scaleDrivePower(power_rf, drive_power_f);
            power_rb = (double) scaleDrivePower(power_rb, drive_power_f);
            power_rb *= MECANUM_RIGHT_BACK_SCALE;

            double max_mw_power = ENCODER_MAX_SIDEWALK_POWER;      // cap the max wheel power for sidewalk
            power_lf = Range.clip(power_lf, -max_mw_power, max_mw_power);
            power_lb = Range.clip(power_lb, -max_mw_power, max_mw_power);
            power_rf = Range.clip(power_rf, -max_mw_power, max_mw_power);
            power_rb = Range.clip(power_rb, -max_mw_power, max_mw_power);
        }

        /// Flip robot if needed
        boolean flip_robot = false;  // grabber facing forward
        if (flip_robot) {
            double p = power_lf;
            power_lf = power_rf;
            power_rf = p;
            power_lb = power_lf;
            power_rb = power_rf;
        }

        /// Set power values for all motors after clipping
        power_lf = Range.clip(power_lf, -1, 1);
        power_lb = Range.clip(power_lb, -1, 1);
        power_rf = Range.clip(power_rf, -1, 1);
        power_rb = Range.clip(power_rb, -1, 1);

        motorRF_.setPower(power_rf);
        motorRB_.setPower(power_rb);
        motorLF_.setPower(power_lf);
        motorLB_.setPower(power_lb);


        /// Use digital pad to control lift
        boolean manual_claw_control = false;
        if (gamepad1.dpad_up || gamepad2.dpad_up) { // raise lift
            power_lift_ = LIFT_UP_POWER;
        } else if (gamepad1.dpad_down || gamepad2.dpad_down) {  // lower lift
            power_lift_ = LIFT_DOWN_POWER;
        }
        motorLift_.setPower(power_lift_);

        /// Test
        if (gamepad1.a) servo_lift_pin_pos_ = LIFT_PIN_PULL;
        servo_lift_pin_.setPosition(servo_lift_pin_pos_);

        ///intake servo
        if (USE_INTAKE_SERVOS) {

            if (y2_cnt_ % 3 == 1) {
                servo_intake_l_.setPosition(INTAKE_POWER_IN);
                servo_intake_r_.setPosition(1 - INTAKE_POWER_IN);
            } else if (y2_cnt_ % 3 == 2) {
                servo_intake_l_.setPosition(INTAKE_POWER_OUT);
                servo_intake_r_.setPosition(1 - INTAKE_POWER_OUT);
            } else if (y2_cnt_ % 3 == 0) {
                servo_intake_l_.setPosition(INTAKE_POWER_BRAKE);
                servo_intake_r_.setPosition(INTAKE_POWER_BRAKE);
            }

        }

        //Dumping
        if (USE_SERVO_DUMP) {
            if (b2_cnt_ % 2 == 1) {                     //if b is pressed once, go down
                servo_dump_.setPosition(DUMP_DOWN);
            } else if (b2_cnt_ % 2 == 0) {              //if b is pressed twice, go up
                servo_dump_.setPosition(DUMP_UP);
            }
        }


        //Intake linear slide & winch
        if (USE_MINERALS_LIFT) {
            /*
            Psuedo Code : if math.abs of gamepad 2 . left stick y is more than 0.1 (ex.), then go to manual (set the flag), do similar for the button control (a || x)
             */
            boolean manual_flag = false;

            double vly = gamepad2.left_stick_y;
            double mineral_lift_enc = motorMineralsLift_.getCurrentPosition();
            power_minerals_lift = 0.0;

            if (Math.abs(gamepad2.left_stick_y) > 0.1 || gamepad2.left_bumper || gamepad2.right_bumper) {
                manual_flag = true;
                mineral_lift_auto_up_flag_ = false;
                mineral_lift_auto_down_flag_ = false;
            } else {
                if (x2_cnt_ != x2_prev_cnt_) {
                    mineral_lift_auto_up_flag_ = true;
                    mineral_lift_auto_down_flag_ = false;
                } else if (a2_cnt_ != a2_prev_cnt_) {
                    mineral_lift_auto_up_flag_ = false;
                    mineral_lift_auto_down_flag_ = true;
                }
            }

            x2_prev_cnt_ = x2_cnt_;
            a2_prev_cnt_ = a2_cnt_;

            power_motor_winch = WINCH_POWER_BRAKE;
            motor_winch_enc = motorWinch_.getCurrentPosition();

            if (manual_flag) {
                power_minerals_lift = -vly;   // todo negative


                if (gamepad2.left_bumper) {
                    if (motor_winch_enc > WINCH_UP_ENC_CNT) {
                        power_motor_winch = WINCH_UP_POWER;
                    }
                } else if (gamepad2.right_bumper) {
                    if (motor_winch_enc < WINCH_MANUAL_DOWN_ENC_CNT) {
                        power_motor_winch = WINCH_DOWN_POWER;
                    }
                }
            } else if (mineral_lift_auto_up_flag_) {         //up
                if (motor_winch_enc > (WINCH_UP_ENC_CNT+200)) {
                    power_motor_winch = WINCH_UP_POWER;
                }

                if (Math.abs(mineral_lift_enc) >= (MINERALS_LIFT_UP_POS - MINERALS_DUMP_RANGE) && Math.abs(mineral_lift_enc) < MINERALS_LIFT_UP_POS) {
                    power_minerals_lift = MINERALS_LIFT_UP_HOLD_POWER;
                } else if (Math.abs(mineral_lift_enc) < (MINERALS_LIFT_UP_POS + MINERALS_DUMP_RANGE) && Math.abs(mineral_lift_enc) >= MINERALS_LIFT_UP_POS) {
                    power_minerals_lift = 0.0;   //not negative because it will droop anyways
                } else if (Math.abs(mineral_lift_enc) >= (MINERALS_LIFT_UP_POS + MINERALS_DUMP_RANGE)) {
                    power_minerals_lift = MINERALS_LIFT_DOWN_POWER;
                } else if (Math.abs(mineral_lift_enc) < (MINERALS_LIFT_UP_POS - MINERALS_DUMP_RANGE)) {
                    power_minerals_lift = MINERALS_LIFT_UP_POWER;
                }
            } else if (mineral_lift_auto_down_flag_) {     //down
                if (motor_winch_enc < WINCH_AUTO_DOWN_ENC_CNT) {
                    power_motor_winch = WINCH_DOWN_POWER;
                }

                if (Math.abs(mineral_lift_enc) >= (MINERALS_LIFT_DOWN_POS - MINERALS_DUMP_RANGE) && Math.abs(mineral_lift_enc) < MINERALS_LIFT_DOWN_POS) {
                    power_minerals_lift = MINERALS_LIFT_UP_HOLD_POWER;
                } else if (Math.abs(mineral_lift_enc) < (MINERALS_LIFT_DOWN_POS + MINERALS_DUMP_RANGE) && Math.abs(mineral_lift_enc) >= MINERALS_LIFT_DOWN_POS) {
                    power_minerals_lift = 0.0;   //not negative because it will droop anyways
                } else if (Math.abs(mineral_lift_enc) >= (MINERALS_LIFT_DOWN_POS + MINERALS_DUMP_RANGE)) {
                    power_minerals_lift = MINERALS_LIFT_DOWN_POWER;
                } else if (Math.abs(mineral_lift_enc) < (MINERALS_LIFT_DOWN_POS - MINERALS_DUMP_RANGE)) {
                    power_minerals_lift = MINERALS_LIFT_UP_POWER;
                }
            }

            telemetry.addData("Minerals Lift EncPos", ": " + String.valueOf(motorMineralsLift_.getCurrentPosition()) + ", Power=" + String.valueOf(motorMineralsLift_.getPower()));
            telemetry.addData("Intake Winch EncPos", ": " + String.valueOf(motorWinch_.getCurrentPosition()) + ", Power=" + String.valueOf(motorWinch_.getPower()));

            power_motor_winch = Range.clip(power_motor_winch, -1, 1);
            motorWinch_.setPower(power_motor_winch);

            if (Math.abs(mineral_lift_enc) >= MAX_MINERALS_LIFT_ENC_COUNT && power_minerals_lift > 0) {
                power_minerals_lift = 0.0;
            } else if (mineral_lift_enc <= MIN_MINERALS_LIFT_ENC_COUNT && power_minerals_lift < 0) {
                power_minerals_lift = 0.0;
            }

            power_minerals_lift = Range.clip(power_minerals_lift, -1, 1);
            motorMineralsLift_.setPower(power_minerals_lift);

        }

        //extension linear slide servo
        if (USE_SERVO_EXTENSION){
            if(gamepad2.right_stick_y == 0){
                servo_extension_.setPosition(SERVO_EXTENSION_STOP);
            }else if (gamepad2.right_stick_y > 0){
                servo_extension_.setPosition(EXTENSION_OUT);
            }else {
                servo_extension_.setPosition(EXTENSION_IN);
            }
        }

        /// Send telemetry data back to driver station for debugging
        boolean no_msg = false;               // set true to disable all msgs to minimize lagging
        boolean show_wheel_power = true;
        boolean show_lift_pos = true;
        boolean show_heading = false;
        boolean show_voltage = false;
        boolean show_mr_range = false;

        if( !no_msg ) {
            telemetry.addData("GiftGears(Y18LM1)", "Team:"+(isRedTeam()?"RED":"BLUE")+", Time:"+String.format("%.2f",curr_time_));

            if( show_wheel_power )  telemetry.addData("WheelPower", "Factor="+String.format("%.2f",drive_power_f)+"LF/RF/LB/RB: " + String.format("%.2f", motorLF_.getPower()) + "/" + String.format("%.2f", motorRF_.getPower()) + "/" + String.format("%.2f", motorLB_.getPower()) + "/" + String.format("%.2f", motorRB_.getPower()));

            if( DEBUG_DRIVE_MOTORS ) {
                telemetry.addData("Lift EncPose", ": "+String.valueOf(motorLift_.getCurrentPosition())+", Power="+String.valueOf(motorLift_.getPower()));
                telemetry.addData("Mineral Lift EncPose", ": "+String.valueOf(motorMineralsLift_.getCurrentPosition())+", Power="+String.valueOf(motorMineralsLift_.getPower()));
            }
            if( show_mr_range && mr_range_!=null ) {
                telemetry.addData("MRRangeSensor", String.format("ultra/opt/dist=%4d/%.2f/%.2f",mr_range_.rawUltrasonic(),mr_range_.cmOptical(),mr_range_.getDistance(DistanceUnit.CM)));
            }
        }

    }


    // Code to run when the op mode is first disabled goes here
    @Override public void stop () {
    }


    /*
     * This method scales the joystick input so for low joystick values, the
     * scaled value is less than linear.  This is to make it easier to drive
     * the robot more precisely at slower speeds.
     */
    double scaleDrivePower(double dVal, double factor) {
        //                    { 0.0, 0.06, 0.13, 0.19, 0.25, 0.31, 0.38, 0.44, 0.50, 0.56, 0.63, 0.69, 0.75, 0.81, 0.88, 0.94, 1.00 };  // linear scale
        double[] scaleArray = {0.0, 0.14, 0.16, 0.18, 0.20, 0.22, 0.24, 0.27, 0.30, 0.34, 0.38, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00};  // Y17, with encoder

        // Get the corresponding index for the scaleDrivePower array.
        int index = (int) (dVal * 16.0);
        if (index < 0) {
            index = -index;
        } else if (index > 16) {
            index = 16;
        }

        double dScale = 0.0;
        if (dVal < 0) {
            dScale = -scaleArray[index];
        } else {
            dScale = scaleArray[index];
        }

        if( USE_ENCODER_FOR_TELEOP ) dScale *= ENCODER_MAX_DRIVE_POWER;
        if( factor>0.0 && factor<=1.5) dScale *= factor;
        return dScale;
    }

    /// Overloaded vaersion with default factor of 1.0
    double scaleDrivePower(double dVal) {
        return scaleDrivePower(dVal, 1.0);
    }

    /// Low sensitivity drive mode for balancing and relic
    double scaleDrivePowerLowSensitivity(double dVal, double factor) {
        //                    { 0.0, 0.06, 0.13, 0.19, 0.25, 0.31, 0.38, 0.44, 0.50, 0.56, 0.63, 0.69, 0.75, 0.81, 0.88, 0.94, 1.00 };  // linear scale
        double[] scaleArray = {0.0, 0.14, 0.16, 0.18, 0.20, 0.22, 0.24, 0.26, 0.28, 0.30, 0.32, 0.34, 0.36, 0.38, 0.40, 0.42, 0.44};  // Y17, with encoder

        // Get the corresponding index for the scaleDrivePower array.
        int index = (int) (dVal * 16.0);
        if (index < 0) {
            index = -index;
        } else if (index > 16) {
            index = 16;
        }

        double dScale = 0.0;
        if (dVal < 0) {
            dScale = -scaleArray[index];
        } else {
            dScale = scaleArray[index];
        }

        if( USE_ENCODER_FOR_TELEOP ) dScale *= ENCODER_MAX_DRIVE_POWER;
        if( factor>0.0 && factor<=1.5) dScale *= factor;
        return dScale;
    }

    /// Scale the robot rotating power
    double scaleRotatePower(double dVal) {
        //                    { 0.0, 0.06, 0.13, 0.19, 0.25, 0.31, 0.38, 0.44, 0.50, 0.56, 0.63, 0.69, 0.75, 0.81, 0.88, 0.94, 1.00 };  // linear scale
        double[] scaleArray = {0.0, 0.35, 0.4, 0.45, 0.50, 0.55, 0.60, 0.65, 0.70, 0.75, 0.78, 0.80, 0.85, 0.90, 0.93, 0.97, 1.0};   // Y17, with encoder for Mecanum 6

        // Get the corresponding index for the scalePower array.
        int index = (int) (dVal * 16.0);
        if (index < 0) {
            index = -index;
        } else if (index > 16) {
            index = 16;
        }

        double dScale = 0.0;
        if (dVal < 0) {
            dScale = -scaleArray[index];
        } else {
            dScale = scaleArray[index];
        }

        if( USE_ENCODER_FOR_TELEOP ) dScale *= ENCODER_MAX_ROTATE_POWER;
        return dScale;
    }

}

