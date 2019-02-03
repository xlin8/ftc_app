/*
 * This is Program for TeleOp Run for FTC 2018-2019 season
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


//y for intake servos - in, out, and off - cont. servos - done and compiles
//b for dumping servo, down and up - 180 servo - done and compiles
//right(down) and left(up) bumpers for winch (servo) that lifts up/down the entire pickup mechanism - not using toggles because they give analog value...
//left toggle for linear slide (intake)
//right toggle for extention linear slide (intake)  todo : program
// D-pad for landing lin-slide (up/down)
//a for lowest-ish mineral linear slide position
//x for highest mineral linear slide position
//right toggle for extension intake linear slide
//right trigger driver 2 = compact front linear slide, intake off, winch up
//left trigger driver 1 = manual pulling of the servo pin


@TeleOp(name="Y18TeleLinearOp", group="GG")
//@Disabled
public class Y18TeleLinearOp extends Y18HardwareLinearOp
{
    static final double  MIN_BUTTON_INTERVAL = 0.3;

    static final double ENCODER_MAX_DRIVE_POWER = 0.8;      // 2017/12/01
    static final double ENCODER_MAX_ROTATE_POWER = 0.6;
    static final double ENCODER_MAX_SIDEWALK_POWER = 0.5;   // 2018/02/09, cap Mecanum wheel sidewalk power at 0.5

    static final float JOYSTICK_DEAD_ZONE = 0.1f;

    static final boolean USE_MECANUM_WHEELS = true;
    static final boolean USE_MECANUM_FOR_SIDEWALK_ONLY = true;
    static final double MECANUM_RIGHT_BACK_SCALE = 1.0;    // compensate the unbalanced robot at right back wheel

    static final boolean USE_LOW_SEN_DRIVE = true;         // use low sensitivity drive mode for relic and balancing
    boolean useLowSenDrive_ = true;

    int [] aCnt_={0, 0};                 // number of times A is pressed for pads 1 and 2
    int [] bCnt_={0, 0};                 // number of times B is pressed for pads 1 and 2
    int [] xCnt_={0, 0};                 // number of times X is pressed for pads 1 and 2
    int [] yCnt_={0, 0};                 // number of times Y is pressed for pads 1 and 2
    int [] lbCnt_={0, 0};                // number of times left_bumper is pressed for pads 1 and 2
    int [] rbCnt_={0, 0};                // number of times right_bumper is pressed for pads 1 and 2
    int [] lsbCnt_={0, 0};               // number of times left_joystick is pressed for pads 1 and 2
    int [] rsbCnt_={0, 0};               // number of times right_joystick is pressed for pads 1 and 2

    double [] lastButtonPressTime_={0.0, 0.0};

    @Override
    public void runOpMode() {
        initialize();

        waitForStart();

        initializeWhenStart();

        while (opModeIsActive()) {
            driveRobot();
        }

        cleanUpAtEndOfRun();
    }

    @Override
    public void initialize() {
        super.initialize();

        motorLF_.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLB_.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRF_.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRB_.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        if( USE_MECANUM_WHEELS ) {
            motorLF_.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            motorLB_.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            motorRF_.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            motorRB_.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }

        if(USE_LIFT) motorLift_.setMode ( DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    void initializeWhenStart() {
        timer_.reset();
        currTime_ = 0.0;

        motorLF_.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLB_.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRF_.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRB_.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        if(USE_LIFT) motorLift_.setMode ( DcMotor.RunMode.RUN_USING_ENCODER );
    }

    void cleanUpAtEndOfRun() {
        // TBD
    }

    void driveRobot() {
        currTime_ = timer_.time();

        checkPressedButton(0);  // Game pad 1
        checkPressedButton(1);  // Game pad 2

        applyWheelPower();

        pullLiftPin();

        driveLiftMotor();
    }

    void checkPressedButton(int pad_id) {
        if ((currTime_ - lastButtonPressTime_[pad_id]) > MIN_BUTTON_INTERVAL) return;

        Gamepad game_pad=(pad_id==0)?gamepad1:gamepad2;

        if (game_pad.x) {
            xCnt_[pad_id]++;
            lastButtonPressTime_[pad_id] = currTime_;
        } else if (game_pad.y) {
            yCnt_[pad_id]++;
            lastButtonPressTime_[pad_id] = currTime_;
        } else if (game_pad.a) {
            aCnt_[pad_id]++;
            lastButtonPressTime_[pad_id] = currTime_;
        } else if (game_pad.b) {
            bCnt_[pad_id]++;
            lastButtonPressTime_[pad_id] = currTime_;
        } else if (game_pad.left_bumper) {
            lbCnt_[pad_id]++;
            lastButtonPressTime_[pad_id] = currTime_;
        } else if (game_pad.right_bumper) {
            rbCnt_[pad_id]++;
            lastButtonPressTime_[pad_id] = currTime_;
        } else if (game_pad.left_stick_button) {
            lsbCnt_[pad_id]++;
            lastButtonPressTime_[pad_id] = currTime_;
        } else if (game_pad.right_stick_button) {
            rsbCnt_[pad_id]++;
            lastButtonPressTime_[pad_id] = currTime_;
        }
    }

    void applyWheelPower() {
        double power_lf = 0;
        double power_lb = 0;
        double power_rf = 0;
        double power_rb = 0;
        double power_sweeper = 0;
        double lsy = 0;
        double lsx = 0;
        double drive_power_factor = 1.0;

        // rsy: right_stick_y ranges from -1 to 1, where -1 is full up, and 1 is full down
        // rsx: right_stick_x ranges from -1 to 1, where -1 is full left and 1 is full right
        double rsy = -gamepad1.right_stick_y;
        double rsx = gamepad1.right_stick_x;

        if (USE_LOW_SEN_DRIVE) {
            useLowSenDrive_ = ((rsbCnt_[0] % 2) == 1);
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
            power_rf = (double) scaleDrivePower(power_rf, drive_power_factor);
            power_lf = (double) scaleDrivePower(power_lf, drive_power_factor);

            if (USE_LOW_SEN_DRIVE && useLowSenDrive_) {
                power_rf = (double) scaleDrivePowerLowSensitivity(power_rf,/*drive_power_factor*/1.0);
                power_lf = (double) scaleDrivePowerLowSensitivity(power_lf,/*drive_power_factor*/1.0);
            }

            power_rf = Range.clip(power_rf, -1, 1);
            power_lf = Range.clip(power_lf, -1, 1);

            /// LR is same as LF, RR is same as RF
            power_lb = power_lf;
            power_rb = power_rf;
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

            power_lf = (double) scaleDrivePower(power_lf, drive_power_factor);
            power_lb = (double) scaleDrivePower(power_lb, drive_power_factor);
            power_rf = (double) scaleDrivePower(power_rf, drive_power_factor);
            power_rb = (double) scaleDrivePower(power_rb, drive_power_factor);
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

        // telemetry.addData("WheelPower", "Factor="+String.format("%.2f",drive_power_factor)+"LF/RF/LB/RB: " + String.format("%.2f", motorLF_.getPower()) + "/" + String.format("%.2f", motorRF_.getPower()) + "/" + String.format("%.2f", motorLB_.getPower()) + "/" + String.format("%.2f", motorRB_.getPower()));
        // telemetry.update();
    }

    double scaleDrivePower(double dVal,
                           double factor) {
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

        dScale *= ENCODER_MAX_DRIVE_POWER;
        if (factor > 0.0 && factor <= 1.5) dScale *= factor;
        return dScale;
    }

    double scaleDrivePowerLowSensitivity(double dVal,
                                         double factor) {
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

        dScale *= ENCODER_MAX_DRIVE_POWER;
        if (factor > 0.0 && factor <= 1.5) dScale *= factor;
        return dScale;
    }

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

        dScale *= ENCODER_MAX_ROTATE_POWER;
        return dScale;
    }

    void pullLiftPin() {
        if (USE_SERVO_LIFT_PIN) {
            if (gamepad1.left_trigger > 0.0) {
                servoLiftPinPos_ = LIFT_PIN_PULL_POS;
                servoLiftPin_.setPosition(servoLiftPinPos_);
            }
        }
    }

    void driveLiftMotor() {
        if (USE_LIFT) {
            liftPower_ = 0.0;

            if (gamepad1.dpad_up || gamepad2.dpad_up) { // raise lift
                liftPower_ = LIFT_UP_POWER;
            } else if (gamepad1.dpad_down || gamepad2.dpad_down) {  // lower lift
                liftPower_ = LIFT_DOWN_POWER;
            }
            motorLift_.setPower(liftPower_);
        }
    }
}
