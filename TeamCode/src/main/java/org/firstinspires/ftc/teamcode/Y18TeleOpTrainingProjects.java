/**
 * Put all common hardware configuration here
 */

/* Created by Melinda */
/*2018-19 FTC Season*/

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.lynx.LynxI2cColorRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.AnalogInputController;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;



/**
 * Common codes
 */

@TeleOp(name="Java TeleOp Training Projects Melinda", group="GG")
@Disabled
public class Y18TeleOpTrainingProjects extends OpMode {

    /// Main timer
    ElapsedTime timer_ = new ElapsedTime();

    /// Motors
    DcMotor LeftMotor;
    DcMotor RightMotor;


    double power_lf = 0.0;
    double power_rf = 0.0;


    ///Map power arrays
    static final double[] map_power_table;
    static {
        map_power_table = new double[]{0, 0.25, 0.3, 0.35, 0.4, 0.45, 0.5, 0.6, 0.65, 0.7, 0.75, 0.8, 0.85, 0.9, 0.95, 1.0, 1.0};
    }

    /// Servo Variables

    /// IMU

    /// REV range sensors


    /// Gamepad Variables
    static final float JOYSTICK_DEAD_ZONE = 0.1f;
    int a1_cnt_;                 // number of times pad1/A is pressed
    int b1_cnt_;                 // number of times pad1/B is pressed
    int x1_cnt_;                 // number of times pad1/X is pressed
    int y1_cnt_;                 // number of times pad1/Y is pressed
    int lb1_cnt_;                // number of times pad1/left_bumper is pressed
    int rb1_cnt_;                // number of times pad1/right_bumper is pressed

    int a2_cnt_;                 // number of times pad2/A is pressed
    int b2_cnt_;                 // number of times pad2/B is pressed
    int x2_cnt_;                 // number of times pad2/X is pressed
    int y2_cnt_;                 // number of times pad2/Y is pressed
    int lb2_cnt_;                // number of times pad2/left_bumper is pressed
    int rb2_cnt_;                // number of times pad2/right_bumper is pressed

    double last_button_time_;
    double last_button_time2_;
    static final double MIN_BUTTON_INTERVAL = 0.3;

    boolean pos_increasing_ = true;
    boolean pos_decreasing = true;

    /// Other variables
    double curr_time_;
    double last_change_time_;

    ///  Constructor
    public Y18TeleOpTrainingProjects() {
    }


    ///  Code to run when the op mode is initialized goes here
    @Override
    public void init() {
        /// Use the hardwareMap to get the dc motors and servos by name.
        LeftMotor = hardwareMap.dcMotor.get("motor1");
        RightMotor = hardwareMap.dcMotor.get("motor2");

        LeftMotor.setPower(0.0);
        RightMotor.setPower(0.0);

        LeftMotor.setDirection(DcMotor.Direction.REVERSE);


        /// Set joystick deadzone, any value below this threshold value will be considered as 0; moved from init() to init_loop() to aovid crash
        if (gamepad1 != null) gamepad1.setJoystickDeadzone(JOYSTICK_DEAD_ZONE);
        if (gamepad2 != null) gamepad2.setJoystickDeadzone(JOYSTICK_DEAD_ZONE);


        curr_time_ = 0.0;
        last_button_time_ = 0.0;
        a1_cnt_ = 0;
        b1_cnt_ = 0;
        x1_cnt_ = 0;
        y1_cnt_ = 0;
        lb1_cnt_ = 0;
        rb1_cnt_ = 0;

        last_button_time2_ = 0.0;
        a2_cnt_ = 0;
        b2_cnt_ = 0;
        x2_cnt_ = 0;
        y2_cnt_ = 0;
        lb2_cnt_ = 0;
        rb2_cnt_ = 0;
    }

    /// This method will be called once before entering loop()
    @Override
    public void init_loop() {
        timer_.reset();
        curr_time_ = 0.0;

    }

    /// This method will be called repeatedly in a loop
    @Override
    public void loop() {
        curr_time_ = timer_.time();

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
            }
        }//gamepad button detection segment


        //////////////////////////////////////////////////////////////////////////**************///


        //Motor power

        //Arcade drive: Left Stick
        double vlx = gamepad1.left_stick_x;
        double vly = gamepad1.left_stick_y;

        power_lf = vlx + vly;
        power_rf = vly - vlx;

        telemetry.addData("vlx", ": " + String.valueOf(vlx) + ", Power=" + String.valueOf(power_lf));
        telemetry.addData("vly", ": " + String.valueOf(vlx) + ", Power=" + String.valueOf(power_rf));

        power_lf = Range.clip(power_lf, -1, 1);
        power_rf = Range.clip(power_rf, -1, 1);

        if (power_lf >= 0) {
            int id = (int) (power_lf * (double)16.0);
            power_lf = map_power_table[id];
        } else if (power_lf < 0){
            int id = -((int)(power_lf * (double)16.0));
            power_lf = -map_power_table[id];
        }

        if (power_rf >= 0) {
            int id = (int) (power_rf * (double)16.0);
            power_rf = map_power_table[id];
        } else if (power_rf < 0){
            int id = -((int) (power_rf * (double)16.0));
            power_rf = -(map_power_table[id]);
        }

        //Arcade drive: Spinning the robot with the right stick

        // double vlx_right = gamepad1.right_stick_x;
        // power_lf = vlx_right;
        // power_rf = -(vlx_right);


        //If the stick moves right, then power_lf must be positive and power_rf must be negative, causing the robot to turn right.
        //If the stick moves left, then power_lf must be negative and power_rf must be positive, causing the robot to turn left.


        //Motor Power

        LeftMotor.setPower(power_lf);
        RightMotor.setPower(power_rf);


        /// buttons
        // //backward with a
        //         if(gamepad1.a){
        //         LeftMotor.setPower(-0);
        //         RightMotor.setPower(-0.1);
        //         } else {
        //            LeftMotor.setPower(0);
        //            RightMotor.setPower(0);
        //      }
        //
        //      // forward with y
        //      if(gamepad1.y){
        //         LeftMotor.setPower(0.1);
        //         RightMotor.setPower(0.1);
        //      } else {
        //        LeftMotor.setPower(0);
        //        RightMotor.setPower(0);
        //      }
        //
        //       //turn left with x
        //         if(gamepad1.x){
        //            LeftMotor.setPower(-0.1);
        //            RightMotor.setPower(0.1);
        //         } else {
        //            LeftMotor.setPower(0);
        //            RightMotor.setPower(0);
        //         }
        //
        //       // turn right with b
        //         if(gamepad1.b) {
        //            LeftMotor.setPower(0.1);
        //            RightMotor.setPower(-0.1);
        //         } else {
        //            LeftMotor.setPower(0);
        //            RightMotor.setPower(0);
        //         }


        //Telemetry
        //telemetry.addData("Front Left Motor EncPos", ": " + String.valueOf(LeftMotor.getCurrentPosition()) + ", Power=" + String.valueOf(LeftMotor.getPower()));
        //telemetry.addData("Front Right Motor EncPos", ": " + String.valueOf(RightMotor.getCurrentPosition()) + ", Power=" + String.valueOf(RightMotor.getPower()));
        telemetry.addData("Status", "Initialized");
        ///telemetry.addLine("booleanExample is true.");

    }//looop bracket

    @Override public void stop() {

    }


    //ALL FUNCTIONS GO HERE!


    /// Return current robot heading based on gyro/IMU reading


}

/// Code to run when the op mode is first disabled goes here

