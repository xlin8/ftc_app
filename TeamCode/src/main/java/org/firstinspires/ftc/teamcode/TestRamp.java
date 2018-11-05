/**
 * Put all common hardware configuration here
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.lynx.LynxI2cColorRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
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

@TeleOp(name="TestRamp", group="GG")
//@Disabled
public class TestRamp extends OpMode {

   /// Main timer
   ElapsedTime timer_ = new ElapsedTime();

   /// Motors
   static final boolean USE_WHEELS = false;
   DcMotor motorRF_;
   DcMotor motorRB_;
   DcMotor motorLF_;
   DcMotor motorLB_;

   static final double  MIN_BUTTON_INTERVAL = 0.3;

   //static final boolean USE_SWEEPER = true;
   static final boolean USE_SWEEPER = false;
   DcMotor motorSweeper_;

   static final boolean USE_LIFT = false;
   //static final boolean USE_LIFT = true;
   DcMotor motorLift_;
   double power_lift = 0;
   static final double LIFT_DOWN_POWER = -1.0;
   static final double LIFT_UP_POWER = 1.0;
   static final double  LIFT_ENC_COUNT_PER_GLYPH = 750;    // 750 for 6inch, new NeverRest 3.7 motor
   static final double  MAX_LIFT_ENC_COUNT = LIFT_ENC_COUNT_PER_GLYPH*3+200;    // 3 glyphs + extra room
   static final double  MIN_LIFT_ENC_COUNT = 0;            // min lift encoder count to prevent reverse
   static final double  LIFT_DIP_ENC_COUNT = LIFT_ENC_COUNT_PER_GLYPH/15;  // max lift encoder dip allowed; auto-hold OFF if set to 0
   static final double  MIN_LIFT_HOLD_POWER = 0.05;        // min power to hold the lift
   static final double  MIN_LIFT_POWER = 0.0;

   //static final boolean USE_GLYPH_WHEELS = true;             // use fly wheels to push/take glyphs
   static final boolean USE_GLYPH_WHEELS = false;             // use fly wheels to push/take glyphs
   DcMotor motor_glyph_left_;
   DcMotor motor_glyph_right_;
   static final double  GLYPH_LEFT_PUSH_POWER = 1.0;

   /// Servos
   //static final boolean USE_INTAKE_SERVO = true;             // use servo to control fly wheels 
   static final boolean USE_INTAKE_SERVO = false;             // use servo to control fly wheels 
   Servo servo_left_intake_;                                 // left intake servo
   Servo servo_right_intake_;                                // right intake servo
   double servo_left_intake_pos_;                            // Left intake servo position
   double servo_right_intake_pos_;                           // right intake servo position
   static final double SERVO_LEFT_INTAKE_INIT = 0.7;         // folded for init
   static final double SERVO_LEFT_INTAKE_GRAB = 0.5;         // grabbing, intake
   static final double SERVO_LEFT_INTAKE_OPEN = 0.3;         // point forward, pusher
   static final double SERVO_LEFT_INTAKE_SHELF = 0.0;        // fold back when shelfing
   static final double SERVO_RIGHT_INTAKE_INIT = 0.3;
   static final double SERVO_RIGHT_INTAKE_GRAB = 0.6;
   static final double SERVO_RIGHT_INTAKE_OPEN = 0.8;
   static final double SERVO_RIGHT_INTAKE_SHELF = 1.0;

   static final boolean USE_CLAW = false;             // use fly wheels to push/take glyphs
   Servo servo_left_jaw_;                                       // servo aligned with Adafruit RGB
   Servo servo_right_jaw_;                                      // servo aligned with MR RGB
   Servo servo_left_jaw2_;                                       // servo aligned with Adafruit RGB
   Servo servo_right_jaw2_;                                      // servo aligned with MR RGB

   static final boolean USE_KICKER = false;
   Servo servo_kicker_;                                   // servo aligned with MR RGB
   static final boolean USE_PUSHER = false;
   Servo servo_pusher_;                                   // servo aligned with MR RGB

   double servo_left_jaw_pos_;                                  // Ada servo position
   double servo_right_jaw_pos_;                                 // MR servo position
   double servo_left_jaw2_pos_;                                  // Ada servo position
   double servo_right_jaw2_pos_;                                 // MR servo position
   double servo_kicker_pos_;                              // glyph kicker position
   double servo_pusher_pos_;                              // glyph pusher position

   static final boolean USE_JEWEL = false;             // use fly wheels to push/take glyphs
   Servo servo_jewel_;
   Servo servo_knock_;

   /// Servo Variables
   static final double  CR_SERVO_STOP = 0.5;
   static final double  LEFT_JAW_OPEN = 1.0;
   static final double  LEFT_JAW_OPEN_SLOW = 0.7;
   static final double  LEFT_JAW_CLOSE = 0.0;
   static final double  LEFT_JAW_CLOSE_HOLD = 0.25;
   static final double  RIGHT_JAW_OPEN = 1.0;
   static final double  RIGHT_JAW_OPEN_SLOW = 0.7;
   static final double  RIGHT_JAW_CLOSE = 0.0;
   static final double  RIGHT_JAW_CLOSE_HOLD = 0.25;
   static final double  PUSHER_PUSH = 1.0;
   static final double  PUSHER_PULL = 0.0;
   static final double  PUSHER_PULL_HOLD = 0.25;

   static final double  KICKER_DOWN = 0.72;
   static final double  KICKER_UP = 0.50;
   static final double  KICKER_UP_MAX = 0.40;
   static final double  KICKER_INIT = KICKER_DOWN ;

   static final double SERVO_JEWEL_INIT = 0.9;
   static final double SERVO_JEWEL_DOWN_POS = 0.35;
   static final double SERVO_KNOCK_INIT = 0.3;
   static final double SERVO_KNOCK_FRONT_POS = 0.4;                     // front = away from pictograph, back = toward it
   static final double SERVO_KNOCK_BACK_POS = 0.2;




   static final boolean USE_RAMP_CLAMP = true;            // use servo to clamp glyph
   Servo servo_front_clamp_;                              // front ramp clamp servo
   Servo servo_back_clamp_;                               // back ramp clamp servo
   double servo_front_clamp_pos_;                         // front clamp servo position
   double servo_back_clamp_pos_;                          // back clamp servo position
   static final double  FRONT_CLAMP_INIT = 0.22;          // initial position to guide glyph in
   static final double  FRONT_CLAMP_GRAB = 0.14;          // grab the glyph when rising the ramp
   static final double  FRONT_CLAMP_RELEASE = 0.24;       // release the glyph to let it fall, parallel to the ramp rail
   static final double  BACK_CLAMP_INIT = 0.26;           // initial position to guide glyph in
   static final double  BACK_CLAMP_GRAB = 0.48;           // grab the glyph when rising the ramp
   static final double  BACK_CLAMP_RELEASE = 0.34;         // release the glyph to let it fall, parallel to the ramp rail

    /////ramp clamp 2..?
    double servo_front_slide_pos;
    double servo_back_slide_pos;
    static final double FRONT_SLIDE_INIT = 0.0;
    static final double BACK_SLIDE_INIT = 0.0;
    static final double FRONT_SLIDE_HOLD = 0.0;
    static final double BACK_SLIDE_HOLD = 0.0;




   static final boolean USE_RAMP = true;                 // use servo to control fly wheels 
   Servo servo_left_ramp_;                                // left intake servo
   Servo servo_right_ramp_;                               // right intake servo
   double servo_left_ramp_pos_;                           // Left intake servo position
   double servo_right_ramp_pos_;                          // right intake servo position
   static final double  HITEC_DIGITAL_SERVO_MAX = 0.9; 
   static final double  HITEC_DIGITAL_SERVO_MIN = 1-HITEC_DIGITAL_SERVO_MAX; 
   static final double  LEFT_RAMP_INIT = CR_SERVO_STOP;  
   static final double  LEFT_RAMP_DROP = HITEC_DIGITAL_SERVO_MAX; //0.75             // Hitec2645 digital, limit the power
   //static final double  LEFT_RAMP_RISE = 1-LEFT_RAMP_DROP;
   static final double  LEFT_RAMP_RISE = HITEC_DIGITAL_SERVO_MIN; //0.15
   static final double  RIGHT_RAMP_INIT = CR_SERVO_STOP;  
   static final double  RIGHT_RAMP_DROP = LEFT_RAMP_RISE;
   static final double  RIGHT_RAMP_RISE = LEFT_RAMP_DROP; 
   static final double  MAX_RAMP_LIFT_ENC_COUNT = LIFT_ENC_COUNT_PER_GLYPH*1.8;   // 1 glyph => 6in; 2.2x => 13.2in; max 14.5in
   static final double  RAMP_LIFT_R0_ENC = LIFT_ENC_COUNT_PER_GLYPH*0.4;          // put the glyph at ground level, rise ramp up a little to avoid the back bar
   static final double  RAMP_LIFT_R1_ENC = LIFT_ENC_COUNT_PER_GLYPH*1.0;          // put the glyph at 2nd row
   static final double  RAMP_LIFT_R2_ENC = LIFT_ENC_COUNT_PER_GLYPH*1.6;          // put the glyph at 3rd row
   static final double  RAMP_LIFT_R3_ENC = MAX_RAMP_LIFT_ENC_COUNT;               // put the glyph at 4th/top row

   /// IMU
   static final boolean USE_IMU  = false;           // use IMU sensor to turn 
   //static final boolean USE_IMU  = false;           // use IMU sensor to turn 
   BNO055IMU imu_;                         // Adafruit or RevHub IMU 
   BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
   Orientation imu_angles_;
   //Acceleration imu_gravity_;
   double heading_ = 0.0;                                  // current heading 


   /// REV range sensors for auto-align/auto-grab
   static final double MAX_DISTANCE = 1000.0;
   static final double MAX_DIST_GLYPH_MR = 25.0;    // 30cm(12in), beam gap, for MR range; max value 28cm
   static final double MIN_DIST_GLYPH_GRAY = 6.0;   // min distance for gray glypy
   static final double MIN_DIST_GLYPH_BROWN = 8.0;  // min distance for brown glypy
   static final double MIN_DIST_GLYPH = 10.0;        // min distance for any glypy
   static final double MAX_DIST_GLYPH_SEEN_BY_REV = 20.0;   // assume

   //static final boolean ENABLE_AUTO_GRAB = true;        // enable AUTO-GRAB
   static final boolean ENABLE_AUTO_GRAB = false;        // enable AUTO-GRAB
   //static final boolean ENABLE_AUTO_GRAB_FOLLOW = true; // enable follow glyph during AUTO-GRAB
   static final boolean ENABLE_AUTO_GRAB_FOLLOW = false; // enable follow glyph during AUTO-GRAB
   static final boolean ENABLE_AUTO_GRAB_FOLLOW_DIAG = false; // enable diagonally follow glyph during AUTO-GRAB
   //static final double AUTO_GRAB_TURN_MAX_DEG = 20; // max angle to turn
   static final double AUTO_GRAB_TURN_MAX_DEG = 30; // max angle to turn
   static final double AUTO_GRAB_TURN_TIME = 1.0;   // max time to turn
  // static final double AUTO_GRAB_TURN_POWER = 0.3;   // max time to turn
   static final double AUTO_GRAB_TURN_POWER = 0.4;   // max time to turn
   static final double AUTO_GRAB_SHIFT_TIME = 1.0;  // max time to shift
   //static final double AUTO_GRAB_SHIFT_POWER = 0.2;  // max time to shift
   static final double AUTO_GRAB_SHIFT_POWER = 0.3;  // max time to shift
   LynxI2cColorRangeSensor rev_range_left_;         // left REV color/range sensor
   LynxI2cColorRangeSensor rev_range_right_;        // right color/range sensor
   double dist_left_ = 0.0;                         // distance value for left REV sensor
   double dist0_left_ = 0.0;                        // distance value for left REV sensor
   double alpha_left_ = 0.0;                        // alpha value for left REV sensor
   double dist_right_ = 0.0;                        // distance value for right REV sensor
   double dist0_right_ = 0.0;                       // distance value for right REV sensor
   double alpha_right_ = 0.0;                       // alpha value for right REV sensor

   static final boolean USE_MR_RANGE = false;       // sensor removed bcz it may cause crash, 2018/01/06
   //static final boolean USE_MR_RANGE = true;
   ModernRoboticsI2cRangeSensor mr_range_;          // MR range sensor 
   double dist_mr_range_ = 0.0; 
   double dist0_mr_range_ = 0.0; 

   double glyph_seen_t_ = 0.0;                      // glyph seen time
   double glyph_seen_heading_ = 0.0;                // heading when glyph is seen 
   boolean glyph_aligned_ = false;
   int    auto_grab_move_ = 0;                      // 0 stop, 1 turn left, 2 turn right, 3 shift left, 4 shift right
   int    prev_auto_grab_move_ = 0;                 // saved previous move

   /// Joy sticks
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

   /// Other variables
   double curr_time_; 

   /// Battery voltage
   double battery_voltage_=0.0;     // battery voltage 


   ///  Constructor
   public TestRamp() {
   }


   ///  Code to run when the op mode is initialized goes here
   @Override public void init() {
      /// Use the hardwareMap to get the dc motors and servos by name.
      if( USE_WHEELS ) {
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
      }

      if(USE_SWEEPER) {
         motorSweeper_ = hardwareMap.dcMotor.get("motor5");
         motorSweeper_.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      }
      if(USE_LIFT) {
         motorLift_ = hardwareMap.dcMotor.get("motor6");
         motorLift_.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      }
      if(USE_GLYPH_WHEELS) {
         motor_glyph_left_ = hardwareMap.dcMotor.get("motor5");
         motor_glyph_left_.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
         motor_glyph_right_ = hardwareMap.dcMotor.get("motor7");
         motor_glyph_right_.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      }

      if( USE_INTAKE_SERVO ) {
         servo_left_intake_ = hardwareMap.servo.get("servo_left_intake");
         servo_right_intake_ = hardwareMap.servo.get("servo_right_intake");
      }
      servo_left_intake_pos_ = SERVO_LEFT_INTAKE_INIT;
      servo_right_intake_pos_ = SERVO_RIGHT_INTAKE_INIT;

      if( USE_RAMP ) {
         servo_left_ramp_ = hardwareMap.servo.get("servo_left_ramp");
         servo_right_ramp_ = hardwareMap.servo.get("servo_right_ramp");
      }
      servo_left_ramp_pos_ = LEFT_RAMP_INIT;
      servo_right_ramp_pos_ = RIGHT_RAMP_INIT;

      if( USE_RAMP_CLAMP ) {
         servo_front_clamp_ = hardwareMap.servo.get("servo_front_clamp");
         servo_back_clamp_ = hardwareMap.servo.get("servo_back_clamp");
      }
      servo_front_clamp_pos_ = FRONT_CLAMP_INIT;
      servo_back_clamp_pos_ = BACK_CLAMP_INIT;

      if( USE_CLAW ) {
      servo_left_jaw_ = hardwareMap.servo.get("servo_left_jaw");
      servo_right_jaw_ = hardwareMap.servo.get("servo_right_jaw");
      servo_left_jaw2_ = hardwareMap.servo.get("servo_left_jaw2");
      servo_right_jaw2_ = hardwareMap.servo.get("servo_right_jaw2");

      servo_left_jaw_pos_ = CR_SERVO_STOP;
      servo_right_jaw_pos_ = CR_SERVO_STOP;
      servo_left_jaw2_pos_ = CR_SERVO_STOP;
      servo_right_jaw2_pos_ = CR_SERVO_STOP;

      servo_left_jaw_.setPosition(servo_left_jaw_pos_);
      servo_right_jaw_.setPosition(servo_right_jaw_pos_);
      servo_left_jaw2_.setPosition(servo_left_jaw2_pos_);
      servo_right_jaw2_.setPosition(servo_right_jaw2_pos_);
      }

      if( USE_KICKER ) {
         servo_kicker_ = hardwareMap.servo.get("servo_kicker");
         servo_kicker_.setPosition(KICKER_INIT);
      }
      servo_kicker_pos_ = KICKER_INIT; 
      if( USE_PUSHER ) {
         servo_pusher_ = hardwareMap.servo.get("servo_pusher");
         servo_pusher_.setPosition(CR_SERVO_STOP);
      }
      servo_pusher_pos_ = CR_SERVO_STOP; 

      if( USE_JEWEL ) {
         servo_jewel_ = hardwareMap.servo.get("servo_jewel");
         servo_knock_ = hardwareMap.servo.get("servo_knock");
         servo_jewel_.setPosition(SERVO_JEWEL_INIT);
         servo_knock_.setPosition(SERVO_KNOCK_INIT);
      }


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

      if( ENABLE_AUTO_GRAB ) {
         rev_range_left_ = hardwareMap.get(LynxI2cColorRangeSensor.class, "rev_range_left");
         rev_range_right_ = hardwareMap.get(LynxI2cColorRangeSensor.class, "rev_range_right");
      }

      if( USE_MR_RANGE ) {
         mr_range_ = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "mr_range");
      }

      power_lift = 0.0; 
      
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

      last_button_time2_ = 0.0;
      a2_cnt_ = 0;
      b2_cnt_ = 0;
      x2_cnt_ = 0;
      y2_cnt_ = 0;
      lb2_cnt_ = 0;
      rb2_cnt_ = 0;

   }


   /// This method will be called once before entering loop()
   @Override public void init_loop() {
      timer_.reset(); 
      curr_time_ = 0.0;

      if(USE_LIFT) motorLift_.setMode ( DcMotor.RunMode.RUN_USING_ENCODER );
   }

   /// This method will be called repeatedly in a loop
   @Override public void loop () { 
      curr_time_ = timer_.time(); 

      if( (curr_time_-last_button_time_) > MIN_BUTTON_INTERVAL) {
         if (gamepad1.x){
            x1_cnt_++;
            last_button_time_ = curr_time_;
         } else if (gamepad1.y) {
            y1_cnt_++;
            last_button_time_ = curr_time_;
         }else if (gamepad1.a) {
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
      }

      double power_gl = 0;
      power_lift = 0.0; 
      servo_left_ramp_pos_ = LEFT_RAMP_INIT; 
      servo_right_ramp_pos_ = RIGHT_RAMP_INIT; 

      if( USE_GLYPH_WHEELS ) {
         if( gamepad1.a ) {
            power_gl = GLYPH_LEFT_PUSH_POWER;
         } else if (gamepad1.b) {
            power_gl = -GLYPH_LEFT_PUSH_POWER; 
         }
         
         power_gl = Range.clip(power_gl, -1, 1);
         motor_glyph_left_.setPower(power_gl);
         motor_glyph_right_.setPower(-power_gl);
         
      } 
      if( USE_INTAKE_SERVO ) {
         double t = timer_.time();
         if( t>last_button_time_+0.2 ) {
            if (gamepad1.dpad_up) {
               servo_left_intake_pos_ += 0.1;
               last_button_time_ = t;
            } else if (gamepad1.dpad_down) {
               servo_left_intake_pos_ -= 0.1;
               last_button_time_ = t;
            } else if (gamepad1.dpad_left) {
               servo_right_intake_pos_ += 0.1;
               last_button_time_ = t;
            } else if (gamepad1.dpad_right) {
               servo_right_intake_pos_ -= 0.1;
               last_button_time_ = t;
            }
         }

         servo_left_intake_pos_ = Range.clip(servo_left_intake_pos_,0,1);
         servo_left_intake_.setPosition( servo_left_intake_pos_ ); 
         servo_right_intake_pos_ = Range.clip(servo_right_intake_pos_,0,1);
         servo_right_intake_.setPosition( servo_right_intake_pos_ ); 

         telemetry.addData("IntakeServoPos", String.format("left=%.2f, right=%.2f",servo_left_intake_.getPosition(),servo_right_intake_.getPosition()));
      }
      if( USE_RAMP ) {
         double t = timer_.time();
         if( t>last_button_time_+0.2 ) {
            if (gamepad1.dpad_up) {
               servo_left_intake_pos_ += 0.1;
               last_button_time_ = t;
            } else if (gamepad1.dpad_down) {
               servo_left_intake_pos_ -= 0.1;
               last_button_time_ = t;
            }
         }
         if( gamepad1.a || gamepad2.a ) {
            servo_left_ramp_pos_ = LEFT_RAMP_RISE; 
         } else if( gamepad1.b || gamepad2.b ) {
            servo_left_ramp_pos_ = LEFT_RAMP_DROP; 
         } 

         //servo_left_ramp_pos_ = Range.clip(servo_left_ramp_pos_,0.2,0.8);
         servo_left_ramp_pos_ = Range.clip(servo_left_ramp_pos_,HITEC_DIGITAL_SERVO_MIN,HITEC_DIGITAL_SERVO_MAX);
         servo_left_ramp_.setPosition( servo_left_ramp_pos_ ); 
         servo_right_ramp_pos_ = 1-servo_left_ramp_pos_; 
         //servo_right_ramp_pos_ = Range.clip(servo_right_ramp_pos_,0.2,0.8);
         servo_right_ramp_pos_ = Range.clip(servo_right_ramp_pos_,HITEC_DIGITAL_SERVO_MIN,HITEC_DIGITAL_SERVO_MAX);
         servo_right_ramp_.setPosition( servo_right_ramp_pos_ ); 

         telemetry.addData("RampServoPos", String.format("left=%.2f/%.2f, right=%.2f/%.2f",servo_left_ramp_pos_,servo_left_ramp_.getPosition(),servo_right_ramp_pos_,servo_right_ramp_.getPosition()));
      }

      if( USE_RAMP_CLAMP ) {
         double t = timer_.time();
         if( t>last_button_time_+0.2 ) {
            if (gamepad1.dpad_left) {
               if( gamepad1.right_trigger<0.5 ) {
                  servo_front_clamp_pos_ += 0.02;
               } else {
                  servo_back_clamp_pos_ += 0.02;
               }
               last_button_time_ = t;
            } else if (gamepad1.dpad_right) {
               if( gamepad1.right_trigger<0.5 ) {
                  servo_front_clamp_pos_ -= 0.02;
               } else {
                  servo_back_clamp_pos_ -= 0.02;
               }
               last_button_time_ = t;
            }
         }

         if( x1_cnt_%4==1 ) {
            servo_front_clamp_pos_ = FRONT_CLAMP_GRAB; 
            servo_back_clamp_pos_ = BACK_CLAMP_GRAB;
         } else if( x1_cnt_%4==2 ) {
            servo_front_clamp_pos_ = FRONT_CLAMP_RELEASE; 
            servo_back_clamp_pos_ = BACK_CLAMP_RELEASE; 
         } else if( x1_cnt_%4==3 ) {
            servo_front_clamp_pos_ = FRONT_CLAMP_INIT; 
            servo_back_clamp_pos_ = BACK_CLAMP_INIT; 
         }
         servo_front_clamp_pos_ = Range.clip(servo_front_clamp_pos_,0.0,1.0);
         servo_front_clamp_.setPosition( servo_front_clamp_pos_ ); 
         servo_back_clamp_pos_ = Range.clip(servo_back_clamp_pos_,0.0,1.0);
         servo_back_clamp_.setPosition( servo_back_clamp_pos_ ); 

         telemetry.addData("RampClampServoPos", String.format("front=%.2f/%.2f, back=%.2f/%.2f",servo_front_clamp_pos_,servo_front_clamp_.getPosition(),servo_back_clamp_pos_,servo_back_clamp_.getPosition()));
      }

      if( USE_LIFT ) { 
         int lift_enc = motorLift_.getCurrentPosition(); 
         boolean manual_ramp_control = false;

         if( gamepad1.dpad_up || gamepad2.dpad_up ) { // raise, auto-close claw
            power_lift = LIFT_UP_POWER;
            manual_ramp_control = true;
         } else if( gamepad1.dpad_down || gamepad2.dpad_down ) {  // lower
            power_lift = LIFT_DOWN_POWER;
            manual_ramp_control = true;
         }

         boolean lift_ramp_for_R0R1 = ((y1_cnt_%2)==1 || (y2_cnt_%2)==1);  // use Y to lift ramp for R0/R1
         boolean lift_ramp_for_R2 = ((x1_cnt_%2)==1 || (x2_cnt_%2)==1);    // use X for lift ramp for R2
         if( manual_ramp_control ) {
            x1_cnt_=0; x2_cnt_=0; 
            y1_cnt_=0; y2_cnt_=0; 
         } else if( !manual_ramp_control && (lift_ramp_for_R0R1 || lift_ramp_for_R2) ) {
            double tg_enc = lift_ramp_for_R2 ? RAMP_LIFT_R2_ENC :  RAMP_LIFT_R1_ENC ;
            if( lift_enc>tg_enc ) {
               power_lift=0.0; 
            } else if( lift_enc>tg_enc-LIFT_DIP_ENC_COUNT) {
               power_lift=MIN_LIFT_HOLD_POWER; 
            } else {
               power_lift=1.0; 
            }
         }

         if( power_lift>0.0 && lift_enc>MAX_RAMP_LIFT_ENC_COUNT ) {  // prevent over-extending
            power_lift = Range.clip(power_lift,-MIN_LIFT_POWER,MIN_LIFT_POWER); 
         } else if( power_lift<0.0 && lift_enc<MIN_LIFT_ENC_COUNT ) {   // prevent reverse
            power_lift = Range.clip(power_lift,-MIN_LIFT_POWER,MIN_LIFT_POWER); 
         } 
         motorLift_.setPower(power_lift);

         telemetry.addData("Lift EncPos", ": "+String.valueOf(motorLift_.getCurrentPosition())+", Power="+String.valueOf(motorLift_.getPower())); 
      } 
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
      dist_left_ = 0.0; 
      dist_right_ = 0.0; 
      dist_mr_range_= 0.0; 
      if( USE_MR_RANGE && mr_range_!=null ) {
         dist_mr_range_ = mr_range_.getDistance(DistanceUnit.CM);
         //if( Double.isNaN(dist_mr_range_) ) dist_mr_range_ = MAX_DISTANCE; 
         if( Double.isNaN(dist_mr_range_) ) dist_mr_range_ = 5.0;  // ultrasonic => optical
      }
      if( ENABLE_AUTO_GRAB ) {
         alpha_left_ = rev_range_left_.alpha();
         dist_left_ = rev_range_left_.getDistance(DistanceUnit.CM); 
         if( Double.isNaN(dist_left_) ) dist_left_ = MAX_DISTANCE; 
         alpha_right_ = rev_range_right_.alpha();
         dist_right_ = rev_range_right_.getDistance(DistanceUnit.CM); 
         if( Double.isNaN(dist_right_) ) dist_right_ = MAX_DISTANCE; 
      }
      return dist_mr_range_; 
   } 

   boolean isGlyphSeen() {
      boolean seen = false;
      getDistances(); 
      if( USE_MR_RANGE ) {
         seen = dist_mr_range_ < MAX_DIST_GLYPH_MR; 
      } else {
         seen = (dist_left_ < MAX_DIST_GLYPH_SEEN_BY_REV) || (dist_right_ < MAX_DIST_GLYPH_SEEN_BY_REV); 
      }
      return seen;
   }

   boolean isGlyphAligned() {
      return (dist_left_>0 && dist_left_<MIN_DIST_GLYPH && dist_right_>0 && dist_right_<MIN_DIST_GLYPH); 
   }

} 
