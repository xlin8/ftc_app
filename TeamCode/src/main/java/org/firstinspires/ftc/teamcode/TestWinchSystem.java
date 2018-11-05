/**
 * Put all common hardware configuration here
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.lynx.LynxI2cColorRangeSensor;
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

@TeleOp(name="TestWinch", group="GG")
//@Disabled
public class TestWinchSystem extends OpMode {

   /// Main timer
   ElapsedTime timer_ = new ElapsedTime();

   /// Motors
   static final boolean USE_WHEELS = false;
   DcMotor motorRF_;
   DcMotor motorRB_;
   DcMotor motorLF_;
   DcMotor motorLB_;

   static final double  MIN_BUTTON_INTERVAL = 0.3;


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

   /// ---------------------------  Servos ------------------------------  ///
   static final double  CR_SERVO_STOP = 0.5;

   static final boolean USE_WINCH = true;
   Servo servo_left_winch;
   Servo servo_right_winch;
   double servo_left_winch_pos;
   double servo_right_winch_pos;

   static final double SERVO_LEFT_WINCH_LOWER_SPEED = 0.40; //lol remember these are CONTINUOUS YOU
   static final double SERVO_RIGHT_WINCH_LOWER_SPEED = 0.60;


   // Robot 1 Clamp
   static final boolean USE_CLAW = false;             // use fly wheels to push/take glyphs
   Servo servo_left_jaw_;                                       // servo aligned with Adafruit RGB
   Servo servo_right_jaw_;                                      // servo aligned with MR RGB
   Servo servo_left_jaw2_;                                       // servo aligned with Adafruit RGB
   Servo servo_right_jaw2_;                                      // servo aligned with MR RGB
   double servo_left_jaw_pos_;                                  // Ada servo position
   double servo_right_jaw_pos_;                                 // MR servo position
   double servo_left_jaw2_pos_;                                  // Ada servo position
   double servo_right_jaw2_pos_;                                 // MR servo position
   double servo_kicker_pos_;                              // glyph kicker position
   double servo_pusher_pos_;                              // glyph pusher position

   //kicker and pusher
   static final boolean USE_KICKER = false;
   Servo servo_kicker_;
   static final double  KICKER_DOWN = 0.72;
   static final double  KICKER_INIT = KICKER_DOWN ;
   static final boolean USE_PUSHER = false;
   Servo servo_pusher_;

   //jewel module
   static final boolean USE_JEWEL = false;             // use fly wheels to push/take glyphs
   Servo servo_jewel_;
   Servo servo_knock_;
   static final double SERVO_JEWEL_INIT = 0.9;
   static final double SERVO_KNOCK_INIT = 0.3;

   //ramp clamp
   static final boolean USE_RAMP_CLAMP = false;            // use servo to clamp glyph
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

   //ramp servos
   static final boolean USE_RAMP = false;                 // use servo to control fly wheels
   Servo servo_left_ramp_;                                // left intake servo
   Servo servo_right_ramp_;                               // right intake servo
   double servo_left_ramp_pos_;                           // Left intake servo position
   double servo_right_ramp_pos_;                          // right intake servo position
   static final double  HITEC_DIGITAL_SERVO_MAX = 0.9;
   static final double  HITEC_DIGITAL_SERVO_MIN = 1-HITEC_DIGITAL_SERVO_MAX;
   static final double  LEFT_RAMP_INIT = CR_SERVO_STOP;
   static final double  LEFT_RAMP_DROP = 0.75;             // Hitec2645 digital, limit the power
   static final double  LEFT_RAMP_RISE = 0.15;
   static final double  RIGHT_RAMP_INIT = CR_SERVO_STOP;
   static final double  MAX_RAMP_LIFT_ENC_COUNT = LIFT_ENC_COUNT_PER_GLYPH*1.8;   // 1 glyph => 6in; 2.2x => 13.2in; max 14.5in
   static final double  RAMP_LIFT_R1_ENC = LIFT_ENC_COUNT_PER_GLYPH*1.0;          // put the glyph at 2nd row
   static final double  RAMP_LIFT_R2_ENC = LIFT_ENC_COUNT_PER_GLYPH*1.6;          // put the glyph at 3rd row




/****************----------------------SENSORS--------------------------*********************8*/
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
   static final double MIN_DIST_GLYPH = 10.0;        // min distance for any glypy
   static final double MAX_DIST_GLYPH_SEEN_BY_REV = 20.0;   // assume

   //static final boolean ENABLE_AUTO_GRAB = true;        // enable AUTO-GRAB
   static final boolean ENABLE_AUTO_GRAB = false;        // enable AUTO-GRAB

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




   ///-------------------------------Joy sticks----------------------------------///
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



   /// ------------------------ Other variables: time, battery voltage -------------------- ///
   double curr_time_;
   /// Battery voltage
   double battery_voltage_=0.0;     // battery voltage





   /////////////////////////////************   Constructor   **********////////////////////////////
   public TestWinchSystem() {
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

      if(USE_WINCH) {
         servo_left_winch = hardwareMap.servo.get("servo_left_winch");
         servo_right_winch = hardwareMap.servo.get("servo_right_winch");
      }
      servo_left_winch_pos = CR_SERVO_STOP;
      servo_right_winch_pos = CR_SERVO_STOP;

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

      if(USE_WINCH){

         if( x1_cnt_%4==0 ) {
            servo_left_winch_pos = CR_SERVO_STOP;//stop
            servo_right_winch_pos = CR_SERVO_STOP;
         } else if (x1_cnt_%4==1){
            servo_left_winch_pos = 0.0;   ///2.0 maybe 1.7..? 0.3 stop 4.0 second stage
            servo_right_winch_pos = 1.0;
         } else if (x1_cnt_%4==2){
            servo_left_winch_pos = CR_SERVO_STOP;//stop
            servo_right_winch_pos = CR_SERVO_STOP;
         } else if (x1_cnt_%4==3){
            servo_left_winch_pos = 1.0;
            servo_right_winch_pos = 0.0;  ///4.0
         }

         if (gamepad1.b){
            servo_left_winch_pos = CR_SERVO_STOP;
            servo_right_winch_pos = CR_SERVO_STOP;
         }
   if(servo_left_winch_pos == CR_SERVO_STOP) {
   if (y1_cnt_ % 4 == 0) {
      servo_left_winch_pos = CR_SERVO_STOP;
      servo_right_winch_pos = CR_SERVO_STOP;
   } else if (y1_cnt_ % 4 == 1) {//resetting intake
      //raise at full speed
      servo_left_winch_pos = 0.0;
      servo_right_winch_pos = 1.0;
   } else if (y1_cnt_ % 4 == 2) {
      servo_left_winch_pos = CR_SERVO_STOP;
      servo_right_winch_pos = CR_SERVO_STOP;
   } else if (y1_cnt_ % 4 == 3) {
      servo_left_winch_pos = 0.7;
      servo_right_winch_pos = 0.3;
      //half for stage 1 lowering. full for stage 2 lowering
      // LOWERING left = 0.6 right = 0.4
      //stage 1 lifting power: L:0.3 R:0.7 1.0s?
      //stage 2 lowering power: L:1.0 R:0.0 5.0?
   }
}

         if( gamepad1.left_bumper ) {
            servo_left_winch_pos = SERVO_LEFT_WINCH_LOWER_SPEED; //lift
            servo_right_winch_pos = SERVO_RIGHT_WINCH_LOWER_SPEED;//lift
         } else if (gamepad1.right_bumper){
            servo_left_winch_pos = SERVO_RIGHT_WINCH_LOWER_SPEED; //lift
            servo_right_winch_pos = SERVO_LEFT_WINCH_LOWER_SPEED;//lift
         }

         double t = timer_.time();
         if( t>last_button_time_+0.2 ) {
            if (gamepad1.dpad_up) {
               servo_left_winch_pos += 0.1;
               servo_right_winch_pos -= 0.1;
               last_button_time_ = t;
            } else if (gamepad1.dpad_down) {
               servo_left_winch_pos -= 0.1;
               servo_right_winch_pos += 0.1;
               last_button_time_ = t;
            }
         }

         servo_left_winch_pos = Range.clip(servo_left_winch_pos,0.0,1.0);
         servo_right_winch_pos = Range.clip(servo_right_winch_pos,0.0,1.0);

         servo_left_winch.setPosition(servo_left_winch_pos);
         servo_right_winch.setPosition(servo_right_winch_pos);

         telemetry.addData("WinchServoPos", String.format("left=%.2f/%.2f, right=%.2f/%.2f",servo_left_winch_pos,servo_left_winch.getPosition(),servo_right_winch_pos,servo_right_winch.getPosition()));


      }

      if( USE_RAMP ) {
         double t = timer_.time();
         if( t>last_button_time_+0.2 ) {
            if (gamepad1.dpad_up) {
               servo_left_ramp_pos_ += 0.1;
               last_button_time_ = t;
            } else if (gamepad1.dpad_down) {
               servo_left_ramp_pos_ -= 0.1;
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
