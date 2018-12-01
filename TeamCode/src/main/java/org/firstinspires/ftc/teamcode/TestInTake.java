/**
 * Put all common hardware configuration here
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.lynx.LynxI2cColorRangeSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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

@TeleOp(name="TestInTake", group="GG")
@Disabled
public class TestInTake extends OpMode {

   /// Main timer
   ElapsedTime timer_ = new ElapsedTime();

   /// Motors
   static final boolean USE_WHEELS = false;
   DcMotor motorRF_;
   DcMotor motorRB_;
   DcMotor motorLF_;
   DcMotor motorLB_;

   //static final boolean USE_SWEEPER = true;
   static final boolean USE_SWEEPER = false;
   DcMotor motorSweeper_;

   static final boolean USE_LIFT = false;
   //static final boolean USE_LIFT = false;
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

   static final boolean USE_GLYPH_WHEELS = true;             // use fly wheels to push/take glyphs
   DcMotor motor_glyph_left_;
   DcMotor motor_glyph_right_;
   static final double  GLYPH_LEFT_PUSH_POWER = 1.0;
   static final double  INTAKE_POWER_DFLT = 0.5;
   double power_intake_ = INTAKE_POWER_DFLT;                 // intake motor power

   /// Servos
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
   public TestInTake() {
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
         power_intake_ = INTAKE_POWER_DFLT; 
      }

      if( USE_INTAKE_SERVO ) {
         servo_left_intake_ = hardwareMap.servo.get("servo_left_intake");
         servo_right_intake_ = hardwareMap.servo.get("servo_right_intake");
      }
      servo_left_intake_pos_ = SERVO_LEFT_INTAKE_INIT;
      servo_right_intake_pos_ = SERVO_RIGHT_INTAKE_INIT;

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
   }

   /// This method will be called repeatedly in a loop
   @Override public void loop () { 
      double power_gl = 0;

      if( USE_GLYPH_WHEELS ) {
         if( gamepad1.a ) {
            power_gl = GLYPH_LEFT_PUSH_POWER;
         } else if (gamepad1.b) {
            power_gl = -GLYPH_LEFT_PUSH_POWER; 
         }

         double lsy = -gamepad1.left_stick_y;   // throttle
         if( Math.abs(lsy)>0.3 ) {
            if( lsy>0 ) {  // push
               power_gl = Math.abs(lsy)*GLYPH_LEFT_PUSH_POWER; 
            } else {  // pull, intake
               power_gl = -1.0*Math.abs(lsy)*GLYPH_LEFT_PUSH_POWER; 
            }
         } 

         double t = timer_.time();
         if( t>last_button_time_+0.2 ) {
            if( gamepad1.x && power_intake_<=0.9 ) {
               power_intake_ += 0.1;
               last_button_time_ = t;
            } else if( gamepad1.y && power_intake_>=0.1 ) {
               power_intake_ -= 0.1;
               last_button_time_ = t;
            }
         } 

         if( last_button_time_>0.0 ) {
            power_intake_ = Range.clip(power_intake_,0,1);
            if( power_gl>0 ) {
               power_gl = power_intake_; 
            } else {
               power_gl = -1.0*power_intake_; 
            }
         }
         
         power_gl = Range.clip(power_gl, -1, 1);
         motor_glyph_left_.setPower(power_gl);
         motor_glyph_right_.setPower(-power_gl);

         telemetry.addData("IntakeMotorPower", String.format("left=%.2f/%.2f, right=%.2f/%.2f; power_intake_=%.2f",power_gl,motor_glyph_left_.getPower(),-power_gl,motor_glyph_right_.getPower(),power_intake_));
         
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
