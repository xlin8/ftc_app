/**
 * Put all common hardware configuration here
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
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

//import com.qualcomm.robotcore.util.Range;


/**
 * Common codes 
 */

//HUB 2 = SIDE HUB
//HUB 3 = BOTTOM HUB
public class Y18CommonCrane extends OpMode {

   int loop_cnt_ = 0;

   /// TIMER
   ElapsedTime timer_ = new ElapsedTime();

   /// DRIVE MOTORS
   DcMotor motorLF_; //port 0 HUB3
   DcMotor motorLB_; //port 1
   DcMotor motorRF_; //port 2
   DcMotor motorRB_; //port 3

   static final double  CR_SERVO_STOP = 0.5;

   /// Crane motors
   static boolean RUN_CRANE_ENC_POS = true;                 // use RUN_TO_POSITION if true
   static boolean ADAPTIVE_CRANE_POWER = true;              // adaptive to adjust the power
   //static final double CRANE_POS_POWER = 0.3;
   static final double CRANE_POS_POWER = 0.4;               // YJ188 motor direct drive

   static boolean USE_CRANE = true;
   static final double CRANE_DOWN_POWER = -1.0;
   static final double CRANE_UP_POWER = 1.0;
   static final int    MAX_CRANE_ENC_COUNT = 1400;         // NR60
   //static final int    MAX_CRANE_ENC_COUNT = 2000;         // YJ188, gear box broken after <50 lifts, can't be used, fall back to double GB53, 2019/01/31 
   static final int    MIN_CRANE_ENC_COUNT = 0;            // min lift encoder count to prevent reverse
   static final int    CRANE_COLLECT_POS = 0;              // min lift encoder count to prevent reverse
   //static final int    CRANE_UP_POS = 1050;                // min lift encoder count to prevent reverse, almost vertical, NR60
   //static final int    CRANE_DUMP_POS = 1150;              // min lift encoder count to prevent reverse
   //static final int    CRANE_UP_POS = 1600;                // min lift encoder count to prevent reverse, almost vertical
   //static final int    CRANE_DUMP_POS = 1800;              // min lift encoder count to prevent reverse
   static final int    CRANE_UP_POS = 850;                // 
   static final int    CRANE_DUMP_POS = 1050;             //

   DcMotor motor_crane_;
   double power_crane_ = 0;
   static boolean USE_CRANE_DOUBLE = true;                // use two motor to drive crane to ensure enough torque; YJ188
   DcMotor motor_crane2_; 
   static final boolean USE_ONE_STAGE_LIFT = false;       // lift main arm directly to dump position if true for max efficiency 

   static final int    CRANE_DUMP_POS_DEPOT = 700;        // dump position for depot side
   static final boolean USE_SWEEPER_FOR_DUMPING = false;   // sweep out to help dumping on depot side

    static final int    CRANE_RESET_POS = -300;            // manually reset arm
    static final double CRANE_RESET_POWER = 0.3;
    static final double CRANE_RESET_TIME = 1.0;            // reset

    /// Sweeper motors
   static final boolean AUTO_SWEEPER = true;              // automatically start the sweeper when the arm is lowered
   static final int MIN_CRANE_ENC_START_SWEEPER = 100;    // 

   static boolean USE_SWEEPER = true;
   static final double SWEEP_IN_POWER = -1.0;
   static final double SWEEP_OUT_POWER = -SWEEP_IN_POWER;
   DcMotor motor_sweeper_;
   double power_sweeper_ = 0;

   static boolean USE_CRANE_ARM = true;
   Servo servo_crane_arm_;
   double servo_crane_arm_pos_ =0.0; 
   static double CRANE_ARM_COLLECT = 0.12;
   static double CRANE_ARM_DUMP = 0.44;        // dump position for crater side
   static double CRANE_ARM_DUMP_DEPOT = 0.62;  // dump position for depot side
   static double CRANE_ARM_INIT = 0.34;
   static boolean TIMED_CRANE_ARM_DUMP = false;   // hold the main arm till the slide is ready
   static double CRANE_ARM_DUMP_HOLD = CRANE_ARM_COLLECT ;
   static double CRANE_ARM_DUMP_TIME = 0.5;      // seconds to dump
   static double CRANE_ARM_DUMP_ENC_RATIO = 0.90;      // ratio to start dump
   double crane_arm_dump_start_t = 0.0;

   static boolean USE_CRANE_WINCH = true;
   Servo servo_crane_winch_;
   double servo_crane_winch_pos_ =0.0; 
   //static double CRANE_WINCH_INIT = 0.0;
   //static double CRANE_WINCH_EXTEND = 0.50;         // winch 1.5dia, 4.72in/120cm per rotation; 20/4.72=4.23 rotation, 4.23/8=0.53
   //static double CRANE_WINCH_MAX_EXTEND = 0.53;     // winch 1.5dia, 4.72in/120cm per rotation; 20/4.72=4.23 rotation, 4.23/8=0.53
   static double CRANE_WINCH_INIT = CR_SERVO_STOP;    // switch to REV smart servo, 2019/02/10
   static double CRANE_WINCH_EXTEND = 1.0;            
   static double CRANE_WINCH_HOLD_ENC = 400;
   static double CRANE_WINCH_HOLD = 0.60;


   /// LIFT
   static final boolean USE_LIFT = true;
   DcMotor motorLift_; //port 3 on HUB2 (motor7)
   double power_lift_ = 0;
   static final double LIFT_DOWN_POWER = -1.0;
   static final double LIFT_UP_POWER = 1.0;
   static final double  LIFT_ENC_COUNT_PER_GLYPH = 750;    // 750 for 6inch, new NeverRest 3.7 motor
   static final double  MAX_LIFT_ENC_COUNT = LIFT_ENC_COUNT_PER_GLYPH*3+200;    // 3 glyphs + extra room
   static final double  MIN_LIFT_ENC_COUNT = 0;            // min lift encoder count to prevent reverse
   static final double  LIFT_DIP_ENC_COUNT = LIFT_ENC_COUNT_PER_GLYPH/15;  // max lift encoder dip allowed; auto-hold OFF if set to 0
   static final double  MIN_LIFT_HOLD_POWER = 0.05;        // min power to hold the lift
   static final double  MIN_LIFT_POWER = 0.0;


   Servo  servo_lift_pin_;                                 // Servo for lift pin
   double servo_lift_pin_pos_;                            // Lift pin servo position
   //static final double  LIFT_PIN_STOP = 0.0;              // INIT position, switch to 180-servo, 2018/11/30
   //static final double  LIFT_PIN_PULL = 1-LIFT_PIN_STOP;
   static final double  LIFT_PIN_STOP = 0.0;              // INIT position, switch to 180-servo, 2018/11/30
   static final double  LIFT_PIN_PULL = 0.75; 

   static final boolean USE_STAB_WHEELS = true; 
   Servo  servo_stab_wheel_;                                // Servo for hold/release the back stabilization wheels
   static final double  STAB_WHEEL_INIT = 0.5;              // INIT position, hold the wheels
   static final double  STAB_WHEEL_RELEASE = 0.4;           // release the wheels

   // Main arm holding servo
   Servo servoArmHolder_;
   static final boolean USE_SERVO_ARM_HOLDER = true;
   static final double SERVO_ARM_HOLD_POS = 0.2;
   static final double SERVO_ARM_RELEASE_POS = 0.9;

   //CR SERVO STOP


   /////***************************** SENSORS *************************************/////

   /// IMU
   static final boolean USE_IMU  = true;           // use IMU sensor to turn
   BNO055IMU imu_;                         // Adafruit or RevHub IMU
   BNO055IMU.Parameters imu_parameters = new BNO055IMU.Parameters();
   Orientation imu_angles_;
   //Acceleration imu_gravity_;
   double heading_ = 0.0;                                  // current heading



   /////***************************** JOY STICKS *************************************/////

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
   double battery_voltage_=0.0;     // battery voltage



   ///  Constructor
   public Y18CommonCrane() {
   }


   ///  Code to run when the op mode is initialized goes here
   @Override public void init() {
      int loop_cnt_ = 0; 

      if(USE_CRANE) {
         //motor_crane_ = hardwareMap.dcMotor.get("motor_crane");
         motor_crane_ = hardwareMap.dcMotor.get("motorMineralsFlip1");
         motor_crane_.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
         motor_crane_.setMode( DcMotor.RunMode.STOP_AND_RESET_ENCODER );
         motor_crane_.setDirection(DcMotor.Direction.REVERSE);  // flip encoder count sign; need for NR60 and GF53; NO NEED for direct drive
         if( RUN_CRANE_ENC_POS ) motor_crane_.setTargetPosition(0);

         if (USE_SERVO_ARM_HOLDER) {
            servoArmHolder_ = hardwareMap.servo.get("servoArmHolder");
            servoArmHolder_.setPosition(SERVO_ARM_HOLD_POS);
         }

         if( USE_CRANE_DOUBLE ) {
            //motor_crane2_ = hardwareMap.dcMotor.get("motor_crane2");
            motor_crane2_ = hardwareMap.dcMotor.get("motorMineralsFlip2");
            motor_crane2_.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor_crane2_.setMode( DcMotor.RunMode.STOP_AND_RESET_ENCODER );
            motor_crane2_.setDirection(DcMotor.Direction.REVERSE);  // flip encoder count sign, need for NR60 and GF53
            if( RUN_CRANE_ENC_POS ) motor_crane2_.setTargetPosition(0);
         }
      }
      if( USE_CRANE_ARM ) {
         //servo_crane_arm_ = hardwareMap.servo.get("servo_crane_arm");
         servo_crane_arm_ = hardwareMap.servo.get("servoDump");
         if( servo_crane_arm_!=null ) {
            servo_crane_arm_.setPosition(CRANE_ARM_INIT); 
            servo_crane_arm_pos_ = CRANE_ARM_INIT; 
         }
      }
      if( USE_CRANE_WINCH ) {
         //servo_crane_winch_ = hardwareMap.servo.get("servo_crane_winch");
         servo_crane_winch_ = hardwareMap.servo.get("servoExtention");
         if( servo_crane_winch_!=null ) {
            servo_crane_winch_.setPosition(CRANE_WINCH_INIT); 
            servo_crane_winch_pos_ = CRANE_WINCH_INIT; 
         }
      }

      if(USE_SWEEPER) {
         //motor_sweeper_ = hardwareMap.dcMotor.get("motor_sweeper");
         motor_sweeper_ = hardwareMap.dcMotor.get("motorIntake");
         motor_sweeper_.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
         motor_sweeper_.setMode( DcMotor.RunMode.STOP_AND_RESET_ENCODER );
      }
      power_sweeper_ = 0.0; 

      if(USE_LIFT) {
         //motorLift_ = hardwareMap.dcMotor.get("motor_lift");
         motorLift_ = hardwareMap.dcMotor.get("motorLift");
         motorLift_.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
         power_lift_ = 0.0;

         //servo_lift_pin_ = hardwareMap.servo.get("servo_lift_pin_");
         servo_lift_pin_ = hardwareMap.servo.get("servoLiftPin");
         servo_lift_pin_pos_ = LIFT_PIN_STOP ; 
      } 

      if( USE_STAB_WHEELS ) {
         servo_stab_wheel_ = hardwareMap.servo.get("servoLittleStabWheels_");
         servo_stab_wheel_.setPosition( STAB_WHEEL_INIT );
      } 


      /// Use the hardwareMap to get the dc motors and servos by name.
      motorLF_ = hardwareMap.dcMotor.get("motor1");
      motorLB_ = hardwareMap.dcMotor.get("motor2");
      motorRF_ = hardwareMap.dcMotor.get("motor3");
      motorRB_ = hardwareMap.dcMotor.get("motor4");

      // Reverse motor, required for RevHub for NR40
      // NOT needed for NR20 used by new robot, 2018/03/20
      //motorLF_.setDirection(DcMotor.Direction.REVERSE);
      //motorLB_.setDirection(DcMotor.Direction.REVERSE);
      //motorRF_.setDirection(DcMotor.Direction.REVERSE);
      //motorRB_.setDirection(DcMotor.Direction.REVERSE);

      motorLF_.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      motorLB_.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      motorRF_.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      motorRB_.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);




      /////***************************** JOY STICKS *************************************/////

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
      loop_cnt_ = 0;
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


} 
