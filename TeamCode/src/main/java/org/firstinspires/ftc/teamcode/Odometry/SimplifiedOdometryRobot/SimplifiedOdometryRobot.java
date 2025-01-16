/* Created by Phil Malone. 2023.
    This class illustrates my simplified Odometry Strategy.
    It implements basic straight line motions but with heading and drift controls to limit drift.
    See the readme for a link to a video tutorial explaining the operation and limitations of the code.
 */

package org.firstinspires.ftc.teamcode.Odometry.SimplifiedOdometryRobot;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.util.MathUtil;
import org.firstinspires.ftc.teamcode.util.PIDController;

import java.util.List;


@Config
public class SimplifiedOdometryRobot {
   // Adjust these numbers to suit your robot.
   private final double ODOM_INCHES_PER_COUNT   = 0.00076698974609375;
   private final boolean INVERT_DRIVE_ODOMETRY  = false;       //  When driving FORWARD, the odometry value MUST increase.  If it does not, flip the value of this constant.
   private final boolean INVERT_STRAFE_ODOMETRY = false;       // DOUBLE CHECK!!!!!!!!!!! When strafing to the LEFT, the odometry value MUST increase.  If it does not, flip the value of this constant.

   private static double DRIVE_GAIN          = 0.052;    // Strength of axial position control
   private static final double DRIVE_ACCEL         = 1.2;     // Acceleration limit.  Percent Power change per second.  1.0 = 0-100% power in 1 sec.
   private static final double DRIVE_TOLERANCE     = 0.5;     // Controller is is "inPosition" if position error is < +/- this amount
   private static final double DRIVE_DEADBAND      = 0.2;     // Error less than this causes zero output.  Must be smaller than DRIVE_TOLERANCE
   private static final double DRIVE_MAX_AUTO      = 0.6;     // "default" Maximum Axial power limit during autonomous

   private static double STRAFE_GAIN         = 0.052;    // Strength of lateral position control
   private static final double STRAFE_ACCEL        = 1.1;     // Acceleration limit.  Percent Power change per second.  1.0 = 0-100% power in 1 sec.
   private static final double STRAFE_TOLERANCE    = 0.5;     // Controller is is "inPosition" if position error is < +/- this amount
   private static final double STRAFE_DEADBAND     = 0.1;     // Error less than this causes zero output.  Must be smaller than DRIVE_TOLERANCE
   private static final double STRAFE_MAX_AUTO     = 0.6;     // "default" Maximum Lateral power limit during autonomous

   private static double YAW_GAIN            = 0.0163;    // Strength of Yaw position control
   private static final double YAW_ACCEL           = 2.4;     // Acceleration limit.  Percent Power change per second.  1.0 = 0-100% power in 1 sec.
   private static final double YAW_TOLERANCE       = 1;     // Controller is is "inPosition" if position error is < +/- this amount
   private static final double YAW_DEADBAND        = 0.25;    // Error less than this causes zero output.  Must be smaller than DRIVE_TOLERANCE
   private static final double YAW_MAX_AUTO        = 0.6;     // "default" Maximum Yaw power limit during autonomous

   // Public Members
   public double driveDistance     = 0; // scaled axial distance (+ = forward)
   public double strafeDistance    = 0; // scaled lateral distance (+ = left)
   public double heading           = 0; // Latest Robot heading from IMU

   // Establish a proportional controller for each axis to calculate the required power to achieve a setpoint.
   public ProportionalControl driveController     = new ProportionalControl(DRIVE_GAIN, DRIVE_ACCEL, DRIVE_MAX_AUTO, DRIVE_TOLERANCE, DRIVE_DEADBAND, false);
   public ProportionalControl strafeController    = new ProportionalControl(STRAFE_GAIN, STRAFE_ACCEL, STRAFE_MAX_AUTO, STRAFE_TOLERANCE, STRAFE_DEADBAND, false);
   public ProportionalControl yawController       = new ProportionalControl(YAW_GAIN, YAW_ACCEL, YAW_MAX_AUTO, YAW_TOLERANCE,YAW_DEADBAND, true);

   public static double translationKp = 0.25;
   public static double translationKd = 0.001;


   public static double headingKP = 0.9;
   public static double headingKD = 0.02;


   public static double translationPositionTolerance = 0.75;
   public static double translationVelocityTolerance = 10.0;

    public static double headingVelocityTolerance= 10.0; // i really dont know about this one, if turn is weird fiddle with this

   public static double headingPositionTolerance=1.0;


   //15mm x probably close to zero for y
   public PIDFController xController = new PIDFController(translationKp, 0.0, 0.00, 0.0);
   public PIDFController yController = new PIDFController(translationKp, 0.0, 0.00, 0.0);
   public PIDController headingController = new PIDController(headingKP, 0.00, 0.00);

   // ---  Private Members

   SparkFunOTOS odo;

  public SparkFunOTOS.Pose2D pose= new SparkFunOTOS.Pose2D();

   // Hardware interface Objects
   public Motor LFMotor;     //  control the left front drive wheel
   public Motor RFMotor;    //  control the right front drive wheel
  public  Motor LBMotor;      //  control the left back drive wheel
   public Motor RBMotor;     //  control the right back drive wheel

   MecanumDrive  mecanumDrive;

   private DcMotor armExtension1;       //  the Axial (front/back) Odometry Module (may overlap with motor, or may not)
   private DcMotor armExtension2;      //  the Lateral (left/right) Odometry Module (may overlap with motor, or may not)

   private LinearOpMode myOpMode;
   //private IMU imu;
   private ElapsedTime holdTimer = new ElapsedTime();  // User for any motion requiring a hold time or timeout.

   public static double ks = 0.0;
   private SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(ks, 6.99, 0.98);


   public final double MAX_VELOCITY = 1.74;

   private int rawDriveOdometer    = 0; // Unmodified axial odometer count
   private int driveOdometerOffset = 0; // Used to offset axial odometer
   private int rawStrafeOdometer   = 0; // Unmodified lateral odometer count
   private int strafeOdometerOffset= 0; // Used to offset lateral odometer
   private double rawHeading       = 0; // Unmodified heading (degrees)
   private double headingOffset    = 0; // Used to offset heading

   private double turnRate           = 0; // Latest Robot Turn Rate from IMU
   private boolean showTelemetry     = false;

   // Robot Constructor
   public SimplifiedOdometryRobot(LinearOpMode opmode) {myOpMode = opmode;}


   /**
    * Robot Initialization:
    *  Use the hardware map to Connect to devices.
    *  Perform any set-up all the hardware devices.
    * @param showTelemetry  Set to true if you want telemetry to be displayed by the robot sensor/drive functions.
    */
   public void initialize(boolean showTelemetry)
   {
      // Initialize the hardware variables. Note that the strings used to 'get' each
      // motor/device must match the names assigned during the robot configuration.

      // !!!  Set the drive direction to ensure positive power drives each wheel forward.
      LFMotor  = setupMotor("LFMotor", DcMotor.Direction.FORWARD);
      RFMotor = setupMotor("RFMotor", DcMotor.Direction.REVERSE);
      LBMotor  = setupMotor( "LBMotor", DcMotor.Direction.FORWARD);
      RBMotor = setupMotor( "RBMotor",DcMotor.Direction.REVERSE);
      armExtension2=myOpMode.hardwareMap.get(DcMotor.class,"lateral");
      armExtension1=myOpMode.hardwareMap.get(DcMotor.class,"axial");

      headingController.enableContinuousInput(-Math.PI, Math.PI);
      headingController.setTolerance(Math.toRadians(3.0),headingVelocityTolerance);//
      xController.setTolerance(translationPositionTolerance,translationVelocityTolerance);
      yController.setTolerance(translationPositionTolerance,translationVelocityTolerance);

      myOpMode.telemetry.addData("x",pose.x);
      myOpMode.telemetry.addData("y",pose.y);
      myOpMode.telemetry.addData("h",pose.h);
      myOpMode.telemetry.update();

      //imu = myOpMode.hardwareMap.get(IMU.class, "imu");

      odo= myOpMode.hardwareMap.get(SparkFunOTOS.class,"odo");
      configureOtos();

      //  Connect to the encoder channels using the name of that channel.

      // Set all hubs to use the AUTO Bulk Caching mode for faster encoder reads
      List<LynxModule> allHubs = myOpMode.hardwareMap.getAll(LynxModule.class);
      for (LynxModule module : allHubs) {
         module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
      }

      // Tell the software how the Control Hub is mounted on the robot to align the IMU XYZ axes correctly
      /*
      RevHubOrientationOnRobot orientationOnRobot =
      new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP,
      RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);
      imu.initialize(new IMU.Parameters(orientationOnRobot));

       */

     mecanumDrive = new MecanumDrive(false,
         LFMotor,RFMotor,LBMotor,RBMotor
     );

      // zero out all the odometry readings.
      //resetOdometry();

      // Set the desired telemetry state
      this.showTelemetry = showTelemetry;
   }

   private void configureOtos() {

      // Set the desired units for linear and angular measurements. Can be either
      // meters or inches for linear, and radians or degrees for angular. If not
      // set, the default is inches and degrees. Note that this setting is not
      // persisted in the sensor, so you need to set at the start of all your
      // OpModes if using the non-default value.
      // myOtos.setLinearUnit(DistanceUnit.METER);
      odo.setLinearUnit(DistanceUnit.INCH);
      // myOtos.setAngularUnit(AnguleUnit.RADIANS);
      odo.setAngularUnit(AngleUnit.DEGREES);

      // Assuming you've mounted your sensor to a robot and it's not centered,
      // you can specify the offset for the sensor relative to the center of the
      // robot. The units default to inches and degrees, but if you want to use
      // different units, specify them before setting the offset! Note that as of
      // firmware version 1.0, these values will be lost after a power cycle, so
      // you will need to set them each time you power up the sensor. For example, if
      // the sensor is mounted 5 inches to the left (negative X) and 10 inches
      // forward (positive Y) of the center of the robot, and mounted 90 degrees
      // clockwise (negative rotation) from the robot's orientation, the offset
      // would be {-5, 10, -90}. These can be any value, even the angle can be
      // tweaked slightly to compensate for imperfect mounting (eg. 1.3 degrees).
      //SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(0.6578631, 0.5828845, -90.0);
      SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(0.5949, 0.0212, -90.0);

      odo.setOffset(offset);

      // Here we can set the linear and angular scalars, which can compensate for
      // scaling issues with the sensor measurements. Note that as of firmware
      // version 1.0, these values will be lost after a power cycle, so you will
      // need to set them each time you power up the sensor. They can be any value
      // from 0.872 to 1.127 in increments of 0.001 (0.1%). It is recommended to
      // first set both scalars to 1.0, then calibrate the angular scalar, then
      // the linear scalar. To calibrate the angular scalar, spin the robot by
      // multiple rotations (eg. 10) to get a precise error, then set the scalar
      // to the inverse of the error. Remember that the angle wraps from -180 to
      // 180 degrees, so for example, if after 10 rotations counterclockwise
      // (positive rotation), the sensor reports -15 degrees, the required scalar
      // would be 3600/3585 = 1.004. To calibrate the linear scalar, move the
      // robot a known distance and measure the error; do this multiple times at
      // multiple speeds to get an average, then set the linear scalar to the
      // inverse of the error. For example, if you move the robot 100 inches and
      // the sensor reports 103 inches, set the linear scalar to 100/103 = 0.971
      odo.setLinearScalar(1.0);
      odo.setAngularScalar(1.0);

      // The IMU on the OTOS includes a gyroscope and accelerometer, which could
      // have an offset. Note that as of firmware version 1.0, the calibration
      // will be lost after a power cycle; the OTOS performs a quick calibration
      // when it powers up, but it is recommended to perform a more thorough
      // calibration at the start of all your OpModes. Note that the sensor must
      // be completely stationary and flat during calibration! When calling
      // calibrateImu(), you can specify the number of samples to take and whether
      // to wait until the calibration is complete. If no parameters are provided,
      // it will take 255 samples and wait until done; each sample takes about
      // 2.4ms, so about 612ms total
      odo.calibrateImu();

      // Reset the tracking algorithm - this resets the position to the origin,
      // but can also be used to recover from some rare tracking errors
      odo.resetTracking();

      // After resetting the tracking, the OTOS will report that the robot is at
      // the origin. If your robot does not start at the origin, or you have
      // another source of location information (eg. vision odometry), you can set
      // the OTOS location to match and it will continue to track from there.
      SparkFunOTOS.Pose2D currentPosition = new SparkFunOTOS.Pose2D(36, 9, 0);
      odo.setPosition(currentPosition);

      // Get the hardware and firmware version
      SparkFunOTOS.Version hwVersion = new SparkFunOTOS.Version();
      SparkFunOTOS.Version fwVersion = new SparkFunOTOS.Version();
      odo.getVersionInfo(hwVersion, fwVersion);

   }
   /**
    *   Setup a drive motor with passed parameters.  Ensure encoder is reset.
    * @param deviceName  Text name associated with motor in Robot Configuration
    * @param direction   Desired direction to make the wheel run FORWARD with positive power input
    * @return the DcMotor object
    */


   private Motor setupMotor (String deviceName, DcMotor.Direction direction) {
      Motor aMotor=new Motor(myOpMode.hardwareMap,deviceName);
      //DcMotor aMotor = myOpMode.hardwareMap.get(DcMotor.class, deviceName);
      boolean isInverted;

      switch (direction) {
         case FORWARD:
            isInverted = false;
            break;
         case REVERSE:
            isInverted = true;
            break;
         default:
            isInverted = false;
      }

      aMotor.setInverted(isInverted);
      aMotor.stopAndResetEncoder();
      aMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
      aMotor.setRunMode(Motor.RunMode.RawPower);

      return aMotor;
   }


   /**
    * Read all input devices to determine the robot's motion
    * always return true so this can be used in "while" loop conditions
    * @return true
    */
   public boolean readSensors() {

      SparkFunOTOS.Pose2D pos = odo.getPosition();


      //rawDriveOdometer = armExtension1.getCurrentPosition() * (INVERT_DRIVE_ODOMETRY ? -1 : 1);
      //rawStrafeOdometer = armExtension2.getCurrentPosition() * (INVERT_STRAFE_ODOMETRY ? -1 : 1);
      driveDistance = pos.x;
      strafeDistance = pos.y;

      //YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
      //AngularVelocity angularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);

      //rawHeading  = orientation.getYaw(AngleUnit.DEGREES);
      rawHeading = pos.h;
      heading     = rawHeading - headingOffset;
      pose = new SparkFunOTOS.Pose2D(pos.x, pos.y, heading);
      //turnRate    = angularVelocity.zRotationRate;

      if (showTelemetry) {
         //  myOpMode.telemetry.addData("Odom Ax:Lat", "%6d %6d", rawDriveOdometer - driveOdometerOffset, rawStrafeOdometer - strafeOdometerOffset);
         // myOpMode.telemetry.addData("Dist Ax:Lat", "%5.2f %5.2f", driveDistance, strafeDistance);
         // myOpMode.telemetry.addData("Head Deg:Rate", "%5.2f %5.2f", heading, turnRate);
      }
      return true;  // do this so this function can be included in the condition for a while loop to keep values fresh.
   }


   //  ########################  Mid level control functions.  #############################3#

   public void moveToPose (double x, double y, double heading, double power, double turnPower, double holdTime) {
      //resetOdometry();
      /*
      strafeController.reset(x,power);             //  Maintain zero strafe drift
      driveController.reset(y, power);  // Achieve desired drive distance
      yawController.reset(heading, turnPower);                          // Maintain last turn angle
      holdTimer.reset();
       */
      SparkFunOTOS.Pose2D currentPose = pose;
      xController.reset();
      yController.reset();
      headingController.reset();

      xController.calculate(currentPose.x, x);
      yController.calculate(currentPose.y, y);
      double targetHeading = Rotation2d.fromDegrees(heading).getRadians();
      headingController.calculate(getRotation().getRadians(), targetHeading);

      while (myOpMode.opModeIsActive() && readSensors()){
         myOpMode.telemetry.addData("x",pose.x);
         myOpMode.telemetry.addData("y",pose.y);
         myOpMode.telemetry.addData("h",pose.h);
         myOpMode.telemetry.update();

         double xFeedback = xController.calculate(pose.x, x);
         double yFeedback = yController.calculate(pose.y, y);
         double headingFeedback = -headingController.calculate(getRotation().getRadians(), targetHeading);

         // implement desired axis powers
         driveWithFieldCentric(xFeedback, yFeedback, headingFeedback);
         // Time to exit?
         if (xController.atSetPoint() && yController.atSetPoint() && headingController.atSetpoint()) {
            if (holdTimer.time() > 0.15) {
               stopRobot();
               //resetHeading();
               //resetOdometry();
               break;   // Exit loop if we are in position, and have been there long enough.
            }
         }
         else {
            holdTimer.reset();
         }
         myOpMode.sleep(10);
      }
      stopRobot();

   }


   /**
    * Drive in the axial (forward/reverse) direction, maintain the current heading and don't drift sideways
    * @param distanceInchesY  Distance to travel.  +ve = forward, -ve = reverse.
    * @param distanceInchesX  Distance to travel.  +ve = forward, -ve = reverse.
    * @param power Maximum power to apply.  This number should always be positive.
    * @param holdTime Minimum time (sec) required to hold the final position.  0 = no hold.
    */
   public void move (double distanceInchesY, double distanceInchesX, double power, double holdTime) {
      //resetOdometry();
      strafeController.reset(distanceInchesX,power);             //  Maintain zero strafe drift
      driveController.reset(distanceInchesY, power);  // Achieve desired drive distance
      yawController.reset();                          // Maintain last turn angle
      holdTimer.reset();

      while (myOpMode.opModeIsActive() && readSensors()){
         // implement desired axis powers
         moveRobot(driveController.getOutput(driveDistance), strafeController.getOutput(strafeDistance), yawController.getOutput(heading));

         // Time to exit?
         if (driveController.inPosition() && yawController.inPosition() && strafeController.inPosition()) {
            if (holdTimer.time() > 0.15) {
               stopRobot();
               //resetHeading();
               //resetOdometry();
               break;   // Exit loop if we are in position, and have been there long enough.
            }
         }
         else {
            holdTimer.reset();
         }
         myOpMode.sleep(10);
      }
      stopRobot();
   }

   /**
    * Strafe in the lateral (left/right) direction, maintain the current heading and don't drift fwd/bwd
    * @param distanceInchesX  Distance to travel.  +ve = left, -ve = right.
    * @param power Maximum power to apply.  This number should always be positive.
    * @param holdTime Minimum time (sec) required to hold the final position.  0 = no hold.
    */
   public void strafe(double distanceInchesX, double power, double holdTime) {
      //resetOdometry();
      driveController.reset(0.0);             //  Maintain zero drive drift
      strafeController.reset(distanceInchesX, power);  // Achieve desired Strafe distance
      yawController.reset();                          // Maintain last turn angle
      holdTimer.reset();

      while (myOpMode.opModeIsActive() && readSensors()){

         // implement desired axis powers
         moveRobot(driveController.getOutput(driveDistance), strafeController.getOutput(strafeDistance), yawController.getOutput(heading));

         // Time to exit?
         if (strafeController.inPosition() && yawController.inPosition()) {
            if (holdTimer.time() > holdTime) {
               stopRobot();
               //resetHeading();
               //resetOdometry();
               break;   // Exit loop if we are in position, and have been there long enough.
            }
         }
         else {
            holdTimer.reset();
         }
         myOpMode.sleep(10);
      }
      stopRobot();
      //resetHeading();
      //resetOdometry();
   }
   public void drive (double distanceInchesY, double power, double holdTime) {
      //resetOdometry();
      driveController.reset(distanceInchesY,power);             //  Maintain zero drive drift
      strafeController.reset(0.0);  // Achieve desired Strafe distance
      yawController.reset();                          // Maintain last turn angle
      holdTimer.reset();

      while (myOpMode.opModeIsActive() && readSensors()){

         // implement desired axis powers
         moveRobot(driveController.getOutput(driveDistance), strafeController.getOutput(strafeDistance), yawController.getOutput(heading));

         // Time to exit?
         if (driveController.inPosition() && yawController.inPosition()) {
            if (holdTimer.time() > holdTime) {
               stopRobot();
               //resetHeading();
               //resetOdometry();
               break;   // Exit loop if we are in position, and have been there long enough.
            }
         }
         else {
            holdTimer.reset();
         }
         myOpMode.sleep(10);
      }
      stopRobot();
      //resetHeading();
      //resetOdometry();
   }

   /**
    * Rotate to an absolute heading/direction
    * @param headingDeg  Heading to obtain.  +ve = CCW, -ve = CW.
    * @param power Maximum power to apply.  This number should always be positive.
    * @param holdTime Minimum time (sec) required to hold the final position.  0 = no hold.
    */
   public void turnTo(double headingDeg, double power, double holdTime) {

      yawController.reset(headingDeg, power);
      while (myOpMode.opModeIsActive() && readSensors()) {

         // implement desired axis powers
         moveRobot(0, 0, yawController.getOutput(heading));

         // Time to exit?
         if (yawController.inPosition()) {
            if (holdTimer.time() > holdTime) {
               break;   // Exit loop if we are in position, and have been there long enough.
            }
         } else {
            holdTimer.reset();
         }
         myOpMode.sleep(10);
      }
      stopRobot();
   }

   //  ########################  Low level control functions.  ###############################

   public void driveWithFieldCentric(double strafeSpeed, double forwardSpeed, double turnSpeed){
       mecanumDrive.driveFieldCentric(strafeSpeed, forwardSpeed, turnSpeed, getHeading());

   }


   /**
    * Drive the wheel motors to obtain the requested axes motions
    * @param drive     Fwd/Rev axis power
    * @param strafe    Left/Right axis power
    * @param yaw       Yaw axis power
    */
   public void moveRobot(double drive, double strafe, double yaw){


      double lF = drive - strafe - yaw;
      double rF = drive + strafe + yaw;
      double lB = drive + strafe - yaw;
      double rB = drive - strafe + yaw;

      double max = Math.max(Math.abs(lF), Math.abs(rF));
      max = Math.max(max, Math.abs(lB));
      max = Math.max(max, Math.abs(rB));

      //normalize the motor values
      if (max > 1.0)  {
         lF /= max;
         rF /= max;
         lB /= max;
         rB /= max;
      }

      /*
      lF *= MAX_VELOCITY;
      rF *= MAX_VELOCITY;
      lB *= MAX_VELOCITY;
      rB *= MAX_VELOCITY;

      lF = driveFeedforward.calculate(lF) / 12.0;
      rF = driveFeedforward.calculate(rF) / 12.0;
      lB = driveFeedforward.calculate(lB) / 12.0;
      rB = driveFeedforward.calculate(rB) / 12.0;

       */

      //send power to the motors
      LFMotor.set(lF);
      RFMotor.set(rF);
      LBMotor.set(lB);
      RBMotor.set(rB);



      if (showTelemetry = true) {
         // myOpMode.telemetry.addData("Axes D:S:Y", "%5.2f %5.2f %5.2f", drive, strafe, yaw);
         // myOpMode.telemetry.addData("Wheels lf:rf:lb:rb", "%5.2f %5.2f %5.2f %5.2f", lF, rF, lB, rB);
         //myOpMode.telemetry.update(); //  Assume this is the last thing done in the loop.

        // myOpMode.telemetry.addData("Axial(D)",  driveDistance);
        // myOpMode.telemetry.addData("Lateral(S)", strafeDistance);
       //  myOpMode.telemetry.addData("heading(Y)", heading);
         //myOpMode.telemetry.addData("LFDistance",LFMotor.getCurrentPosition());
        // myOpMode.telemetry.addData("RFDistance",RFMotor.getCurrentPosition());
         //myOpMode.telemetry.addData("LBDistance",LBMotor.getCurrentPosition());



         //myOpMode.telemetry.addData("driveController",driveController.getOutput(strafeDistance));
         //myOpMode.telemetry.addData("strafeController",strafeController.getOutput(strafeDistance));

        // myOpMode.telemetry.addData("driveDistance",driveDistance);
         //myOpMode.telemetry.addData("strafeDistance",strafeDistance);


         //myOpMode. telemetry.update();
      }
   }

   /**
    * Stop all motors.
    */
   public void stopRobot() {
      moveRobot(0,0,0);
   }

   /**
    * Set odometry counts and distances to zero.
    */
   public void resetOdometry() {
      readSensors();
      odo.resetTracking();
      /*
      driveOdometerOffset = rawDriveOdometer;
      driveDistance = 0.0;
      driveController.reset(0);

      strafeOdometerOffset = rawStrafeOdometer;
      strafeDistance = 0.0;
      strafeController.reset(0);
       */
   }

   /**
    * Reset the robot heading to zero degrees, and also lock that heading into heading controller.
    */
   public void resetHeading() {
      //readSensors();
      headingOffset = (headingOffset + getHeading()-90.0) % 360;
      //headingOffset = rawHeading;
      yawController.reset(0);
      //heading = 0;
   }

   public double getHeading() {return getRotation().getDegrees();}

   public Rotation2d getRotation() {
      return Rotation2d.fromDegrees(heading);
   }
   public double getTurnRate() {return turnRate;}

   public void setPos(double x,double y,double heading){
      headingOffset=0;
      odo.setPosition(new SparkFunOTOS.Pose2D(x,y,heading));
      readSensors();
   }

   /**
    * Set the drive telemetry on or off
    */
   public void showTelemetry(boolean show){
      showTelemetry = show;
   }

   public class ProportionalControl {
      double lastOutput;
      double gain;
      double accelLimit;
      double defaultOutputLimit;
      double liveOutputLimit;
      double setPoint;
      double tolerance;
      double deadband;
      boolean circular;
      boolean inPosition;
      ElapsedTime cycleTime = new ElapsedTime();

      public ProportionalControl(double gain, double accelLimit, double outputLimit, double tolerance, double deadband, boolean circular) {
         this.gain = gain;
         this.accelLimit = accelLimit;
         this.defaultOutputLimit = outputLimit;
         this.liveOutputLimit = outputLimit;
         this.tolerance = tolerance;
         this.deadband = deadband;
         this.circular = circular;
         reset(0.0);
      }

      /**
       * Determines power required to obtain the desired setpoint value based on new input value.
       * Uses proportional gain, and limits rate of change of output, as well as max output.
       *
       * @param input Current live control input value (from sensors)
       * @return desired output power.
       */
      public double getOutput(double input) {
         double error = setPoint - input;
         double dV = cycleTime.seconds() * accelLimit;
         double output;

         // normalize to +/- 180 if we are controlling heading
         if (circular) {
            while (error > 180) error -= 360;
            while (error <= -180) error += 360;
         }

         inPosition = (Math.abs(error) <= tolerance);

         // Prevent any very slow motor output accumulation
         if (Math.abs(error) <= deadband) {
            output = 0;
         } else {
            // calculate output power using gain and clip it to the limits
            output = (error * gain);
            output = Range.clip(output, -liveOutputLimit, liveOutputLimit);

            // Now limit rate of change of output (acceleration)
            if ((output - lastOutput) > dV) {
               output = lastOutput + dV;
            } else if ((output - lastOutput) < -dV) {
               output = lastOutput - dV;
            }
         }

         lastOutput = output;
         cycleTime.reset();
         return output;
      }

      public boolean inPosition() {
         return inPosition;
      }

      public double getSetpoint() {
         return setPoint;
      }

      /**
       * Saves a new setpoint and resets the output power history.
       * This call allows a temporary power limit to be set to override the default.
       *
       * @param setPoint
       * @param powerLimit
       */
      public void reset(double setPoint, double powerLimit) {
         liveOutputLimit = Math.abs(powerLimit);
         this.setPoint = setPoint;
         reset();
      }

      /**
       * Saves a new setpoint and resets the output power history.
       *
       * @param setPoint
       */
      public void reset(double setPoint) {
         liveOutputLimit = defaultOutputLimit;
         this.setPoint = setPoint;
         reset();
      }

      /**
       * Leave everything else the same, Just restart the acceleration timer and set output to 0
       */
      public void reset() {
         cycleTime.reset();
         inPosition = false;
         lastOutput = 0.0;
      }

   }
}



//****************************************************************************************************
//****************************************************************************************************

