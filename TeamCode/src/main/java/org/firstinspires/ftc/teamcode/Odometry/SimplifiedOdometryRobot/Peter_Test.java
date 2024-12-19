package org.firstinspires.ftc.teamcode.Odometry.SimplifiedOdometryRobot;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class Peter_Test extends LinearOpMode {

   public CRServo intakeLeft;
   public CRServo intakeRight;
   public Servo intakeServo;

   public DcMotor climbHook;
   private PIDController armController;

   boolean climbButtonPressed = false;

   // PID constants
   public static double p = 0.052, i = 0.02, d = 0.001;  // Tuning constants for PID controller
   public static double f = 0.1;  // Feedforward constant

   private final double ticks_in_degree = 8192 / 360.0;  // Encoder ticks per degree

   private DcMotorEx armPivot;
   private double target = 40;  // Initial target position for the arm

   double newTarget = 0;
   DcMotor armExtension1;
   public DcMotor armExtension2;

   final double SAFE_DRIVE_SPEED = 0.9;
   final double SAFE_STRAFE_SPEED = 0.9;
   final double SAFE_YAW_SPEED = 0.7;
   final double HEADING_HOLD_TIME = 10.0;

   ElapsedTime stopTime = new ElapsedTime();
   boolean autoHeading = false;

   long lastTime = System.nanoTime();

   private double lastLF = 0.0;
   private double lastLB = 0.0;
   private double lastRF = 0.0;
   private double lastRB = 0.0;
   TrapezoidProfile profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(1.73, 0.5), new TrapezoidProfile.State(3.0, 0.0));

   SimplifiedOdometryRobot robot = new SimplifiedOdometryRobot(this);

   private ElapsedTime etime = new ElapsedTime();

   @Override
   public void runOpMode() throws InterruptedException {

      telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

      armExtension1 = hardwareMap.get(DcMotor.class, "lateral");
      armExtension1.setDirection(DcMotorSimple.Direction.FORWARD);
      armExtension1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


      armExtension2 = hardwareMap.get(DcMotor.class, "armExtension");
      armExtension2.setDirection(DcMotorSimple.Direction.FORWARD);
      armExtension2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

      intakeLeft = hardwareMap.get(CRServo.class, "intakeLeft");
      intakeLeft.setDirection(CRServo.Direction.FORWARD);

      intakeRight = hardwareMap.get(CRServo.class, "intakeRight");
      intakeRight.setDirection(CRServo.Direction.REVERSE);

      intakeServo = hardwareMap.get(Servo.class, "intakeServo");


      armController = new PIDController(p, i, d);
      telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

      armPivot = hardwareMap.get(DcMotorEx.class, "armPivot");
      armPivot.setDirection(DcMotorSimple.Direction.REVERSE);
      armPivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      armPivot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

      climbHook = hardwareMap.get(DcMotor.class, "axial");
      climbHook.setDirection(DcMotorSimple.Direction.REVERSE);
      climbHook.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

      robot.initialize(true);

      while (opModeInInit()) {
         robot.readSensors();

         telemetry.addData("lf", 0.0);
         telemetry.addData("rf", 0.0);
         telemetry.addData("lb", 0.0);
         telemetry.addData("rb", 0.0);
         telemetry.addData("target", 0.0);
         telemetry.update();

      }
      etime.reset();

      while (opModeIsActive()) {
         robot.readSensors();
         long currentTime = System.nanoTime();

         // Allow the driver to reset the gyro by pressing both small gamepad buttons
         /*
         if (gamepad1.options && gamepad1.share) {
            robot.resetHeading();
            robot.resetOdometry();
         }

         // read joystick values and scale according to limits set at top of this file
         double drive = -gamepad1.right_stick_x * SAFE_DRIVE_SPEED;
         double strafe = gamepad1.right_stick_y * SAFE_STRAFE_SPEED;
         double yaw = -gamepad1.left_stick_x * SAFE_YAW_SPEED;

         // Drive the robot
         robot.moveRobot(drive, strafe, yaw);

         // If auto heading is on, override manual yaw with the value generated by the heading controller.
         if (autoHeading) {
            yaw = robot.yawController.getOutput(robot.getHeading());
         }

          */
         TrapezoidProfile.State state = profile.calculate(etime.seconds());



         robot.moveRobot(state.velocity / robot.MAX_VELOCITY, 0.0, 0.0);


         double currentLF = robot.LFMotor.getCurrentPosition() * 0.0012186958;
         double currentLB = robot.LBMotor.getCurrentPosition() * 0.0012186958;
         double currentRF = robot.RFMotor.getCurrentPosition() * 0.0012186958;
         double currentRB = robot.RBMotor.getCurrentPosition() * 0.0012186958;
         double loopTime = (currentTime-lastTime) / 1E9;

         telemetry.addData("ms", (currentTime-lastTime)/1E6);
         telemetry.addData("lf", (currentLF-lastLF)/loopTime);
         telemetry.addData("rf", (currentRF-lastRF)/loopTime);
         telemetry.addData("lb", (currentLB-lastLB)/loopTime);
         telemetry.addData("rb", (currentRB-lastRB)/loopTime);
         telemetry.addData("target", state.velocity);
         telemetry.update();
         lastTime = currentTime;
         lastLF = currentLF;
         lastLB = currentLB;
         lastRF = currentRF;
         lastRB = currentRB;
      }
   }
}
