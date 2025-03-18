package org.firstinspires.ftc.teamcode.IntoTheDeep.Auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Odometry.SimplifiedOdometryRobot.SimplifiedOdometryRobot;

@Autonomous
@Config
public class SpecimenAuto extends LinearOpMode {

   private SimplifiedOdometryRobot robot = new SimplifiedOdometryRobot(this);

   public CRServo intakeLeft;
   public CRServo intakeRight;
   public Servo intakeServo;
   public DcMotor armExtension1;
   public DcMotor armExtension2;

   private PIDController armController;
   public static double p = 0.1, i = 0.0, d = 0.002;  // Adjust PID constants
   public static double f = 0.1; // Feedforward value for holding against gravity
   private final double ticks_in_degree = 8192 / 360;

   double target = 0;
   private DcMotorEx armPivot;

   double highBasket = 148;


   private ElapsedTime timer = new ElapsedTime();  // Create a timer


   @Override
   public void runOpMode() {

      telemetry.addData("x", robot.pose.x);
      telemetry.addData("y", robot.pose.y);
      telemetry.addData("h", robot.pose.h);
      telemetry.update();


      telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

      // Initialize the robot hardware & Turn on telemetry
      robot.initialize(true);

      intakeLeft = hardwareMap.get(CRServo.class, "intakeLeft");
      intakeLeft.setDirection(CRServo.Direction.FORWARD);

      intakeRight = hardwareMap.get(CRServo.class, "intakeRight");
      intakeRight.setDirection(CRServo.Direction.REVERSE);

      intakeServo = hardwareMap.get(Servo.class, "intakeServo");

      armExtension1 = hardwareMap.get(DcMotor.class, "lateral");
      armExtension1.setDirection(DcMotorSimple.Direction.REVERSE);
      armExtension1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

      armExtension2 = hardwareMap.get(DcMotor.class, "armExtension");
      armExtension2.setDirection(DcMotorSimple.Direction.FORWARD);
      armExtension2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

      armController = new PIDController(p, i, d);
      telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

      armPivot = hardwareMap.get(DcMotorEx.class, "armPivot");
      armPivot.setDirection(DcMotorSimple.Direction.REVERSE);
      armPivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      armPivot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

      telemetry.addData(">", "Touch Play to run Auto");
      telemetry.update();

   while (opModeInInit()) {
      waitForStart();
      robot.setPos(72, 10, 90);
      robot.resetHeading();
      armController.reset();
   }

      if (opModeIsActive()) {
         robot.setPos(72, 10, 90);


         // arm on and up, make sure it is holding intake
         intakeServo.setPosition(0.2);
         intakeRight.setPower(0.6);
         intakeLeft.setPower(0.6);

         armExtension1.setPower(1);
         armExtension2.setPower(1);

         //arm up to high bar
         setArmTarget(72);
         runArmPID();
         waitForArmToReachTarget();

         //move to bar, stop extension
         robot.moveToPose(72,23,90, 1.0, 1.0, 0.1);
         armExtension2.setPower(0);
         armExtension1.setPower(0);


         setArmTarget(65);
         runArmPID();
         waitForArmToReachTarget();

         intakeLeft.setPower(-0.8);
         intakeRight.setPower(-0.8);

         robot.moveToPose(72,20,90,0.2,1.0,0.1);

         setArmTarget(45);
         runArmPID();

         robot.moveToPose(96,10,0,1.0,1.0,0.1);
      }
   }

   private void stopIntake() {
      intakeLeft.setPower(0);
      intakeRight.setPower(0);
      intakeServo.setPosition(0.4);
   }

   private void outtake() {
      intakeServo.setPosition(0.9);
      waitFor(100);
      intakeLeft.setPower(1.0); // 0.8 for not spitting but 1.0 maybe stop jam
      intakeRight.setPower(1.0);
      waitFor(400);
   }

   public void runArmPID() {
      double armPos = -armPivot.getCurrentPosition() / ticks_in_degree;
      double pidOutput = armController.calculate(armPos, target);
      double ff = Math.cos(Math.toRadians(armPos)) * f;
      double power = pidOutput + ff;

      power = Math.max(-0.5, Math.min(1.0, power));

      armPivot.setPower(power);

      telemetry.addData("ArmPos", armPos);
      telemetry.addData("ArmTarget", target);
      telemetry.addData("PID Output", pidOutput);
      telemetry.addData("Feedforward", ff);
      telemetry.addData("Motor Power", power);
      telemetry.update();
   }

   public void setArmTarget(double newTarget) {
      target = newTarget;
   }

   public void waitForArmToReachTarget() {
      double armPos = -armPivot.getCurrentPosition() / ticks_in_degree;
      while (Math.abs(armPos - target) > 3) {
         runArmPID();
         armPos = -armPivot.getCurrentPosition() / ticks_in_degree;
         telemetry.update();
      }
      runArmPID();
      telemetry.addData("Arm reached target", target);
      telemetry.update();
   }

   public void intake() {
      intakeServo.setPosition(0.4);
      intakeLeft.setPower(1);
      intakeRight.setPower(1);
      setArmTarget(23);
      runArmPID();
      waitForArmToReachTarget();
      waitFor(100);
      stopIntake();
   }

   public void waitFor(long milliseconds){

      timer.reset();  // Reset the timer
      while (opModeIsActive() && timer.milliseconds() < milliseconds) {
         runArmPID();
         telemetry.update();
      }
   }
}