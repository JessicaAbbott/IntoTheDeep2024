package org.firstinspires.ftc.teamcode.IntoTheDeep2025.Tele;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
 import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Odometry.SimplifiedOdometryRobot.SimplifiedOdometryRobot;

@Autonomous
public class climbAutomationTest extends LinearOpMode {

   public CRServo intakeLeft;
   public CRServo intakeRight;
   public Servo intakeServo;

   public Servo climbServo;
   public DcMotor climbHook;
   private PIDController armController;
   private PIDController extensionController;
   public static double extensionKp = 0.32, extensionKi = 0, extensionKd = 0.0;  // Tuning constants for extension PID controller
   public static double extensionKf = 0.0;
   private final double ticks_in_inch = (8192 * 0.5249330709);//change this constant

   double ExtensionTarget = 15;
   private DcMotorEx armExtension1;
   private DcMotorEx armExtension2;


   // PID constants
   public static double p = 0.052, i = 0.02, d = 0.001;  // Tuning constants for PID controller
   public static double f = 0.1;  // Feedforward constant

   private final double ticks_in_degree = 8192 / 360.0;  // Encoder ticks per degree

   private DcMotorEx armPivot;
   private double target = 0;  // Initial target position for the arm

   double newTarget = 0;

   SimplifiedOdometryRobot robot = new SimplifiedOdometryRobot(this);

   @Override
   public void runOpMode() throws InterruptedException {
      telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

      intakeLeft = hardwareMap.get(CRServo.class, "intakeLeft");
      intakeLeft.setDirection(CRServo.Direction.FORWARD);

      intakeRight = hardwareMap.get(CRServo.class, "intakeRight");
      intakeRight.setDirection(CRServo.Direction.REVERSE);

      intakeServo = hardwareMap.get(Servo.class, "intakeServo");

      climbServo = hardwareMap.get(Servo.class, "climbServo");

      armController = new PIDController(p, i, d);
      telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

      armPivot = hardwareMap.get(DcMotorEx.class, "armPivot");
      armPivot.setDirection(DcMotorSimple.Direction.REVERSE);
      armPivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      armPivot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

      armExtension1 = hardwareMap.get(DcMotorEx.class, "armExtension");
      armExtension1.setDirection(DcMotorSimple.Direction.FORWARD);
      armExtension1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      armExtension1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

      armExtension2 = hardwareMap.get(DcMotorEx.class, "lateral");
      armExtension2.setDirection(DcMotorSimple.Direction.REVERSE);
      armExtension2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      armExtension2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

      climbHook = hardwareMap.get(DcMotor.class, "axial");
      climbHook.setDirection(DcMotorSimple.Direction.FORWARD);
      climbHook.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

      robot.initialize(true);
      while (opModeInInit()) {
         waitForStart();
      }

      while (opModeIsActive()) {
         climbHook.setPower(0);
         extensionIn();
         delay(800);
         setArmTarget(138);
         runArmPID();
         waitForArmToReachTarget();
         fullExtension();
         delay(2500);
         climbServo.setPosition(0.275);
         stopExtension();
         delay(400);
         firstAscent(); // get it to pull up a bit farther so that the hook can get over
         climbHook.setPower(0.2);
         setArmTarget(142);
         runArmPID();
         fullExtension();
         delay(1000);
         stopExtension();
         retractToHook();
         sleep(2000);
         stopExtension();
         //ClimbHookRelease();
        // retractToHook();
        // sleep(3000);
         //fullRetraction();
         holdPosition();
         stop();

         double armPos = -armPivot.getCurrentPosition() / ticks_in_degree;  // Convert encoder ticks to degrees
         double armPidOutput = armController.calculate(armPos, target);  // Calculate PID output
         double armff = Math.cos(Math.toRadians(target)) * f;  // Feedforward term
         double armPower = armPidOutput + armff;  // Final motor armPower (PID + Feedforward)

         armPower = Math.max(-0.4, Math.min(1.75, armPower));

         armPivot.setPower(armPower);  // Set motor armPower to the calculated value

         telemetry.addData("target", target);
         telemetry.addData("armPos", armPos);
         telemetry.update();
      }
   }

   public void extensionIn() {
      armExtension2.setPower(-1);
      armExtension1.setPower(-1);
      climbHook.setPower(0);
      sleep(800);
   }

   public void stopExtension(){
      armExtension2.setPower(0);
      armExtension1.setPower(0);
   }

   public void delay(long time) {
      sleep(time);
   }

   public void servoOut() {
      climbServo.setPosition(0.275);// this is the perfect down position
   }

   public void firstAscent() {
      servoOut();
      climbHook.setPower(1.0);
      setArmTarget(157);
      runArmPID();
      delay(1100);
   }

    public void fullExtension(){
       armExtension1.setPower(1);// extend
       armExtension2.setPower(1);
       sleep(1100);
       armExtension1.setPower(0);// stop
       armExtension2.setPower(0);
    }
    public void retractToHook(){
       armExtension1.setPower(-1);
       armExtension2.setPower(-1);
    }

   public void ClimbHookRelease() {
      climbHook.setPower(-1);
      sleep(300);
      climbHook.setPower(-0.1);
   }

   public void fullRetraction() {
      armExtension1.setPower(-1);
      armExtension2.setPower(-1);
      setArmTarget(-23);
      runArmPID();
      sleep(1000);
      armExtension1.setPower(0);
      armExtension2.setPower(0);
   }

   public void holdPosition() {
      sleep(1000000000);
   }

   public void runArmPID() {
      double armPos = -armPivot.getCurrentPosition() / ticks_in_degree;
      double pidOutput = armController.calculate(armPos, target);
      double ff = Math.cos(Math.toRadians(armPos)) * f;
      double power = pidOutput + ff;

      power = Math.max(-0.4, Math.min(1.0, power));

      armPivot.setPower(power);

      telemetry.addData("ArmPos", armPos);
      telemetry.addData("ArmTarget", target);
      telemetry.addData("PID Output", pidOutput);
      telemetry.addData("Feedforward", ff);
      telemetry.addData("Motor Power", power);
   }

   public void setArmTarget(double newTarget) {
      target = newTarget;
   }

   public void waitForArmToReachTarget() {
      double armPos = -armPivot.getCurrentPosition() / ticks_in_degree;
      while (Math.abs(armPos - target) > 5) {
         runArmPID();
         armPos = -armPivot.getCurrentPosition() / ticks_in_degree;
         telemetry.update();
      }
      runArmPID();
      telemetry.addData("Arm reached target", target);
   }

}


