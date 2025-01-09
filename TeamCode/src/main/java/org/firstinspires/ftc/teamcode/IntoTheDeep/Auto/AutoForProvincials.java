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

import org.firstinspires.ftc.teamcode.Odometry.SimplifiedOdometryRobot.SimplifiedOdometryRobot;

@Autonomous
@Config
public class AutoForProvincials extends LinearOpMode {

   private SimplifiedOdometryRobot robot = new SimplifiedOdometryRobot(this);

   public CRServo intakeLeft;
   public CRServo intakeRight;
   public Servo intakeServo;
   public DcMotor armExtension1;
   public DcMotor armExtension2;

   private PIDController armController;
   public static double p = 0.05, i = 0.0, d = 0.002;  // Adjust PID constants
   public static double f = 0.1; // Feedforward value for holding against gravity
   private final double ticks_in_degree = 8192 / 360;

   double target = 0;
   private DcMotorEx armPivot;

   @Override
   public void runOpMode() {

     telemetry.addData("x",robot.pose.x);
     telemetry.addData("y",robot.pose.y);
      telemetry.addData("h",robot.pose.h);
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
      armExtension1.setDirection(DcMotorSimple.Direction.FORWARD);
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
      armPivot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


      telemetry.addData(">", "Touch Play to run Auto");
      telemetry.update();

      robot.setPos(36,9,0);

      waitForStart();
      //robot.resetHeading();
      robot.setPos(36,9,0);
      armController.reset();

      if (opModeIsActive()) {

          robot.moveToPose(36,30,0,1.0,1.0,0.1);
         telemetry.addLine("first point reached");
         telemetry.update();

      }

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
      while (Math.abs(armPos - target) > 6) {
         runArmPID();
         armPos = -armPivot.getCurrentPosition() / ticks_in_degree;
         telemetry.update();
      }
      runArmPID();
      telemetry.addData("Arm reached target", target);
   }

   public void intake() {
      intakeServo.setPosition(0.7);
      intakeLeft.setPower(1);
      intakeRight.setPower(1);
      setArmTarget(23);


   }

   public void outtake() {
      intakeServo.setPosition(1.0);
      intakeLeft.setPower(0.8);
      intakeRight.setPower(0.8);
   }

   public void stopIntake(){
      intakeLeft.setPower(0);
      intakeRight.setPower(0);
   }


   public void Extend(){
      armExtension1.setPower(1);
      armExtension2.setPower(1);
   }
   public void stopExtension(){
      armExtension1.setPower(0);
      armExtension2.setPower(0);
   }


}





