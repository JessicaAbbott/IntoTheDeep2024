package org.firstinspires.ftc.teamcode.IntoTheDeep2025.Tele;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp

public class KateTeleForReference extends OpMode{

   public DcMotor leftMotor = null;
   public DcMotor rightMotor = null;
   public DcMotor pivotLeft;
   public DcMotor pivotRight;
   public DcMotor extensionLeft;
   public DcMotor extensionRight;


   boolean automateOutButtonPressed = false;
   boolean automateOuttake = false;
   double startOuttakeTime;
   boolean automateButtonPressed = false;
   boolean automatOuttake = false;
   double startTime;
   boolean ButtonPressed = false;
   boolean automate = false;
   double sTime;
   boolean buttonpressed = false;
   boolean automat = false;
   double Time;

   private PIDController controller;

   public static double p = 0.001, i = 0, d = 0.00005;
   public static double f = 0.01;

   public static int target = 2670;

   public final double ticksDegree = 700 / 100.0;

   double time;
   boolean wasRecentlyPressed = false;
   int accelerateCount = 0;

   double currentPower = 0;

   @Override
   public void init()    {

      controller = new PIDController(p, i, d);
      telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

      leftMotor = hardwareMap.get(DcMotor.class, "leftMotor");
      rightMotor = hardwareMap.get(DcMotor.class, "rightMotor");
      pivotLeft = hardwareMap.get(DcMotor.class, "pivotL");
      pivotRight = hardwareMap.get(DcMotor.class, "pivotR");
      extensionLeft = hardwareMap.get(DcMotor.class, "extensionLeft");
      extensionRight = hardwareMap.get(DcMotor.class, "extensionRight");

      leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
      rightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
      pivotLeft.setDirection(DcMotorSimple.Direction.FORWARD);
      pivotRight.setDirection(DcMotorSimple.Direction.REVERSE);
      extensionLeft.setDirection(DcMotorSimple.Direction.REVERSE);
      extensionRight.setDirection(DcMotorSimple.Direction.REVERSE);
      pivotLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      pivotRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      pivotLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      pivotRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      pivotLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      pivotRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


      telemetry.addData("position", pivotRight.getCurrentPosition());
      telemetry.update();
      telemetry.addData(">", "Robot Ready. Press Play.");
   }
   @Override
   public void start(){

   }

   @Override
   public void loop() {

      double left = 0;
      double right = 0;
      double pivot = 0;
      double extension = 0;

      if(gamepad1.dpad_left){left = -0.8; right = 0.8;}
      else if(gamepad1.dpad_right){left = 0.8; right = -0.8;}
      else if(gamepad1.dpad_up){left = 0.6; right = 0.6;}
      else if(gamepad1.dpad_down){left = -0.6; right = -0.6;}
      else if(gamepad1.left_trigger > 0 || gamepad1.right_trigger > 0) {

         if (!wasRecentlyPressed) {
            time = getRuntime();
            wasRecentlyPressed = true;
            accelerateCount = 1;
         }

         double targetPower = -gamepad1.left_trigger + gamepad1.right_trigger;

         if (targetPower < 0 && (-0.01 * (accelerateCount * accelerateCount)) > targetPower) {
            if (getRuntime() - time >= 0.01) {
               currentPower = -0.01 * (accelerateCount * accelerateCount);
               time = getRuntime();
               accelerateCount += 1;
            }
         }
         else if (targetPower > 0 && (0.01 * (accelerateCount * accelerateCount)) < targetPower) {
            if (getRuntime() - time >= 0.01) {
               currentPower = 0.01 * (accelerateCount * accelerateCount);
               time = getRuntime();
               accelerateCount += 1;
            }
         }
         else {
            currentPower = targetPower;
         }

         left = right = currentPower;

      }
      else{
         wasRecentlyPressed = false;
         currentPower = 0;
         accelerateCount = 0;
      }

      left = Math.min(Math.max((left + 0.9 * gamepad1.left_stick_x), -1), 1);
      right = Math.min(Math.max((right - 0.9 * gamepad1.left_stick_x), -1), 1);

      telemetry.addData("Lpower", left);
      telemetry.addData("Rpower", right);
      telemetry.addData("position", pivotRight.getCurrentPosition());
      telemetry.update();
      leftMotor.setPower(left);
      rightMotor.setPower(right);


//        if (gamepad2.a) intakeServo.setPosition(0.7); //open
//        else if (gamepad2.b) intakeServo.setPosition(0.3); //close

      if (gamepad2.right_stick_y > 0) extension = 1;
      else if (gamepad2.right_stick_y < 0) extension = -1;
      else extension = 0;


      if (gamepad2.left_stick_y > 0) pivot = 1;
      else if (gamepad2.left_stick_y < 0) pivot = -1;
//        else pivot = 0;

      if (gamepad2.dpad_up) pivot = -0.7;
      else if (gamepad2.dpad_down) pivot = 0.4;
//        else pivot = 0;

      extensionRight.setPower(extension);
      extensionLeft.setPower(extension);

      pivotRight.setPower(pivot);
      pivotLeft.setPower(pivot);

      telemetry.addData("extension", extension);
      telemetry.addData("arm", pivot);
      telemetry.update();

      if (gamepad2.x) {
         if (!automateOutButtonPressed) {    // Toggle automate outtake
            if (automateOuttake) {
               automateOuttake = false;
            }
            else {
               automateOuttake = true;
               startOuttakeTime = getRuntime();
            }
         }
         automateOutButtonPressed = true;
      }
      else {
         automateOutButtonPressed = false;
      }
      if (automateOuttake) {
         if (getRuntime() - startOuttakeTime < 0.5) {
            telemetry.addData("time", getRuntime());
            telemetry.addData("startTime", startOuttakeTime);
            extensionLeft.setPower(-1);
            extensionRight.setPower(-1);
         }
         controller.setPID(p, i, d);
         int armPos = pivotRight.getCurrentPosition();
         double pid = controller.calculate(armPos, target);
         double ff = Math.cos(Math.toRadians(target / ticksDegree)) * f;

         double power = pid + ff;

         pivotLeft.setPower(-power);
         pivotRight.setPower(-power);

         telemetry.addData("power", power);
         telemetry.addData("pos", armPos);
         telemetry.addData("target", target);
         telemetry.update();
      }
      if (gamepad2.y) {
         if (!automateButtonPressed) {    // Toggle automate outtake
            if (automatOuttake) {
               automatOuttake = false;
            }
            else {
               automatOuttake = true;
               startTime = getRuntime();
            }
         }
         automateButtonPressed = true;
      }
      else {
         automateButtonPressed = false;
      }
      if (automatOuttake) {
         if (getRuntime() - startTime < 0.7) {
            telemetry.addData("time", getRuntime());
            telemetry.addData("startTime", startTime);
//                extensionLeft.setPower(1);
//                extensionRight.setPower(1);

            if (getRuntime() - startTime > 0.2) {
               controller.setPID(0.0009, i, d);
               int armPos = pivotRight.getCurrentPosition();
               double pid = controller.calculate(armPos, 0);
               double ff = Math.cos(Math.toRadians(0 / ticksDegree)) * f;

               double power = pid + ff;

               pivotLeft.setPower(-power);
               pivotRight.setPower(-power);

               telemetry.addData("power", power);
               telemetry.addData("pos", armPos);
               telemetry.addData("target", target);
               telemetry.update();
            }
         }
      }
      if (gamepad1.x) {
         if (!ButtonPressed) {    // Toggle automate outtake
            if (automate) {
               automate = false;
            }
            else {
               automate = true;
               sTime = getRuntime();
            }
         }
         ButtonPressed = true;
      }
      else {
         ButtonPressed = false;
      }
      if (automate) {
         if (getRuntime() - sTime < 2.2) {
            telemetry.addData("time", getRuntime());
            telemetry.addData("startTime", sTime);
//                extensionLeft.setPower(-1);
//                extensionRight.setPower(-1);
         }
         controller.setPID(p, i, d);
         int armPos = pivotRight.getCurrentPosition();
         double pid = controller.calculate(armPos, 1700);
         double ff = Math.cos(Math.toRadians(1700 / ticksDegree)) * f;

         double power = pid + ff;

         pivotLeft.setPower(-power);
         pivotRight.setPower(-power);

         telemetry.addData("power", power);
         telemetry.addData("pos", armPos);
         telemetry.addData("target", target);
         telemetry.update();
      }

   }

   @Override
   public void stop(){

   }
}

