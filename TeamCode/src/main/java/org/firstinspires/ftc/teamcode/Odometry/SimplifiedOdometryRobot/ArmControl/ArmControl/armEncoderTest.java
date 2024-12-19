package org.firstinspires.ftc.teamcode.Odometry.SimplifiedOdometryRobot.ArmControl.ArmControl;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous
public class armEncoderTest extends LinearOpMode {

   DcMotor armPivot;

   @Override
   public void runOpMode()  {

      armPivot=hardwareMap.get(DcMotorEx.class,"armPivot");
      armPivot.setDirection(DcMotorSimple.Direction.REVERSE);
      armPivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      armPivot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();
      // wait for start
      while (opModeIsActive());

         double armPos = armPivot.getCurrentPosition() * 8192/360;

         telemetry.addData("armPos", armPos);
         telemetry.update();
   }
}

