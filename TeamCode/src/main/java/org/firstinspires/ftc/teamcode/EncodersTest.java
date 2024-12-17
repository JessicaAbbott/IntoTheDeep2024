package org.firstinspires.ftc.teamcode;

import android.icu.text.MeasureFormat;
import android.icu.util.MeasureUnit;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous
public class EncodersTest extends LinearOpMode {

   DcMotor axial;
   DcMotor lateral;

   @Override
   public void runOpMode()  {

      axial = hardwareMap.get(DcMotor.class, "axial");
      lateral = hardwareMap.get(DcMotor.class, "lateral");

      axial.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      lateral.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

      axial.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      lateral.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


      waitForStart();

      while (opModeIsActive()) {



         double axialDistance = axial.getCurrentPosition() * 6.28318 /8192 ;
         double lateralDistance = lateral.getCurrentPosition() * 6.28318 /8192;


         telemetry.addData("axial", -axialDistance);
         telemetry.addData("lateral", lateralDistance);
         telemetry.update();
   }
}
}
