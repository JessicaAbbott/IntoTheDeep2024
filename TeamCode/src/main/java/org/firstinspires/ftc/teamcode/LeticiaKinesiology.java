package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous
public class LeticiaKinesiology extends LinearOpMode {

   public Servo toeMotor;

   @Override
   public void runOpMode() throws InterruptedException {
      toeMotor = hardwareMap.get(Servo.class, "toeMotor");

      waitForStart();

      while (opModeIsActive()){

         toeMotor.setPosition(0.2);


         sleep(300);

         toeMotor.setPosition(-0.2);

         sleep(300);

         toeMotor.setPosition(0.2);

         sleep(300);

         toeMotor.setPosition(-0.2);

         sleep(300);


         toeMotor.setPosition(0.2);

         sleep(300);

         toeMotor.setPosition(-0.2);

         sleep(300);


         toeMotor.setPosition(0.2);

         sleep(300);

         toeMotor.setPosition(-0.2);

         sleep(300);


         toeMotor.setPosition(0.2);

         sleep(300);

         toeMotor.setPosition(-0.2);

         loop();

      }
   }




}
