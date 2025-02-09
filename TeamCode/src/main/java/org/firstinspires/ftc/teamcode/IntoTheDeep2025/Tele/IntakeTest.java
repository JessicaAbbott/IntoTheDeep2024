package org.firstinspires.ftc.teamcode.IntoTheDeep2025.Tele;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class IntakeTest extends OpMode {
   public CRServo intakeLeft;
   public CRServo intakeRight;
   public Servo intakeServo;




   @Override
   public void init(){
   intakeLeft=hardwareMap.get(CRServo.class,"intakeLeft");
   intakeRight=hardwareMap.get(CRServo.class,"intakeRight");
   intakeServo=hardwareMap.get(Servo.class,"intakeServo");

   intakeLeft.setDirection(CRServo.Direction.FORWARD);
   intakeRight.setDirection(CRServo.Direction.REVERSE);
}

   @Override
   public void loop() {



      intakeServo.setPosition(0.7);
      intakeLeft.setPower(-1.0); // 0.8 for not spitting but 1.0 maybe stop jam
      intakeRight.setPower(-1.0);
      sleep(200);
      intakeLeft.setPower(1.0); // 0.8 for not spitting but 1.0 maybe stop jam
      intakeRight.setPower(1.0);
      sleep(4000000);

   }

   private void sleep(double time) {
      sleep(time);
   }

}

