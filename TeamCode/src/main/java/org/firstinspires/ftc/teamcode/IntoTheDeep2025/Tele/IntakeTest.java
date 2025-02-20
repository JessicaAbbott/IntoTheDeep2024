package org.firstinspires.ftc.teamcode.IntoTheDeep2025.Tele;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
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
      intakeServo.setPosition(0.2);
      intakeServo.setPosition(0.2);
      intakeServo.setPosition(0.7);
      intakeServo.setPosition(0.2);
      intakeServo.setPosition(0.7);
      intakeServo.setPosition(0.2);
    intakeServo.setPosition(0.7);
      intakeServo.setPosition(0.2);
    intakeServo.setPosition(0.7);
      intakeServo.setPosition(0.2);
      intakeServo.setPosition(0.7);
      intakeServo.setPosition(0.2);
     intakeServo.setPosition(0.7);
      intakeServo.setPosition(0.2);
  intakeServo.setPosition(0.7);
      intakeServo.setPosition(0.2);
    intakeServo.setPosition(0.7);
      intakeServo.setPosition(0.2);

   }

}

