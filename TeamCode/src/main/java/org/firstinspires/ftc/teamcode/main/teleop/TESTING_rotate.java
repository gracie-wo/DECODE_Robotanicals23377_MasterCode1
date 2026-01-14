package org.firstinspires.ftc.teamcode.main.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "ROTATE", group = "Testing")
public class TESTING_rotate extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        Servo rotator = hardwareMap.get(Servo.class, "rotator");


        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();


        if (isStopRequested()) return;

        while (opModeIsActive()) {

            if(gamepad1.right_bumper){
                //red
                rotator.setPosition(0.57);
            }

            if(gamepad1.left_bumper){
                //blue
               rotator.setPosition(0.38);
            }



        }


    }
}