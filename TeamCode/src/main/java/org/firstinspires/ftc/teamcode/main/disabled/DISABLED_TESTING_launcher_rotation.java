package org.firstinspires.ftc.teamcode.main.disabled;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled
@TeleOp(name = "launcher rotation", group = "Testing")
public class DISABLED_TESTING_launcher_rotation extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Servo rotator = hardwareMap.get(Servo.class, "rotator");

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();



        if (isStopRequested()) return;

        while (opModeIsActive()) {

            if(gamepad1.a){
                rotator.setPosition(1);
            }

            if(gamepad1.b){
                rotator.setPosition(0);
            }

            if(gamepad1.x){
                rotator.setPosition(0.5);
            }
        }


    }
}