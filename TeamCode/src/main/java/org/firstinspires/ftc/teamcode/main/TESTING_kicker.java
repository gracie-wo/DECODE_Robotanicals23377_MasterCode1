package org.firstinspires.ftc.teamcode.main;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

@Disabled
@TeleOp(name = "2 servos kicker", group = "Testing")
public class TESTING_kicker extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        CRServo kicker1 = hardwareMap.get(CRServo.class, "kicker1");
        CRServo kicker2 = hardwareMap.get(CRServo.class, "kicker2");


        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();


        if (isStopRequested()) return;

        while (opModeIsActive()) {

            if(gamepad1.right_bumper){
                kicker1.setPower(1);
            }

            if(gamepad1.left_bumper){
                kicker1.setPower(-1);
            }

            if(gamepad1.b){
                kicker1.setPower(0);
            }

            if(gamepad1.a){
                kicker2.setPower(1);
            }

            if(gamepad1.x){
                kicker2.setPower(-1);
            }
            


        }


    }
}