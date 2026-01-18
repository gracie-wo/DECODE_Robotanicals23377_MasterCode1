package org.firstinspires.ftc.teamcode.main.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Intake Only", group = "Testing")
public class TESTING_intakeOnly extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        DcMotor intake = hardwareMap.dcMotor.get("intake");
        Servo spindex = hardwareMap.get(Servo.class, "spindex");

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();


        if (isStopRequested()) return;

        while (opModeIsActive()) {

            if(gamepad1.y){
                intake.setPower(1);
            }

            if(gamepad1.x){
                intake.setPower(0);
            }

            if(gamepad1.b){
                intake.setPower(-1);
            }

            if(gamepad1.dpad_up){
                spindex.setPosition(0);
            }

            if(gamepad1.dpad_right){
                spindex.setPosition(0.43);
            }


            if(gamepad1.dpad_down){
                spindex.setPosition(0.87);
            }



        }


    }
}