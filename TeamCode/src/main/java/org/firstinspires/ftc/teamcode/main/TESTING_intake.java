package org.firstinspires.ftc.teamcode.main;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "intake w/ rubber bands", group = "Testing")
public class TESTING_intake extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        DcMotor intake = hardwareMap.dcMotor.get("intake");


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
            if(gamepad1.a){
                intake.setPower(-1);
            }

        }


    }
}