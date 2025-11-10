package org.firstinspires.ftc.teamcode.main;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.TouchSensor;

@TeleOp(name = "launcher", group = "Testing")
public class TESTING_launcher extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor launcher = hardwareMap.dcMotor.get("launch");

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();



        if (isStopRequested()) return;

        while (opModeIsActive()) {

            if(gamepad1.a){
                launcher.setPower(1);
            }

            if(gamepad1.b){
                launcher.setPower(-1);
            }

            if(gamepad1.x){
                launcher.setPower(0);
            }
        }


    }
}