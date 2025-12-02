package org.firstinspires.ftc.teamcode.main;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.TouchSensor;

@TeleOp(name = "intake w servo", group = "Testing")
public class TESTING_intake_w_servo extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        CRServo intake_rotate = hardwareMap.get(CRServo.class, "intake_rotate");

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();



        if (isStopRequested()) return;

        while (opModeIsActive()) {

            if(gamepad1.a){
                intake_rotate.setPower(1);
            }

            if(gamepad1.b){
                intake_rotate.setPower(-1);
            }

            if(gamepad1.x){
                intake_rotate.setPower(0);
            }
        }


    }
}