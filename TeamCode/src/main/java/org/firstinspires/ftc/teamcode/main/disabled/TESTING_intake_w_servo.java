package org.firstinspires.ftc.teamcode.main.disabled;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

@Disabled
@TeleOp(name = "intake w servo", group = "Testing")
public class TESTING_intake_w_servo extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {


        CRServo sweeper1R = hardwareMap.get(CRServo.class, "sweeper1");
        CRServo sweeper1L = hardwareMap.get(CRServo.class, "sweeper2");
        CRServo sweeper2R = hardwareMap.get(CRServo.class, "sweeper3");
        CRServo sweeper2L = hardwareMap.get(CRServo.class, "sweeper4");

        DcMotor intake = hardwareMap.dcMotor.get("intake");


        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();



        if (isStopRequested()) return;

        while (opModeIsActive()) {

            if(gamepad1.dpad_up){
                intake.setPower(1);
            }


            if(gamepad1.a){
                sweeper1R.setPower(-1);

            }

            if(gamepad1.b){
                sweeper1L.setPower(1);
            }

            if(gamepad1.x){
                sweeper2R.setPower(1);
            }

            if(gamepad1.y){
                sweeper2L.setPower(1);
            }
        }


    }
}