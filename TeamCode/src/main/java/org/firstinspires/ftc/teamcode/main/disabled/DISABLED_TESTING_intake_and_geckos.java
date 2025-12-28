package org.firstinspires.ftc.teamcode.main.disabled;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;


@Disabled
@TeleOp(name = "intake+geckos", group = "Testing")
public class DISABLED_TESTING_intake_and_geckos extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        CRServo sweeper1R = hardwareMap.get(CRServo.class, "sweeper1");
        CRServo sweeper1L = hardwareMap.get(CRServo.class, "sweeper2");
        CRServo sweeper2R = hardwareMap.get(CRServo.class, "sweeper3");
        CRServo sweeper2L = hardwareMap.get(CRServo.class, "sweeper4");

        DcMotor boot = hardwareMap.dcMotor.get("boot");
        boot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();



        if (isStopRequested()) return;

        while (opModeIsActive()) {

            if(gamepad1.a){
                sweeper1R.setPower(1);
                sweeper1L.setPower(-1);
            }

            if(gamepad1.b){
                sweeper1R.setPower(1);
                sweeper1L.setPower(-1);
                sweeper2R.setPower(1);
                sweeper2L.setPower(-1);
            }

            if(gamepad1.x){
                sweeper1R.setPower(0);
                sweeper1L.setPower(0);
                sweeper2R.setPower(0);
                sweeper2L.setPower(0);
            }

            if(gamepad1.y){
                boot.setPower(0.89);
            }

            if(gamepad1.right_bumper){
                boot.setPower(0);
            }
        }


    }
}