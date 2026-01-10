package org.firstinspires.ftc.teamcode.main;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Disabled
@TeleOp(name = "2 motor (1 encoded) and 1 servo", group = "Testing")
public class TESTING_intake extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        DcMotor intake = hardwareMap.dcMotor.get("intake");
        Servo kicker_rotate = hardwareMap.get(Servo.class, "kicker1");
        CRServo kicker_continuous = hardwareMap.get(CRServo.class, "kicker2");
        Servo spindex = hardwareMap.get(Servo.class, "spindex");
        DcMotor launcher = hardwareMap.dcMotor.get("launcher");
        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        DcMotor rotate = hardwareMap.dcMotor.get("rotate");

        rotate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rotate.setTargetPosition(0);
        rotate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rotate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        boolean in_position = false;
        int ready = 0;
        boolean launchOn = false;
        int rotate_state = 0;
        int current_state = 0;
        int kicker_start = 0;
        ElapsedTime timer = new ElapsedTime();


        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();



        if (isStopRequested()) return;

        while (opModeIsActive()) {

            telemetry.addData("Position", in_position);
            telemetry.update();

//            spindex.setPosition(0.47);

            if(kicker_start == 0){
                timer.reset();
                kicker_continuous.setPower(0.1);
                kicker_rotate.setPosition(0.3);
                kicker_start = 1;
            }

            if(kicker_start == 1 && timer.time() > 0.1){
                kicker_continuous.setPower(0);
                kicker_start = 2;
            }

            if(gamepad1.y){
                in_position = false;
                intake.setPower(1);
            }
            if(gamepad1.x){
                in_position = false;
                intake.setPower(0);
            }

            //kicker put ball up into launcher
            if(gamepad2.b) {
//NOTE TO SELF: WHEN CODING INTAKE SPINDEX PLEASE SET IN_POSITION TO TRUE WHEN IT SPINS TO LUANCH POSITION AT THE END
//PLEASE DON"T FORGET PLEASE PLEASE
                if(!in_position){
                    spindex.setPosition(0.05);
                    ready = 1;
                    timer.reset();
                } else {
                    launchOn = true;
                    in_position = launch(spindex, launchOn, timer, kicker_continuous, kicker_rotate, rotate_state, current_state);
                }
            }

            if(ready == 1 && timer.time() > 0.6){
                ready = 0;
                in_position = true;
                launchOn = true;
                in_position = launch(spindex, launchOn, timer, kicker_continuous, kicker_rotate, rotate_state, current_state);

            }

            if(gamepad2.x){
                in_position = false;
                launcher.setPower(0.75);
            }

            if(gamepad2.y){
                in_position = false;
                launcher.setPower(0);
            }


//            if(gamepad1.dpad_up){
//                rotate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                rotate.setTargetPosition(537);
//                rotate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                rotate.setPower(0.5);
//            }
//            if(gamepad1.dpad_right){
//                rotate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                rotate.setTargetPosition(0);
//                rotate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                rotate.setPower(0.5);
//            }
//            if(gamepad1.dpad_down){
//                rotate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                rotate.setTargetPosition(2150);
//                rotate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                rotate.setPower(0.5);
//            }
//            if(gamepad1.dpad_left){
//                rotate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                rotate.setTargetPosition(1000);
//                rotate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                rotate.setPower(0.5);
//            }

            if(gamepad1.dpad_up){
                in_position = false;
                spindex.setPosition(0.085);
            }

            if(gamepad1.dpad_down){
                in_position = false;
                spindex.setPosition(0.94);
            }

            if(gamepad1.dpad_right){
                in_position = false;
                spindex.setPosition(0.5);
            }

            //for launch positions: 0, 0.85, 0.43
            if(gamepad1.right_bumper){
                spindex.setPosition(0);
                in_position = true;
            }
        }


    }

    //launch spindex positions: 0, 0.43, 0.86
    //launch rotator positions: 0.3, 0.6
    public boolean launch(Servo spindex, boolean launchOn, ElapsedTime timer, CRServo kicker_continuous, Servo kicker_rotate, int rotate_state, int current_state){
        boolean end_state = false;
        int wait_time = 0;

        spindex.setPosition(0);
        kicker_continuous.setPower(1);
        timer.reset();
        kicker_rotate.setPosition(0.6);
        rotate_state = 1;

        while(launchOn) {
            if (rotate_state == 1 && (timer.time() > 0.5)) {
                kicker_rotate.setPosition(0.3);
                current_state++;
                rotate_state = 0;
                timer.reset();

                if(end_state){
                    kicker_continuous.setPower(0);
                    launchOn = false;
                }
            }

            if(current_state == 1 && timer.time() > 0.3){
                spindex.setPosition(0.43);
                timer.reset();
                current_state++;
                wait_time = 1;
            }

            if(current_state == 3 && timer.time() > 0.3){
                wait_time = 1;
                spindex.setPosition(0.87);
                current_state++;
                end_state = true;
                timer.reset();
            }

            if(wait_time == 1 && timer.time() > 0.4){
                kicker_rotate.setPosition(0.6);
                rotate_state = 1;
                wait_time = 0;
                timer.reset();
            }
        }

        return false;
    }
}