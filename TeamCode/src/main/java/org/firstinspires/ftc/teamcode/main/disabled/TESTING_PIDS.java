package org.firstinspires.ftc.teamcode.main.disabled;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorControllerEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

@Disabled
@TeleOp(name = "LimeLight PIDS", group = "Tele Op")
public class TESTING_PIDS extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        Servo rotator = hardwareMap.get(Servo.class, "rotator");

        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(1);
        LLResult llResult = limelight.getLatestResult();

        DcMotor launcher = hardwareMap.dcMotor.get("launcher");
        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Servo kicker_rotate = hardwareMap.get(Servo.class, "kicker1");
        CRServo kicker_continuous = hardwareMap.get(CRServo.class, "kicker2");
        Servo spindex = hardwareMap.get(Servo.class, "spindex");

        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        VoltageSensor controlHubVoltageSensor;
        controlHubVoltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");
        double voltChange = voltSpeed(controlHubVoltageSensor);

        final double NEWR_P = 5.0;
        final double NEWR_I = 0.2;
        final double NEWR_D = 0.7;
        final double NEWR_F = 11.0;

        DcMotorControllerEx motorControllerExR = (DcMotorControllerEx)launcher.getController();
        int motorIndexR = ((DcMotorEx)launcher).getPortNumber();

        PIDFCoefficients pidfOrigR = motorControllerExR.getPIDFCoefficients(motorIndexR, DcMotor.RunMode.RUN_USING_ENCODER);

        PIDFCoefficients pidfNewR = new PIDFCoefficients(NEWR_P, NEWR_I, NEWR_D, NEWR_F);
        motorControllerExR.setPIDFCoefficients(motorIndexR, DcMotor.RunMode.RUN_USING_ENCODER, pidfNewR);

        PIDFCoefficients pidfModifiedR = motorControllerExR.getPIDFCoefficients(motorIndexR, DcMotor.RunMode.RUN_USING_ENCODER);


        ElapsedTime timer = new ElapsedTime();

        boolean camera_on = false;
        boolean launchDistanceChange = false;
        double launchPosition = 0.5;
        int omg = 0;
        int position = 0;
        limelight.start();



        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();


        if (isStopRequested()) return;

        while (opModeIsActive()) {
            llResult = limelight.getLatestResult();

            if (llResult != null && llResult.isValid()) {
                Pose3D botPose = llResult.getBotpose_MT2();
                telemetry.addData("Distance", getDistanceFromTags(llResult.getTa()));
                telemetry.addData("Tx", llResult.getTx());
                telemetry.addData("Ty", llResult.getTy());
                telemetry.addData("Ta", llResult.getTa());
                telemetry.addData("BotPose", botPose.toString());
//            telemetry.addData("Yaw", botPose.getOrientation().getYaw());
                telemetry.update();
            }

            if(gamepad1.right_bumper){
                launchDistanceChange = true;
                position = 1;
                timer.reset();
            }

            if(position == 1 && timer.time() > 3.5){
                kicker_continuous.setPower(1);
                timer.reset();
                kicker_rotate.setPosition(0.6);
            }

            if(gamepad1.left_bumper){
                launchDistanceChange = false;
                launcher.setPower(0);
            }

            if(launchDistanceChange && llResult != null && llResult.isValid()){
                double distance = getDistanceFromTags(llResult.getTa());
                double launchPower = (0.0025 * distance) + voltChange;
                launcher.setPower(launchPower);
            }

            if(gamepad1.dpad_up){
                launcher.setPower(0.475);
            }


            if(gamepad1.dpad_right){
                launcher.setPower(0.45);
            }

            if (gamepad1.dpad_down) {

                launcher.setPower(0);
            }

            if(gamepad1.dpad_left){
                launcher.setPower(0.5);
            }

            if(gamepad1.b){
                kicker_continuous.setPower(1);
                timer.reset();
                kicker_rotate.setPosition(0.6);
                omg = 1;
            }

            if(omg == 1 && timer.time() > 0.8){
                kicker_rotate.setPosition(0.3);
                kicker_continuous.setPower(0);
                omg = 0;
            }

            if(gamepad1.a){
                spindex.setPosition(0);
            }

            if(gamepad1.x){
                spindex.setPosition(0.43);
            }

            if(gamepad1.y){
                spindex.setPosition(0.86);
            }

            if(gamepad2.dpad_up){
                rotator.setPosition(1);
            }

            if(gamepad2.dpad_right){
                rotator.setPosition(0.25);
            }

            if(gamepad2.dpad_down){
                rotator.setPosition(0);
            }
        }


    }

    public double getDistanceFromTags(double ta){
        //CHANGE SCALE NUM (CALCULATE)

        double scale = 29280.39;
        double distance = Math.sqrt(scale/ta);
        return distance;
    }

    public double voltSpeed(VoltageSensor controlHubVoltageSensor){
        double voltage = controlHubVoltageSensor.getVoltage();

        if(voltage >= 13.1){
            return 0.05;
        } else if (voltage >= 12.6){
            return 0.1;
        } else if (voltage >= 12.1){
            return 0.125;
        } else if (voltage >= 11.6){
            return 0.15;
        } else if (voltage >= 11.1){
            return 0.175;
        } else if (voltage >= 10.6){
            return 0.2;
        } else {
            return 0.225;
        }
    }
}