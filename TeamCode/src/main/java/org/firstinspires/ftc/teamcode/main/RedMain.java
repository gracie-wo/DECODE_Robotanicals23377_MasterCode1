package org.firstinspires.ftc.teamcode.main;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorControllerEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

@TeleOp(name = "Red Main", group = "Main")
public class RedMain extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        NormalizedColorSensor sensor = hardwareMap.get(NormalizedColorSensor.class, "colorSensor");
        double hue = 0.0;

        DcMotor frontLeft = hardwareMap.dcMotor.get("frontLeft");
        DcMotor backLeft = hardwareMap.dcMotor.get("backLeft");
        DcMotor frontRight = hardwareMap.dcMotor.get("frontRight");
        DcMotor backRight = hardwareMap.dcMotor.get("backRight");

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);


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


        GoBildaPinpointDriver odo = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
//OFFSETS NEED TO BE CHANGED
        odo.setOffsets(44, 60.2, DistanceUnit.MM);


        DcMotor intake = hardwareMap.dcMotor.get("intake");

        Servo kicker_rotate = hardwareMap.get(Servo.class, "kicker1");
        CRServo kicker_continuous = hardwareMap.get(CRServo.class, "kicker2");
        Servo spindex = hardwareMap.get(Servo.class, "spindex");
        Servo rotator = hardwareMap.get(Servo.class, "rotator");

        DcMotor launcher = hardwareMap.dcMotor.get("launcher");
        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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


        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(2);
        LLResult llResult = limelight.getLatestResult();
        limelight.start();

        //rotator positions
        boolean camera_on = false;
        //basically center = 0.8
        //range rn: 0.25 - 1
        double launchPosition = 0.38;

        //intake and launch kicker
        boolean in_position = false;
        int ready = 0;
        boolean launchOn = false;
        int rotate_state = 0;
        int current_state = 0;
        int kicker_start = 0;
        ElapsedTime timer = new ElapsedTime();

        //intake
        int intake_position = 0;
        boolean spinToLaunch = false;

        //launching
        boolean launchDistanceChange = false;


        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            NormalizedRGBA colors = sensor.getNormalizedColors();
            hue = JavaUtil.colorToHue(colors.toColor());

            //limelight
            llResult = limelight.getLatestResult();

            if(hue < 350 && hue > 225){
                telemetry.addData("Color Detected:", "Purple");
                telemetry.update();
            }

            if(hue > 90 && hue < 225){
                telemetry.addData("Color Detected:", "Green");
                telemetry.update();
            }

            if(hue > 350 || hue < 90){
                telemetry.addData("Color Detected:", "NONE");
            }

            //run kicker quickly
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


            double y = gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = -gamepad1.left_stick_x;
            double rx = -gamepad1.right_stick_x;


            // This button choice was made so that it is hard to hit on accident,
            // it can be freely changed based on preference.
            // The equivalent button is start on Xbox-style controllers.
            if (gamepad1.options) {
                imu.resetYaw();
            }

            if (gamepad2.options){
                imu.resetYaw();
            }

            double botHeading = 0.0;

            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.1;  // Counteract imperfect strafing

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            frontLeft.setPower(frontLeftPower* 1);
            backLeft.setPower(backLeftPower* 1);
            frontRight.setPower(frontRightPower* 1);
            backRight.setPower(backRightPower* 1);


            //gamepad1
            //intake
            if(gamepad1.y){
                in_position = false;
                intake.setPower(1);
            }

            if(gamepad1.dpad_up){
                in_position = false;
                spindex.setPosition(0.1);
            }

            if(gamepad1.dpad_right){
                in_position = false;
                spindex.setPosition(0.56);
            }

            if(gamepad1.dpad_down){
                in_position = false;
                spindex.setPosition(1);
            }

            if(gamepad1.dpad_left){
                spindex.setPosition(0);
                in_position = true;
            }

            if(gamepad1.x){
                in_position = false;
                intake.setPower(0);
            }

            if(gamepad2.a){
                spindex.setPosition(1);
            }

            //turn camera on/off
            if(gamepad1.right_bumper){
                in_position = false;
                rotator.setPosition(0.57);
                camera_on = true;
            }

            if (camera_on && llResult != null && llResult.isValid()){
                Pose3D botPose = llResult.getBotpose_MT2();
                telemetry.addData("Distance", getDistanceFromTags(llResult.getTa()));
                telemetry.addData("Tx" , llResult.getTx());
                telemetry.addData("Ty", llResult.getTy());
                telemetry.addData("Ta", llResult.getTa());
                telemetry.addData("BotPose", botPose.toString());
//            telemetry.addData("Yaw", botPose.getOrientation().getYaw());
                telemetry.update();
            }

            if(gamepad1.left_bumper){
                in_position = false;
                camera_on = false;
                telemetry.clear();
                telemetry.addData("Camera:", "Off");
                telemetry.update();
            }


            if(camera_on && llResult.getTx() < -1){
                if(launchPosition > 0.25){
                    launchPosition -= 0.0001;
                    rotator.setPosition(launchPosition);
                }
            }

            if(camera_on && llResult.getTx() > 1){
                if(launchPosition < 1){
                    launchPosition += 0.0001;
                    rotator.setPosition(launchPosition);
                }
            }

            //gamepad2
            //kicker up to spindex (automatic)
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

            //power launch motor
            if(gamepad2.dpad_up){
                in_position = false;
                launchDistanceChange = true;
                //may need to delete
                voltChange = voltSpeed(controlHubVoltageSensor);

            }

            if(gamepad2.dpad_down){
                in_position = false;
                launchDistanceChange = false;
                launcher.setPower(0);
            }

            if(launchDistanceChange && llResult != null && llResult.isValid()){
                double distance = getDistanceFromTags(llResult.getTa());
                double launchPower = (0.0025 * distance) + voltChange;
                launcher.setPower(launchPower);
            }

        }

    }

    public double getDistanceFromTags(double ta){
        //CHANGE SCALE NUM (CALCULATE)

        double scale = 29280.39;
        double distance = Math.sqrt(scale/ta);
        return distance;
    }

    public boolean launch(Servo spindex, boolean launchOn, ElapsedTime timer, CRServo kicker_continuous, Servo kicker_rotate, int rotate_state, int current_state){
        boolean end_state = false;
        int wait_time = 0;

        spindex.setPosition(0.04);
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
                spindex.setPosition(0.45);
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