package org.firstinspires.ftc.teamcode.main.auto;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.limelightvision.LLFieldMap;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorControllerEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumDrive;

import java.util.List;

@Config
@Autonomous(name = "Blue Front (Far)", group = "Autonomous")
public class BlueBack extends LinearOpMode {
    private long startTime;
    private void initTime(){
        startTime = System.currentTimeMillis();
    }

    private boolean hasBeenTime(int milli){
        return (System.currentTimeMillis() - startTime) >= milli;
    }

    private ElapsedTime timer = new ElapsedTime();

    //    //------------------------------------MOTORS--------------------------------------------
    public class Intake {
        private DcMotorEx intake;

        public Intake(HardwareMap hardwareMap){
            intake = hardwareMap.get(DcMotorEx.class, "intake");
            intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        public class IntakeOn implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                intake.setPower(1);
                return false;
            }
        }

        public Action intakeOn() {
            return new IntakeOn();
        }

        public class IntakeOff implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                intake.setPower(0);
                return false;
            }
        }

        public Action intakeOff() {
            return new IntakeOff();
        }
    }

    public class Launcher {
        private DcMotorEx launcher;

        private double voltChange;

//         final double NEWR_P = 5.0;
//         final double NEWR_I = 0.2;
//         final double NEWR_D = 0.7;
//         final double NEWR_F = 11;

        public Launcher(HardwareMap hardwareMap){
            launcher = hardwareMap.get(DcMotorEx.class, "launcher");
            launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//             PIDFCoefficients pidfNewR = new PIDFCoefficients(NEWR_P, NEWR_I, NEWR_D, NEWR_F);
//            launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfNewR);
            VoltageSensor controlHubVoltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");
            voltChange = voltSpeed(controlHubVoltageSensor);

        }

        public class LaunchOn implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                double launchPower = (0.0025 * 185) + voltChange;
                launcher.setPower(launchPower);
                return false;
            }
        }

        public Action launchOn() {
            return new LaunchOn();
        }

        public class LaunchOff implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                launcher.setPower(0);
                return false;
            }
        }

        public Action launchOff() {
            return new LaunchOff();
        }
    }

    //----------------------------SERVOS---------------------------------------------------
    public class Spindex {
        private Servo spindex;
        private boolean initialized = false;

        public Spindex(HardwareMap hardwareMap){
            spindex = hardwareMap.get(Servo.class, "spindex");
        }

        //intake
        public class SpindexIntakeOne implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                //old value - 0.085
                spindex.setPosition(0.1);
                return false;
            }
        }

        public Action spindexIntakeOne(){
            return new SpindexIntakeOne();
        }


        public class SpindexIntakeTwo implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                //old value = 0.52
                spindex.setPosition(0.56);
                return false;
            }
        }

        public Action spindexIntakeTwo(){
            return new SpindexIntakeTwo();
        }


        public class SpindexIntakeThree implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                //old value - 0.96
                spindex.setPosition(1.0);
                return false;
            }
        }

        public Action spindexIntakeThree(){
            return new SpindexIntakeThree();
        }

        //launch
        public class SpindexLaunchOne implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                spindex.setPosition(0.04);
                return false;
            }
        }

        public Action spindexLaunchOne(){
            return new SpindexLaunchOne();
        }

        public class SpindexLaunchTwo implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                spindex.setPosition(0.45);
                return false;
            }
        }

        public Action spindexLaunchTwo(){
            return new SpindexLaunchTwo();
        }

        public class SpindexLaunchThree implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                spindex.setPosition(0.87);
                return false;
            }
        }

        public Action spindexLaunchThree(){
            return new SpindexLaunchThree();
        }
    }

    public class KickerCont {
        private CRServo kickerCont;

        public KickerCont(HardwareMap hardwareMap){
            kickerCont = hardwareMap.get(CRServo.class, "kicker2");
        }

        public class KickerContOn implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                kickerCont.setPower(1);
                return false;
            }
        }

        public Action kickerContOn(){
            return new KickerContOn();
        }


        public class KickerContOff implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                kickerCont.setPower(0);
                return false;
            }
        }

        public Action kickerContOff(){
            return new KickerContOff();
        }
    }

    public class KickerRotate {
        private Servo kickerRotate;

        public KickerRotate(HardwareMap hardwareMap){
            kickerRotate = hardwareMap.get(Servo.class, "kicker1");
        }

        public class KickerRotateUp implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                kickerRotate.setPosition(0.6);
                return false;
            }
        }

        public Action kickerRotateUp(){
            return new KickerRotateUp();
        }


        public class KickerRotateDown implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                kickerRotate.setPosition(0.3);
                return false;
            }
        }

        public Action kickerRotateDown(){
            return new KickerRotateDown();
        }

        public class KickerRotateDownInit implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                kickerRotate.setPosition(0.3);
                return false;
            }
        }

        public Action kickerRotateDownInit(){
            return new KickerRotateDownInit();
        }
    }

    public class Rotator {
        private Servo rotator;
        private double launchPosition = 0.8;
        private boolean adjusted = false;

        public Rotator(HardwareMap hardwareMap){
            rotator = hardwareMap.get(Servo.class, "rotator");
        }

        public class Rotate implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                rotator.setPosition(0.38);
                return false;
            }
        }

        public Action rotate(){
            return new Rotate();
        }
    }

    @Override
    public void runOpMode() {

        //instantiate at (0,0)
        Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(0));

        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);


        TrajectoryActionBuilder nothing = drive.actionBuilder(initialPose)
                .strafeToConstantHeading (new Vector2d(0, 25));


        Action move = nothing.build();
        waitForStart();
        if (isStopRequested()) return;


        // ------------------------- RUN AUTO -------------------------
        Actions.runBlocking(
                new SequentialAction(
                        move
                )
        );


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