package org.firstinspires.ftc.teamcode.tuning;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.acmerobotics.dashboard.config.Config;


@Config
@Autonomous(name = "PID Test", group = "RoadRunner")
public class PIDTuning extends LinearOpMode{
    DcMotorEx flywheel;

    ElapsedTime timer = new ElapsedTime();
    private double lastError = 0;
    private double integralSum = 0;
    public static double Kp = 0.0;
    public static double Ki = 0.0;
    public static double Kd = 0.0;
    public static double Kf = 0.0;


    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    public static double targetPosition = 5000;

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap);
        TelemetryPacket packet = new TelemetryPacket();
        dashboard.setTelemetryTransmissionInterval(25);
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        flywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flywheel.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();
        int targetPosition = 5000;
        while(opModeIsActive()){
            double power = PIDControl(targetPosition, flywheel.getCurrentPosition());
            packet.put("power", power);
            packet.put("position", flywheel.getCurrentPosition());
            packet.put("error", lastError);

            flywheel.setPower(power);
            dashboard.sendTelemetryPacket(packet);


        }

    }

    public double PIDControl(double reference, double state){
        double error = reference - state;
        integralSum += error * timer.seconds();
        double derivative = (error - lastError) / timer.seconds();
        lastError = error;

        timer.reset();

        double output = (error * Kp) + (derivative * Kd) + (integralSum * Ki) + (reference * Kf);
        return output;
    }
}