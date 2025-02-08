package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name="ParkRR")
public class BasicAuto extends LinearOpMode {

    public class ScoringClaw{
        private Servo claw;
        public ScoringClaw(HardwareMap hardwareMap){
            claw = hardwareMap.get(Servo.class, "Claw");
        }

        public class Open implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                claw.setPosition(0);
                return false;
            }
        }
        public Action open() {
            return new Open();
        }

        public class Close implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                claw.setPosition(1);
                return false;
            }
        }
        public Action close() {
            return new Close();
        }
    }


    @Override
    public void runOpMode(){
        /* X+ is to the right
         *  Y+ is away from you
         *  0 Heading is towards back of field */
        Pose2d startpose = new Pose2d(-24, 72, Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startpose);
        ScoringClaw scoringClaw = new ScoringClaw(hardwareMap);

        TrajectoryActionBuilder park = drive.actionBuilder(startpose)
                .strafeTo(new Vector2d(-69, 72))
                .waitSeconds(1);


        Actions.runBlocking(new ParallelAction(
                scoringClaw.close()
        ));

        waitForStart();

        if (isStopRequested())return;

        Actions.runBlocking( new ParallelAction(
                park.build(),
                scoringClaw.open()
        ));

    }
}