package org.firstinspires.ftc.teamcode.actions;
import android.content.pm.LauncherApps;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


public class ArmActions {
    private DcMotor leftSlide;
    private DcMotor rightSlide;
    private CRServo intake;
    private Servo intakePivot;
    private Servo bucketPivot;
    private Servo clawPivot;
    private Servo claw;


    public ArmActions(HardwareMap hardwareMap) {
        leftSlide = hardwareMap.get(DcMotor.class, "leftSlide");
        rightSlide = hardwareMap.get(DcMotor.class, "rightSlide");
        intake = hardwareMap.get(CRServo.class, "intake");
        intakePivot = hardwareMap.get(Servo.class, "flip");
        bucketPivot = hardwareMap.get(Servo.class, "bucket_pivot");
        clawPivot = hardwareMap.get(Servo.class, "hooks");
        claw = hardwareMap.get(Servo.class, "claw");

        leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


    }

    public Action runIntake(boolean slow) {
        return new Action() {
            boolean initalized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initalized) {
                    intakePivot.setPosition(1);
                    if (!slow) {
                        intake.setPower(0.8);
                    } else {
                        intake.setPower(0.2);
                    }
                }
                return initalized;
            }
        };
    }

    ;

    public Action reverseIntake() {
        return new Action() {
            boolean initalized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                intakePivot.setPosition(0);
                if (!initalized) {
                    intake.setPower(-0.2);
                }
                return initalized;
            }
        };
    }

    ;

    public Action raiseArm() {
        return new Action() {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    rightSlide.setPower(0.8);
                    leftSlide.setPower(0.8);
                    initialized = true;
                }

                double pos = leftSlide.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos < 2000) {
                    return true;
                } else {
                    leftSlide.setPower(0);
                    rightSlide.setPower(0);
                    return false;
                }
            }
        };
    }

    public Action deposit() {
        return new Action() {
            private boolean initialized;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    bucketPivot.setPosition(1);
                    while (leftSlide.getCurrentPosition() < 1000 && rightSlide.getCurrentPosition() < 1000) {
                        leftSlide.setPower(1);
                        rightSlide.setPower(1);
                    }
                    leftSlide.setPower(0);
                    rightSlide.setPower(0);
                    bucketPivot.setPosition(0);

                }

                return initialized;
            }
        };
    }

    public Action closeClaw() {
        return new Action() {
            private boolean initialized;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {

                claw.setPosition(0);

                return initialized;
            }
        };
    }

    public Action openClaw() {
        return new Action() {
            private boolean initialized;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    claw.setPosition(0);
                }

                return initialized;
            }
        };
    }

    public Action raiseClaw() {
        return new Action() {
            private boolean initialized;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                clawPivot.setPosition(.5);

                return initialized;
            }
        };
    }

    public Action lowerClaw() {
        return new Action() {
            private boolean initialized;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    clawPivot.setPosition(.43);
                }

                return initialized;
            }
        };
    }

    public Action lowerArm() {
        return new Action() {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    rightSlide.setPower(-0.2);
                    leftSlide.setPower(-0.2);
                    initialized = true;
                }

                double pos = leftSlide.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos > 20) {
                    return true;
                } else {
                    leftSlide.setPower(0);
                    rightSlide.setPower(0);
                    return false;
                }
            }
        };
    }
}
