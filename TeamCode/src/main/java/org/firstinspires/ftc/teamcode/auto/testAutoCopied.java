/*

// BadBlueBackstage


/* Copyright (c) 2019 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


package org.firstinspires.ftc.teamcode.drive.opmode.Auto;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.MecanumDrive;
//import org.firstinspires.ftc.teamcode.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;


import java.util.List;

/*
 * This OpMode illustrates the basics of TensorFlow Object Detection,
 * including Java Builder structures for specifying Vision parameters.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.

@Autonomous(name = "BlueAudience", group = "Concept")
public class testAutoCopied extends LinearOpMode {

    MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

    private DcMotorEx leftFront = null;
    private DcMotorEx rightRear = null;
    private DcMotorEx leftRear = null;
    private DcMotorEx rightFront = null;


    static final double     COUNTS_PER_MOTOR_REV    = 28 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = 3.75 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.45;
    static final double     TURN_SPEED              = 0.3;
    static final double     SLOW_SPEED = 0.3;


    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
    };

    /**
     * The variable to store our instance of the TensorFlow Object Detection processor.




    @Override
    public void runOpMode() {


        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");



        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftRear.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.FORWARD);

        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
        waitForStart();




        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // -------------------- Right Trajectories -----------
        Trajectory right_traj1 = drive.trajectoryBuilder(new Pose2d())
                .strafeRight(10)
                .build();
        /*TrajectorySequence right_trajTurn1 = drive.trajectorySequenceBuilder(right_traj1.end())
                .turn(Math.toRadians(-90))
                .waitSeconds(17)
                .build();
        Trajectory right_traj2 = drive.trajectoryBuilder(right_traj1.end())
                .forward(26)
                .build();
        Trajectory right_traj3 = drive.trajectoryBuilder(right_traj2.end())
                .back(6)
                .build();
        /*TrajectorySequence right_trajTurn2 = drive.trajectorySequenceBuilder(right_traj3.end())
                .turn(Math.toRadians(-90))
                .build();
        Trajectory right_traj4 = drive.trajectoryBuilder(right_traj3.end())
                .strafeLeft(12)
                .build();
        /*TrajectorySequence right_trajTurn3 = drive.trajectorySequenceBuilder(right_traj4.end())
                .turn(Math.toRadians(-90))
                .build();
        Trajectory right_traj5 = drive.trajectoryBuilder(right_traj4.end())
                .forward(31)
                .build();
        TrajectorySequence right_trajTurn1 = drive.trajectorySequenceBuilder(right_traj5.end())
                .turn(Math.toRadians(87.5))
                .waitSeconds(15)
                .build();
        Trajectory right_traj6 = drive.trajectoryBuilder(right_trajTurn1.end())
                .forward(90)
                .build();

                {
                    drive.followTrajectory(right_traj1);
                    //drive.followTrajectorySequence(right_trajTurn);
                    //drive.followTrajectorySequence(right_trajTurn1);
                    drive.followTrajectory(right_traj2);
                    //drive.followTrajectorySequence(right_trajTurn2);
                    drive.followTrajectory(right_traj3);
                    //drive.followTrajectorySequence(right_trajTurn3);
                    drive.followTrajectory(right_traj4);
                    drive.followTrajectory(right_traj5);
                    drive.followTrajectorySequence(right_trajTurn1);
                    drive.followTrajectory(right_traj6);
                    //armpose(-4);





                    sleep(100000);

                }
        }
        // end runOpMode()

    /**
     * Initialize the TensorFlow Object Detection processor.
     */

        // Choose a camera resolution. Not all cameras support all resolutions.
        //builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.


        // Build the Vision Portal, using the above settings.

        // Set confidence threshold for TFOD recognitions, at any time.
        //tfod.setMinResultConfidence(0.75f);

        // Disable or re-enable the TFOD processor at any time.
        //visionPortal.setProcessorEnabled(tfod, true);

       // end method initTfod()

    /**
     * Add telemetry about TensorFlow Object Detection (TFOD) recognitions.
     */


    /*public void intake(String mode, double power){
        Intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        if (mode == "intake"){
            Intake.setPower(power);
        }

        if (mode == "outtake"){
            Intake.setPower(-power);
        }
        if (mode == "stop"){
            Intake.setPower(0);
        }

    }*/


    /*
        public void armDown(double distance, double power) {

            //Reset Encoders
            slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


            slide.setPower(-power);


            while (-slide.getCurrentPosition() < distance) {
                telemetry.addData("Arm Encoder", slide.getCurrentPosition());
                telemetry.update();
            }

            slide.setPower(0);

            sleep(500);

        }


        public void armUp(double power, String mode) {

            //Reset Encoders
            slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            slide.setPower(power);
            if (mode == "Up"){
                slide.setPower(power);
            }

            if (mode == "Down"){
                slide.setPower(-power);
            }
            if (mode == "stop"){
                slide.setPower(0);
            }



            slide.setPower(0);

            sleep(1000);

        }*/
// end class
