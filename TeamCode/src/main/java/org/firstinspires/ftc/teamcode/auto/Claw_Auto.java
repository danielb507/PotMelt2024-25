package org.firstinspires.ftc.teamcode.auto;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.auto.actions.ArmActions;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.MecanumDrive;

import kotlin.jvm.internal.TypeParameterReference;

@Config
@Autonomous(name = "BLUE_TEST_AUTO_PIXEL", group = "Autonomous")
public class Claw_Auto extends LinearOpMode {
    HardwareMap hardwareMap;


    @Override
    public void runOpMode() {

        ArmActions armActions = new ArmActions(hardwareMap);
        //ArmActions  Arm = new ArmActions(hardwareMap);

        Pose2d startPose = new Pose2d(37, 63, Math.toRadians(180));
        Pose2d bucketPose = new Pose2d(56, 58, 180);

        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        TrajectoryActionBuilder bucket_traj_1 = drive.actionBuilder(startPose)
                .strafeTo(new Vector2d(startPose.position.x, startPose.position.y - 10))
                //pre-loaded sample
                .strafeToLinearHeading(bucketPose.position, bucketPose.heading)
                .stopAndAdd(new ParallelAction(armActions.deposit()))
                .waitSeconds(1)
                .stopAndAdd(new ParallelAction(armActions.depositReset()))
                //first sample
                .strafeToLinearHeading(new Vector2d(37, 25), 0)
                .stopAndAdd(armActions.runIntake(false))
                .waitSeconds(2)
                .strafeToLinearHeading(bucketPose.position, bucketPose.heading)
                .stopAndAdd(armActions.reverseIntake())
                .stopAndAdd(new ParallelAction(armActions.deposit()))
                .stopAndAdd(new ParallelAction(armActions.depositReset()))
                //second sample
                .strafeToLinearHeading(new Vector2d(50, 25), 0)
                .stopAndAdd(armActions.runIntake(false))
                .waitSeconds(2)
                .strafeToLinearHeading(bucketPose.position, bucketPose.heading)
                .stopAndAdd(armActions.reverseIntake())
                .stopAndAdd(new ParallelAction(armActions.deposit()))
                .stopAndAdd(new ParallelAction(armActions.depositReset()))
                // third sample
                .strafeToLinearHeading(new Vector2d(60, 25), 0)
                .stopAndAdd(armActions.runIntake(false))
                .waitSeconds(2)
                .strafeToLinearHeading(bucketPose.position, bucketPose.heading)
                .stopAndAdd(armActions.reverseIntake())
                .stopAndAdd(new ParallelAction(armActions.deposit()))
                .stopAndAdd(new ParallelAction(armActions.depositReset()));


        while (!isStopRequested() && !opModeIsActive()) {
            telemetry.update();
        }

        telemetry.update();
        waitForStart();

        if (isStopRequested()) return;

        Action bucket_action_1;

        bucket_action_1 = bucket_traj_1.build();

        Actions.runBlocking(
                new SequentialAction(
                        bucket_action_1
                )
        );


    }

}

