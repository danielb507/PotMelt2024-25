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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.actions.ArmActions;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.MecanumDrive;

import kotlin.jvm.internal.TypeParameterReference;

@Config
@Autonomous(name = "Claw_Autosdlfjdsj", group = "Autonomous")
public class Claw_Auto extends LinearOpMode {

    @Override
    public void runOpMode() {

        Pose2d startPose = new Pose2d(-10, 63, Math.toRadians(90));
        Pose2d subPoseMid = new Pose2d(0, 35, Math.toRadians(90));

        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        if (hardwareMap == null) {
            telemetry.addData("Error", "hardwareMap is not initialized");
            telemetry.update();
            return;
        }

        ArmActions armActions = new ArmActions(hardwareMap);
        //ArmActions  Arm = new ArmActions(hardwareMap);



        TrajectoryActionBuilder traj_1 = drive.actionBuilder(startPose)
                .strafeTo(new Vector2d(subPoseMid.position.x - 10, subPoseMid.position.y + 10));
                /*
                //pre-loaded sample
                .strafeToLinearHeading(bucketPose.position, bucketPose.heading)
                .stopAndAdd(new ParallelAction(armActions.deposit()))
                .waitSeconds(1)
                .stopAndAdd(new ParallelAction(armActions.resetArm()));
                /*
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

                 */

        TrajectoryActionBuilder traj_2 = drive.actionBuilder(startPose)
                        .strafeTo(new Vector2d(startPose.position.x, startPose.position.y));

        Actions.runBlocking(armActions.raiseClaw());
        Actions.runBlocking(armActions.closeClaw());

        while (!isStopRequested() && !opModeIsActive()) {
            telemetry.update();
        }
        telemetry.update();
        waitForStart();

        if (isStopRequested()) return;

        Action trajectory_1;
        Action trajectory_2;

        trajectory_1 = traj_1.build();
        trajectory_2 = traj_2.build();

        Actions.runBlocking(
                new SequentialAction(
                        armActions.raiseArm(),
                        trajectory_1,
                        armActions.lowerArm(3000),
                        trajectory_2,
                        armActions.lowerArm(20),
                        trajectory_2
                )
        );
    }
}
