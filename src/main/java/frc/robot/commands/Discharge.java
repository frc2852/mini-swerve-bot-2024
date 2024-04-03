// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class Discharge extends SequentialCommandGroup {
  public Discharge(Intake intake, Conveyor conveyor, Shooter shooter) {
    addCommands(
        // Run the conveyor and shooter again to discharge the game piece
        new ParallelCommandGroup(
            new RunCommand(() -> conveyor.runConveyorForward(), conveyor),
            new RunCommand(() -> shooter.divertGamePiece(), shooter))
            .until(() -> !conveyor.isGamePieceAmpReady()),

        // Wait for 0.5 seconds
        new WaitCommand(0.5),

        // Finally, stop the conveyor and shooter
        new ParallelCommandGroup(
            new InstantCommand(() -> conveyor.stopConveyor(), conveyor),
            new InstantCommand(() -> shooter.stopShooter(), shooter)));
  }
}