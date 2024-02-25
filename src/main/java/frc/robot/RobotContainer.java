// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Commands.IntakeSetAng;
import frc.robot.Commands.IntakeSetSpeed;
import frc.robot.Subsystems.IntakeSubsystem;

public class RobotContainer {

  CommandXboxController DriveController = new CommandXboxController(0);
  //CommandXboxController coDriveController = new CommandXboxController(1);
  //Trigger xButton = DriveController.x();  
  //Trigger yButton = DriveController.y();  
  //Trigger bButton = DriveController.b();  
  //Trigger ltrigger = DriveController.leftTrigger();
  //Trigger rtrigger = DriveController.rightTrigger();
  //Trigger noteAquired = new Trigger(IntakeSubsystem.getInstance()::getHasNote);
  //Trigger dPadUp = DriveController.povUp();
  //Trigger dPadDown = DriveController.povDown();

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {

    //xButton.onTrue(new IntakeSetAng(IntakeAng.Speaker));
    //yButton.onTrue(new IntakeSetAng(IntakeAng.Amp));
    //bButton.onTrue(new IntakeSetAng(IntakeAng.Extended));

    //ltrigger.and(noteAquired.negate())
    //  .whileTrue(new IntakeSetSpeed(IntakeSpeed.In));
    //rtrigger.and(IntakeSubsystem.getInstance()::getAtSetpoint)
    //  .whileTrue(new IntakeSetSpeed(IntakeSpeed.Out));

    

    //dPadUp.onTrue(new ClimberExtend());
    //dPadDown.onTrue(new ClimberRetract());
}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
