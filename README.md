
![blocks_logo](https://github.com/Team135BlackKnights/135-Blocks/assets/49589065/488cddd6-688f-4a2d-b0a6-cc395535b318)

# 135 Blocks
This is designed to be a template-based framework for FRC programming.

This is designed so that the only files that need changing to combine different subsystems is RobotContainer.java if used properly.

### Built-in Features for Each Block
- Drivetrain support
  - Swerve
  - Mechanum
  - Tank
- Multi-Vendor support
  - CTRE Motors
  - REV Spark Maxes
  - REV Spark Flexes
- Motor Constant Container
  - Prevent mistakes/crash-causing errors from SysID via checks
  - Put all SysID in ONE place
- Dynamic PID Tuning 
  - Loggable Tuning Numbers allow updates
- SysId Selector
  - Use a testing controller to select SysId tests on different subsystems
  - Automatically appends all tests needed for the new block to the selector
- Drive to Pose Command
  - Slow/Fast mode, where each has custom max accel/velocity
  - Go to a given Pose2d
  - Pathfinds on its own using ADStar planning
- Aim to Pose Command
  - Aim at a given Pose2d
  - Works IN PARALLEL with both TeleOp drive/Pathplanner drive  
- Orchestra
  - Orchestra for CTRE
  - Plays a given `.chrp` file to ALL connected TalonFXs



### SubsystemChecker Overhaul: 
The base branch includes an overhauled Subsystem system named `SubsystemChecker`. It provides system checking and outputs data to Advantage Scope, where all of this data is visible. All Subsystems should extend `SubsystemChecker` and supply their own TestCommand to confirm ALL functionality of that subsystem, that way Advantage Scope can run everything on its own. 
All SubsystemChecker extensions include:
- Self-Checking hardware, which throws errors to the Advantage Scope if any faults occur ANYWHERE on the device.
  - Talon FX
  - CAN Spark Base
  - Pigeon 2 / NavX 2
  - CANCoder
  - PWMMotor
- Easy to use SystemCheckCommand, which will run THAT subsystem's check, confirming functionally by throwing faults if speeds/positions are off.
- Built-in Orchestra, allowing easy Orchestra use.

### Directory Structure
Preferably, do NOT touch any of the `drive/*` contents, as that could cause a merge error!
##### However, some files do need to change interactions with the base, so it is understandable in some situations.

An example branch directory could be:

```
    src/main/java/frc/robot/
        commands/
        drive/
            ...
        exampleBranchName/
            exampleCommand.java
        subsystems/
        drive/
            ...
        exampleBranchName/
            exampleSubsystem.java
        utils/
        drive/
            ...
        exampleBranchName/
            Constants.java
        Constants.java
    Main.java
    Robot.java
    RobotContainer.java
```

## Usage

Refer to the [PyDriverStation](https://github.com/Team135BlackKnights/PyDriverStation) repository for instructions on how to use the API for the neural network.


## Credits

We'd like to extend our deepest gratitude to FRC teams 254, 449, 1678, 1690, 3015, 5516, 6328 and so many others for their code and ideas. Links to their codebases are here:  
[254  (The Cheesy Poofs)](https://github.com/Team254): Setpoint Generator (accel limiting)  
[1690 (Orbit)](https://github.com/team1690): Skid Detection Algorithm  
[3015 (Ranger Robotics)](https://github.com/3015rangerrobotics): Pathplanner
[6328 (Mechanical Advantage)](https://github.com/Mechanical-Advantage): Advantage Kit, Advantage Scope, 250 Hz Advanced Skeleton Swerve, URCL, DriveToPose  
[ChatGPT](https://chat.openai.com/): For helping us feel not alone

## Blocks
### Wrappers

All blocks need two wrappers to be used in order to function:
- `MotorConstantContainer`: A wrapper that holds characterization values (ks, kv, ka, kp, kd) of a particular motor. Throws an error if an incorrect value is input.
- `IO`: A wrapper that allows for efficient communication with component. Needs to be written independently for each component. Check the `utils` folders for example implementations.

## MatchState 

 This value lets you see what part of the FRC Match the robot is in, including endgame/test. Works during practice matches.

## Workflow
 This repository is designed so that multiple programmers can merge their work from separate branches without any issues. In order to accomplish this, create a folder with the name of each branch under all of the `utils` folders.  If this is done correctly, the only merge issues that  
 need to be resolved should be in RobotContainer.java.
