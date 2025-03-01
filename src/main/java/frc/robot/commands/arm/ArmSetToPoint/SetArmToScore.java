package frc.robot.commands.arm.ArmSetToPoint;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmConstants;
import frc.robot.RobotContainer;

public class SetArmToScore extends Command {
    private int level;
    public SetArmToScore(int level) { 
        this.level = level;
    }

    @Override
    public void execute() {
        switch (level) {
            case 0:
                // Score pos for trof
                
                break; 
            case 1:
                // Score pos for level 1
                
                break; 
            case 2:
                // Score pos for level 2
                
                break;
            
            case 3:
                // Score pos for level 3

                break;
        } 
    }
}
