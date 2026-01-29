package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

// For testing
import edu.wpi.first.wpilibj2.command.PrintCommand;

public class AutoPlans extends SubsystemBase {


    PathPlannerAuto test, otherTest;

    public AutoPlans(){
        registerCommands();

        test = new PathPlannerAuto(new PrintCommand("test"));
        otherTest = new PathPlannerAuto("test");
        
    }

    public void registerCommands(){
        NamedCommands.registerCommand("test",new PrintCommand("test"));
    }

    public Command getAutonomousCommand(int commandIndex){
        switch(commandIndex){
            default:
                return none
            case 0:
                
            case 1:
        }
    }
    
}
