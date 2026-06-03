// import edu.wpi.first.util.sendable.SendableBuilder;
// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.DriverStation.Alliance;

// public class MatchDetails{
//     public double getMatchTime(){
//         return DriverStation.getMatchTime();
//     }

//     public String getFirstActive(){
//         if(DriverStation.getGameSpecificMessage.isEmpty()){
//             return "n/A";
//         }
//         else{
//             switch (DriverStation.getGameSpecificMessage().charAt(0)){
//                 case 'R': return "Blue";
//                 case 'B': return "Red";
//             }
//         }
//     }

//     public Alliance currentActiveAlliance(){
//         if (!DriverStation.isTeleopEnabled()){
//             if (DriverStation.isAutonomousEnabled()){
//                 return DriverStation.getAlliance();
//             }
//             else{
//                 return null;
//             }
//         }
//         else{
//             if (getMatchTime() <= 30.0){
//                 return DriverStation.getAlliance();
//             }
//             else{
//                 if (getMatchTime() <= 55.0 || (80.0 < getMatchTime() && getMatchTime() <= 105.0)){
//                     switch (getFirstActive()){
//                         case "Red": return Alliance.Blue;
//                         case "Blue": return Alliance.Red;
//                     }
//                 }
//                 if ((getMatchTime() <= 130.0 && getMatchTime() > 105.0) || (getMatchTime()))
//             }
//         }
//     }

//     public void initSendable(SendableBuilder builder){
//         super.initSendable(builder);
//         builder.setSmartDashboardType(getName());

//     }
// }