# application
In this repository, we store the public versions of the application that was developed in the VARobot project group. It is the followup work based on the mastertheses of Michael Wieneke and Jonas Eilers and their SPEARED Framework. More information about the project group you can find here: https://varobot-pg.github.io/. 
## prequisites
- Unity 2019.3.0f6
- Android development tools
- Windows SDK (in case you want to build for HoloLens)
## Geting started
- Clone this repository
- Open this folder with unity: SPEAREDV2.0
- You can now build the application for your phone using the unity build (alternatively you can also use 'Make and Run' to deploy after build to your phone)
- If you want to program for the Dobot Magician, you can open the 'Dobot' scenario when the app has startet
- If you want to program for the NXT (real robot), you can open the 'Lego Mindstorms' scenario when the app has startet
  - Before doing that you need to deploy and start the BTNavigator lejos program (you can use eclipse with Lejos installed for this purpose) (you can find the program her: LeJOS_Code_for_NXT_Robot\speared)
  - Enable bluetooth on your phone
  - If the program is started on the robot you can connect to the Lego Mindstorms NXT robot via bluetooth
- If you want to program the 'Grab and Drop' scenario you can open it when the app has startet
- Notice for the latter two cases to reset after you have simulated ('Simulate') or executed ('Make and Run') one run. You can reset by presening the related button in the top left corner
