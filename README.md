To clear RoboRIO deploy directory to clear NamedCommands in AutoBuilder and auto choices in SendableChooser object:

1) Open terminal
2) ssh -l admin roboRIO-2059-frc.local
3) rm -r /home/lvuser/deploy

Basic vision tuning strats:

1) "For all pipelines, exposure time should be set as low as possible while still allowing for the target to be reliably tracked. This allows for faster processing as decreasing exposure will increase your camera FPS."
2) "Unlike with retroreflective tape, AprilTag tracking is not very dependent on lighting consistency. If you have trouble detecting tags due to low light, you may want to try increasing exposure, but this will likely decrease your achievable framerate."
3) "Cranking your exposure as low as it goes and increasing your gain/brightness. This will decrease the effects of motion blur and increase FPS."
