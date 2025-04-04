#!/bin/bash
# Replace <your_service_name> with your Docker service name.

# Open a new Terminal window for the localization launch
osascript -e 'tell application "Terminal" to do script "cd OneDrive/Documents/GitHub/racecar_docker/home/racecar_ws/ && docker compose exec racecar bash -c \"source install/setup.bash && ros2 launch localization localize.launch.xml; exec bash\""' 

# Open a new Terminal window for the wall follower launch
osascript -e 'tell application "Terminal" to do script "docker compose exec racecar bash -c \"source install/setup.bash && ros2 launch wall_follower wall_follower.launch.xml; exec bash\""' 

# Open a new Terminal window for the racecar simulator launch
osascript -e 'tell application "Terminal" to do script "docker compose exec racecar bash -c \"source install/setup.bash && ros2 launch racecar_simulator simulate.launch.xml; exec bash\""' 

# Run the convergence plot subscriber in the current terminal
python3 convergence_plot.py