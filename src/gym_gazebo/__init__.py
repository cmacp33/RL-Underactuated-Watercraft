import logging
from gym.envs.registration import register

logger = logging.getLogger(__name__)

# Gazebo
# ----------------------------------------

register(
    id='GazeboCartPole-v0',
    entry_point='gym_gazebo.envs.gazebo_cartpole:GazeboCartPolev0Env',
)

register(
	id='BoatEnv-v0',
	entry_point='boat_gazebo.nodes.boat_env:Boat_Env',
	max_episode_steps=3000,
)

