import numpy as np
from stable_baselines3 import PPO

class PolicyExecutor:
    """
    A ROS-independent class to load a Stable Baselines3 model
    and predict actions from observations.
    """
    def __init__(self, model_path: str):
        """
        Initializes the PolicyExecutor by loading the trained model.
        Args:
            model_path (str): The absolute path to the .zip file of the trained model.
        """
        try:
            self.model = PPO.load(model_path)
            print(f"Successfully loaded model from {model_path}")
        except Exception as e:
            print(f"Error loading model: {e}")
            self.model = None

    def predict_action(self, observation: np.ndarray) -> np.ndarray | None:
        """
        Predicts an action based on the given observation.
        Args:
            observation (np.ndarray): The observation from the environment.
        Returns:
            np.ndarray | None: The predicted action, or None if the model is not loaded.
        """
        if self.model is None:
            return None
        
        # Use deterministic=True for deployment to get the most likely action
        action, _states = self.model.predict(observation, deterministic=True)
        return action