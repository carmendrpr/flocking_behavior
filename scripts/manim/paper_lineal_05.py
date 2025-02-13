"""
paper_lineal_05.py
"""

from manim import *
import numpy as np


class DroneTrajectory(Scene):
    def construct(self):
        # Load an image (ensure the file is in the same directory or provide the full path)
        image = ImageMobject("base_figure.png")  
        image.scale(0.45)
        self.add(image)
        
        # Define initial and final positions for each drone
        drone_0_positions = np.array([[-3.9, -0.02, 0], [3.1, -0.02, 0]])
        drone_1_positions = np.array([[-4.8, 1.5, 0], [2.2, 1.5, 0]])
        drone_2_positions = np.array([[-4.8, -1.60, 0], [2.2, -1.60, 0]])

        # Create initial points
        drone0_initial_point = Dot(drone_0_positions[0], color=BLACK)
        drone1_initial_point = Dot(drone_1_positions[0], color=BLACK)
        drone2_initial_point = Dot(drone_2_positions[0], color=BLACK)

        # Create final points
        drone0_final_point = Dot(drone_0_positions[1], color=BLACK)
        drone1_final_point = Dot(drone_1_positions[1], color=BLACK)
        drone2_final_point = Dot(drone_2_positions[1], color=BLACK)

        # Labels for initial points
        distance = 0.1
        drone0_initial_label = Text(
            "Drone 0", color=BLACK).scale(0.3).next_to(drone0_initial_point, UR, buff=distance)
        drone1_initial_label = Text(
            "Drone 1", color=BLACK).scale(0.3).next_to(drone1_initial_point, UR, buff=distance)
        drone2_initial_label = Text(
            "Drone 2", color=BLACK).scale(0.3).next_to(drone2_initial_point, UR, buff=distance)

        # Labels for final points
        drone0_final_label = Text(
            "Drone 0", color=BLACK).scale(0.3).next_to(drone0_final_point, UR, buff=distance)
        drone1_final_label = Text(
            "Drone 1", color=BLACK).scale(0.3).next_to(drone1_final_point, UR, buff=distance)
        drone2_final_label = Text(
            "Drone 2", color=BLACK).scale(0.3).next_to(drone2_final_point, UR, buff=distance)


        # Create a triangle connecting the initial points
        initial_triangle = Polygon(
            drone_0_positions[0], 
            drone_1_positions[0], 
            drone_2_positions[0],
            color=GREEN,
            fill_color=GREEN,
            fill_opacity=0.7
        )
        final_triangle = Polygon(
            drone_0_positions[1], 
            drone_1_positions[1], 
            drone_2_positions[1],
            color=GREEN,
            fill_color=GREEN,
            fill_opacity=0.7
        )
        self.add(initial_triangle)

        # Draw initial points
        self.add(drone0_initial_point, drone1_initial_point, drone2_initial_point,
                 drone0_initial_label, drone1_initial_label, drone2_initial_label)

        # # Debug
        # grid = NumberPlane(x_range=[-10, 10, 1.0], y_range=[-10, 10, 1.0])
        # self.add(grid)
        # self.add(drone0_final_point, drone1_final_point, drone2_final_point,
        #          drone0_final_label, drone1_final_label, drone2_final_label)
        # self.add(final_triangle)
        # self.wait(1)

        # Animate transition: move the points and adjust the triangle
        self.play(
            Transform(
                initial_triangle, 
                final_triangle
            ),
            Transform(drone0_initial_point, drone0_final_point),
            Transform(drone1_initial_point, drone1_final_point),
            Transform(drone2_initial_point, drone2_final_point),
            Transform(drone0_initial_label, drone0_final_label),
            Transform(drone1_initial_label, drone1_final_label),
            Transform(drone2_initial_label, drone2_final_label),
            run_time=60
        )

        # # Wait after animation
        # self.wait(1)

        
if __name__ == "__main__":
    folder = "manim_media/"
    
    from manim import tempconfig
    with tempconfig({
        "save_first_frame": False,
        "save_last_frame": False,
        "media_dir": folder,
        "frame_rate": 60,
        "quality": "high_quality",
        "pixel_height": 2400,
        "pixel_width": 4200,
        "disable_caching": True,
        "preview": True}):
        DroneTrajectory().render()