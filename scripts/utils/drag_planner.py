import numpy as np
import yaml
import collision
import matplotlib.pyplot as plt

from corgipath.collision import BoundingVolumeHierarchy
from corgipath.planning import HybridAstar
from corgipath.search_space import DefaultHybridGrid, DefaultHybridNode, HybridSuccessor
from corgipath.matplot import static_draw as draw
from corgipath.matplot import live_draw as live
from corgipath.matplot.utils import pick_color, auto_scale
from scipy.interpolate import interp1d
from typing import Tuple, Dict, List

class StableTopContactPushServer:
    def __init__(self, velocity_candidate):
        with open('../config/config.yaml', 'r') as f:
            self.config = yaml.load(f, Loader=yaml.FullLoader)
            
        self.world_bound    = self.config['planner']['world_bound']
        self.start          = self.config['planner']['start']
        self.goal           = self.config['planner']['goal']
        self.grid_size      = self.config['planner']['grid_size']

        self.planner = HybridAstar()
        self.planner.collision_system = self._get_collision_system()
        self.planner.collision_system.build()
        self.planner.search_space = self._get_search_space(self._get_custom_successor_template(velocity_candidate))
        self.planner.search_space.reset()

    def _get_collision_system(self):
        bvh = BoundingVolumeHierarchy(bounds=self.world_bound)        
        bvh.agent_collision = collision.Poly.from_box(collision.Vector(0.0, 0.0), self.config['pullee']['WIDTH'], self.config['pullee']['HEIGHT'])
        return bvh
    
    def _get_custom_successor_template(self, velocity_candidate):
        assert velocity_candidate.shape[1] == 3, "velocity_candidate shape should be (n, 3)"
        # In Hybrid A*, minimum distance to forward >= diagonal length.
        diagonal = np.sqrt(2.0) * self.grid_size
        forward = (diagonal, diagonal)
        edges = (forward,)

        # Make template
        template = []
        for displacement, cost in edges:
            for v in velocity_candidate:
                template.append(HybridSuccessor.from_heading_with_dist(v[2], self.config['dragger']['unit_v_speed'], cost))
            # template.append(HybridSuccessor.from_heading_with_dist(0.0, self.config['dragger']['unit_v_speed'], cost))
            # template.append(HybridSuccessor.from_heading_with_dist(-np.pi/10, self.config['dragger']['unit_v_speed'], cost))
            # template.append(HybridSuccessor.from_heading_with_dist(-np.pi/3, self.config['dragger']['unit_v_speed'], cost))
            # template.append(HybridSuccessor.from_heading_with_dist(np.pi/10, self.config['dragger']['unit_v_speed'], cost))
            # template.append(HybridSuccessor.from_heading_with_dist(np.pi/3, self.config['dragger']['unit_v_speed'], cost))
            # template.append(HybridSuccessor.from_heading_with_dist(-np.pi/5, self.config['dragger']['unit_v_speed'], cost))
        return template
    
    def _get_search_space(self, successor_template):
        # Define the search space
        search_space = DefaultHybridGrid(
            dxy=self.grid_size,
            dtheta=np.radians(1), node_type=DefaultHybridNode)
        search_space.successor_template = successor_template
        return search_space

    @staticmethod
    def _cartesian_heuristic(query: DefaultHybridNode, goal: DefaultHybridNode) -> float:
        qx, qy, qrad = query.xyt
        gx, gy, grad = goal.xyt
        qxy = np.array((qx, qy))
        gxy = np.array((gx, gy))
        dist_error = np.linalg.norm(gxy - qxy)
        angle_error = np.abs(qrad - grad)
        
        # Heuristic score
        k = 1e-1
        h_score = dist_error + k / (dist_error + 1e-5) * angle_error
        return h_score
    
    @staticmethod
    def _cartesian_terminal_condition(query: DefaultHybridNode, goal: DefaultHybridNode) -> bool:
        qx, qy, qrad = query.xyt
        gx, gy, grad = goal.xyt
        # print("====================")
        # print(qx, qy, qrad)
        # print(gx, gy, grad)
        # print(np.linalg.norm(np.array((qx, qy)) - np.array((gx, gy))) < 0.01)
        # print(np.abs(qrad - grad) < 0.01)
        # print(np.linalg.norm(np.array((qx, qy)) - np.array((gx, gy))) < 0.01 and np.abs(qrad - grad) < 0.01)
        # return np.linalg.norm(np.array((qx, qy)) - np.array((gx, gy))) < 0.01
        # return np.abs(qrad - grad) < 0.01
        return np.linalg.norm(np.array((qx, qy)) - np.array((gx, gy))) < 0.01 and np.abs(qrad - grad) < 0.01
    
    @staticmethod
    def live_draw_options(ax):
        styles = {
            "focus_current_node": {
                "color": "r",
                "fill": False,
                "coordinates_type": "directional circle",
                "coordinates_size": 0.05,
            },
            "open_list": {
                "color": "gold",
                "fill": False,
                "coordinates_type": "directional circle",
                "coordinates_size": 0.05,
            },
            "path_reconstruction": {
                "color": "b",
                "fill": True,
                "coordinates_type": "directional circle",
                "coordinates_size": 0.05,
            },
        }
        live_draw_options = {
            "focus_current_node": live.LiveDrawOption(
                draw_func=lambda xyt: draw.draw_coordinates(ax, xyt, style=styles["focus_current_node"]),
                # pause_after=0.05,
                # wait_key=True,
            ),
            "open_list": live.LiveDrawOption(
                draw_func=lambda xyt: draw.draw_coordinates(ax, xyt, style=styles["open_list"]),
            ),
            "path_reconstruction": live.LiveDrawOption(
                draw_func=lambda xyt: draw.draw_coordinates(ax, xyt, style=styles["path_reconstruction"]),
                pause_before=0.1,
            ),
        }
        return live_draw_options

    def plan(self):
        fig, ax = plt.subplots()

        # Draw background objects (environment-related information)
        draw.draw_grid(ax, grid=self.planner.search_space, drawing_bounds=self.world_bound, style={"color": "0.8", "linewidth": 0.5})

        # Draw background objects (agent-related objects)
        agent_shape = self.planner.collision_system.agent_collision
        draw.draw_shape(ax, agent_shape, at=self.start, style={"color": pick_color(0.7, "turbo"), "fill": True}) 
        draw.draw_shape(ax, agent_shape, at=self.goal,  style={"color": pick_color(0.7, "rainbow"), "fill": True}) 
        auto_scale(ax)
        if self.config['planner']['live_plot']:
            self.planner.set_live_draw_options(self.live_draw_options(ax))
        waypoints = self.planner.solve(self.start, self.goal, fn_heuristic=self._cartesian_heuristic, fn_terminal_condition=self._cartesian_terminal_condition)

        color = list(pick_color(0.3, "rainbow"))
        color[3] = 0.8  # Set alpha
        draw.draw_waypoints(
            ax,
            waypoints,
            agent_shape,
            show_shape=True,
            shape_style={"color": color, "fill": False},
            show_coordinates=False,
        )

        # # Wait for closing the plot
        plt.pause(30)
        return waypoints
        

