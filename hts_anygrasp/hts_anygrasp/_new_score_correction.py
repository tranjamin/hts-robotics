#### WHAT TO DO WITH OUT TIMING
    # WE NEED TO KEEP TRACK OF BOTH PICKUP AND PLAN


def correct_scores(self, gg: GraspNetGroup, cloud: o3d.cuda.pybind.geometry.PointCloud, save_folder: str) -> None:
    """
    Corrects the scores of the grasp group.

    Parameters:
        gg: the grasp group
        cloud: the point cloud
        save_folder: the main directory of the data being stored
    """

    original_xyz: npt.NDArray[np.float64] = np.array([g.translation for g in gg])
    geometric_centroid = cloud.get_center()

    # gets the original grasp scores
    original_scores: npt.NDArray[np.float64] = gg.scores
    self.save_grasps_in_polar(gg, save_folder, "grasp_score_original", c=gg.scores)

    # calculate a simplified inertia and mass
    SLICE_LAYER_HEIGHT = 0.02
    total_mass: float = 0
    total_ixx: float = 0
    layer_base_height: float = self.Z_COORDS_MIN
    layer_masses = []
    layer_rsquareds = []
    while (layer_base_height < self.Z_COORDS_MAX):
        self.get_logger().info(f"Slicing layer at height {layer_base_height}")

        # filter cloud and grasps by height
        bb: Any = o3d.geometry.AxisAlignedBoundingBox(
            min_bound=[-math.inf, -math.inf, layer_base_height],
            max_bound=[math.inf, math.inf, layer_base_height + SLICE_LAYER_HEIGHT]
        )
        layer_cloud: Any = cloud.crop(bb)

        if layer_cloud.is_empty():
            self.get_logger().info(f"Skipping... {layer_cloud}")
            layer_masses.append(0)
            layer_base_height += SLICE_LAYER_HEIGHT
            continue

        # approximate layer as disk
        layer_points: npt.NDArray[np.float64] = np.asarray(layer_cloud.points)
        rsquared: npt.NDArray[np.float64] = np.square(layer_points[:,0] - centroid[0]) + np.square(layer_points[:,1] - centroid[1])
        average_rsquared: float= float(np.mean(rsquared))

        # approximate mass
        layer_mass: float = average_rsquared*np.pi*SLICE_LAYER_HEIGHT # alternatively, we could do convex hull
        layer_masses.append(layer_mass)
        layer_rsquareds.append(average_rsquared)
        total_mass += layer_mass

        layer_base_height += SLICE_LAYER_HEIGHT

    # calculate centre of mass from circles
    best_centroid_estimate = 0
    best_centroid_diff = math.inf
    for i in range(len(layer_masses)):
        under_mass = sum(layer_masses[:i])
        over_mass = sum(layer_masses[i:])
        mass_diff = abs(over_mass - under_mass)
        if mass_diff < best_centroid_diff:
            best_centroid_estimate = i*SLICE_LAYER_HEIGHT
            best_centroid_diff = mass_diff
    
    # calculate inertia
    for i, layer_mass in enumerate(layer_masses):
        layer_ixx: float = 1/4*layer_mass*layer_rsquareds[i] + 1/3*layer_mass*(SLICE_LAYER_HEIGHT**2)
        shifted_ixx: float = layer_ixx + layer_mass*((best_centroid_estimate - layer_base_height)**2)
        total_ixx += shifted_ixx

    # calculates the distance away from the centroid
    distance_above_centroid: npt.NDArray[np.float64] = original_xyz[:, 2] - best_centroid_estimate

    # calculates the stable score
    stable_score: npt.NDArray[np.float64] = np.abs(distance_above_centroid)/(np.max(np.abs(distance_above_centroid))*1.5)
    self.save_grasps_in_polar(gg, save_folder, "stable_score", c=stable_score)

    # calculates the grasp score without stable score
    unstable_scores: npt.NDArray[np.float64] = (original_scores/(1 - stable_score)).clip(max=1.0)
    self.save_grasps_in_polar(gg, save_folder, "grasp_score_without_stable", c=unstable_scores)

    # calculate the estimated (sqrt) lambda
    lambdas: npt.NDArray[np.float64] = -np.sign(distance_above_centroid)*np.sqrt(total_mass*9.81*np.abs(distance_above_centroid)/(total_ixx + total_mass*distance_above_centroid**2))

    # normalise lambdas
    if np.max(lambdas):
        lambdas = lambdas/np.abs(np.max(lambdas))

    self.save_grasps_in_polar(gg, save_folder, "lambdas", c=lambdas)

    # lambdas = lambdas.clip(min=0.0)

    # multiply grasp scores by this
    gg.scores = gg.scores*(1 - lambdas)
    self.save_grasps_in_polar(gg, save_folder, "lambda corrected score", c=gg.scores)

    grasp_factors = []

    # now we determine the grasps whose region does not pass through the centre of the object in 2D
    for grasp in gg:
        # we figure out the direction of closing in 2D
        closing_direction = grasp.rotation_matrix[:2, 0]
        grasp_centre = grasp.translation[:2]

        # get key points:
        left_finger_tip = grasp.translation + np.array([grasp.depth, -grasp.width/2, 0]) @ grasp.rotation_matrix.T
        right_finger_tip = grasp.translation + np.array([grasp.depth, grasp.width/2, 0]) @ grasp.rotation_matrix.T
        tip_centre = grasp.translation + np.arary([grasp.depth, 0, 0]) @ grasp.rotation_matrix.T

        # we project it to the perp distance
        dist = grasp_centre - centroid[:2]
        min_dist = dist - np.dot(dist, closing_direction) * closing_direction
        
        # we need to decide whether our grippers are on the opposite side or not
        # min_dist = (1 if sign(min_dist) == sign(min_finger_dist) else -1)*min_finger_dist

        # clip everything
        grasp_factor = np.linalg.norm(min_dist)
        grasp_factors.append(grasp_factor)
    
    grasp_factors = np.array(grasp_factors)
    grasp_factors /= np.max(grasp_factors)

    self.save_grasps_in_polar(gg, save_folder, "custom_stability score", c=grasp_factors)
    gg.scores = grasp_factors * gg.scores
    self.save_grasps_in_polar(gg, save_folder, "final corrected scores", c=gg.scores)