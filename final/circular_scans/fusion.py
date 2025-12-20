import os
import argparse
import numpy as np
import open3d as o3d


def fuse_npy_clouds(run_dir, voxel_size=0.01, save_individual=True):
    pcs = []

    for f in sorted(os.listdir(run_dir)):
        if f.startswith("cloud_") and f.endswith(".npy"):
            npy_path = os.path.join(run_dir, f)

            # Extract index from filename (e.g., "cloud_0.npy" -> 0)
            idx = int(f.replace("cloud_", "").replace(".npy", ""))
            
            # Load corresponding pose file (base_to_camera transformation)
            pose_path = os.path.join(run_dir, f"pose_{idx}.npy")
            if not os.path.exists(pose_path):
                print(f"Warning: pose file not found: {pose_path}, skipping {f}")
                continue
            
            base_to_camera = np.load(pose_path)
            # Inverse to get camera_to_base transformation
            # camera_to_base = np.linalg.inv(base_to_camera)

            # Load dict {"xyz":..., "rgb":...}
            data = np.load(npy_path, allow_pickle=True).item()
            xyz = np.asarray(data["xyz"])
            rgb = np.asarray(data["rgb"])


            # Transform points from camera frame to base (world) frame
            # Convert to homogeneous coordinates
            xyz_h = np.hstack([xyz, np.ones((xyz.shape[0], 1))])  # N×4
            # Transform: base_to_camera @ points_in_camera_frame
            xyz_base = (base_to_camera @ xyz_h.T).T[:, :3]  # N×3

            # -------------------------------
            # FILTER: remove points where y > 0.75
            # -------------------------------
            # mask = xyz_base[:, 1] <= 0.75
            # xyz_base = xyz_base[mask]
            # rgb = rgb[mask]


            # mask = xyz_base[:, 2] > -0.2
            # xyz_base = xyz_base[mask]
            # rgb = rgb[mask]


            # mask = xyz_base[:, 0] > -0.6
            # xyz_base = xyz_base[mask]
            # rgb = rgb[mask]

            # mask = xyz_base[:, 0] < -
            # xyz_base = xyz_base[mask]
            # rgb = rgb[mask]
            # -------------------------------

            # Build Open3D cloud
            pc = o3d.geometry.PointCloud()
            pc.points = o3d.utility.Vector3dVector(xyz_base)
            pc.colors = o3d.utility.Vector3dVector(rgb)

            pcs.append(pc)
            print("Loaded and transformed", f, f"({xyz_base.shape[0]} points left)")

            # Save individual PLY
            if save_individual:
                ply_path = os.path.join(run_dir, f.replace(".npy", ".ply"))
                o3d.io.write_point_cloud(ply_path, pc)
                print("Saved individual PLY:", ply_path)

    if not pcs:
        print("No clouds found.")
        return

    # Merge all clouds
    merged = o3d.geometry.PointCloud()
    for pc in pcs:
        merged += pc

    # Downsample (averaging XYZ + RGB)
    fused = merged.voxel_down_sample(voxel_size)

    # Save fused cloud
    fused_path = os.path.join(run_dir, "fused_cloud.ply")
    o3d.io.write_point_cloud(fused_path, fused)
    print("Saved fused cloud:", fused_path)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("run_dir", help="Directory containing cloud_*.npy files")
    parser.add_argument("--voxel_size", type=float, default=0.01)
    parser.add_argument("--no_individual", action="store_true",
                        help="Don’t save per-cloud .ply")
    args = parser.parse_args()

    fuse_npy_clouds(
        args.run_dir,
        args.voxel_size,
        save_individual=not args.no_individual,
    )
