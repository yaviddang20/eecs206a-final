---
layout: default
title: EECS 206A Final Project
permalink: /
---

<div class="hero">
  <div class="hero-content">
    <div class="hero-badge">EECS 206A Final Project — Group 48</div>
    <h1 class="hero-title">Hand–Eye Calibrated 3D Gaussian Splatting with a Robot Manipulator</h1>
  </div>
</div>

<div class="container">

  <!-- ================= INTRODUCTION ================= -->

  <section class="overview">
    <h2>1. Introduction</h2>

    <h3>(a) Objective</h3>
    <p>
      This project aims to integrate, robotic manipulation, hand–eye
      calibration, and neural scene representations into a single system for automated 3D scene reconstruction. A camera is rigidly mounted to the wrist of a UR7e robot arm
      such that the camera an be actively repositioned to acquire multi-view observations of a scene. Using
      known robot kinematics and an estimated hand–eye transformation, all camera
      poses are expressed in a common world coordinate frame and used to train a
      3D Gaussian Splatting model.
    </p>

    <h3>(b) Motivation and Technical Challenges</h3>
    <p>
      Accurate and efficient 3D reconstruction is a foundational capability for robotic perception and manipulation. Classical structure-from-motion pipelines estimate camera poses from large collections of images to reconstruct a 3D scene, making them time-consuming and data-hungry. In contrast, my approach leverages robot-provided kinematics to obtain camera poses directly, reducing the required number of images by an order of magnitude. I demonstrate how a single robot arm can autonomously capture a 3D scene that can subsequently be used to plan grasping tasks based on the resulting reconstruction. Successful deployment of this system requires both a robust scene reconstruction model and accurate transformation of the camera frame into the world frame.
    </p>

    <h3>(c) Applications</h3>
    <p>
      The resulting system is applicable to a range of robotic tasks requiring
      accurate scene understanding, including grasp planning, bin picking,
      inspection, and the construction of digital twins for simulation and motion
      planning. The core idea of this project (egocentric 3D reconstruction with a mobile component of a robot) will be increasingly useful as robots become more capable/mobile and 3D representations become a necessary component for navigation and developing world models.
    </p>
  </section>

  <!-- ================= DESIGN ================= -->

  <section class="features">
    <h2>2. System Design</h2>

    <h3>(a) Design Requirements</h3>
    <p>
      My system is designed to produce 3D reconstructions aligned to the
      robot base frame. This requires a rigid camera mount with minimal camera movement and a robust method of acquiring the necessary transformation between the camera optical frame and the world frame. The design also requires a 3DGS model that can reliably and rapidly reconstruct 3D scenes from a small but informative collection of images.
    </p>

    <h3>(b) Planned Design </h3>
    <p>
      An RGB camera is rigidly mounted to the wrist of the UR7e robot end-effector. A hand–eye
      calibration procedure estimates the fixed transformation between the camera
      and end-effector frames. The robot detects the ArUco tag in the center of the scene and plans a 360 trajectory around the center. Camera poses are computed via forward kinematics composed with the hand–eye transformation. The known camera poses are featurized as inputs into the 3D Gaussian Splatting optimization process.
    </p>

    <h3>(c) Design Choies and Trade-offs</h3>
    <p>
    Leveraging robot-provided camera poses removes the need for structure-from-motion and eliminates its associated pose-estimation errors. However, this shifts the reconstruction quality dependence to the accuracy of the camera-to-world frame transformation. I adopt 3D Gaussian Splatting (3DGS) instead of neural radiance fields due to its significantly faster reconstruction speed while achieving comparable visual fidelity. Although a RealSense sensor can directly capture depth to produce 3D point clouds, there is a considerable depth error and frame-to-frame inconsistency. As a result, RealSense point clouds are often irregular and poorly suited for high-quality surface reconstruction, in contrast to the smoother representations produced by 3DGS.
    </p>

    <h3>(d) Robustness, Durability, and Efficiency</h3>
    <ul>
      <li>
        <strong>Robustness:</strong> The system leverages camera-world frame transformations for pose estimation for more reliable camera pose estimation. It generalizes to any AruCo tag and can support capturing a scene on any surface as long as the scene is not too large or too tall.
      </li>
      <li>
        <strong>Durability:</strong> The camera mount remains rigidly fixed onto the wrist and a low velocity on the arm minimizes any camera movement or rotation relative to the wrist mount.
      </li>
      <li>
        <strong>Efficiency:</strong> By eliminating multi-view pose estimation, the
        pipeline significantly reduces image requirements and computation, enabling online reconstruction suitable
        for downstream grasp planning.
      </li>
    </ul>

  </section>

  <!-- ================= IMPLEMENTATION ================= -->

  <section class="methodology">
    <h2>3. Implementation</h2>

    <h3>(a) Hardware Setup </h3>
    <p>
      I use a RealSense RGB-D camera, but any non-depth RGB camera suffices. I built a simple custom camera mount with two M4 through-holes to screw into the wrist end effector and a 0.5 in through-hole for the RealSense. The mount was 3D printed with polylactic acid (PLA). To connect the camera to the computer, I used an active USB hub and two cables (camera-to-hub and computer-to-hub) and velcro straps to keep the cables from interfering with perception and motion.
      
     For the scene, I used a wallet & keychain, small wood cube, green plastic cup, and a 5 cm ArUco tag. The scene was placed on top of a tabletop placed below the end-effector of the arm. The scene can consist of any object as long as the object is not too tall or too large. For robustness, I add an ArUco tag to indicate the center of the scene but the tag is not necessary if the center of the scene is directly under the end effector's default position.
    </p>

    <figure style="max-width: 500px; margin: 2rem auto;">
      <div style="width: 100%; max-width: 500px; height: 400px; overflow: hidden; border-radius: 8px; box-shadow: 0 4px 6px rgba(0, 0, 0, 0.1); display: flex; align-items: center; justify-content: center; background: #f9fafb;">
        <img src="{{ '/assets/images/mount.png' | relative_url }}" alt="Camera mount design" style="max-width: 100%; max-height: 100%; width: auto; height: auto; object-fit: contain;">
      </div>
      <figcaption style="text-align: center; margin-top: 0.75rem; font-style: italic; color: #6b7280; font-size: 0.9rem;">
        Custom 3D-printed camera mount design with M4 through-holes for attachment to the wrist end-effector and half inch through-hole to mount the RealSense camera.
      </figcaption>
    </figure>

    <div style="display: grid; grid-template-columns: 1fr 1fr; gap: 2rem; margin: 2rem 0; max-width: 800px;">
      <figure style="margin: 0;">
        <div style="width: 100%; max-width: 350px; height: 300px; overflow: hidden; border-radius: 8px; box-shadow: 0 4px 6px rgba(0, 0, 0, 0.1); display: flex; align-items: center; justify-content: center; background: #f9fafb;">
          <img src="{{ '/assets/images/gripper.png' | relative_url }}" alt="Gripper with camera mount" style="max-width: 100%; max-height: 100%; width: auto; height: auto; object-fit: contain;">
        </div>
        <figcaption>
          Close-up view of the gripper with the custom 3D-printed camera mount attached to the wrist end-effector.
        </figcaption>
      </figure>
      <figure style="margin: 0;">
        <div style="width: 100%; max-width: 350px; height: 300px; overflow: hidden; border-radius: 8px; box-shadow: 0 4px 6px rgba(0, 0, 0, 0.1); display: flex; align-items: center; justify-content: center; background: #f9fafb;">
          <img src="{{ '/assets/images/full_body.png' | relative_url }}" alt="Full robot setup" style="max-width: 100%; max-height: 100%; width: auto; height: auto; object-fit: contain;">
        </div>
        <figcaption>
        Full view of the UR7e robot arm with the camera-mounted gripper positioned above the scene and 15 cm ArUco tag attached to the base.
        </figcaption>
      </figure>
    </div>

    <h3>(b) Software </h3>
    <p>
      <strong>ArUco frame transformation node</strong> — Service node that detects the AR marker tag and stores the transformation from the camera to the base frame. It handles both the AR marker attached to the arm and the AR marker at the center of the scene.<br>
      <strong>Trajectory planning node</strong> — Node for planning trajectories for hand-eye calibration and scene revolving by calling MoveIt 2. This node stops for 0.25 seconds before calling the picture-taking node.<br>
      <strong>Picture-taking node</strong> — Service node that takes a picture and saves the picture and position of the camera relative to the world frame.<br>
      <strong>Modal node</strong> — Node that launches a containerized cloud environment to process RealSense images and train the 3DGS model on an A100 GPU. The trained model outputs the 3D point cloud.
    </p>

    <figure style="max-width: 800px; margin: 2rem auto;">
      <img src="{{ '/assets/images/nodes.png' | relative_url }}" alt="ROS2 nodes architecture" style="width: 100%; height: auto; border-radius: 8px; box-shadow: 0 4px 6px rgba(0, 0, 0, 0.1);">
      <figcaption style="text-align: center; margin-top: 0.75rem; font-style: italic; color: #6b7280; font-size: 0.9rem;">
        ROS2 node architecture and communication flow.
      </figcaption>
    </figure>

    <h3>(c) Complete Pipeline</h3>
    <ol>
      <li><strong>Hand-eye calibration</strong> — The robot arm moves into a position to detect the ArUco tag located at the base. The 15cm ArUco tag is detected and the camera-to-world frame is calculated and stored.</li>
      <li><strong>Scene center identification</strong> — (Optional if scene center is placed under end-effector and coordinates are known a priori) The arm moves into a birds-eye view position such that the 5 cm ArUco tag on the table is visible. The ArUco tag center is used as the center for step 3.</li>
      <li><strong>Circular Trajectory</strong> — The arm plans and executes a trajectory to circle around the center with a tunable radius (default 40 cm) with 9 waypoints for a total of 10 images. At each waypoint, the robot stops and the picture-taking node is called to take a picture of the scene.</li>
      <li><strong>3DGS Training</strong> — The RealSense images and camera poses are preprocessed as input to train the 3DGS model and output a 3D point cloud (.ply) file. Training on 10 images takes less than 20 seconds on an A100.</li>
    </ol>
  </section>

  <!-- ================= RESULTS ================= -->

  <section class="results">
    <h2>4. Results</h2>

    <h3>Pipeline Trajectories</h3>
    <p>
      The robot arm successfully executes a three-stage pipeline to autonomously capture multi-view images of a scene for 3D reconstruction as described in 3C.
    </p>
    <style>
      @media (max-width: 768px) {
        .trajectory-videos-grid {
          grid-template-columns: 1fr !important;
        }
      }
    </style>
    <script>
      var tag = document.createElement('script');
      tag.src = "https://www.youtube.com/iframe_api";
      var firstScriptTag = document.getElementsByTagName('script')[0];
      firstScriptTag.parentNode.insertBefore(tag, firstScriptTag);

      var players = {};
      var videoIds = ['KbX0mmM3LaY', 'Vzll8Dj2dX8', 'Xz6AsCTOUIA'];
      var checkIntervals = {};
      
      function onYouTubeIframeAPIReady() {
        videoIds.forEach(function(videoId, index) {
          var playerId = 'video' + (index + 1);
          var container = document.getElementById(playerId);
          if (container) {
            container.innerHTML = '';
            players[playerId] = new YT.Player(playerId, {
              videoId: videoId,
              playerVars: {
                'autoplay': 1,
                'loop': 1,
                'playlist': videoId,
                'mute': 1,
                'controls': 0,
                'modestbranding': 1,
                'playsinline': 1,
                'rel': 0,
                'iv_load_policy': 3,
                'enablejsapi': 1
              },
              events: {
                'onReady': function(event) {
                  event.target.playVideo();
                  var player = event.target;
                  checkIntervals[playerId] = setInterval(function() {
                    try {
                      var currentTime = player.getCurrentTime();
                      var duration = player.getDuration();
                      if (duration && currentTime && (duration - currentTime) < 0.3) {
                        player.seekTo(0, true);
                      }
                    } catch(e) {}
                  }, 50);
                },
                'onStateChange': function(event) {
                  if (event.data === YT.PlayerState.ENDED) {
                    event.target.seekTo(0);
                    event.target.playVideo();
                  }
                }
              }
            });
          }
        });
      }
    </script>
    <div class="trajectory-videos-grid" style="display: grid; grid-template-columns: repeat(3, 1fr); gap: 1.5rem; margin: 2rem 0; max-width: 1200px; width: 100%;">
      <figure style="margin: 0;">
        <div style="position: relative; padding-bottom: 177.78%; height: 0; overflow: hidden; border-radius: 8px; box-shadow: 0 4px 6px rgba(0, 0, 0, 0.1); background: #000;">
          <div id="video1" style="position: absolute; top: 0; left: 0; width: 100%; height: 100%;"></div>
        </div>
        <figcaption style="text-align: center; margin-top: 0.75rem; font-style: italic; color: #6b7280; font-size: 0.9rem;">
          <strong>Part 1:</strong> Hand-eye calibration (7X speed)
        </figcaption>
      </figure>
      <figure style="margin: 0;">
        <div style="position: relative; padding-bottom: 177.78%; height: 0; overflow: hidden; border-radius: 8px; box-shadow: 0 4px 6px rgba(0, 0, 0, 0.1); background: #000;">
          <div id="video2" style="position: absolute; top: 0; left: 0; width: 100%; height: 100%;"></div>
        </div>
        <figcaption style="text-align: center; margin-top: 0.75rem; font-style: italic; color: #6b7280; font-size: 0.9rem;">
          <strong>Part 2:</strong> Scene center identification (4X speed)
        </figcaption>
      </figure>
      <figure style="margin: 0;">
        <div style="position: relative; padding-bottom: 177.78%; height: 0; overflow: hidden; border-radius: 8px; box-shadow: 0 4px 6px rgba(0, 0, 0, 0.1); background: #000;">
          <div id="video3" style="position: absolute; top: 0; left: 0; width: 100%; height: 100%;"></div>
        </div>
        <figcaption style="text-align: center; margin-top: 0.75rem; font-style: italic; color: #6b7280; font-size: 0.9rem;">
          <strong>Part 3:</strong> Circular trajectory (7X speed & partial)
        </figcaption>
      </figure>
    </div>

    <h3>3D Reconstruction Quality</h3>
    <p>
      The system produces 3D reconstructions with consistent geometry given a sparse collection of only 10 images. The 3DGS point cloud output is depicted below. There are some clear issues with catching objects that are farther away from the center and closer to the periphery of the circular trajectory such as the green cup. Additionally some artifacting is visible and points outside of the scene should be filtered out. More images (either by increasing the number of waypoints or varying height of the robot arm / camera) could increase the quality of the reconstruction at the cost of increased speed. For online real-time reconstruction, the system achieves sub-20 second training times on an A100 GPU, making it suitable for rapid scene understanding and grasp planning applications.
    </p>

    <div style="width: 100vw; position: relative; left: 50%; right: 50%; margin-left: -50vw; margin-right: -50vw; padding: 0 20px;">
      <figure style="max-width: 1000px; margin: 0 auto;">
        <iframe
          loading="lazy"
          src="{{ '/viewer/viewer.html' | relative_url }}?splat={{ '/assets/models/point_cloud.spz' | relative_url }}"
          style="width: 100%; height: 500px; border: 2px solid #e5e7eb; border-radius: 8px; display: block; box-shadow: 0 4px 6px rgba(0, 0, 0, 0.1);">
        </iframe>
        <figcaption>
          3D Gaussian Splatting (3DGS) reconstruction of the captured scene (NOTE: similar but not exact same scene as trajectory demonstration). <strong>Interactive viewer:</strong> Click and drag to rotate, scroll to zoom, right-click and drag to pan.
        </figcaption>
      </figure>
    </div>
  </section>


  <!-- ================= CONCLUSION ================= -->

  <section class="conclusion">
    <h2>5. Conclusion & Future Directions </h2>
    <p>
      This project successfully demonstrates the integration of robotic manipulation, hand–eye calibration, and neural scene representations for automated 3D scene reconstruction. By leveraging robot-provided kinematics instead of structure-from-motion, the system achieves accurate 3D reconstructions with significantly fewer images (10 vs. hundreds typically required). The system produces reasonably dense 3D point clouds with consistent geometry in under 20 seconds, making it suitable for real-time robotic applications such as grasp planning and scene understanding.
    </p>
    <p>
      The key contributions of this work include: (1) a complete pipeline from robot control to 3D reconstruction, (2) robust pose estimation through hand–eye calibration that eliminates the need for feature matching, and (3) efficient scene reconstruction using 3DGS that trades real-time performance for high-quality geometric representations. 

      Given the limited times and number of team members, I was limited to only implementing the online 3D reconstruction pipeline. Future directions from the original proposal would include a pick-and-place demonstration using only the 3D scene and implementing FisherRF for next-best view planning. FisherRF is a method that maximizes the Fisher information (minimizes variance) of the rendered observations with respect to the scene representation parameters to select the next best camera pose from a set of candidate views. This would require more complicated trajectory planning and collision detection and avoidance of known objects.
    </p>
  </section>

  <!-- ================= TEAM ================= -->

  <section class="team">
    <h2>6. Team</h2>
    <div class="team-members">
      <div class="team-member">
        <h3>David Yang</h3>
        <p>Graduate student at UC Berkeley in computational biology with research experience in deep learning for protein design and an interest in robotics.</p>
      </div>
    </div>
  </section>

  <!-- ================= RESOURCES ================= -->

  <section class="links">
    <h2>Resources</h2>
    <div class="link-buttons">
      <a href="https://github.com/yaviddang20/eecs206a-final" class="btn btn-primary" target="_blank">
        GitHub Repository
      </a>
    </div>
  </section>

</div>
