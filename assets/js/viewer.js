import * as THREE from "three";
import { PLYLoader } from "three/addons/loaders/PLYLoader.js";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";


// Wait for DOM to be ready
function initViewer() {
  const container = document.getElementById("viewer");
  
  if (!container) {
    console.error("Viewer container not found");
    return;
  }

  // Show loading message
  container.innerHTML = '<div style="display: flex; align-items: center; justify-content: center; height: 100%; color: #666; font-family: sans-serif;">Loading 3D model...</div>';

  const scene = new THREE.Scene();
  scene.background = new THREE.Color(0xf5f5f5);

  // Get container dimensions
  const width = container.clientWidth || 800;
  const height = container.clientHeight || 500;

  const camera = new THREE.PerspectiveCamera(60, width / height, 0.01, 100);
  camera.position.set(0.3, 0.3, 0.3);

  const renderer = new THREE.WebGLRenderer({ antialias: true });
  renderer.setSize(width, height);
  renderer.setPixelRatio(window.devicePixelRatio);
  
  // Clear loading message and add canvas
  container.innerHTML = '';
  container.appendChild(renderer.domElement);

  const controls = new OrbitControls(camera, renderer.domElement);
  controls.enableDamping = true;
  controls.dampingFactor = 0.05;

  scene.add(new THREE.AmbientLight(0xffffff, 0.6));
  const light = new THREE.DirectionalLight(0xffffff, 0.8);
  light.position.set(1, 1, 1);
  scene.add(light);

  // Handle window resize
  function onWindowResize() {
    const newWidth = container.clientWidth;
    const newHeight = container.clientHeight;
    camera.aspect = newWidth / newHeight;
    camera.updateProjectionMatrix();
    renderer.setSize(newWidth, newHeight);
  }
  window.addEventListener('resize', onWindowResize);

  // Get the PLY file path from Jekyll or use fallback
  const plyPath = window.PLY_FILE_PATH || '/assets/models/point_cloud.ply';
  
  console.log("Attempting to load PLY from:", plyPath);
  console.log("Full URL would be:", window.location.origin + plyPath);

  const loader = new PLYLoader();
  
  loader.load(
    plyPath,
    (geometry) => {
      console.log("PLY file loaded successfully");
      console.log("Geometry vertices:", geometry.attributes.position.count);
      geometry.computeVertexNormals();
      
      // Check if geometry has colors
      const hasColors = geometry.attributes.color !== undefined;
      
    //   const material = new THREE.PointsMaterial({
    //     size: 0.002,
    //     vertexColors: hasColors,
    //     color: hasColors ? 0xffffff : 0x2563eb // Use blue if no colors
    //   });
    console.log("geometry.attributes.color", geometry.attributes.color);
      const material = new THREE.PointsMaterial({
        size: 0.01,
        vertexColors: true,
        sizeAttenuation: true
      });
      
      
      const points = new THREE.Points(geometry, material);
      scene.add(points);
      
      // Auto-fit camera to geometry
      const box = new THREE.Box3().setFromObject(points);
      const center = box.getCenter(new THREE.Vector3());
      const size = box.getSize(new THREE.Vector3());
      const maxDim = Math.max(size.x, size.y, size.z);
      const fov = camera.fov * (Math.PI / 180);
      let cameraZ = Math.abs(maxDim / 2 / Math.tan(fov / 2));
      cameraZ *= 1.5; // Add some padding
      camera.position.set(center.x, center.y, center.z + cameraZ);
      camera.lookAt(center);
      controls.target.copy(center);
      controls.update();
    },
    (progress) => {
      // Loading progress (optional)
      if (progress.total > 0) {
        const percent = (progress.loaded / progress.total) * 100;
        console.log(`Loading: ${percent.toFixed(0)}%`);
      }
    },
    (error) => {
      console.error("Error loading PLY file:", error);
      console.error("Error details:", {
        message: error.message,
        path: plyPath,
        fullUrl: window.location.origin + plyPath
      });
      
      let errorMessage = "Failed to load 3D model";
      if (error.message) {
        errorMessage += `: ${error.message}`;
      }
      
      container.innerHTML = `
        <div style="display: flex; flex-direction: column; align-items: center; justify-content: center; height: 100%; color: #dc2626; font-family: sans-serif; padding: 20px; text-align: center;">
          <p style="font-size: 18px; margin-bottom: 10px;">⚠️ ${errorMessage}</p>
          <p style="font-size: 14px; color: #666;">Path: ${plyPath}</p>
          <p style="font-size: 12px; color: #999; margin-top: 10px;">Note: Your PLY file is 512MB - this may be too large for web loading. Consider reducing the point cloud density.</p>
          <p style="font-size: 12px; color: #999; margin-top: 5px;">Check browser console (F12) for details</p>
        </div>
      `;
    }
  );

  function animate() {
    requestAnimationFrame(animate);
    controls.update();
    renderer.render(scene, camera);
  }
  animate();
}

// Initialize when DOM is ready
if (document.readyState === 'loading') {
  document.addEventListener('DOMContentLoaded', initViewer);
} else {
  initViewer();
}
