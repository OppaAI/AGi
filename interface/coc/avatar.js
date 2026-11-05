import * as THREE from 'three';
import { GLTFLoader } from 'three/addons/loaders/GLTFLoader.js';
import { VRMLoaderPlugin, VRMUtils } from '@pixiv/three-vrm';

const canvas = document.getElementById('robotVrmCanvas');
const status = document.getElementById('statusText');

if (canvas) {
  try {
    const renderer = new THREE.WebGLRenderer({ canvas, alpha: true, antialias: true, preserveDrawingBuffer: true, powerPreference: 'low-power' });
    renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 1.25));
    renderer.outputColorSpace = THREE.SRGBColorSpace;
    renderer.setClearColor(0x000000, 0);

    const scene = new THREE.Scene();
    scene.add(new THREE.HemisphereLight(0xffffff, 0x8c79aa, 2.0));
    const key = new THREE.DirectionalLight(0xffffff, 2.3);
    key.position.set(-1, 2, 3);
    scene.add(key);
    const camera = new THREE.PerspectiveCamera(30, 1, 0.1, 30);
    camera.position.set(0, 1.05, 3.6);

    const loader = new GLTFLoader();
    loader.register((parser) => new VRMLoaderPlugin(parser));
    loader.load('/assets/AGi.vrm', (gltf) => {
      const vrm = gltf.userData.vrm;
      if (!vrm) throw new Error('The avatar file did not contain a VRM model');
      VRMUtils.removeUnnecessaryVertices(gltf.scene);
      VRMUtils.combineSkeletons(gltf.scene);
      scene.add(vrm.scene);
      vrm.scene.rotation.y = Math.PI;

      const bounds = new THREE.Box3().setFromObject(vrm.scene);
      const center = bounds.getCenter(new THREE.Vector3());
      vrm.scene.position.x -= center.x;
      vrm.scene.position.y -= bounds.min.y;
      const height = Math.max(bounds.max.y - bounds.min.y, 1.2);
      camera.position.y = height * 0.56;
      camera.position.z = height * 2.15;
      camera.lookAt(0, height * 0.53, 0);

      const resize = () => {
        const { width, height: viewHeight } = canvas.getBoundingClientRect();
        if (!width || !viewHeight) return;
        renderer.setSize(width, viewHeight, false);
        camera.aspect = width / viewHeight;
        camera.updateProjectionMatrix();
      };
      new ResizeObserver(resize).observe(canvas);
      resize();

      const clock = new THREE.Clock();
      let lastFrame = 0;
      const animate = (now) => {
        requestAnimationFrame(animate);
        if (now - lastFrame < 33) return;
        lastFrame = now;
        vrm.update(clock.getDelta());
        renderer.render(scene, camera);
      };
      animate(0);

      // Reuse a lightweight rendered portrait beside Grace's chat messages.
      setTimeout(() => {
        try {
          window.graceAvatarImage = renderer.domElement.toDataURL('image/png');
          document.querySelectorAll('.message.grace').forEach((message) => {
            if (message.querySelector('.grace-mini-avatar')) return;
            const portrait = document.createElement('img');
            portrait.className = 'grace-mini-avatar';
            portrait.alt = 'Grace';
            portrait.src = window.graceAvatarImage;
            message.prepend(portrait);
          });
        } catch (error) {
          console.debug('Could not create the small chat portrait:', error);
        }
      }, 800);
    }, undefined, (error) => {
      console.error('Could not load AGi.vrm:', error);
      if (status) status.textContent = 'Avatar unavailable';
    });
  } catch (error) {
    console.error('WebGL avatar initialization failed:', error);
    if (status) status.textContent = '3D avatar unavailable';
  }
}
