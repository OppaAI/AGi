/* Grace's 3D VRM is shown when the AIVA PC presence heartbeat is online. */
(() => {
  'use strict';

  const canvas = document.getElementById('robotVrmCanvas');
  const container = document.getElementById('avatarContainer');
  if (!canvas || !container) return;

  let avatar = null;
  let frame = 0;
  let loadPromise = null;
  let aivaOnline = false;
  let auroraOnline = window.agiAuroraOnline === true;
  let nextFrameAt = 0;

  async function loadAvatar() {
    if (avatar) return avatar;
    if (loadPromise) return loadPromise;
    loadPromise = (async () => {
      const THREE = await import('three');
      const { GLTFLoader } = await import('three/addons/loaders/GLTFLoader.js');
      const { VRMLoaderPlugin, VRMUtils } = await import('@pixiv/three-vrm');

      const renderer = new THREE.WebGLRenderer({
        canvas, alpha: true, antialias: true, powerPreference: 'low-power',
      });
      renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 1.25));
      renderer.outputColorSpace = THREE.SRGBColorSpace;
      renderer.setClearColor(0x000000, 0);

      const scene = new THREE.Scene();
      scene.add(new THREE.HemisphereLight(0xffffff, 0x8c79aa, 2.0));
      const key = new THREE.DirectionalLight(0xffffff, 2.3);
      key.position.set(-1, 2, 3);
      scene.add(key);
      const camera = new THREE.PerspectiveCamera(30, 1, 0.1, 30);

      const loader = new GLTFLoader();
      loader.register((parser) => new VRMLoaderPlugin(parser));
      const gltf = await loader.loadAsync('/assets/Grace.vrm');
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
      const modelHeight = Math.max(bounds.max.y - bounds.min.y, 1.2);
      camera.position.set(0, modelHeight * 0.56, modelHeight * 2.15);
      camera.lookAt(0, modelHeight * 0.53, 0);

      const resize = () => {
        if (!aivaOnline) return;
        const { width, height } = canvas.getBoundingClientRect();
        if (!width || !height) return;
        renderer.setSize(width, height, false);
        camera.aspect = width / height;
        camera.updateProjectionMatrix();
      };
      const resizeObserver = new ResizeObserver(resize);
      resizeObserver.observe(canvas);
      avatar = { renderer, scene, camera, vrm, clock: new THREE.Clock(), resize };
      return avatar;
    })();
    try {
      return await loadPromise;
    } catch (error) {
      loadPromise = null;
      throw error;
    }
  }

  function stopRender() {
    cancelAnimationFrame(frame);
    frame = 0;
    nextFrameAt = 0;
    if (avatar) avatar.clock.getDelta();
  }

  function render(now) {
    if (!aivaOnline || !avatar) return;
    frame = requestAnimationFrame(render);
    if (now < nextFrameAt) return;
    nextFrameAt = now + 33;
    avatar.vrm.update(avatar.clock.getDelta());
    avatar.renderer.render(avatar.scene, avatar.camera);
  }

  async function syncPresence() {
    container.classList.toggle('aiva-online', aivaOnline && auroraOnline);
    container.classList.toggle('grace-only', aivaOnline && !auroraOnline);
    if (!aivaOnline) {
      stopRender();
      return;
    }
    try {
      await loadAvatar();
      avatar.resize();
      if (!frame) frame = requestAnimationFrame(render);
    } catch (error) {
      console.error('Could not load AGi.vrm:', error);
      container.classList.remove('aiva-online', 'grace-only');
      const presence = document.getElementById('aivaPresence');
      if (presence) presence.textContent = 'AIVA avatar unavailable';
    }
  }

  window.addEventListener('agi-aiva-presence', (event) => {
    aivaOnline = Boolean(event.detail?.online);
    syncPresence();
  });
  window.addEventListener('agi-aurora-presence', (event) => {
    auroraOnline = Boolean(event.detail?.online);
    syncPresence();
  });
})();
