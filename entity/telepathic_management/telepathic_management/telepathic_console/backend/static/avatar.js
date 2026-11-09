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
  let idleTime = 0;
  let cameraMode = 'portrait';

  function frameCamera(mode = cameraMode) {
    cameraMode = mode;
    const zoomSwitch = document.getElementById('avatarZoomSwitch');
    if (zoomSwitch) zoomSwitch.checked = mode === 'portrait';
    document.querySelector('.chamber-panel')?.classList.toggle('portrait-zoom', mode === 'portrait');
    if (!avatar) return;
    const { camera, modelHeight } = avatar;
    const distance = mode === 'full' ? 2.3 : 0.72;
    const targetHeight = mode === 'full' ? 0.52 : 0.8;
    camera.position.set(0, modelHeight * targetHeight, modelHeight * distance);
    camera.lookAt(0, modelHeight * targetHeight, 0);
    camera.updateProjectionMatrix();
  }

  // Apply the default portrait framing and switch state before the VRM loads.
  frameCamera();

  function rememberRestPose(vrm) {
    const bones = {};
    const humanoid = vrm.humanoid;
    // The Aiko VRM viewer drives raw bones directly. VRM 3.x otherwise
    // copies the normalized T-pose over them on every update.
    if (humanoid) humanoid.autoUpdateHumanBones = false;
    for (const name of ['spine', 'chest', 'neck', 'head', 'leftUpperArm', 'rightUpperArm', 'leftLowerArm', 'rightLowerArm', 'leftHand', 'rightHand', 'leftUpperLeg', 'rightUpperLeg', 'leftLowerLeg', 'rightLowerLeg']) {
      const bone = humanoid?.getRawBoneNode(name);
      if (bone) bones[name] = { bone, x: bone.rotation.x, y: bone.rotation.y, z: bone.rotation.z };
    }
    // Let the arms and legs hang loosely while Grace rests in the chamber.
    if (bones.neck) bones.neck.x -= 0.09;
    if (bones.head) { bones.head.x -= 0.24; bones.head.z -= 0.06; }
    if (bones.leftUpperArm) bones.leftUpperArm.z = 1.28;
    if (bones.rightUpperArm) bones.rightUpperArm.z = -1.28;
    if (bones.leftLowerArm) { bones.leftLowerArm.x += 0.12; bones.leftLowerArm.z += 0.08; }
    if (bones.rightLowerArm) { bones.rightLowerArm.x += 0.12; bones.rightLowerArm.z -= 0.08; }
    if (bones.leftUpperLeg) bones.leftUpperLeg.x += 0.06;
    if (bones.rightUpperLeg) bones.rightUpperLeg.x += 0.08;
    if (bones.leftLowerLeg) bones.leftLowerLeg.x -= 0.06;
    if (bones.rightLowerLeg) bones.rightLowerLeg.x -= 0.08;
    return bones;
  }

  function applyIdle() {
    const t = idleTime;
    const breathe = Math.sin((2 * Math.PI * t) / 3.5);
    const drift = (frequency, phase = 0) => Math.sin(2 * Math.PI * frequency * t + phase);
    const pose = avatar.restPose;
    const apply = (name, x = 0, y = 0, z = 0) => {
      const item = pose[name];
      if (item) item.bone.rotation.set(item.x + x, item.y + y, item.z + z);
    };
    // Match Aiko's gentle, asynchronous breathing and tiny head micro-sway.
    apply('spine', 0.007 * Math.sin((2 * Math.PI * (t - 0.08)) / 3.5));
    apply('chest', 0.014 * breathe);
    apply('head', 0.0045 * drift(0.47, 0.4), 0.016 * drift(0.41) + 0.007 * drift(0.53, 2.1), 0.006 * drift(0.19, 1.1));
    apply('neck', 0.003 * drift(0.67, 1.7), 0.004 * drift(0.59), 0);
    apply('leftUpperArm');
    apply('rightUpperArm');
    apply('leftLowerArm');
    apply('rightLowerArm');
    apply('leftHand');
    apply('rightHand');
    apply('leftUpperLeg');
    apply('rightUpperLeg');
    apply('leftLowerLeg');
    apply('rightLowerLeg');
  }

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
      scene.add(new THREE.HemisphereLight(0xd7faff, 0x164279, 1.6));
      const key = new THREE.DirectionalLight(0xedfaff, 1.8);
      key.position.set(-1, 2, 3);
      scene.add(key);
      const rim = new THREE.DirectionalLight(0x53dfff, 1.5);
      rim.position.set(1, 1, -1);
      scene.add(rim);
      const camera = new THREE.PerspectiveCamera(30, 1, 0.1, 30);

      const loader = new GLTFLoader();
      loader.register((parser) => new VRMLoaderPlugin(parser));
      const gltf = await loader.loadAsync('/static/assets/Grace.vrm');
      const vrm = gltf.userData.vrm;
      if (!vrm) throw new Error('The avatar file did not contain a VRM model');
      // Match the body skin to the cooler face without tinting clothes or hair.
      const bodySkinTint = new THREE.Color().setRGB(0.78, 1.05, 1.35);
      const adjustedMaterials = new Set();
      gltf.scene.traverse((object) => {
        const materials = Array.isArray(object.material) ? object.material : [object.material];
        for (const material of materials) {
          if (!material || adjustedMaterials.has(material) || !/Body_00_SKIN/i.test(material.name)) continue;
          adjustedMaterials.add(material);
          material.color?.multiply(bodySkinTint);
          material.shadeColorFactor?.multiply(bodySkinTint);
        }
      });
      VRMUtils.removeUnnecessaryVertices(gltf.scene);
      VRMUtils.combineSkeletons(gltf.scene);
      scene.add(vrm.scene);
      vrm.scene.rotation.y = Math.PI;

      const bounds = new THREE.Box3().setFromObject(vrm.scene);
      const center = bounds.getCenter(new THREE.Vector3());
      vrm.scene.position.x -= center.x;
      vrm.scene.position.y -= bounds.min.y;
      const modelHeight = Math.max(bounds.max.y - bounds.min.y, 1.2);
      // frameCamera applies the close chest portrait after avatar state is set.

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
      avatar = { renderer, scene, camera, vrm, modelHeight, restPose: rememberRestPose(vrm), baseY: vrm.scene.position.y, clock: new THREE.Clock(), resize };
      vrm.expressionManager?.setValue('blink', 1);
      resize();
      // Capture the loaded model itself for the header and conversation badges.
      const selectedMode = cameraMode;
      frameCamera('portrait');
      applyIdle();
      vrm.update(0);
      renderer.render(scene, camera);
      try {
        document.body.style.setProperty('--grace-portrait', `url("${canvas.toDataURL('image/webp')}")`);
        document.body.classList.add('avatar-portrait-ready');
      } catch (_) { /* Keep the neutral badge if canvas capture is unavailable. */ }
      frameCamera(selectedMode);
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
    const delta = Math.min(avatar.clock.getDelta(), 0.05);
    const reducedMotion = window.matchMedia('(prefers-reduced-motion: reduce)').matches;
    if (!reducedMotion) idleTime += delta;
    avatar.vrm.scene.position.y = avatar.baseY + (reducedMotion ? 0 : Math.sin(idleTime * 0.8) * avatar.modelHeight * 0.006);
    applyIdle();
    avatar.vrm.update(delta);
    avatar.renderer.render(avatar.scene, avatar.camera);
  }

  document.getElementById('avatarZoomSwitch')?.addEventListener('change', (event) => {
    frameCamera(event.target.checked ? 'portrait' : 'full');
  });

  async function syncPresence() {
    container.classList.toggle('aurora-online', auroraOnline);
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
      console.error('Could not load Grace.vrm:', error);
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
