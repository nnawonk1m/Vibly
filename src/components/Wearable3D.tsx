import { useGLTF } from '@react-three/drei/native';
import { Canvas, useFrame, useThree } from '@react-three/fiber/native';
import React, { Suspense, useMemo, useRef, useState } from 'react';
import { ActivityIndicator, StyleSheet, View, ViewStyle } from 'react-native';
import { Gesture, GestureDetector } from 'react-native-gesture-handler';
import * as THREE from 'three';
import { COLORS } from "../constants/theme";

const MODEL = require('../../assets/models/vibly.glb');

// Warm the loader so the first render isn't a stall.
useGLTF.preload(MODEL);

/** How hard the model chases its target rotation. Lower = heavier, more inertia. */
const DAMPING = 0.12;
/** Idle turntable speed, radians per second. */
const IDLE_SPIN = 0.3;
/** Milliseconds of no touch before the turntable resumes. */
const IDLE_DELAY = 2500;
/** Longest edge of the model, in world units, after normalisation. */
const FIT_SIZE = 2;

const PITCH_LIMIT = Math.PI / 2.2;
const ZOOM_MIN = 0.8;
const ZOOM_MAX = 1.4;

const clamp = (v: number, min: number, max: number) =>
  Math.min(max, Math.max(min, v));

type Controls = {
  yaw: number;
  pitch: number;
  zoom: number;
  dragging: boolean;
  lastTouch: number;
};

type MeshProps = {
  controls: React.MutableRefObject<Controls>;
  onReady: () => void;
};

function WearableMesh({ controls, onReady }: MeshProps) {
  const pivot = useRef<THREE.Group>(null!);
  const { scene } = useGLTF(MODEL) as unknown as { scene: THREE.Group };
  const { size } = useThree();
  const fitScale = Math.min(1, size.width / size.height) * 0.65;

  // Onshape exports in millimetres, with the origin wherever you sketched it.
  // Re-centre on the bounding box and normalise the longest edge to FIT_SIZE so
  // the camera framing works regardless of the part's real dimensions.
  const model = useMemo(() => {
    const root = scene.clone(true);

    const box = new THREE.Box3().setFromObject(root);
    const size = box.getSize(new THREE.Vector3());
    const center = box.getCenter(new THREE.Vector3());

    root.position.sub(center);

    const holder = new THREE.Group();
    holder.add(root);
    holder.scale.setScalar(FIT_SIZE / Math.max(size.x, size.y, size.z));

    // CAD exports often arrive fully rough and non-metallic. Nudge toward a
    // moulded-plastic read; delete this block if your Onshape appearances
    // already look right.
    holder.traverse((child) => {
      const mesh = child as THREE.Mesh;
      if (!mesh.isMesh) return;
      const material = mesh.material as THREE.MeshStandardMaterial;
      if (material?.isMeshStandardMaterial) {
        material.color.set(COLORS.primary);
        material.roughness = 0.45;
        material.metalness = 0.1;
      }
    });

    onReady();
    return holder;
  }, [scene, onReady]);

  useFrame((_state, delta) => {
    const c = controls.current;
    const group = pivot.current;
    if (!group) return;

    if (!c.dragging && Date.now() - c.lastTouch > IDLE_DELAY) {
      c.yaw += IDLE_SPIN * delta;
    }

    // Exponential ease toward the target. Frame-rate independent enough for
    // 60/120 Hz phones without a full spring solver.
    group.rotation.y += (c.yaw - group.rotation.y) * DAMPING;
    group.rotation.x += (c.pitch - group.rotation.x) * DAMPING;

    const target = c.zoom * fitScale;
    const current = group.scale.x;
     group.scale.setScalar(current + (target - current) * DAMPING);
  });

  return (
    <group ref={pivot}>
      <primitive object={model} />
    </group>
  );
}

export type Wearable3DProps = {
  style?: ViewStyle;
  /** Tint of the loading spinner. */
  accentColor?: string;
};

export default function Wearable3D({
  style,
  accentColor = '#7C9CFF',
}: Wearable3DProps) {
  const [loading, setLoading] = useState(true);

  const controls = useRef<Controls>({
    yaw: 0,
    pitch: 0,
    zoom: 1,
    dragging: false,
    lastTouch: 0,
  });

  const gesture = useMemo(() => {
    // runOnJS keeps the callbacks on the JS thread, where the three.js scene
    // lives — otherwise the ref writes happen on the UI thread and are lost.
    const pan = Gesture.Pan()
      .runOnJS(true)
      .onBegin(() => {
        controls.current.dragging = true;
      })
      .onChange((e) => {
        const c = controls.current;
        c.yaw += e.changeX * 0.01;
        c.pitch = clamp(c.pitch + e.changeY * 0.01, -PITCH_LIMIT, PITCH_LIMIT);
        c.lastTouch = Date.now();
      })
      .onFinalize(() => {
        controls.current.dragging = false;
        controls.current.lastTouch = Date.now();
      });

    const pinch = Gesture.Pinch()
      .runOnJS(true)
      .onChange((e) => {
        const c = controls.current;
        c.zoom = clamp(c.zoom * e.scaleChange, ZOOM_MIN, ZOOM_MAX);
        c.lastTouch = Date.now();
      })
      .onFinalize(() => {
        controls.current.lastTouch = Date.now();
      });

    return Gesture.Simultaneous(pan, pinch);
  }, []);

  const handleReady = useMemo(() => () => setLoading(false), []);

  return (
    <View style={[styles.container, style]}>
      <Canvas
        camera={{ position: [0, 0, 5], fov: 40 }}
        gl={{ antialias: true }}
        onCreated={({ gl }) => {
          gl.setClearColor(0x000000, 0);
        }}
      >
        <ambientLight intensity={0.7} />
        <directionalLight position={[4, 5, 6]} intensity={1.7} />
        <directionalLight position={[-5, -2, -4]} intensity={0.55} />
        <directionalLight position={[0, 6, -5]} intensity={0.4} />

        <Suspense fallback={null}>
          <WearableMesh controls={controls} onReady={handleReady} />
        </Suspense>
      </Canvas>

      {/* Touch surface sits above the Canvas so r3f's PanResponder
          never claims the gesture. */}
      <GestureDetector gesture={gesture}>
        <View style={StyleSheet.absoluteFill} collapsable={false} />
      </GestureDetector>

      {loading && (
        <View style={styles.overlay} pointerEvents="none">
          <ActivityIndicator size="large" color={accentColor} />
        </View>
      )}
    </View>
  );
}

const styles = StyleSheet.create({
  container: {
    flex: 0,
    overflow: 'hidden',
  },
  overlay: {
    ...StyleSheet.absoluteFill,
    alignItems: 'center',
    justifyContent: 'center',
  },
});