"use client";
import * as THREE from "three";
import { createRoot } from "react-dom/client";
import React, { useRef, useState, useEffect } from "react";
import { Canvas, useFrame, ThreeElements } from "@react-three/fiber";
import { CameraControls, OrbitControls, PerspectiveCamera, useHelper } from "@react-three/drei";

function Box(props: ThreeElements["mesh"]) {
  const ref = useRef<THREE.Mesh>(null!);
  const [hovered, hover] = useState(false);
  const [clicked, click] = useState(false);
  useFrame((state, delta) => (ref.current.rotation.x += delta));
  return (
    <mesh
      {...props}
      ref={ref}
      scale={clicked ? 1.5 : 1}
      onClick={(event) => click(!clicked)}
      onPointerOver={(event) => hover(true)}
      onPointerOut={(event) => hover(false)}
    >
      <boxGeometry args={[1, 1, 1]} />
      <meshStandardMaterial color={hovered ? "hotpink" : "orange"} />
    </mesh>
  );
}

function Plane(props: ThreeElements["mesh"]) {
  return (
    <mesh {...props} rotation={[-Math.PI / 2, 0, 0]}>
      <planeGeometry args={[200, 200]} />
      <meshStandardMaterial/>
    </mesh>
  );
}

function Moon(props: ThreeElements["mesh"]) {
  return (
    <mesh {...props}>
      <sphereGeometry args={[10, 32, 32]} />
      <meshStandardMaterial color="grey" />
    </mesh>
  );
}

function Buildings(props: ThreeElements["mesh"]) {
  return (
    <mesh {...props}>
      <boxGeometry args={[20, 1, 20]} />
      <meshStandardMaterial color="black" />
    </mesh>
  );
}

import { Model } from './Model';
function Scene() {
  const camera = useRef<THREE.PerspectiveCamera>(null!);
  useHelper(camera, THREE.CameraHelper);
  useFrame((state, delta) => {
    camera.current?.lookAt(0, 0, 0);
    camera.current.position.z = 200;
    camera.current.position.y = 20;
    });

  return (
    <>
      <OrbitControls />
      <PerspectiveCamera ref={camera} makeDefault  />
      <gridHelper args={[100, 100]} />
      <ambientLight intensity={Math.PI / 2} />
      <directionalLight position={[-10, 10, 10]} intensity={Math.PI / 2} />
      {/* <spotLight position={[10, 10, 10]} angle={0.15} penumbra={1} decay={0} intensity={Math.PI} /> */}
      {/* <pointLight position={[-10, -10, -10]} decay={0} intensity={Math.PI} /> */}
      <Moon position={[0, 45, 0]} />
      {/* <Plane position={[0, 0, 0]} /> */}
      <Model position={[0,1,0]}/>
    </>
  );
}

export default function App() {
  return (
    <div id="root" className="h-screen bg-grey overflow-hidden">
      <Canvas>
        <Scene />
      </Canvas>
    </div>
  );
}
