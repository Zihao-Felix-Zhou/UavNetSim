import { useEffect, useMemo, useRef, useState } from 'react'
import { Canvas, useFrame, useThree } from '@react-three/fiber'
import { Grid, Line, OrbitControls } from '@react-three/drei'
import * as THREE from 'three'

const MATERIAL_COLORS = {
  itu_concrete: '#5f6669',
  itu_brick: '#80584d',
  itu_glass: '#4f747c',
  itu_metal: '#8b9294',
  itu_wood: '#74644f',
  itu_medium_dry_ground: '#52604d',
  itu_wet_ground: '#315a68',
}

function CameraTarget({ scene }) {
  const controls = useRef()
  const { camera } = useThree()
  useEffect(() => {
    const scale = Math.max(scene.size_x, scene.size_y)
    const relief = Math.max(0, ...(scene.terrain?.vertices || []).map((point) => point.z))
    camera.position.set(
      scene.size_x * 1.25,
      Math.max(scale, relief + scale * 0.65),
      scene.size_y * 0.75,
    )
    controls.current?.target.set(scene.size_x / 2, relief * 0.28 + 12, -scene.size_y / 2)
    controls.current?.update()
  }, [camera, scene])
  return <OrbitControls ref={controls} makeDefault maxPolarAngle={Math.PI / 2.05} minDistance={40} maxDistance={1800} />
}

function Building({ feature, selected }) {
  const baseHeight = useMemo(
    () => feature.footprint.reduce((total, point) => total + (point.z || 0), 0) / feature.footprint.length,
    [feature],
  )
  const geometry = useMemo(() => {
    const shape = new THREE.Shape()
    feature.footprint.forEach((point, index) => {
      if (index === 0) shape.moveTo(point.x, point.y)
      else shape.lineTo(point.x, point.y)
    })
    shape.closePath()
    const result = new THREE.ExtrudeGeometry(shape, {
      depth: feature.height,
      bevelEnabled: false,
    })
    result.rotateX(-Math.PI / 2)
    return result
  }, [feature])
  return (
    <mesh geometry={geometry} position={[0, baseHeight, 0]} castShadow receiveShadow>
      <meshStandardMaterial
        color={MATERIAL_COLORS[feature.material] || MATERIAL_COLORS.itu_concrete}
        roughness={selected ? 0.36 : 0.62}
        metalness={feature.material === 'itu_metal' ? 0.52 : 0.08}
      />
    </mesh>
  )
}

function Surface({ feature }) {
  const baseHeight = useMemo(
    () => feature.footprint.reduce((total, point) => total + (point.z || 0), 0) / feature.footprint.length,
    [feature],
  )
  const geometry = useMemo(() => {
    const shape = new THREE.Shape()
    feature.footprint.forEach((point, index) => {
      if (index === 0) shape.moveTo(point.x, point.y)
      else shape.lineTo(point.x, point.y)
    })
    shape.closePath()
    const result = new THREE.ShapeGeometry(shape)
    result.rotateX(-Math.PI / 2)
    return result
  }, [feature])
  return (
    <mesh geometry={geometry} position={[0, baseHeight + (feature.category === 'water' ? 0.11 : 0.07), 0]} receiveShadow>
      <meshStandardMaterial
        color={MATERIAL_COLORS[feature.material] || MATERIAL_COLORS.itu_medium_dry_ground}
        roughness={feature.category === 'water' ? 0.28 : 0.96}
        metalness={feature.category === 'water' ? 0.08 : 0}
      />
    </mesh>
  )
}

function Terrain({ terrain }) {
  const geometry = useMemo(() => {
    const result = new THREE.BufferGeometry()
    const elevations = terrain.vertices.map((point) => point.z)
    const minimum = Math.min(...elevations)
    const range = Math.max(1, Math.max(...elevations) - minimum)
    const lowColor = new THREE.Color('#35413c')
    const highColor = new THREE.Color('#647069')
    result.setAttribute('position', new THREE.Float32BufferAttribute(
      terrain.vertices.flatMap((point) => [point.x, point.z, -point.y]),
      3,
    ))
    result.setAttribute('color', new THREE.Float32BufferAttribute(
      terrain.vertices.flatMap((point) => {
        const color = lowColor.clone().lerp(highColor, (point.z - minimum) / range)
        return [color.r, color.g, color.b]
      }),
      3,
    ))
    result.setIndex(terrain.faces.flat())
    result.computeVertexNormals()
    return result
  }, [terrain])
  return (
    <mesh geometry={geometry} receiveShadow>
      <meshBasicMaterial vertexColors side={THREE.DoubleSide} />
    </mesh>
  )
}

function Drone({ node, selected, onSelect }) {
  const group = useRef()
  useFrame((state) => {
    if (group.current) group.current.rotation.y = state.clock.elapsedTime * 0.45 + node.id
  })
  const position = [node.position[0], node.position[2], -node.position[1]]
  return (
    <group ref={group} position={position} onClick={(event) => { event.stopPropagation(); onSelect(node.id) }}>
      <mesh castShadow>
        <sphereGeometry args={[selected ? 3.8 : 3.2, 16, 12]} />
        <meshStandardMaterial color={selected ? '#f3a63a' : '#d9dee0'} emissive={selected ? '#6a3b08' : '#172124'} />
      </mesh>
      <mesh rotation={[0, 0, Math.PI / 2]}><cylinderGeometry args={[0.55, 0.55, 15, 8]} /><meshStandardMaterial color="#6f797c" /></mesh>
      <mesh rotation={[Math.PI / 2, 0, 0]}><cylinderGeometry args={[0.55, 0.55, 15, 8]} /><meshStandardMaterial color="#6f797c" /></mesh>
      {[[7, 0], [-7, 0], [0, 7], [0, -7]].map(([x, z], index) => (
        <mesh key={index} position={[x, 0, z]} rotation={[0, index * 0.3, 0]}>
          <cylinderGeometry args={[3.2, 3.2, 0.28, 20]} />
          <meshStandardMaterial color="#f3a63a" transparent opacity={0.62} />
        </mesh>
      ))}
      <mesh position={[0, -node.position[2] / 2, 0]}>
        <cylinderGeometry args={[0.12, 0.12, node.position[2], 6]} />
        <meshBasicMaterial color="#89a7ad" transparent opacity={selected ? 0.32 : 0.12} />
      </mesh>
    </group>
  )
}

function PacketArc({ arc, nodes }) {
  const pulse = useRef()
  const source = nodes.find((node) => node.id === arc.source)
  const destination = nodes.find((node) => node.id === arc.destination)
  const curve = useMemo(() => {
    if (!source || !destination) return null
    const start = new THREE.Vector3(source.position[0], source.position[2], -source.position[1])
    const end = new THREE.Vector3(destination.position[0], destination.position[2], -destination.position[1])
    const lift = Math.max(15, start.distanceTo(end) * 0.12)
    const middle = start.clone().lerp(end, 0.5).add(new THREE.Vector3(0, lift, 0))
    return new THREE.QuadraticBezierCurve3(start, middle, end)
  }, [destination, source])
  useFrame(() => {
    if (!pulse.current || !curve) return
    const progress = Math.min(1, (Date.now() - arc.createdAt) / 900)
    pulse.current.position.copy(curve.getPoint(progress))
  })
  if (!curve) return null
  const color = arc.status === 'failed' ? '#ef5c50' : arc.status === 'success' ? '#46c8b0' : '#f3a63a'
  return (
    <group>
      <Line points={curve.getPoints(30)} color={color} lineWidth={1.4} transparent opacity={0.7} />
      <mesh ref={pulse}><sphereGeometry args={[1.8, 10, 8]} /><meshBasicMaterial color={color} /></mesh>
    </group>
  )
}

export default function SceneViewport({ scene, nodes, arcs, selectedNode, onSelectNode }) {
  return (
    <Canvas shadows dpr={[1, 1.7]} camera={{ fov: 46, near: 0.5, far: 5000 }}>
      <color attach="background" args={['#171a1c']} />
      <fog attach="fog" args={['#171a1c', 650, 1600]} />
      <ambientLight intensity={0.58} />
      <directionalLight position={[250, 500, 180]} intensity={1.9} castShadow shadow-mapSize={[2048, 2048]} />
      {scene.terrain
        ? <Terrain terrain={scene.terrain} />
        : <mesh position={[scene.size_x / 2, -0.2, -scene.size_y / 2]} rotation={[-Math.PI / 2, 0, 0]} receiveShadow>
          <planeGeometry args={[scene.size_x + 100, scene.size_y + 100]} />
          <meshStandardMaterial color="#24282a" roughness={0.92} />
        </mesh>}
      {!scene.terrain && <Grid position={[scene.size_x / 2, 0, -scene.size_y / 2]} args={[scene.size_x, scene.size_y]} cellSize={20} cellThickness={0.45} cellColor="#42494b" sectionSize={100} sectionThickness={0.8} sectionColor="#5d6668" fadeDistance={900} />}
      {scene.features.filter((feature) => feature.category === 'building').map((feature) => <Building key={feature.id} feature={feature} />)}
      {scene.features.filter((feature) => feature.category === 'water' || (!scene.terrain && feature.category === 'terrain')).map((feature) => <Surface key={feature.id} feature={feature} />)}
      {scene.features.filter((feature) => feature.category === 'road').map((feature) => (
        <Line key={feature.id} points={feature.footprint.map((point) => [point.x, (point.z || 0) + 0.22, -point.y])} color="#697173" lineWidth={3} />
      ))}
      {nodes.map((node) => <Drone key={node.id} node={node} selected={selectedNode === node.id} onSelect={onSelectNode} />)}
      {arcs.map((arc) => <PacketArc key={`${arc.id}-${arc.destination}`} arc={arc} nodes={nodes} />)}
      <CameraTarget scene={scene} />
    </Canvas>
  )
}
