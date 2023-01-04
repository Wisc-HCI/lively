/*
Auto-generated by: https://github.com/pmndrs/gltfjsx
*/

import { useGLTF } from '@react-three/drei'
import mesh from './camera_visor.glb'

export default function Model() {
  const { nodes } = useGLTF(mesh)

  return [{ type: 'group', children: [{ type: 'raw', geometry: nodes.camera_visor.geometry, material: nodes.camera_visor.material }]}]
}

useGLTF.preload(mesh)