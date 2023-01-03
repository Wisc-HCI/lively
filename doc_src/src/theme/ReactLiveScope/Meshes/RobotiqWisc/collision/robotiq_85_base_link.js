import RobotiqCollision85BaseLink from './robotiq_85_base_link.glb';
import { useGLTF } from '@react-three/drei'

export default function Model(props) {
  const { nodes } = useGLTF(RobotiqCollision85BaseLink)
  return (
    [{ type: 'group', rotation:[-Math.PI/2,0,0], children: [{ type: 'raw', geometry: nodes.mesh_0.geometry, material: nodes.mesh_0.material }] }]
  )
}

useGLTF.preload(RobotiqCollision85BaseLink)