import RobotiqVisual85FingerLink from './robotiq_85_finger_link.glb';
import { useGLTF } from '@react-three/drei'

export default function Model(props) {
  const { nodes } = useGLTF(RobotiqVisual85FingerLink)
  return (
    [{type:'group',rotation:[-Math.PI/2,0,0],children:[
        {type:'raw',geometry:nodes.mesh_0.geometry, material:nodes.mesh_0.material},
        {type:'raw',geometry:nodes.mesh_0_1.geometry, material:nodes.mesh_0_1.material},
        {type:'raw',geometry:nodes.mesh_0_2.geometry, material:nodes.mesh_0_2.material}
    ]}]
  )
}

useGLTF.preload(RobotiqVisual85FingerLink)