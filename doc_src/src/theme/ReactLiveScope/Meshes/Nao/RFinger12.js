/*
Auto-generated by: https://github.com/pmndrs/gltfjsx
*/


import { useGLTF } from '@react-three/drei';
import RFinger12Mesh from './RFinger12.glb';

export default function Model(props) {

  const { nodes, materials } = useGLTF(RFinger12Mesh)
  return [{type:'raw', geometry:nodes.RFinger12.geometry, material:materials.RFinger12UV,scale:[0.01, 0.01, 0.01]}]
}

useGLTF.preload(RFinger12Mesh)