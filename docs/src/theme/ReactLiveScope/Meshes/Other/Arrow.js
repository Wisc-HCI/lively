import { useGLTF } from '@react-three/drei'
import ArrowFile from './Arrow.glb';

export default function Model(props) {
  const { nodes, materials } = useGLTF(ArrowFile)
  return [{type:'raw',geometry:nodes.Arrow.geometry,material:materials['Default OBJ'],scale:[1,1,1]}]
}

useGLTF.preload(ArrowFile)
