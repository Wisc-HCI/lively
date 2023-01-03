import { useGLTF } from '@react-three/drei'
import FlagFile from './Flag.glb';

export default function Model(props) {
  const { nodes, materials } = useGLTF(FlagFile)
  return [
      { type: 'group', children: [
        {
            type:'raw',
            geometry:nodes.Flag.geometry,
            material:materials.Material
          }
      ]}
    ]
}

useGLTF.preload(FlagFile)
