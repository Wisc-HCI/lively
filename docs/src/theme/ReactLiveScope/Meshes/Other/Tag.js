import { useGLTF } from '@react-three/drei'
import TagFile from './Tag.glb';

export default function Model(props) {
  const { nodes, materials } = useGLTF(TagFile)
  return [
      { type: 'group', children: [
        {
            type:'raw',
            geometry:nodes.Tag.geometry,
            material:materials.Material
          }
      ]}
    ]
}

useGLTF.preload(TagFile)

