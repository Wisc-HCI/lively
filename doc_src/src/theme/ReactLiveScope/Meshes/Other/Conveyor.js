import { useGLTF } from '@react-three/drei'
import ConveyorFile from './Conveyor.glb';

export default function Model(props) {
  const { nodes, materials } = useGLTF(ConveyorFile)
  return [
      { type: 'group', rotation: [-Math.PI / 2, 0, 0], children: [
        {
          type:'raw',
          geometry:nodes.Belt.geometry,
          material:materials.BeltMaterial
        },
        {
          type:'raw',
          geometry:nodes.Conveyor.geometry,
          material:materials.ConveyorMaterial
        },
        {
          type:'raw',
          geometry:nodes.InsideBack.geometry,
          material:materials.InsideBackMaterial
        }
      ]}
    ]
}

useGLTF.preload(ConveyorFile)